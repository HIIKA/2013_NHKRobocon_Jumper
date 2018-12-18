/************************************************
ジャンプ制御部
スイッチ入力ピン				:RA3
フォトインタラプタ入力ピン		:RA4
移動中入力ピン				:RA5 <-移動モーター制御部より
ロータリーエンコーダー入力ピン1	:RA0 ->encoder RC0 //左回転//正しい回転方向
ロータリーエンコーダー入力ピン2	:RA1 ->encoder RC1 //右回転//逆
ロータリーエンコーダー入力ピン3	:RA2 ->encoder RC2 //停止信号
1回ジャンプ入力ピン			:RB0 <-センサー制御部より
無限ジャンプ入力ピン			:RB1 <-センサー制御部より
スイッチ確認LED出力ピン		:RB7
特別動作中LED出力ピン		:RB3
エラーLEDピン				:RB4
巻取り中出力ピン			:RB5 ->移動モーター制御部へ
特別動作中出力ピン			:RB6 ->センサー制御部へ
特別動作前進出力ピン		:RB2 ->移動モーター制御部へ
じぇじぇじぇ出力ピン			:RC4 ->音声制御部へ
モーター制御出力ピン1			:RC0
モーター制御出力ピン2			:RC1
モーター制御出力ピン3			:RC2
モーター制御出力ピン4			:RC3

************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(all)

#define SWITCHINPUT 	pin_a3//スイッチ入力
#define INTERRUPT	 	pin_a4//フォトインタラプタ入力
#define MOVING 			pin_a5//移動中入力
#define LEFTTURN 		pin_a0//左回り入力
#define RIGHTTURN 		pin_a1//右回り入力
#define STOPTURN 		pin_a2//停止 入力
#define ONEJUMP 		pin_b0//一回ジャンプ（センサー制御部から入力）
#define INFINITYJUMP 	pin_b1//複数回ジャンプ（センサー制御部から入力）
#define SWITCHLED 		pin_b7//スイッチを認識した時に光らせる
#define SPECIALLED 		pin_b3//特別シーケンスを行っているときに光る
#define ERRLED 			pin_b4//エラーが起こると光る
#define WINDING 		pin_b5//巻取り中に出力
#define SEQUENCEMODE 	pin_b6//特別シーケンスを行っているときに出力する
#define SPFORWARD 		pin_b2//特別シーケンスで前進するときに出力する
#define JEJEJEOUT		pin_c4//ジャンプするときにじぇじぇじぇ
#define TACTSWITCH		pin_c7//タクトスイッチで性回転の微調整
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms

//input(SWWITCHINPT)	スイッチが入るとTrue
//input(INTERRUPT)		切れ目の時True
//input(MOVING)			移動中の時True
//input(～～TURN)		Trueのとき、その方向に回転してる,STOPは止まってる



void initializing(void){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS設定
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//回転停止でスタート
	set_tris_a(0x1F);//0b00011111
	set_tris_b(0x03);//0b00000011
	set_tris_c(0x80);//0b10000000
	Setup_timer_0(T0_DIV_256);//1count 16us
}

void change_c(int port){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(deadtime);
	output_c((input_c() & 0xf0) | port);
}
void Exception_ERR(void){
	change_c(0);//出力なし
	output_low (WINDING);
	output_low (SEQUENCEMODE);
	output_low (SPFORWARD);
	while(true){
		output_toggle(ERRLED);//無限ループ
		delay_ms(1000);
	}
}

//巻き上げ処理、これでジャンプも行う
int Sequence_Winding(int1 flag=0){//切れ目を見つけたいだけならflag1
	int counter =0;
	
	change_c( WISESTOP);
	while(!input(STOPTURN)){//ジャンプモーター停止待ち
		output_toggle(ERRLED);delay_ms(100);
	}
	output_low (ERRLED);
	
	output_high(WINDING);
	if(flag&&input(INTERRUPT)){//フラグがあり、すでに切れ目にいるならまわさない
		change_c( WISESTOP);//巻き上げ停止
		output_low (WINDING);
		return 0;//正常終了
	}
	
	change_c(CLOCKWISE);//巻き上げを行う
	
	set_timer0(0);
	while(input(INTERRUPT)){//今切れ目にあるなら
		if(get_timer0() > 20500){//300ms以内に切れ目から抜けないとエラー
			Exception_ERR();
			return 1;
		}
		output_c((input_c() & 0xf0) | CLOCKWISE);//pwm
		delay_us(19);
		output_c((input_c() & 0xf0) | 0);//pwm
		delay_us(25);
	}
	
	change_c( CLOCKWISE);//PWMで0になっても良いよう再度巻き上げを行う
	set_timer0(0);
	while(!input(INTERRUPT)){//歯車の切れ目に行くまで巻き上げ
		if(get_timer0()>=(LONG_MAX-100)){//オーバーフロー直前
			set_timer0(0);//タイマリセットして
			counter++;//インクリメント
		}
		if(counter == 1 && get_timer0() > 30000L){//約1.5秒以内にまた切れ目を見つけれないとエラー
			Exception_ERR();
			return 1;
		}
		//ストールの疑い,0.5秒たっても停止信号が来ていたら
		if( input(INTERRUPT) &&( ( (get_timer0() >= 30000 && counter == 0) || counter >= 1) && input(STOPTURN) ) ){
			Exception_ERR();
			return 1;
		}
	}
	change_c( WISESTOP);//停止
	output_low(WINDING);//巻取り終了ー
	return 0;
}

void Sequence_Onejump(){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//前進
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//JUMP
	output_high(JEJEJEOUT);
	//ジャンプ
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//前進
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//スペシャルモード終了
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Infinityjump(){
	int i;
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//前進
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);

	for(i=0;i<5;i++){
		output_high(JEJEJEOUT);
		//ジャンプ
		Sequence_Winding();
		output_low (JEJEJEOUT);
	}
	
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Twojump(){
	output_high(SWITCHLED);
	//縄が来るの待機
	delay_ms(600);
	
	//じぇじぇじぇ
	output_high(JEJEJEOUT);
	//ジャンプ
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//縄が来るのまた待機
	delay_ms(600);
	
	//再度ジャンプ
	output_high(JEJEJEOUT);
	//ジャンプ
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//着地待機
	delay_ms(800);
	output_low (SWITCHLED);
}
int main(void)
{
	initializing();
	
	while(input(MOVING)){}
	
	//起動時の巻き上げ
	Sequence_Winding(true);
	
	while(true)
	{
		if(input(MOVING)){
			continue;
		}else{//移動中でない時
			if(input(TACTSWITCH)){//微調整スイッチ...いきいますよー
				output_high(WINDING);
				change_c(CLOCKWISE);
				while(input(TACTSWITCH)){}//動かすんです。
				change_c(WISESTOP);
				output_low (WINDING);
			}
		}
		
		if(input(ONEJUMP)){
			Sequence_Onejump();
		}
		
		if(input(INFINITYJUMP)){
			Sequence_Infinityjump();
		}
		
		if((!input(MOVING))&&input(SWITCHINPUT)){
			Sequence_Twojump();
		}
		
		
	}
return 0;
}
