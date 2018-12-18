/************************************************
ジャンプ制御部
ロータリーエンコーダー入力ピン1	:RA0 ->encoder RC0 //左回転//正しい回転方向
ロータリーエンコーダー入力ピン2	:RA1 ->encoder RC1 //右回転//逆
ロータリーエンコーダー入力ピン3	:RA2 ->encoder RC2 //停止信号
スイッチ入力ピン				:RA3
フォトインタラプタ入力ピン		:RA4
移動中入力ピン				:RA5 <-移動モーター制御部より
1回ジャンプ入力ピン			:RB0 <-センサー制御部より
無限ジャンプ入力ピン			:RB1 <-センサー制御部より
スイッチ確認LED出力ピン		:RB7
特別動作中LED出力ピン		:RB3
エラーLEDピン				:RB4
巻取り中出力ピン			:RB5 ->移動モーター制御部へ
特別動作中出力ピン			:RB6 ->センサー制御部へ
特別動作前進出力ピン		:RB2 ->移動モーター制御部へ
じぇじぇじぇ出力ピン			:RC4 ->音声制御部へ
ダミースイッチ入力ピン			:RC6 <-ダミースイッチ
タクトスイッチ入力ピン			:RC7 <-スイッチ
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


//TRYマクロですわ
#define TRY				/* TRY-CATCH */
#define THROW(name)		goto TRY_TAG_ERROR_##name
#define CATCH(name)		goto TRY_TAG_RESUME_LAST; TRY_TAG_ERROR_##name:
#define FINALLY			TRY_TAG_RESUME_LAST:

#define bool int1
#define LEFTTURN 		pin_a0//左回り入力
#define RIGHTTURN 		pin_a1//右回り入力
#define STOPTURN 		pin_a2//停止 入力
#define SWITCHINPUT 	pin_a3//スイッチ入力
#define INTERRUPT	 	pin_a4//フォトインタラプタ入力
#define MOVING 			pin_a5//移動中入力
#define ONEJUMP 		pin_b0//一回ジャンプ（センサー制御部から入力）
#define INFINITYJUMP 	pin_b1//複数回ジャンプ（センサー制御部から入力）
#define SPFORWARD 		pin_b2//特別シーケンスで前進するときに出力する
#define SPECIALLED 		pin_b3//特別シーケンスを行っているときに光る
#define ERRLED 			pin_b4//エラーが起こると光る
#define WINDING 		pin_b5//巻取り中に出力
#define SEQUENCEMODE 	pin_b6//特別シーケンスを行っているときに出力する
#define SWITCHLED 		pin_b7//スイッチを認識した時に光らせる
#define JEJEJEOUT		pin_c4//ジャンプするときにじぇじぇじぇ
#define DAMMYSWITCH		pin_c6//ダミースイッチ入力
#define TACTSWITCH		pin_c7//タクトスイッチで性回転の微調整
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms

//input(SWWITCHINPT)	スイッチが入るとTrue
//input(INTERRUPT)		切れ目の時True
//input(MOVING)			移動中の時True
//input(〜〜TURN)		Trueのとき、その方向に回転してる,STOPは止まってる



void timing_bit(bool bits){
	output_bit(SWITCHLED,bits);
	output_bit(SPECIALLED,bits);
	output_bit(ERRLED,bits);
}

void initializing(void){
	bool delayflag;//ちかちかにのみ使う
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS設定
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//回転停止でスタート
	set_tris_a(0x3F);//0b00111111
	set_tris_b(0x03);//0b00000011
	set_tris_c(0xC0);//0b11000000
	Setup_timer_0(T0_DIV_256);//1count 16us

	//起動待機処理
	output_high(SEQUENCEMODE);//モーターとか動かないように
	while(!input(TACTSWITCH)){//押すの待機
		timing_bit(++delayflag);//ちかちか
		delay_ms(200);
	}
	timing_bit(false);
	while( input(TACTSWITCH));//離すの待機
	output_low (SEQUENCEMODE);//起動待機終了
	while(input(MOVING)){}//まさか動いてねぇよな？
	//起動待機処理終了
}

void change_c(int port){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(deadtime);
	output_c((input_c() & 0xf0) | port);
}
void Exception_ERR(long ms=1000){
	change_c(0);//出力なし
	output_low (WINDING);
	output_low (SEQUENCEMODE);
	output_low (SPFORWARD);
	while(true){
		output_toggle(ERRLED);//無限ループ
		delay_ms(ms);
	}
}


//巻き上げ処理、これでジャンプも行う
int Sequence_Winding(unsigned long num=1){//0の時は巻き上げのみ行う
	unsigned int timecounter =0;//15625カウント(0.25秒)で1インクリメント
	unsigned long jumpcounter =0;//ジャンプの回数を数える。
	unsigned int timingcounter=0;//タイミングカウンタ、timecounterとの違いはいちいち飛ぶたびに初期化される
	bool edgeflag ;
	int duty=20;
	int nonduty=30;
	TRY{
		change_c( WISESTOP);
		while(!input(STOPTURN)){//ジャンプモーター停止待ち
		}
		
		output_high(WINDING);
		if(num==0&&input(INTERRUPT)){//フラグがあり、すでに切れ目にいるならまわさない
			THROW(END);//関数処理終了
		}
		
		if(num==0)num+=1;//切れ目の位置にあるのでなければ1回と同じ扱い
		
		edgeflag=input(INTERRUPT);
		change_c(CLOCKWISE);//巻き上げを行う
		set_timer0(0);
		do{//while(jumpcounter<num);
			if(get_timer0()>=15625L){//指定回数で
				set_timer0(0);//タイマリセットして
				if(timecounter<uINT_MAX)timecounter++;//限界行ったら数えるのあきらめる
				timingcounter++;//インクリメント
				if(nonduty > 0){//まだPWM中だったら
					duty+=5;//Dutyの比を増やす
					nonduty-=5;//減らす
				}
			}
			
			if(nonduty > 0){//まだPWMが終わってなければ
				output_c((input_c() & 0xf0) | 0);//pwm
				delay_us(nonduty);
			}
			output_c((input_c() & 0xf0) | CLOCKWISE);//pwm
			delay_us(duty);

			//ストールの疑い,停止信号が来ていたら
			if(timecounter >= 7  && input(STOPTURN) ){//約1.75秒後に停止信号が来ていたらエラー
				THROW(ERR);//ERR Throw
			}
			//タイミングをとる
			if(timingcounter >= 7){
				timing_bit(true);
			}else{
				timing_bit(false);
			}

			if(edgeflag!=input(INTERRUPT)){//インタラプタに変化があった時
				edgeflag=input(INTERRUPT);//0なら切れ目から抜ける時、1なら切れ目に入る時
				if(num==0){//切れ目探しのときはラチェット入る前のずれを考慮した時間を入れない
					if(edgeflag){//切れ目に入ったら
						jumpcounter++;//ジャンプしたとみなすとループ終了
						timingcounter=0;
					}
				}else{
					if(edgeflag&&timecounter >= 7){//動き出した後切れ目に入ったら
						jumpcounter++;//ジャンプしたとみなす
						timingcounter=0;
					}
				}
			}
			
		}while(jumpcounter<num);//多くなったら抜ける
		
	}CATCH(ERR){
		Exception_ERR();
		return 1;
	}CATCH(END){
		//do nothing
	}FINALLY{
		change_c( WISESTOP);//停止
		output_low(WINDING);//巻取り終了ー
		return 0;//正常終了
	}
}

void Sequence_Onejump(){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//前進
	output_high(SPFORWARD);
	delay_ms(1000);
	timing_bit(false);
	delay_ms(1000);
	output_low (SPFORWARD);//二秒で停止
	timing_bit(true);
	delay_ms(200);
	
		//ジャンプ
		Sequence_Winding(1);
	
	//前進
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//スペシャルモード終了
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Infinityjump(unsigned long cont = uLONG_MAX){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//前進
	output_high(SPFORWARD);
	delay_ms(1000);
	timing_bit(false);
	delay_ms(1000);
	output_low (SPFORWARD);//2秒で停止
	timing_bit(true);
	delay_ms(200);
	
		//ジャンプ
		Sequence_Winding(cont);//無限の時はuLONG_MAX
	
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Twojump(){
	output_high(SWITCHLED);
	output_high(SEQUENCEMODE);
	//縄が来るの待機
	delay_ms(200);
	timing_bit(false);
	delay_ms(400);
	timing_bit(true);
	delay_ms(200);//total=800
	
		//ジャンプ
		Sequence_Winding(2);
	
	output_low (SWITCHLED);
	//着地待機
	delay_ms(400);
	output_low (SEQUENCEMODE);
}
int main(void)
{
	int duty;
	int nonduty;
	bool movingflag=true;//エッジ検出のみに使う
	int timecounter=0;
	delay_ms(100);
	initializing();

	//起動時の巻き上げ
	Sequence_Winding(0);
	
	while(true)
	{
		
		if(movingflag!=input(MOVING)){
			movingflag=input(MOVING);
			if(!movingflag){//移動中ではなくなったとき
				set_timer0(0);
				timecounter=0;
			}
		}
		
		if(!input(MOVING)){
			//移動中でない時
			if(input(TACTSWITCH)){//微調整スイッチ...いきいますよー
				//duty初期化
				duty=20;
				nonduty=30;
				//巻き上げ開始
				output_high(WINDING);
				change_c(CLOCKWISE);
				set_timer0(0);
				while(input(TACTSWITCH)){
					if(get_timer0() > 30000L && nonduty !=0){
						set_timer0(0);
						duty+=5;
						nonduty-=5;//いずれは0になる
					}
					output_c((input_c() & 0xf0) | CLOCKWISE);
					delay_us(duty);
					if(nonduty!=0) {
						output_c((input_c() & 0xf0) | 0);
						delay_us(nonduty);
					}
				}//動かすんです。
				change_c(WISESTOP);
				output_low (WINDING);
			}
		}
		
		if(input(ONEJUMP)){
			Sequence_Onejump();
			movingflag=true;//管轄外に行ったので動いていたことにする
		}
		
		if(input(INFINITYJUMP)){
			Sequence_Infinityjump(5);
			movingflag=true;//管轄外に行ったので動いていたことにする
		}
		
		if(get_timer0() >= 62500L){
			set_timer0(0);
			if(timecounter!=uINT_MAX)timecounter++;
		}
		
		if((!input(MOVING))&& timecounter >= 1 && input(SWITCHINPUT) && !input(DAMMYSWITCH)){
			output_high(JEJEJEOUT);
			Sequence_Twojump();
			output_low (JEJEJEOUT);
			movingflag=true;//管轄外に行ったので動いていたことにする
		}
	}
return 0;
}
