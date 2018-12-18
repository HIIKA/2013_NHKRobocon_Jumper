/************************************************
ジャンプ制御部
インタラプタ確認出力			:RA0 ->encoder RC6 
カウント入力						:RA1 ->encoder RC5 
ロータリーエンコーダー入力ピン3	:RA2 ->encoder RC4 //停止信号
スイッチ入力ピン				:RA3
ダミー入力ピン					:RA4
フォトインタラプタ入力ピン		:RA5
起動タクト	スイッチピン		:RA6
無限回モード入力ピン			:RA7
解放タクトスイッチ入力			:RB0 
エラーLED			:RB1 
1回ジャンプ入力		:RB2 
無限ジャンプ入力ピン		:RB3 
特別動作前進出力				:RB4
巻取り中出力ピン			:RB5 
特別動作中出力ピン			:RB6 
移動中入力			:RB7 
モーター制御出力ピン1			:RC0
モーター制御出力ピン2			:RC1
モーター制御出力ピン3			:RC2
モーター制御出力ピン4			:RC3
じぇじぇじぇ出力ピン			:RC4 
はーい！出力ピン				:RC5 
シリアル通信TX1					:RC6
シリアル通信RX1					:RC7
************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use RS232(baud=15200,xmit=pin_c6,rcv=pin_c7,INVERT)

#use fast_io(all)



//TRYマクロですわ
#define TRY				/* TRY-CATCH */
#define THROW(name)		goto TRY_TAG_ERROR_##name
#define CATCH(name)		goto TRY_TAG_RESUME_LAST; TRY_TAG_ERROR_##name:
#define FINALLY			TRY_TAG_RESUME_LAST:

#define bool int1
#define CONFIRMOUT 		pin_a0//確認出力
#define COUNTIN 		pin_a1//カウント入力
#define STOPTURN 		pin_a2//停止 入力
#define SWITCHINPUT 	pin_a3//スイッチ入力
#define DAMMYSWITCH		pin_a4//ダミースイッチ入力
#define INTERRUPT	 	pin_a5//フォトインタラプタ入力
#define INITIALSWITCH	pin_a6//起動するときに押すスイッチ
#define INFINITYMODE	pin_a7//無限回ジャンプの時にHigh
#define TACTSWITCH		pin_b0//タクトスイッチで性回転する
#define ERRLED 			pin_b1//エラーが起こると光る
#define ONEJUMP 		pin_b2//一回ジャンプ（センサー制御部から入力）
#define INFINITYJUMP 	pin_b3//複数回ジャンプ（センサー制御部から入力）
#define SPFORWARD 		pin_b4//特別シーケンスで前進するときに出力する
#define WINDING 		pin_b5//巻取り中に出力
#define SEQUENCEMODE 	pin_b6//特別シーケンスを行っているときに出力する
#define MOVING 			pin_b7//移動中入力
#define JEJEJEOUT		pin_c4//ジャンプするときにじぇじぇじぇ
#define HEYOUT			pin_c5//はーい！
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms
#define SWITCHCHAR		'A'//スイッチ認識信号文字
#define ENDSWITCHCHAR	'a'//スイッチ認識終了
#define SPBEGINCHAR		'B'//自動動作に入った時の文字
#define SPENDCHAR		'b'//自動動作が終了した時の文字
#define WAITINGCHAR		'C'//起動待機時の文字
#define WAITEDCHAR		'c'//起動完了時の文字
#define ERRCHAR			'E'//エラーが出ている時の文字
#define FORWARDCHAR		'F'//前進命令が出た時の文字
#define ENDFORWARDCHAR	'f'//前進命令終了
#define SIGNALCHAR		'G'//ジャンプ
#define ENDSIGNALCHAR	'g'//ジャンプタイミング消す
#define CLEARCHAR		'H'//信号CLEAR
#define INFCLEARCHAR	'h'//無限回ジャンプの時の信号CLEAR
#define YELLOWCHAR		'I'//黄色に光る
#define ENDYELLOWCHAR	'i'//黄色消える
#define COUNT1			'j'
#define COUNT2			'k'
#define COUNT3			'l'
#define COUNTEND		'm'
bool infinity_flag=0;



//input(SWWITCHINPT)	スイッチが入るとTrue
//input(INTERRUPT)		切れ目の時True
//input(MOVING)			移動中の時True
//input(～～TURN)		Trueのとき、その方向に回転してる,STOPは止まってる



void timing_bit(bool bits){
	if(bits){
		putc(SIGNALCHAR);
	}else{
		putc(ENDSIGNALCHAR);
	}
}

void yellow_blink(bool bits){
	if(bits){
		putc(YELLOWCHAR);
	}else{
		putc(ENDYELLOWCHAR);
	}
}


void initializing(void){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS設定
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//回転停止でスタート
	//a bはtrisを設定しない
	set_tris_a(0xfe);//0b11111110
	set_tris_b(0x8d);//0b10001101
	set_tris_c(0x80);//0b10000000
	Setup_timer_0(T0_DIV_256);//1count 16us
	
	if(input(INFINITYMODE)){
		putc(INFCLEARCHAR);//起動時無限回時信号クリア
		infinity_flag=true;
	}else{
		putc(CLEARCHAR);//起動時信号クリア
		infinity_flag=false;
	}
	
	//起動待機処理
	output_high(SEQUENCEMODE);//モーターとか動かないように
	putc(WAITINGCHAR);
	while( input(INITIALSWITCH));//離すの待機
	while(!input(INITIALSWITCH));//押すの待機
	putc(WAITEDCHAR);
	while( input(INITIALSWITCH));//離すの待機
	delay_ms(300);
	output_low (SEQUENCEMODE);//起動待機終了
	while(input(MOVING)){}//まさか動いてねぇよな？
	//起動待機処理終了
}

void change_c(int port){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(DEADTIME);
	output_c((input_c() & 0xf0) | port);
}
void Exception_ERR(void){
	change_c(0);//出力なし
	output_low (WINDING);
	output_high (SEQUENCEMODE);//動かないようにセンサ停止
	output_low (SPFORWARD);
	putc(ERRCHAR);
	output_high(ERRLED);
	while(true){
		if(! input(MOVING) && input(INITIALSWITCH)){
			reset_cpu();//復帰できるようになりました
		}
	}
}


//巻き上げ処理、これでジャンプも行う
int Sequence_Winding(unsigned long num=1){//0の時は巻き上げのみ行う
	unsigned long jumpcounter =0;//ジャンプの回数を数える。
	unsigned int timecounter =0;//12500カウント(0.2秒)で1インクリメント
	unsigned int timingcounter=0;//タイミングカウンタ、timecounterとの違いはいちいち飛ぶたびに初期化される
	unsigned int stopcounter=0;//モーターが停止している回数を0.2秒おきに数える
	bool edgeflag=false ;//インタラプタのエッジを見るのに使う
	bool gapflag=false ;//巻き上げのみの時のフラグの引き継ぎ
	int duty=20;
	int nonduty=30;
	TRY{
		change_c( WISESTOP);
		while(!input(STOPTURN)){//ジャンプモーター停止待ち
		}
		if(num==0){
			gapflag=true;//切れ目さがし
			num+=1;//切れ目の位置にあるのでなければ1回と同じ扱い
		}
		
		output_high(WINDING);
		if(gapflag&&input(INTERRUPT)){//フラグがあり、すでに切れ目にいるならまわさない
			THROW(END);//関数処理終了
		}
		
		output_high(CONFIRMOUT);//タイマーリセット命令
		edgeflag=input(INTERRUPT);
		change_c(CLOCKWISE);//巻き上げを行う
		set_timer0(0);
		do{//while(jumpcounter<num);
			if(get_timer0()>=12500L){//指定回数で
				set_timer0(0);//タイマリセットして
				if(timecounter<uINT_MAX)timecounter++;//限界行ったら数えるのあきらめる
				timingcounter++;//インクリメント
				//ストール検出
				if(input(STOPTURN)){
					stopcounter++;//インクリメント
					if(stopcounter>=5){//1.0秒止まっていたらストールの疑い
						THROW(ERR);//ERR Throw
					}
				}else{
					stopcounter=0;//停止信号が来ていなければカウンタリセット;
				}
				//PWM処理
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
			
			//タイミングをとる
			if(timingcounter >= 3){
				timing_bit(true);
			}else{
				timing_bit(false);
			}
			
			if(edgeflag!=input(INTERRUPT)){//インタラプタに変化があった時
				edgeflag=input(INTERRUPT);//0なら切れ目から抜ける時、1なら切れ目に入る時
				if(edgeflag){//切れ目に入ったら
					if(gapflag){//切れ目探しのときはおしまい
						break;//ジャンプしたとみなすとループ終了
					}
					if(timecounter >= 3 && input(COUNTIN)){//動き出しから一定時間経って一定距離進んでいたら
						jumpcounter++;//ジャンプしたとみなす
						timingcounter=0;
						output_high(CONFIRMOUT);//タイマーリセット命令
					//	duty=0;//PWMreset
					//	nonduty=50;
					}
				}
			}
			
			if(!input(COUNTIN)){//回数入力がlowなら
				output_low(CONFIRMOUT);//確認してない
			}
			
		}while(jumpcounter<num);//多くなってたら
		
	}CATCH(ERR){
		Exception_ERR();
		return 1;
	}CATCH(END){
		//do nothing
	}FINALLY{
		change_c( WISESTOP);//停止
		output_low(WINDING);//巻取り終了ー
		timing_bit(false);//タイミングとるのは消す
		return 0;//正常終了
	}
}

void Sequence_Onejump(void){
	int i;
	putc(SPBEGINCHAR);
	output_high(SEQUENCEMODE);
	output_high(HEYOUT);
	
	delay_ms(500);
	putc(COUNT1);
	delay_ms(500);
	putc(COUNT2);
	delay_ms(500);
	putc(COUNT3);
	delay_ms(500);
	putc(COUNTEND);
		//ジャンプ
		Sequence_Winding(1);
	
	//スペシャルモード終了
	putc(SPENDCHAR);
	output_low (SEQUENCEMODE);
	output_low (HEYOUT);
}

void Sequence_Infinityjump(unsigned long cont = uLONG_MAX){
	int i;
	putc(SPBEGINCHAR);
	output_high(SEQUENCEMODE);
	output_high(HEYOUT);
	
	delay_ms(500);
	putc(COUNT1);
	delay_ms(500);
	putc(COUNT2);
	delay_ms(500);
	putc(COUNT3);
	delay_ms(500);
	putc(COUNTEND);
	
		//ジャンプ
		Sequence_Winding(cont);//無限の時はuLONG_MAX
	
	putc(SPENDCHAR);
	output_low (HEYOUT);
	output_low (SEQUENCEMODE);
}

void Sequence_Twojump(void){
	putc(SWITCHCHAR);
	output_high(SEQUENCEMODE);
	//縄が来るの待機
	delay_ms(200);
	timing_bit(false);
	delay_ms(400);
	timing_bit(true);
	delay_ms(200);//total=800
	
		//ジャンプ
		Sequence_Winding(2);
	
	putc(ENDSWITCHCHAR);
	//着地待機
	delay_ms(400);
	output_low (SEQUENCEMODE);
}

void Sequence_Auto2j(void){
	putc(SPBEGINCHAR);
	output_high(SEQUENCEMODE);
	putc(FORWARDCHAR);
	output_high(SPFORWARD);
	delay_ms(1400);
	timing_bit(false);
	delay_ms(400);
	timing_bit(true);
	delay_ms(200);//total=2000
	putc(ENDFORWARDCHAR);
	output_low(SPFORWARD);
	output_high(JEJEJEOUT);
		//ジャンプ
		Sequence_Winding(2);
	output_low (JEJEJEOUT);
	putc(SPENDCHAR);
	//着地待機
	delay_ms(400);
	output_low (SEQUENCEMODE);
}

void Sequence_ManualTurn(void){
	//duty初期化
	int duty=20;
	int nonduty=30;
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

int main(void)
{
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
				Sequence_Winding(0);//脚が下りたのを直す
			}
		}
		
		if(! input(MOVING) && ! movingflag &&  timecounter >= 1){
			//移動中でない時
			if(input(TACTSWITCH)){//微調整スイッチ...いきいますよー
				Sequence_ManualTurn();
			}
			//起動巻き上げをしたいときには
			if(input(INITIALSWITCH)){
				Sequence_Winding(0);
			}
		}
		
		if(input(ONEJUMP)){
			if(input(INFINITYJUMP)){
				Sequence_Auto2j();
				movingflag=true;//管轄外に行ったので動いていたことにする
				timecounter=0;//一応時間リセット
				continue;
			}else{//if(input(ONEJUMP)&& ! input(INFINITYJUMP))
				Sequence_Onejump();
				movingflag=true;//管轄外に行ったので動いていたことにする
				timecounter=0;//一応時間リセット
				continue;
			}
		}else{// !input(ONEJUMP)&&input(INFINITYJUMP)
			if(input(INFINITYJUMP)){
				if(infinity_flag){//無限ジャンプするべきか否か
					Sequence_Infinityjump(uLONG_MAX);
				}else{
					Sequence_Infinityjump(6);
				}
				
				movingflag=true;//管轄外に行ったので動いていたことにする
				timecounter=0;//一応時間リセット
				continue;
			}
		}
		
		if(get_timer0() >= 62500L){
			set_timer0(0);
			if(timecounter!=uINT_MAX)timecounter++;
		}
		
		if(! movingflag && !input(MOVING) && timecounter >= 1 && input(SWITCHINPUT) && !input(DAMMYSWITCH)){
			output_high(JEJEJEOUT);
			Sequence_Twojump();
			output_low (JEJEJEOUT);
			movingflag=true;//管轄外に行ったので動いていたことにする
			timecounter=0;//一応時間リセット
			continue;
		}
	}
return 0;
}
