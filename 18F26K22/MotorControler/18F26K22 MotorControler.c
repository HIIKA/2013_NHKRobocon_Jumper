/************************************************
モーター制御部
モーター制御出力① 	:RC0～RC3
モーター制御出力② 	:RC4～RC7
移動中出力ピン		:RB7 ->ジャンプ制御部へ
巻取中入力ピン		:RB6 <-ジャンプ制御部へ
CCP1			:RA4(CCP5)->LED
CCP2			:RB5(CCP3)->LED
エラーLEDピン		:RB4
モーター前進入力 	:RB3 <-センサー制御部より
モーター後進入力 	:RB2 <-センサー制御部より
モーター面舵入力 	:RB1 <-センサー制御部より
モーター取舵入力 	:RB0 <-センサー制御部より


************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(all)


#define change_pin1 pin_a0//a0は左側
#define change_pin2 pin_a1//a1は右側
#define MOVINGPIN Pin_b7
#define WINDINGPIN Pin_b6
#define CCP1PIN Pin_a4
#define CCP2PIN Pin_b5
#define ERRLED pin_b4
#define STOP	0b10011001
#define TIMER0INTERVAL 0xFFDF //50us
#define CHANGEINTERVAL 0xF831 //1ms
#define PR2 49

int FORWARD	=	0;
int BACK	=	0;
int LEFT	=	0;
int RIGHT	=	0;

int recent = STOP;//現在のモーター状況をバックアップ(input_c()のままだとPWMで認識できなくなる)


#inline //この程度の計算はインライン展開させる
long DetermineDuty(int percent){
	return (4*(PR2+1)/100)*(long)percent;
}

void initializing(){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ);
	//TRIS設定
	set_tris_a(0x0f);
	set_tris_b(0b01001111);
	set_tris_c(0);
	output_a(0x00);
	output_b(0x00);
	output_c(0x00);
	//周期＝（PR2＋１）×4*1/f×（TMR2のプリスケール値）
	//デューティ＝DC1×1/f×（TMR2のプリスケール値）
	//duty cycle = value / [ 4 * (PR2 +1 ) ]
	Setup_ccp5(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);
	Setup_ccp3(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);
	Setup_timer_0(T0_INTERNAL | T0_DIV_8);
	Setup_timer_2(T2_DIV_BY_16,PR2,1);//20khz
	set_timer0(TIMER0INTERVAL);
	//PWM
	Set_pwm5_duty(DetermineDuty(100));
	Set_pwm3_duty(DetermineDuty(100));
	//モーターの回転方向を決定する
	if(input(change_pin1)){
		FORWARD	=	FORWARD	| 0b01010000;
		BACK	=	BACK	| 0b10100000;
		LEFT	=	LEFT	| 0b01010000;
		RIGHT	=	RIGHT	| 0b10100000;
	}else{
		FORWARD	=	FORWARD	| 0b10100000;
		BACK	=	BACK	| 0b01010000;
		LEFT	=	LEFT	| 0b10100000;
		RIGHT	=	RIGHT	| 0b01010000;
	}
	if(input(change_pin2)){
		FORWARD	=	FORWARD	| 0b00000101;
		BACK	=	BACK	| 0b00001010;
		LEFT	=	LEFT	| 0b00001010;
		RIGHT	=	RIGHT	| 0b00000101;
	}else{
		FORWARD	=	FORWARD	| 0b00001010;
		BACK	=	BACK	| 0b00000101;
		LEFT	=	LEFT	| 0b00000101;
		RIGHT	=	RIGHT	| 0b00001010;
	}
	
	
	//割り込み許可
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_TIMER0);
}

void Exception_signal_competition(void){//通信競合時の処理
	output_c(STOP);
}
void Exception_winding(void){
	output_c(STOP);
}

void Exception_signal_none(void){
	output_c(STOP);
}


#INT_TIMER0
void mainloop(void){
	disable_interrupts(INT_TIMER0);//このループ内の処理が伸びた時を考えてループ処理中はﾀｲﾏｰ停止
	long interval = TIMER0INTERVAL;//通常は20khzで割り込み
 	static long DeparturePwmCounter = 10000L;
	
	if( (int)input(pin_b0)+(int)input(pin_b1)+(int)input(pin_b2)+(int)input(pin_b3)>1 ){//信号が競合してる
		Exception_signal_competition();//競合エラー
		recent=STOP;
		output_high(ERRLED);
		goto label_function_exit;//関数の終了部へ飛ぶ
	}else{
		output_low(ERRLED);
	}
	if(input(WINDINGPIN)){//巻取り中のため働きたくありません。
		Exception_winding();
		recent=STOP;
		goto label_function_exit;//関数の終了部へ飛ぶ
	}
	
	if(input(pin_b3)){//forward
		if(recent!=FORWARD){
			output_c(0);//dead time
			DeparturePwmCounter=0;
			interval=CHANGEINTERVAL;//1ms変化待ち
			recent=FORWARD;
		}else{
			output_c(FORWARD);
		}
	}
	if(input(pin_b2)){//back
		if(recent!=BACK){
			output_c(0);//dead time
			DeparturePwmCounter=0;
			interval=CHANGEINTERVAL;//1ms変化待ち
			recent=BACK;
		}else{
			output_c(BACK);
		}
	}
	if(input(pin_b1)){//right
		if(recent!=RIGHT){
			output_c(0);//dead time
			DeparturePwmCounter=0;
			interval=CHANGEINTERVAL;//1ms変化待ち
			recent=RIGHT;
		}else{
			output_c(RIGHT);
		}
	}
	if(input(pin_b0)){//left
		if(recent!=LEFT){
			output_c(0);//dead time
			DeparturePwmCounter=0;
			interval=CHANGEINTERVAL;//1ms変化待ち
			recent=LEFT;
		}else{
			output_c(LEFT);
		}
	}

	//オーバーフロー対策
	if(DeparturePwmCounter==LONG_MAX){
		DeparturePwmCounter=1L;
	}

	//発信PWM制御
	if(DeparturePwmCounter++ == 0){
		Set_pwm5_duty(DetermineDuty(50));
		Set_pwm3_duty(DetermineDuty(50));
	}else if(DeparturePwmCounter >= 10000L){
		Set_pwm5_duty(DetermineDuty(100));
		Set_pwm3_duty(DetermineDuty(100));
	}

	//PWM
	if(recent==FORWARD){
		if(input(CCP1PIN)){
			output_c( input_c() | (FORWARD & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (FORWARD & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
	}else if(recent==BACK){
		if(input(CCP1PIN)){
			output_c( input_c()| (BACK & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (BACK & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
	}else if(recent==RIGHT){
		if(input(CCP1PIN)){
			output_c( input_c() | (RIGHT & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (RIGHT & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
	}else if(recent==LEFT){
		if(input(CCP1PIN)){
			output_c( input_c() | (LEFT & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (LEFT & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
	}
	
	if(input(pin_b0) ==0 && input(pin_b1) ==0 && input(pin_b2) ==0 && input(pin_b3) ==0){//信号がどれも来てない
		Exception_signal_none();
		interval=CHANGEINTERVAL;//1ms変化待ち
		recent=STOP;
		output_low(MOVINGPIN);
	}else{
		output_high(MOVINGPIN);
	}
	//関数を終了させたいときのラベル
	label_function_exit:
	set_timer0(interval);
	enable_interrupts(INT_TIMER0);//ﾀｲﾏｰ再開
}

void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
