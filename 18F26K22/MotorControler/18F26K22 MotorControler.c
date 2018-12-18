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

#use delay(clock = 64Mhz)

#use fast_io(all)

//TRYマクロですわ
#define TRY                 /* TRY-CATCH */
#define THROW(name)         goto TRY_TAG_ERROR_##name
#define CATCH(name)   goto TRY_TAG_RESUME_LAST; TRY_TAG_ERROR_##name:
#define FINALLY       TRY_TAG_RESUME_LAST:



#define change_pin1 pin_a0//a0は左側
#define change_pin2 pin_a1//a1は右側
#define MOVINGPIN Pin_b7
#define WINDINGPIN Pin_b6
#define CCP1PIN Pin_a4
#define CCP2PIN Pin_b5
#define ERRLED pin_b4
#define STOP	0b10011001
#define TIMER0INTERVAL 0xFFDF //17us
#define CHANGEINTERVAL 0xF831 //1ms
#define PR2 49


#define pin_forward pin_b3
#define pin_back pin_b2
#define pin_right pin_b1
#define pin_left pin_b0

int FORWARD	=	0;
int BACK	=	0;
int LEFT	=	0;
int RIGHT	=	0;

int recent = STOP;//現在のモーター状況をバックアップ(input_c()だとPWMで認識できなくなる)

int1 lean_rightflag	=0;
int1 lean_leftflag	=0;

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


#INLINE
void PWM(void){
	// a & 0xf =>変化しない
	// a & 0   =>0になる(強制)
	//PWM
	if(recent==FORWARD){
		if(input(CCP1PIN)){
			output_c((input_c() & 0xf0) | (FORWARD & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (FORWARD & 0xf0));//上位4bitを設定する
		}else{
			output_c(input_c() & 0x0f);//上位4bitを0にする
		}
	}else if(recent==BACK){
		if(input(CCP1PIN)){
			output_c((input_c() & 0xf0) | (BACK & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (BACK & 0xf0));//上位4bitを設定する
		}else{
			output_c(input_c() & 0x0f);//上位4bitを0にする
		}
	}else if(recent==RIGHT){
		if(input(CCP1PIN)){
			output_c( (input_c() & 0xf0) | (RIGHT & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (RIGHT & 0xf0));//上位4bitを設定する
		}else{
			output_c(input_c() & 0x0f);//上位4bitを0にする
		}
	}else if(recent==LEFT){
		if(input(CCP1PIN)){
			output_c( (input_c() & 0xf0) | (LEFT & 0x0f));//下位4bitを設定する
		}else{
			output_c(input_c() & 0xf0);//下位4bitを0にする
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (LEFT & 0xf0));//上位4bitを設定する
		}else{
			output_c(input_c() & 0x0f);//上位4bitを0にする
		}
	}
}

#INT_TIMER0
void mainloop(void){
	disable_interrupts(INT_TIMER0);//このループ内の処理が伸びた時を考えてループ処理中はﾀｲﾏｰ停止
	unsigned long interval = TIMER0INTERVAL;//通常は20khzで割り込み
	static unsigned long DeparturePwmCounter = 16000L;
	TRY{
		if(recent==STOP){
			output_c(STOP);
		}
		//前と後ろが同時 または 右と左が同時
		if( (input(pin_left)&&input(pin_right)) || (input(pin_back)&&input(pin_forward)) ){//信号が競合してる
			THROW(SIGCOMP);//関数の終了部へ飛ぶ
		}else{
			output_low(ERRLED);
		}
		if(input(WINDINGPIN)){//巻取り中のため働きたくありません。
			THROW(WIND);//関数の終了部へ飛ぶ
		}
		if(input(pin_forward)&&!input(pin_back)&&!input(pin_right)&&!input(pin_left)){//forward
			if(recent!=FORWARD|| lean_rightflag || lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=FORWARD;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=false;
			}else{
				output_c(FORWARD);
			}
		}
		if(!input(pin_forward)&&input(pin_back)&&!input(pin_right)&&!input(pin_left)){//back
			if(recent!=BACK|| lean_rightflag || lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=BACK;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=false;
			}else{
				output_c(BACK);
			}
		}
		if(!input(pin_forward)&&!input(pin_back)&&input(pin_right)&&!input(pin_left)){//right
			if(recent!=RIGHT){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=RIGHT;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=false;
			}else{
				output_c(RIGHT);
			}
		}
		if(!input(pin_forward)&&!input(pin_back)&&!input(pin_right)&&input(pin_left)){//left
			if(recent!=LEFT){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=LEFT;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=false;
			}else{
				output_c(LEFT);
			}
		}
		//前進+右
		if(input(pin_forward)&&!input(pin_back)&&input(pin_right)&&!input(pin_left)){
			if(recent!=FORWARD|| !lean_rightflag|| lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=FORWARD;
				output_high(MOVINGPIN);
				lean_rightflag=true;
				lean_leftflag=false;
			}else{
				output_c(FORWARD);
			}
		}
		//前進+左
		if(input(pin_forward)&&!input(pin_back)&&!input(pin_right)&&input(pin_left)){
			if(recent!=FORWARD|| lean_rightflag|| !lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=FORWARD;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=true;
			}else{
				output_c(FORWARD);
			}
		}
		//後進+右
		if(!input(pin_forward)&&input(pin_back)&&input(pin_right)&&!input(pin_left)){
			if(recent!=BACK|| lean_rightflag|| !lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=BACK;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=true;
			}else{
				output_c(BACK);
			}
		}
		//後進+左
		if(!input(pin_forward)&&input(pin_back)&&!input(pin_right)&&input(pin_left)){
			if(recent!=BACK|| !lean_rightflag|| lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms変化待ち
				recent=BACK;
				output_high(MOVINGPIN);
				lean_rightflag=true;
				lean_leftflag=false;
			}else{
				output_c(BACK);
			}
		}
		

		//オーバーフロー対策
		if(DeparturePwmCounter==ULONG_MAX){
			DeparturePwmCounter=16000L;
		}

		//発信PWM制御
		if(DeparturePwmCounter == 0){
			Set_pwm5_duty(DetermineDuty(60));
			Set_pwm3_duty(DetermineDuty(60));
			if(lean_rightflag){
				DeparturePwmCounter=16000L;
				Set_pwm3_duty(100);
			}
			if(lean_leftflag){
				DeparturePwmCounter=16000L;
				Set_pwm5_duty(100);
			}
		}else if(DeparturePwmCounter == 5000L){
			Set_pwm5_duty(DetermineDuty(75));
			Set_pwm3_duty(DetermineDuty(75));
		}else if(DeparturePwmCounter == 10000L){
			Set_pwm5_duty(DetermineDuty(85));
			Set_pwm3_duty(DetermineDuty(85));
		}else if(DeparturePwmCounter == 15000L){
			Set_pwm5_duty(DetermineDuty(100));
			Set_pwm3_duty(DetermineDuty(100));
		}
		
		DeparturePwmCounter++;
		
		//PWM用の点滅処理
		PWM();
		
		if(input(pin_left) ==0 && input(pin_right) ==0 && input(pin_back) ==0 && input(pin_forward) ==0){//信号がどれも来てない
			THROW(NONSIG);
		}else{
			output_high(MOVINGPIN);
		}
		
	}CATCH(SIGCOMP){//Exception_signal_competitio
		output_c(0);//dead time
		interval=CHANGEINTERVAL;//1ms変化待ち
		recent=STOP;//停止
		output_high(ERRLED);
	}CATCH(WIND){//Exception_winding
		output_c(0);//dead time
		interval=CHANGEINTERVAL;//1ms変化待ち
		recent=STOP;//停止
		Set_pwm5_duty(DetermineDuty(100));
		Set_pwm3_duty(DetermineDuty(100));
	}CATCH(NONSIG){//Exception_signal_none
		if(recent!=STOP){
			output_c(0);//dead time
			interval=CHANGEINTERVAL;//1ms変化待ち
			recent=STOP;//停止
		}
		output_low(MOVINGPIN);
	}FINALLY{
		set_timer0(interval);
		enable_interrupts(INT_TIMER0);//ﾀｲﾏｰ再開
	}
}

void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
