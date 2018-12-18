/*
カラーLED制御部
出力(CCP1)			:RC2
出力(CCP2)			:RC1
出力(CCP4)			:RB0
信号入力ピン1		:RA0
信号入力ピン2		:RA1
信号入力ピン3		:RA2
信号入力ピン4		:RA3

*/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR
#device high_ints = true

#use delay(clock = 64000000)
#use RS232(baud=19200,xmit=pin_c6,rcv=pin_c7,INVERT)


#use standard_io(all)

#define PR2 49
#define TIMER0INTERVAL 0xFC19//1ms
#define bool int1

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

bool switchflag=0;
bool spflag=0;
bool waitflag=0;
bool errflag=0;
bool forwardflag=0;
bool signalflag=0;
bool infmode=0;
bool yellowflag=0;
#define MOVING pin_a0
#define MOTORERR pin_a1
#define SENSORERR pin_a2
#define mog3 pin_a3// あまり

//色変更の奴
#define Set_RED_duty Set_pwm1_duty
#define Set_BLUE_duty Set_pwm2_duty
#define Set_GREEN_duty Set_pwm4_duty
//色の選定の参考
//http://lowlife.jp/yasusii/static/color_chart.html
//のfloatを100倍すればいい


#inline //この程度の計算はインライン展開させる
long DetermineDuty(unsigned int percent){
	if(percent>100)percent=100;
	return (4*(PR2+1)/100)*(long)percent;//正論理
}

void initializing(void){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS設定
	set_tris_a(0);
	set_tris_b(0);
	set_tris_c(0x80);
	output_a(0x00);
	output_b(0x00);
	output_c(0x80);
	//周期＝（PR2＋１）×4*1/f×（TMR2のプリスケール値）
	//デューティ＝DC1×1/f×（TMR2のプリスケール値）
	//duty cycle = value / [ 4 * (PR2 +1 ) ]
	Setup_ccp1(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_c2
	Setup_ccp2(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_c1
	Setup_ccp4(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_b0
	
	Setup_timer_0(T0_DIV_16);//1count 1us
	Setup_timer_2(T2_DIV_BY_16,PR2,1);//20khz
	
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_RDA);
	enable_interrupts(INT_TIMER0);
	set_timer0(TIMER0INTERVAL);
	
	printf("start");
}

#inline
void CreateColor(unsigned int red,unsigned int green,unsigned int blue){
	if(red	>100)red	=100;
	if(green>100)green	=100;
	if(blue	>100)blue	=100;
	Set_RED_duty	(DetermineDuty(red));
	Set_GREEN_duty	(DetermineDuty(green));
	Set_BLUE_duty	(DetermineDuty(blue));
}


#INT_TIMER0
void timer0(void){
	set_timer0(TIMER0INTERVAL);
	if(errflag){
		CreateColor(100,0,0);//red
		return;
	}
	if(input(MOTORERR)){
		CreateColor(100,10,10);//red
		return;
	}
	if(input(SENSORERR)){
		CreateColor(100,20,20);//red
		return;
	}
	if(input(MOVING)){
		CreateColor(49, 15, 80);//purple
		return;
	}
	if(yellowflag){
		CreateColor(100,100,0);//yellow
		return;
	}
	if(signalflag){
		CreateColor(100, 65, 0);//Orange
		return;
	}
	if(forwardflag){
		CreateColor(49, 15, 80);//purple
		return;
	}
	if(spflag){
		CreateColor(0, 100, 0);//Green
		return;
	}
	if(switchflag){
		CreateColor(10, 100, 10);//LightGreen
		return;
	}
	if(waitflag){
		CreateColor(100,100,100);//white
		return;
	}
	
	if(infmode){
		//無限モードのときは通常時の色が違う
		CreateColor(0,100,100);
	}else{
		//何も信号がなければ適当に光らせる
		CreateColor(0,0, 100);
	}
}

//受信時の処理
#INT_RDA high
void interrupt_rcv(void){
	char shingo = getc();
	putc(shingo);//echo
	switch(shingo){
	case SWITCHCHAR:
		switchflag=true;
		break;
	case ENDSWITCHCHAR:
		switchflag=false;
		break;
	case SPBEGINCHAR:
		spflag=true;
		break;
	case SPENDCHAR:
		spflag=false;
		break;
	case WAITINGCHAR:
		waitflag=true;
		break;
	case WAITEDCHAR:
		waitflag=false;
		break;
	case ERRCHAR:
		errflag=true;
		break;
	case FORWARDCHAR:
		forwardflag=true;
		break;
	case ENDFORWARDCHAR:
		forwardflag=false;
		break;
	case SIGNALCHAR:
		signalflag=true;
		break;
	case ENDSIGNALCHAR:
		signalflag=false;
		break;
	case YELLOWCHAR:
		yellowflag=true;
		break;
	case ENDYELLOWCHAR:
		yellowflag=false;
		break;
	case CLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		yellowflag=false;
		infmode=false;//インフィニティモードではない
		break;
	case INFCLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		infmode=true;//インフィニティモードなんだな
		break;
	}
	timer0();//即座に変更を反応させるためにここで呼び出しておく
	
}


void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
