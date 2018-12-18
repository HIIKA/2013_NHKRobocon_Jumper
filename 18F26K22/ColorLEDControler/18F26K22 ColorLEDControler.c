/*
カラーLED制御部
出力1G			:RB0
出力1R			:RB1
出力1B			:RB2
出力2G			:RB3
出力2R			:RB4
出力2B			:RB5
出力3G			:RA0
出力3R			:RA1
出力3B			:RA2
信号入力ピン1		:RC0
信号入力ピン2		:RC1
信号入力ピン3		:RC2
信号入力ピン4		:RC3
シリアル通信TX1					:RC6
シリアル通信RX1					:RC7
CCPを使わないことにした
*/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR
#device high_ints = true

#use delay(clock = 64000000)
#use RS232(baud=152000,xmit=pin_c6,rcv=pin_c7,INVERT)


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
#define COUNT1			'j'
#define COUNT2			'k'
#define COUNT3			'l'
#define COUNTEND		'm'

unsigned int jumpingcount=0;
bool switchflag=0;
bool spflag=0;
bool waitflag=0;
bool errflag=0;
bool forwardflag=0;
bool signalflag=0;
bool infmode=0;
bool yellowflag=0;

#define OUT1G pin_b0
#define OUT1R pin_b1
#define OUT1B pin_b2
#define OUT2G pin_b3
#define OUT2R pin_b4
#define OUT2B pin_b5
#define OUT3G pin_a0
#define OUT3R pin_a1
#define OUT3B pin_a2

#define MOVING pin_c0
#define MOTORERR pin_c1
#define SENSORERR pin_C2
#define mog3 pin_c3// あまり




void initializing(void){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS設定
	set_tris_a(0);
	set_tris_b(0);
	set_tris_c(0b10001111);
	output_a(0x00);
	output_b(0x00);
	output_c(0x80);
	
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_RDA);
	
	printf("start");
}

#inline
void CreateLEDColor(unsigned int lednum, int1 red,int1 green,int1 blue){
	if(lednum==1){
		output_bit(OUT1R,red);
		output_bit(OUT1G,green);
		output_bit(OUT1B,blue);
	}else if(lednum==2){
		output_bit(OUT2R,red);
		output_bit(OUT2G,green);
		output_bit(OUT2B,blue);
	}else if(lednum==3){
		output_bit(OUT3R,red);
		output_bit(OUT3G,green);
		output_bit(OUT3B,blue);
	}
}
#inline
void CreateLEDColorALL(int1 red,int1 green,int1 blue){
	output_bit(OUT1R,red);
	output_bit(OUT1G,green);
	output_bit(OUT1B,blue);
	output_bit(OUT2R,red);
	output_bit(OUT2G,green);
	output_bit(OUT2B,blue);
	output_bit(OUT3R,red);
	output_bit(OUT3G,green);
	output_bit(OUT3B,blue);
}


void set_color(void){
	if(errflag){
		CreateLEDColorALL(1,0,0);//red
		return;
	}
	if(input(MOTORERR)){
		CreateLEDColor(1,1,0,0);//red
		CreateLEDColor(2,1,1,1);//white
		CreateLEDColor(3,1,1,1);//white
		return;
	}
	if(input(SENSORERR)){
		CreateLEDColor(1,1,1,1);//white
		CreateLEDColor(2,1,0,0);//red
		CreateLEDColor(3,1,1,1);//white
		return;
	}
	if(input(MOVING)){
		CreateLEDColorALL(1, 0, 1);//purple
		return;
	}
	if(jumpingcount==1){
		CreateLEDColor(1,1,1,1);//white
		CreateLEDColor(2,1,1,1);//white
		CreateLEDColor(3,1,1,0);//yellow
		return;
	}else if(jumpingcount==2){
		CreateLEDColor(1,1,1,1);//white
		CreateLEDColor(2,1,1,0);//yellow
		CreateLEDColor(3,1,1,0);//yellow
		return;
	}else if(jumpingcount==3){
		CreateLEDColor(1,1,1,0);//yellow
		CreateLEDColor(2,1,1,0);//yellow
		CreateLEDColor(3,1,1,0);//yellow
		return;
	}
	if(yellowflag){
		CreateLEDColorALL(1,1,0);//yellow
		return;
	}
	if(signalflag){
		CreateLEDColor(1,1, 1, 0);//yellow
		CreateLEDColor(2,1, 0, 0);//red
		CreateLEDColor(3,1, 1, 0);//yellow
		return;
	}
	if(forwardflag){
		CreateLEDColor(1,1, 0, 1);//purple
		CreateLEDColor(2,1, 1, 1);//white
		CreateLEDColor(3,1, 0, 1);//purple
		return;
	}
	if(spflag){
		CreateLEDColorALL(0, 1, 0);//Green
		return;
	}
	if(switchflag){
		CreateLEDColor(1,0, 1, 0);//Green
		CreateLEDColor(2,1, 1, 1);//white
		CreateLEDColor(3,0, 1, 0);//Green
		return;
	}
	if(waitflag){
		CreateLEDColorALL(1,1,1);//white
		return;
	}
	
	if(infmode){
		//無限モードのときは通常時の色が違う
		CreateLEDColorALL(0,1,1);
	}else{
		//何も信号がなければ適当に光らせる
		CreateLEDColorALL(0,0,1);
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
	case COUNT1:
		jumpingcount=1;
		break;
	case COUNT2:
		jumpingcount=2;
		break;
	case COUNT3:
		jumpingcount=3;
		break;
	case COUNTEND:
		jumpingcount=0;
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
		jumpingcount=0;
		break;
	case INFCLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		infmode=true;//インフィニティモードなんだな
		jumpingcount=0;
		break;
	}
	set_color();
	
}


void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
