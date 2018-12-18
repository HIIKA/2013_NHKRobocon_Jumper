/*
�J���[LED���䕔
�o��1G			:RB0
�o��1R			:RB1
�o��1B			:RB2
�o��2G			:RB3
�o��2R			:RB4
�o��2B			:RB5
�o��3G			:RA0
�o��3R			:RA1
�o��3B			:RA2
�M�����̓s��1		:RC0
�M�����̓s��2		:RC1
�M�����̓s��3		:RC2
�M�����̓s��4		:RC3
�V���A���ʐMTX1					:RC6
�V���A���ʐMRX1					:RC7
CCP���g��Ȃ����Ƃɂ���
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

#define SWITCHCHAR		'A'//�X�C�b�`�F���M������
#define ENDSWITCHCHAR	'a'//�X�C�b�`�F���I��
#define SPBEGINCHAR		'B'//��������ɓ��������̕���
#define SPENDCHAR		'b'//�������삪�I���������̕���
#define WAITINGCHAR		'C'//�N���ҋ@���̕���
#define WAITEDCHAR		'c'//�N���������̕���
#define ERRCHAR			'E'//�G���[���o�Ă��鎞�̕���
#define FORWARDCHAR		'F'//�O�i���߂��o�����̕���
#define ENDFORWARDCHAR	'f'//�O�i���ߏI��
#define SIGNALCHAR		'G'//�W�����v
#define ENDSIGNALCHAR	'g'//�W�����v�^�C�~���O����
#define CLEARCHAR		'H'//�M��CLEAR
#define INFCLEARCHAR	'h'//������W�����v�̎��̐M��CLEAR
#define YELLOWCHAR		'I'//���F�Ɍ���
#define ENDYELLOWCHAR	'i'//���F������
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
#define mog3 pin_c3// ���܂�




void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
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
		//�������[�h�̂Ƃ��͒ʏ펞�̐F���Ⴄ
		CreateLEDColorALL(0,1,1);
	}else{
		//�����M�����Ȃ���ΓK���Ɍ��点��
		CreateLEDColorALL(0,0,1);
	}
}

//��M���̏���
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
		infmode=false;//�C���t�B�j�e�B���[�h�ł͂Ȃ�
		jumpingcount=0;
		break;
	case INFCLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		infmode=true;//�C���t�B�j�e�B���[�h�Ȃ񂾂�
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
