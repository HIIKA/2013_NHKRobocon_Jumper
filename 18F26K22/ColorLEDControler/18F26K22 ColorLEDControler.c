/*
�J���[LED���䕔
�o��(CCP1)			:RC2
�o��(CCP2)			:RC1
�o��(CCP4)			:RB0
�M�����̓s��1		:RA0
�M�����̓s��2		:RA1
�M�����̓s��3		:RA2
�M�����̓s��4		:RA3

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
#define mog3 pin_a3// ���܂�

//�F�ύX�̓z
#define Set_RED_duty Set_pwm1_duty
#define Set_BLUE_duty Set_pwm2_duty
#define Set_GREEN_duty Set_pwm4_duty
//�F�̑I��̎Q�l
//http://lowlife.jp/yasusii/static/color_chart.html
//��float��100�{����΂���


#inline //���̒��x�̌v�Z�̓C�����C���W�J������
long DetermineDuty(unsigned int percent){
	if(percent>100)percent=100;
	return (4*(PR2+1)/100)*(long)percent;//���_��
}

void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
	set_tris_a(0);
	set_tris_b(0);
	set_tris_c(0x80);
	output_a(0x00);
	output_b(0x00);
	output_c(0x80);
	//�������iPR2�{�P�j�~4*1/f�~�iTMR2�̃v���X�P�[���l�j
	//�f���[�e�B��DC1�~1/f�~�iTMR2�̃v���X�P�[���l�j
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
		//�������[�h�̂Ƃ��͒ʏ펞�̐F���Ⴄ
		CreateColor(0,100,100);
	}else{
		//�����M�����Ȃ���ΓK���Ɍ��点��
		CreateColor(0,0, 100);
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
	case CLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		yellowflag=false;
		infmode=false;//�C���t�B�j�e�B���[�h�ł͂Ȃ�
		break;
	case INFCLEARCHAR:
		switchflag=false;
		waitflag=false;
		spflag=false;
		forwardflag=false;
		signalflag=false;
		errflag=false;
		infmode=true;//�C���t�B�j�e�B���[�h�Ȃ񂾂�
		break;
	}
	timer0();//�����ɕύX�𔽉������邽�߂ɂ����ŌĂяo���Ă���
	
}


void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
