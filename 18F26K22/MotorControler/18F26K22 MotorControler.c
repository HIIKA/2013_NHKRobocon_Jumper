/************************************************
���[�^�[���䕔
���[�^�[����o�͇@ 	:RC0�`RC3
���[�^�[����o�͇A 	:RC4�`RC7
�ړ����o�̓s��		:RB7
���撆���̓s��		:RB6
CCP1			:RA4(CCP5)->LED
CCP2			:RB5(CCP3)->LED
�G���[LED�s��		:RB4
���[�^�[�O�i���� 	:RB3
���[�^�[��i���� 	:RB2
���[�^�[�ʑǓ��� 	:RB1
���[�^�[���Ǔ��� 	:RB0


************************************************/
#include <18f26K22.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)

#define MOVINGPIN Pin_b7
#define WINDINGPIN Pin_b6
#define CCP1PIN Pin_a4
#define CCP2PIN Pin_b5
#define ERRLED pin_b4
#define FORWARD	0b10101010
#define BACK	0b01010101
#define LEFT	0b10100101
#define RIGHT	0b01011010
#define STOP	0b10011001
#define TIMER0INTERVAL 0xFFDF //50us
#define CHANGEINTERVAL 0xF831 //1ms
#define PR2 49

int recent = STOP;//���݂̃��[�^�[�󋵂��o�b�N�A�b�v(input_c()�̂܂܂���PWM�ŔF���ł��Ȃ��Ȃ�)

#inline //���̒��x�̌v�Z�̓C�����C���W�J������
long DetermineDuty(int percent){
	return (4*(PR2+1)/100)*(long)percent;
}

void Exception_signal_competition(void){//�ʐM�������̏���
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
	disable_interrupts(INT_TIMER0);//���̃��[�v���̏������L�т������l���ă��[�v����������ϰ��~
	long interval = TIMER0INTERVAL;//�ʏ��20khz�Ŋ��荞��
	
	if( (int)input(pin_b0)+(int)input(pin_b1)+(int)input(pin_b2)+(int)input(pin_b3)>1 ){//�M�����������Ă�
		Exception_signal_competition();//�����G���[
		recent=STOP;
		output_high(ERRLED);
		goto label_function_exit;//�֐��̏I�����֔��
	}else{
		output_low(ERRLED);
	}
	if(input(WINDINGPIN)){//����蒆�̂��ߓ�����������܂���B
		Exception_winding();
		recent=STOP;
		goto label_function_exit;//�֐��̏I�����֔��
	}
	
	if(input(pin_b3)){//forward
		if(recent!=FORWARD){
			output_c(STOP);
			interval=CHANGEINTERVAL;//1ms�ω��҂�
		}else{
			output_c(FORWARD);
		}
		recent=FORWARD;
	}
	if(input(pin_b2)){//back
		if(recent!=BACK){
			output_c(STOP);
			interval=CHANGEINTERVAL;//1ms�ω��҂�
		}else{
			output_c(BACK);
		}
		recent=BACK;
	}
	if(input(pin_b1)){//right
		if(recent!=RIGHT){
			output_c(STOP);
			interval=CHANGEINTERVAL;//1ms�ω��҂�
		}else{
			output_c(RIGHT);
		}
		recent=RIGHT;
	}
	if(input(pin_b0)){//left
		if(recent!=LEFT){
			output_c(STOP);
			interval=CHANGEINTERVAL;//1ms�ω��҂�
		}else{
			output_c(LEFT);
		}
		recent=LEFT;
	}

	//PWM
	switch(recent){
	case FORWARD:
		if(input(CCP1PIN)){
			output_c( input_c() | (FORWARD & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (FORWARD & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
		break;
	case BACK:
		if(input(CCP1PIN)){
			output_c( input_c()| (BACK & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (BACK & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
		break;
	case RIGHT:
		if(input(CCP1PIN)){
			output_c( input_c() | (RIGHT & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (RIGHT & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
		break;
	case LEFT:
		if(input(CCP1PIN)){
			output_c( input_c() | (LEFT & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( input_c() | (LEFT & 0xf0));
		}else{
			output_c(input_c() & 0x0f);
		}
		break;
	}
	
	if(input(pin_b0) ==0 && input(pin_b1) ==0 && input(pin_b2) ==0 && input(pin_b3) ==0){//�M�����ǂ�����ĂȂ�
		Exception_signal_none();
		output_c(STOP);//��~
		recent=STOP;
		output_low(MOVINGPIN);
	}else{
		output_high(MOVINGPIN);
	}
	//�֐����I�����������Ƃ��̃��x��
	label_function_exit:
	set_timer0(interval);
	enable_interrupts(INT_TIMER0);//��ϰ�ĊJ
}

void main(void)
{
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ);
	//TRIS�ݒ�
	set_tris_a(0);
	set_tris_b(0x4f);
	set_tris_c(0);
	output_a(0x00);
	output_b(0x00);
	output_c(0x00);
	//�������iPR2�{�P�j�~4*1/f�~�iTMR2�̃v���X�P�[���l�j
	//�f���[�e�B��DC1�~1/f�~�iTMR2�̃v���X�P�[���l�j
	//duty cycle = value / [ 4 * (PR2 +1 ) ]
	Setup_ccp5(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);
	Setup_ccp3(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);
	Setup_timer_0(T0_INTERNAL | T0_DIV_8);
	Setup_timer_2(T2_DIV_BY_16,PR2,1);//20khz
	set_timer0(TIMER0INTERVAL);
	//PWM
	Set_pwm5_duty(DetermineDuty(100));
	Set_pwm3_duty(DetermineDuty(100));
	
	//���荞�݋���
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_TIMER0);
	
	while(true)
	{
	}
}
