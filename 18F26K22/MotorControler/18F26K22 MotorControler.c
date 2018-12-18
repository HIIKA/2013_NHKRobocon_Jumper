/************************************************
���[�^�[���䕔
���[�^�[����o�͇@ 	:RC0�`RC3
���[�^�[����o�͇A 	:RC4�`RC7
�ړ����o�̓s��		:RB7 ->�W�����v���䕔��
���撆���̓s��		:RB6 <-�W�����v���䕔��
CCP1			:RA4(CCP5)->LED
CCP2			:RB5(CCP3)->LED
�G���[LED�s��		:RB4
���[�^�[�O�i���� 	:RB3 <-�Z���T�[���䕔���
���[�^�[��i���� 	:RB2 <-�Z���T�[���䕔���
���[�^�[�ʑǓ��� 	:RB1 <-�Z���T�[���䕔���
���[�^�[��Ǔ��� 	:RB0 <-�Z���T�[���䕔���


************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64Mhz)

#use fast_io(all)

//TRY�}�N���ł���
#define TRY                 /* TRY-CATCH */
#define THROW(name)         goto TRY_TAG_ERROR_##name
#define CATCH(name)   goto TRY_TAG_RESUME_LAST; TRY_TAG_ERROR_##name:
#define FINALLY       TRY_TAG_RESUME_LAST:



#define change_pin1 pin_a0//a0�͍���
#define change_pin2 pin_a1//a1�͉E��
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

int recent = STOP;//���݂̃��[�^�[�󋵂��o�b�N�A�b�v(input_c()����PWM�ŔF���ł��Ȃ��Ȃ�)

int1 lean_rightflag	=0;
int1 lean_leftflag	=0;

#inline //���̒��x�̌v�Z�̓C�����C���W�J������
long DetermineDuty(int percent){
	return (4*(PR2+1)/100)*(long)percent;
}

void initializing(){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ);
	//TRIS�ݒ�
	set_tris_a(0x0f);
	set_tris_b(0b01001111);
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
	//���[�^�[�̉�]���������肷��
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
	
	
	//���荞�݋���
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_TIMER0);
}


#INLINE
void PWM(void){
	// a & 0xf =>�ω����Ȃ�
	// a & 0   =>0�ɂȂ�(����)
	//PWM
	if(recent==FORWARD){
		if(input(CCP1PIN)){
			output_c((input_c() & 0xf0) | (FORWARD & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (FORWARD & 0xf0));//���4bit��ݒ肷��
		}else{
			output_c(input_c() & 0x0f);//���4bit��0�ɂ���
		}
	}else if(recent==BACK){
		if(input(CCP1PIN)){
			output_c((input_c() & 0xf0) | (BACK & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (BACK & 0xf0));//���4bit��ݒ肷��
		}else{
			output_c(input_c() & 0x0f);//���4bit��0�ɂ���
		}
	}else if(recent==RIGHT){
		if(input(CCP1PIN)){
			output_c( (input_c() & 0xf0) | (RIGHT & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (RIGHT & 0xf0));//���4bit��ݒ肷��
		}else{
			output_c(input_c() & 0x0f);//���4bit��0�ɂ���
		}
	}else if(recent==LEFT){
		if(input(CCP1PIN)){
			output_c( (input_c() & 0xf0) | (LEFT & 0x0f));//����4bit��ݒ肷��
		}else{
			output_c(input_c() & 0xf0);//����4bit��0�ɂ���
		}
		if(input(CCP2PIN)){
			output_c( (input_c() & 0x0f) | (LEFT & 0xf0));//���4bit��ݒ肷��
		}else{
			output_c(input_c() & 0x0f);//���4bit��0�ɂ���
		}
	}
}

#INT_TIMER0
void mainloop(void){
	disable_interrupts(INT_TIMER0);//���̃��[�v���̏������L�т������l���ă��[�v����������ϰ��~
	unsigned long interval = TIMER0INTERVAL;//�ʏ��20khz�Ŋ��荞��
	static unsigned long DeparturePwmCounter = 16000L;
	TRY{
		if(recent==STOP){
			output_c(STOP);
		}
		//�O�ƌ�낪���� �܂��� �E�ƍ�������
		if( (input(pin_left)&&input(pin_right)) || (input(pin_back)&&input(pin_forward)) ){//�M�����������Ă�
			THROW(SIGCOMP);//�֐��̏I�����֔��
		}else{
			output_low(ERRLED);
		}
		if(input(WINDINGPIN)){//����蒆�̂��ߓ�����������܂���B
			THROW(WIND);//�֐��̏I�����֔��
		}
		if(input(pin_forward)&&!input(pin_back)&&!input(pin_right)&&!input(pin_left)){//forward
			if(recent!=FORWARD|| lean_rightflag || lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms�ω��҂�
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
				interval=CHANGEINTERVAL;//1ms�ω��҂�
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
				interval=CHANGEINTERVAL;//1ms�ω��҂�
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
				interval=CHANGEINTERVAL;//1ms�ω��҂�
				recent=LEFT;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=false;
			}else{
				output_c(LEFT);
			}
		}
		//�O�i+�E
		if(input(pin_forward)&&!input(pin_back)&&input(pin_right)&&!input(pin_left)){
			if(recent!=FORWARD|| !lean_rightflag|| lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms�ω��҂�
				recent=FORWARD;
				output_high(MOVINGPIN);
				lean_rightflag=true;
				lean_leftflag=false;
			}else{
				output_c(FORWARD);
			}
		}
		//�O�i+��
		if(input(pin_forward)&&!input(pin_back)&&!input(pin_right)&&input(pin_left)){
			if(recent!=FORWARD|| lean_rightflag|| !lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms�ω��҂�
				recent=FORWARD;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=true;
			}else{
				output_c(FORWARD);
			}
		}
		//��i+�E
		if(!input(pin_forward)&&input(pin_back)&&input(pin_right)&&!input(pin_left)){
			if(recent!=BACK|| lean_rightflag|| !lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms�ω��҂�
				recent=BACK;
				output_high(MOVINGPIN);
				lean_rightflag=false;
				lean_leftflag=true;
			}else{
				output_c(BACK);
			}
		}
		//��i+��
		if(!input(pin_forward)&&input(pin_back)&&!input(pin_right)&&input(pin_left)){
			if(recent!=BACK|| !lean_rightflag|| lean_leftflag){
				output_c(0);//dead time
				DeparturePwmCounter=0;
				interval=CHANGEINTERVAL;//1ms�ω��҂�
				recent=BACK;
				output_high(MOVINGPIN);
				lean_rightflag=true;
				lean_leftflag=false;
			}else{
				output_c(BACK);
			}
		}
		

		//�I�[�o�[�t���[�΍�
		if(DeparturePwmCounter==ULONG_MAX){
			DeparturePwmCounter=16000L;
		}

		//���MPWM����
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
		
		//PWM�p�̓_�ŏ���
		PWM();
		
		if(input(pin_left) ==0 && input(pin_right) ==0 && input(pin_back) ==0 && input(pin_forward) ==0){//�M�����ǂ�����ĂȂ�
			THROW(NONSIG);
		}else{
			output_high(MOVINGPIN);
		}
		
	}CATCH(SIGCOMP){//Exception_signal_competitio
		output_c(0);//dead time
		interval=CHANGEINTERVAL;//1ms�ω��҂�
		recent=STOP;//��~
		output_high(ERRLED);
	}CATCH(WIND){//Exception_winding
		output_c(0);//dead time
		interval=CHANGEINTERVAL;//1ms�ω��҂�
		recent=STOP;//��~
		Set_pwm5_duty(DetermineDuty(100));
		Set_pwm3_duty(DetermineDuty(100));
	}CATCH(NONSIG){//Exception_signal_none
		if(recent!=STOP){
			output_c(0);//dead time
			interval=CHANGEINTERVAL;//1ms�ω��҂�
			recent=STOP;//��~
		}
		output_low(MOVINGPIN);
	}FINALLY{
		set_timer0(interval);
		enable_interrupts(INT_TIMER0);//��ϰ�ĊJ
	}
}

void main(void)
{
	initializing();
	
	while(true)
	{
	}
}
