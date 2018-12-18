/************************************************
�W�����v���䕔
���[�^���[�G���R�[�_�[���̓s��1	:RA0 ->encoder RC0 //����]//��������]����
���[�^���[�G���R�[�_�[���̓s��2	:RA1 ->encoder RC1 //�E��]//�t
���[�^���[�G���R�[�_�[���̓s��3	:RA2 ->encoder RC2 //��~�M��
�X�C�b�`���̓s��				:RA3
�t�H�g�C���^���v�^���̓s��		:RA4
�ړ������̓s��				:RA5 <-�ړ����[�^�[���䕔���
1��W�����v���̓s��			:RB0 <-�Z���T�[���䕔���
�����W�����v���̓s��			:RB1 <-�Z���T�[���䕔���
�X�C�b�`�m�FLED�o�̓s��		:RB7
���ʓ��쒆LED�o�̓s��		:RB3
�G���[LED�s��				:RB4
����蒆�o�̓s��			:RB5 ->�ړ����[�^�[���䕔��
���ʓ��쒆�o�̓s��			:RB6 ->�Z���T�[���䕔��
���ʓ���O�i�o�̓s��		:RB2 ->�ړ����[�^�[���䕔��
�������������o�̓s��			:RC4 ->�������䕔��
�^�N�g�X�C�b�`���̓s��			:RC7 <-�X�C�b�`
���[�^�[����o�̓s��1			:RC0
���[�^�[����o�̓s��2			:RC1
���[�^�[����o�̓s��3			:RC2
���[�^�[����o�̓s��4			:RC3

************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(all)


//TRY�}�N���ł���
#define TRY				/* TRY-CATCH */
#define THROW(name)		goto TRY_TAG_ERROR_##name
#define CATCH(name)		goto TRY_TAG_RESUME_LAST; TRY_TAG_ERROR_##name:
#define FINALLY			TRY_TAG_RESUME_LAST:

#define bool int1
#define LEFTTURN 		pin_a0//��������
#define RIGHTTURN 		pin_a1//�E������
#define STOPTURN 		pin_a2//��~ ����
#define SWITCHINPUT 	pin_a3//�X�C�b�`����
#define INTERRUPT	 	pin_a4//�t�H�g�C���^���v�^����
#define MOVING 			pin_a5//�ړ�������
#define ONEJUMP 		pin_b0//���W�����v�i�Z���T�[���䕔������́j
#define INFINITYJUMP 	pin_b1//������W�����v�i�Z���T�[���䕔������́j
#define SPFORWARD 		pin_b2//���ʃV�[�P���X�őO�i����Ƃ��ɏo�͂���
#define SPECIALLED 		pin_b3//���ʃV�[�P���X���s���Ă���Ƃ��Ɍ���
#define ERRLED 			pin_b4//�G���[���N����ƌ���
#define WINDING 		pin_b5//����蒆�ɏo��
#define SEQUENCEMODE 	pin_b6//���ʃV�[�P���X���s���Ă���Ƃ��ɏo�͂���
#define SWITCHLED 		pin_b7//�X�C�b�`��F���������Ɍ��点��
#define JEJEJEOUT		pin_c4//�W�����v����Ƃ��ɂ�����������
#define TACTSWITCH		pin_c7//�^�N�g�X�C�b�`�Ő���]�̔�����
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms

//input(SWWITCHINPT)	�X�C�b�`�������True
//input(INTERRUPT)		�؂�ڂ̎�True
//input(MOVING)			�ړ����̎�True
//input(�`�`TURN)		True�̂Ƃ��A���̕����ɉ�]���Ă�,STOP�͎~�܂��Ă�



void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//��]��~�ŃX�^�[�g
	set_tris_a(0x1F);//0b00011111
	set_tris_b(0x03);//0b00000011
	set_tris_c(0x80);//0b10000000
	Setup_timer_0(T0_DIV_256);//1count 16us
}

void change_c(int port){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(deadtime);
	output_c((input_c() & 0xf0) | port);
}
void Exception_ERR(long ms=1000){
	change_c(0);//�o�͂Ȃ�
	output_low (WINDING);
	output_low (SEQUENCEMODE);
	output_low (SPFORWARD);
	while(true){
		output_toggle(ERRLED);//�������[�v
		delay_ms(ms);
	}
}

//�����グ�����A����ŃW�����v���s��
int Sequence_Winding(unsigned long num=1){//0�̎��͊����グ�̂ݍs��
	unsigned int timecounter =0;//15625�J�E���g(0.25�b)��1�C���N�������g
	unsigned long jumpcounter =0;//�W�����v�̉񐔂𐔂���B
	bool edgeflag ;
	int duty=20;
	int nonduty=30;
	TRY{
		change_c( WISESTOP);
		while(!input(STOPTURN)){//�W�����v���[�^�[��~�҂�
		}
		
		output_high(WINDING);
		if(num==0&&input(INTERRUPT)){//�t���O������A���łɐ؂�ڂɂ���Ȃ�܂킳�Ȃ�
			THROW(END);//�֐������I��
		}
		
		if(num==0)num+=1;//�؂�ڂ̈ʒu�ɂ���̂łȂ����1��Ɠ�������
		
		edgeflag=input(INTERRUPT);
		change_c(CLOCKWISE);//�����グ���s��
		set_timer0(0);
		do{//while(jumpcounter<num);
			if(get_timer0()>=15625L){//�w��񐔂�
				set_timer0(0);//�^�C�}���Z�b�g����
				if(timecounter<uINT_MAX)timecounter++;//���E�s�����琔����̂�����߂�
				if(nonduty > 0){//�܂�PWM����������
					duty+=5;//Duty�̔�𑝂₷
					nonduty-=5;//���炷
				}
			}
			
			if(nonduty > 0){//�܂�PWM���I����ĂȂ����
				output_c((input_c() & 0xf0) | 0);//pwm
				delay_us(nonduty);
			}
			output_c((input_c() & 0xf0) | CLOCKWISE);//pwm
			delay_us(duty);

			//�X�g�[���̋^��,��~�M�������Ă�����
			if(timecounter >= 7  && input(STOPTURN) ){//��1.75�b��ɒ�~�M�������Ă�����G���[
				THROW(ERR);//ERR Throw
			}
			
			if(edgeflag!=input(INTERRUPT)){//�C���^���v�^�ɕω�����������
				edgeflag=input(INTERRUPT);//0�Ȃ�؂�ڂ��甲���鎞�A1�Ȃ�؂�ڂɓ��鎞
				output_bit(JEJEJEOUT,!edgeflag);//�؂�ڂ��甲����Ƃ�JEJEJE,���鎞0
				if(num==0){//�؂�ڒT���̂Ƃ��̓��`�F�b�g����O�̂�����l���������Ԃ����Ȃ�
					if(edgeflag){//�؂�ڂɓ�������
						jumpcounter++;//�W�����v�����Ƃ݂Ȃ��ƃ��[�v�I��
					}
				}else{
					if(edgeflag&&timecounter >= 7){//�����o������؂�ڂɓ�������
						jumpcounter++;//�W�����v�����Ƃ݂Ȃ�
					}
				}
			}
			
		}while(jumpcounter<num);//�����Ȃ����甲����
		
	}CATCH(ERR){
		Exception_ERR();
		return 1;
	}CATCH(END){
		//do nothing
	}FINALLY{
		change_c( WISESTOP);//��~
		output_low(WINDING);//�����I���[
		return 0;//����I��
	}
}

void Sequence_Onejump(){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//�W�����v
	Sequence_Winding(1);
	
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//�X�y�V�������[�h�I��
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Infinityjump(unsigned long cont = uLONG_MAX){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);

		//�W�����v
		Sequence_Winding(cont);//�����̎���uLONG_MAX
	
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}

void Sequence_Twojump(){
	output_high(SWITCHLED);
	output_high(SEQUENCEMODE);
	//�ꂪ����̑ҋ@
	delay_ms(600);
	
	//�W�����v
	Sequence_Winding(2);
	
	//���n�ҋ@
	delay_ms(400);
	output_low (SWITCHLED);
	output_low (SEQUENCEMODE);
}
int main(void)
{
	int duty;
	int nonduty;
	delay_ms(100);
	initializing();
	
	while(input(MOVING)){}
	
	//�N�����̊����グ
	Sequence_Winding(0);
	
	while(true)
	{
		if(input(MOVING)){
			continue;
		}else{//�ړ����łȂ���
			if(input(TACTSWITCH)){//�������X�C�b�`...�������܂���[
				//duty������
				duty=20;
				nonduty=30;
				//�����グ�J�n
				output_high(WINDING);
				change_c(CLOCKWISE);
				set_timer0(0);
				while(input(TACTSWITCH)){
					if(get_timer0() > 30000L && nonduty !=0){
						set_timer0(0);
						duty+=5;
						nonduty-=5;//�������0�ɂȂ�
					}
					output_c((input_c() & 0xf0) | CLOCKWISE);
					delay_us(duty);
					if(nonduty!=0) {
						output_c((input_c() & 0xf0) | 0);
						delay_us(nonduty);
					}
				}//��������ł��B
				change_c(WISESTOP);
				output_low (WINDING);
			}
		}
		
		if(input(ONEJUMP)){
			Sequence_Onejump();
		}
		
		if(input(INFINITYJUMP)){
			Sequence_Infinityjump(5);
		}
		
		if((!input(MOVING))&&input(SWITCHINPUT)){
			Sequence_Twojump();
		}
		
		
	}
return 0;
}
