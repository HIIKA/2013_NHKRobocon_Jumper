/************************************************
�W�����v���䕔
�X�C�b�`���̓s��				:RA3
�t�H�g�C���^���v�^���̓s��		:RA4
�ړ������̓s��				:RA5 <-�ړ����[�^�[���䕔���
���[�^���[�G���R�[�_�[���̓s��1	:RA0 ->encoder RC0 //����]//��������]����
���[�^���[�G���R�[�_�[���̓s��2	:RA1 ->encoder RC1 //�E��]//�t
���[�^���[�G���R�[�_�[���̓s��3	:RA2 ->encoder RC2 //��~�M��
1��W�����v���̓s��			:RB0 <-�Z���T�[���䕔���
�����W�����v���̓s��			:RB1 <-�Z���T�[���䕔���
�X�C�b�`�m�FLED�o�̓s��		:RB7
���ʓ��쒆LED�o�̓s��		:RB3
�G���[LED�s��				:RB4
����蒆�o�̓s��			:RB5 ->�ړ����[�^�[���䕔��
���ʓ��쒆�o�̓s��			:RB6 ->�Z���T�[���䕔��
���ʓ���O�i�o�̓s��		:RB2 ->�ړ����[�^�[���䕔��
�������������o�̓s��			:RC4 ->�������䕔��
���[�^�[����o�̓s��1			:RC0
���[�^�[����o�̓s��2			:RC1
���[�^�[����o�̓s��3			:RC2
���[�^�[����o�̓s��4			:RC3

************************************************/
#include <18f26K22.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(all)

#define SWITCHINPUT 	pin_a3//�X�C�b�`����
#define INTERRUPT	 	pin_a4//�t�H�g�C���^���v�^����
#define MOVING 			pin_a5//�ړ����o��
#define LEFTTURN 		pin_a0//��������
#define RIGHTTURN 		pin_a1//�E������
#define STOPTURN 		pin_a2//��~ ����
#define ONEJUMP 		pin_b0//���W�����v�i�Z���T�[���䕔������́j
#define INFINITYJUMP 	pin_b1//������W�����v�i�Z���T�[���䕔������́j
#define SWITCHLED 		pin_b7//�X�C�b�`��F���������Ɍ��点��
#define SPECIALLED 		pin_b3//���ʃV�[�P���X���s���Ă���Ƃ��Ɍ���
#define ERRLED 			pin_b4//�G���[���N����ƌ���
#define WINDING 		pin_b5//����蒆�Ɍ���
#define SEQUENCEMODE 	pin_b6//���ʃV�[�P���X���s���Ă���Ƃ��ɏo�͂���
#define SPFORWARD 		pin_b2//���ʃV�[�P���X�őO�i����Ƃ��ɏo�͂���
#define JEJEJEOUT		pin_c4//�W�����v����Ƃ��ɂ�����������
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms

void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//��]��~�ŃX�^�[�g
	set_tris_a(0x1F);//0b00011111
	set_tris_b(0x03);//0b00000011
	set_tris_c(0);
	Setup_timer_0(T0_DIV_256);//1count 16us
}

void delay_deadtime(void){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(deadtime);
}
void Exception_ERR(void){
	delay_deadtime();
	output_c((input_c() & 0xf0) | 0);//�o�͂Ȃ�
	while(true){
		output_toggle(ERRLED);//�������[�v
		delay_ms(1000);
	}
}

//�����グ�����A����ŃW�����v���s��
int Sequence_Winding(){
	output_c((input_c() & 0xf0) | WISESTOP);
	while(!input(STOPTURN)){//�W�����v���[�^�[�҂�
		output_toggle(ERRLED);delay_ms(200);
	}
	output_low (ERRLED);
	
	output_high(WINDING);
	delay_deadtime();
	output_c((input_c() & 0xf0) | CLOCKWISE);//�����グ���s��


	set_timer0(0);
	while(input(INTERRUPT)){//���؂�ڂɂ���Ȃ�
		if(get_timer0() > 12500){//200ms�ȓ��ɐ؂�ڂ��甲���Ȃ��ƃG���[
			Exception_ERR();
			return 1;
		}
		output_c((input_c() & 0xf0) | CLOCKWISE);//pwm
		delay_us(20);
		output_c((input_c() & 0xf0) | 0);//pwm
		delay_us(25);
	}

	delay_deadtime();
	output_c((input_c() & 0xf0) | CLOCKWISE);//�����グ���s��
	delay_ms(100);//�ǉ���0.1�b
	set_timer0(0);
	while(!input(INTERRUPT)){//���Ԃ̐؂�ڂɍs���܂Ŋ����グ
		if(get_timer0() > 62500L){//��1�b�ȓ��ɂ܂��؂�ڂ�������Ȃ��ƃG���[
			Exception_ERR();
			return 1;
		}
	}
	delay_deadtime();
	output_c((input_c() & 0xf0) | WISESTOP);//��~
	output_low(WINDING);//�����I���[
	return 0;
}

void Sequence_Onejump(){
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//JUMP
	output_high(JEJEJEOUT);
	//�W�����v
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);
	
	//�X�y�V�������[�h�I��
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}
void Sequence_Infinityjump(){
	int i;
	output_high(SPECIALLED);
	output_high(SEQUENCEMODE);
	//�O�i
	output_high(SPFORWARD);
	delay_ms(2000);
	output_low (SPFORWARD);

	for(i=0;i<5;i++){
		output_high(JEJEJEOUT);
		//�W�����v
		Sequence_Winding();
		output_low (JEJEJEOUT);
	}
	
	output_low (SPECIALLED);
	output_low (SEQUENCEMODE);
}
void Sequence_Twojump(){
	output_high(SWITCHLED);
	//�ꂪ����̑ҋ@
	delay_ms(600);
	
	//������������
	output_high(JEJEJEOUT);
	//�W�����v
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//�ꂪ����̂܂��ҋ@
	delay_ms(600);
	
	//�ēx�W�����v
	output_high(JEJEJEOUT);
	//�W�����v
	Sequence_Winding();
	output_low (JEJEJEOUT);
	
	//���n�ҋ@
	delay_ms(800);
	output_low (SWITCHLED);
}
int main(void)
{
	initializing();
	
	
	while(input(MOVING)){}
	//�N�����̊����グ
	if(Sequence_Winding() != 0){
		Exception_ERR();
	}
	while(true)
	{
		if(input(MOVING)){
			continue;
		}
		
		if(input(ONEJUMP)){
			Sequence_Onejump();
		}
		
		if(input(INFINITYJUMP)){
			Sequence_Infinityjump();
		}
		
		if((!input(MOVING))&&input(SWITCHINPUT)){
			Sequence_Twojump();
		}
		
		
	}
return 0;
}
