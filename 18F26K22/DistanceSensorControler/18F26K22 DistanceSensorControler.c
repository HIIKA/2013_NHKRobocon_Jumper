/************************************************
�����Z���T�[���䕔
RB7=>���ʃV�[�P���X���̓����~���󂯎����̓s��
		���̓s��	<=>	�o�̓s��
			RA0	<=>	RB6 
			RA1	<=>	RB5
			RA2	<=>	RB4
			RA3	<=>	RB3
			RA4	<=>	RB2
			RA5	<=>	RB1
			RC3 <=> RC4
//RC0 <=> RC7
//RC1 <=> RC6
//RC2 <=> RC5

RB0=>LED1//�Ȃ񂩂Ȃ����ĂȂ��Z���T�[�����I
************************************************/
#include <18f26K22.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)


#define SSW_DELAY 800				//���߂�ő�̋���[mm]
#define INTERVALTIME 0xF8AD
#define DETECTDISTANCE 250 //��������
#define ERR_LED pin_b0//�G���[�o�͗p
#define SPECIALPIN pin_b7//���ʓ���
//�o�̓s���Ɠ��̓s���i�O���[�o���ϐ��j
int outputpins[]	={pin_b6,pin_b5};//,pin_b4,pin_b3,pin_b2,pin_b1,pin_c4
int inputpins[]		={pin_a0,pin_a1};//,pin_a2,pin_a3,pin_a4,pin_a5,pin_c3
int1 flag_ERR=false;

void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ);
	//TRIS�ݒ�
	set_tris_a(0);
	set_tris_b(0x80);
	set_tris_c(0);
	output_a(0x00);
	output_b(0x00);
	output_c(0x00);
	//�^�C�}�[�Z�b�g�A�b�v
	setup_timer_0(T0_INTERNAL|T0_DIV_256);//�C���^�[�o���p
	set_timer0(INTERVALTIME);
	setup_timer_3(T3_INTERNAL|T3_DIV_BY_8);//�Z���T�[�Ɏg��
	
	//���荞�݋���
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_timer0);

	output_low (ERR_LED);
}


/*�����̑���
�Ԃ��l�͈̔�
0 �` SSW_DELAY					:���苗��
SSW_DELAY+1 �` SSW_DELAY+99		:����͈͊O
SSW_DELAY+100 �`				:�Z���T�[��������
*/
long check_ssw(int ssw_sig){
	long nagasa;
	output_drive(ssw_sig);		//SSW���Ȃ����s�����o�͂ɐݒ�
	//ssw_tris=ssw_tris&~ssw_port;
	output_low(ssw_sig);
	delay_us(2);
	output_high(ssw_sig);				//�p���X�o�͖��߂̑��M
	delay_us(5);
	output_low(ssw_sig);
	output_float(ssw_sig);//SSW���Ȃ����s������͂ɐݒ�
	//ssw_tris=ssw_tris|ssw_port;
	
	set_timer3(0);
	while(!input(ssw_sig))
	{
		if(get_timer3() > (760*2))//�K�i�ł�750us��ɒl��Ԃ��Ă���͂�
		{
			return SSW_DELAY + 200;//�Z���T�[�������Ȃ�����
		}
	}			//�p���X�܂ő҂�
	
	set_timer3(0);
	while(get_timer3()<SSW_DELAY*12){
		if(!input(ssw_sig)){
			nagasa=get_timer3()/12;//������Ԃ�
			return nagasa;
		}
	}
	return SSW_DELAY+10;//over����
}

void ExceptionDistanceERR(void)
{
	int i;
	for(i=0; i < sizeof(outputpins)/sizeof(outputpins[0]); ++i)//���ׂ�OFF
	{
		output_low(outputpins[i]);
	}
	output_high(ERR_LED);
	flag_err=true;//�G���[����
}

#INT_timer0
int interval(void)
{
	long distance;
	int i;
	static int operating=0;

	
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)
	{
		//�������Ƃ�
		distance=check_ssw(inputpins[i]);
		//���ĂȂ����_��
		if(distance < SSW_DELAY+100)//����͈͂̎�(over���܂�)
		{
			//�����Ő���Ȃ�X���[
			if(operating!=0 && operating!=inputpins[i])
			{
				output_low(outputpins[i]);
				continue;//���݉ғ�����pin�ł͂Ȃ���
			}
			
			//���ʓ��쒆�̓Z���T�[�͂��ׂč쓮���Ȃ�
			if(input(SPECIALPIN)){
				output_low(outputpins[i]);
				continue;
			}
			
			//�G���[�t���O�������Ă���Ƃ������ׂē��삵�Ȃ�
			if(flag_err){
				output_low(outputpins[i]);
				continue;
			}
			
			
			if(distance < DETECTDISTANCE)//�w�苗�����̎�
			{
				output_high(outputpins[i]);
				operating=inputpins[i];//���݉ғ������w��
				continue;
			}else{
				output_low(outputpins[i]);
				operating=0;//�ǂ���ғ����ĂȂ�
			}
		}
		else if(SSW_DELAY+100 <= distance){
			ExceptionDistanceERR();
		}
	}
	set_timer0(INTERVALTIME);
	return 0;
}


int main(void)
{
	initializing();
	
	while(true)
	{
	}
		
	return 0;
}
