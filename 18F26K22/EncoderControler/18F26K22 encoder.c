/*************************************************
���[�^���[�G���R�[�_�[���䕔
���[�^���[�G���R�[�_�[���͇@	:RB0 <-�G���R�[�_�[
���[�^���[�G���R�[�_�[���͇A	:RB1 <-�G���R�[�_�[
90�x�ŏo��						:RC6 ->�W�����v���䕔 RA0
���Z�b�g����					:RC5 ->�W�����v���䕔 RA1
��~�o��						:RC4 ->�W�����v���䕔 RA2

**************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR
#DEVICE HIGH_INTS=TRUE

#use delay(clock = 64000000)

#use fast_io(all)

#define ENCODER_A pin_b0
#define ENCODER_B pin_b1
#define NORMALOUT	pin_c6
#define MESUREIN	pin_c5//�p�x���胂�[�h
#define STOPTURN	pin_c4

signed long g_count = 0;
unsigned long count = 0;

void initializing(void){
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	set_tris_a(0);
	set_tris_b(0x03);//RB7-2:IN1-0:OUT
	set_tris_c(0b00100000);
	output_a(0x00);
	output_c(0);
	setup_timer_0(RTCC_INTERNAL | RTCC_DIV_1);
	enable_interrupts(INT_EXT);
	enable_interrupts(INT_EXT1);	
	enable_interrupts(INT_TIMER0);
	enable_interrupts(GLOBAL);
	set_timer0(0xC600);
}

#inline
void g_count_add(void)
{
	if(g_count==LONG_MAX){
		g_count=LONG_MIN;
	}else{
		g_count+=1;
	}
}
#inline
void g_count_sub(void)
{
	if(g_count==LONG_MIN){
		g_count=LONG_MAX;
	}else{
		g_count-=1;
	}
}

#INT_EXT high
void ext0(void)
{
	int a, b;
	a = input(ENCODER_A);
	b = input(ENCODER_B);
	if (a && b)
	{
		ext_int_edge(0, H_TO_L);
		ext_int_edge(1, H_TO_L);
		g_count_sub();
		output_low (STOPTURN);
		//output_c(RIGHTTURN);
	}
	else if (!a && !b)
	{
		ext_int_edge(0, L_TO_H);
		ext_int_edge(1, L_TO_H);
		g_count_sub();
		output_low (STOPTURN);
		//output_c(RIGHTTURN);
	}
	else if (a && !b)
	{
		ext_int_edge(0, H_TO_L);
		ext_int_edge(1, L_TO_H);
		g_count_add();
		output_low (STOPTURN);
		//output_c(LEFTTURN);
	}
	else if (!a && b)
	{
		ext_int_edge(0, L_TO_H);
		ext_int_edge(1, H_TO_L);
		g_count_add();
		output_low (STOPTURN);
		//output_c(LEFTTURN);
	}
}

#INT_EXT1 high
void ext1(void)
{
	int a, b;
	a = input(ENCODER_A);
	b = input(ENCODER_B);
	if (a && b)
	{
		ext_int_edge(0, H_TO_L);
		ext_int_edge(1, H_TO_L);
		g_count_add();
		output_low (STOPTURN);
		//output_c(LEFTTURN);
	}
	else if (!a && !b)
	{
		ext_int_edge(0, L_TO_H);
		ext_int_edge(1, L_TO_H);
		g_count_add();
		output_low (STOPTURN);
		//output_c(LEFTTURN);
	}
	else if (a && !b)
	{
		ext_int_edge(0, H_TO_L);
		ext_int_edge(1, L_TO_H);
		g_count_sub();
		output_low (STOPTURN);
		//output_c(RIGHTTURN);
	}
	else if (!a && b)
	{
		ext_int_edge(0, L_TO_H);
		ext_int_edge(1, H_TO_L);
		g_count_sub();
		output_low (STOPTURN);
		//output_c(RIGHTTURN);
	}
}

#INT_TIMER0
void timer()
{
	static unsigned int stop_count = 0;
	static int1 reset_flag=0;
	if (g_count == count)
	{
		stop_count++;
	}
	else
	{
		stop_count = 0;
	}

	if (stop_count == 40)
	{
		stop_count = 0;
		output_high(STOPTURN);
		//output_c(STOPTURN);
	}
	
	count = g_count;
	
	//�J�E���^���Z�b�g����
	if(reset_flag!=input(MESUREIN)){
		reset_flag=input(MESUREIN);
		if(reset_flag){//high�ɂȂ��Ă���
			output_low (NORMALOUT);
			g_count=0;//���Z�b�g
		}
	}
	
	if(abs(g_count)>=100&&reset_flag){//���胂�[�h��
		output_high(NORMALOUT);
	}else{
		output_low (NORMALOUT);
	}
	
	set_timer0(0xC600);
}

//���C���֐�/////////////////////////////////////////

void main(){
	initializing();
	
	//�N�������荞�݃G�b�W�ݒ�
	if (input(ENCODER_A))
		ext_int_edge(0, H_TO_L);
	else
		ext_int_edge(0, L_TO_H);

	if (input(ENCODER_B))
		ext_int_edge(1, H_TO_L);
	else
		ext_int_edge(1, L_TO_H);
	

	while(true)
	{
	}
}
