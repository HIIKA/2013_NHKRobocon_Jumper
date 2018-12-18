#include <18f26K22.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR
#use delay(clock = 64000000)

#byte port_a = 0xF80
#byte port_b = 0xF81
#byte port_c = 0xF82

#use fast_io(all)

#define PR2 49

//�F�ύX�̓z
#define Set_RED_duty Set_pwm2_duty
#define Set_GREEN_duty Set_pwm1_duty
#define Set_Blue_duty Set_pwm4_duty
//�F�̑I��̎Q�l
//http://lowlife.jp/yasusii/static/color_chart.html
//��float��100�{����΂���


#inline //���̒��x�̌v�Z�̓C�����C���W�J������
long DetermineDuty(unsigned int percent){
	if(percent>=100)percent=100;
	return (4*(PR2+1)/100)*(long)percent;//���_���Ȃ̂ŋt�]����̂͂��@��
}

void CreateColor(unsigned int red,unsigned int green,unsigned int blue){
	if(red	>100)red	=100;
	if(green>100)green	=100;
	if(blue	>100)blue	=100;
	Set_RED_duty	(DetermineDuty(red));
	Set_GREEN_duty	(DetermineDuty(green));
	Set_BLUE_duty	(DetermineDuty(blue));
}
void main(void)
{
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
	set_tris_a(0);
	set_tris_b(0);
	set_tris_c(0);
	output_a(0x00);
	output_b(0x00);
	output_c(0x00);
	//�������iPR2�{�P�j�~4*1/f�~�iTMR2�̃v���X�P�[���l�j
	//�f���[�e�B��DC1�~1/f�~�iTMR2�̃v���X�P�[���l�j
	//duty cycle = value / [ 4 * (PR2 +1 ) ]
	Setup_ccp1(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_c2
	Setup_ccp2(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_c1
	Setup_ccp4(CCP_PWM | CCP_USE_TIMER1_AND_TIMER2);//pin_c6
	
	Setup_timer_2(T2_DIV_BY_16,PR2,1);//20khz
	Set_RED_duty(DetermineDuty(0));
	Set_GREEN_duty(DetermineDuty(0));
	Set_BLUE_duty(DetermineDuty(0));

	while(true)
	{
		CreateColor(100,0,0);
		delay_ms(100);
		CreateColor(0,100,0);
		delay_ms(100);
		CreateColor(0,0,100);
		delay_ms(100);
	}
}
