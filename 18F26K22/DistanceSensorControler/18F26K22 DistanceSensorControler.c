/************************************************
�����Z���T�[���䕔
			RB7=>���ʃV�[�P���X���̓����~���󂯎����̓s��
			RB0=>LED1//�Ȃ񂩂Ȃ����ĂȂ��Z���T�[�����I
�O�i�Z���T�[			:RA0
��i�Z���T�[			:RA1
�E����Z���T�[			:RA2
������Z���T�[			:RA3
���W�����v�Z���T�[	:RA4
������W�����v�Z���T�[	:RA5
����2��W�����v�Z���T�[	:RC3
�O�i�o��				:RB2
��i�o��				:RB3
�E����o��				:RB4
������o��				:RB5
���W�����v�o��		:RB6
������W�����v�o��		:RB1

************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)


#define SSW_DELAY 800				//���߂�ő�̋���[mm]
#define INTERVALTIME 0xF8AD
#define DETECTDISTANCE 200 //��������
#define AUTO2JDISTANCE 100 //auto2j�̔�������
#define ERR_LED pin_b0//�G���[�o�͗p
#define SPECIALPIN pin_b7//���ʓ���
//�o�̓s���Ɠ��̓s���i�O���[�o���ϐ��j


#define datasize int8//�f�[�^�T�C�Y(�Z���T�[��8�ȏ�ɂȂ�����int16)
unsigned datasize trans_data=0;//���M����f�[�^�e�r�b�g������󋵂�\��
unsigned datasize before_data=0;//�O�񑗐M�����Ƃ��̏�

/*
int1 output_low(out_forward);
int1 out_back;
int1 out_right;
int1 out_left;
int1 out_onejump;
int1 out_infjump;
*/

#define was_forward()	bit_test(before_data,0)//�O�i���Ă�����?
#define was_back()		bit_test(before_data,1)
#define was_right()		bit_test(before_data,2)
#define was_left()		bit_test(before_data,3)
#define is_forward()	bit_test(trans_data,0)//�O�i���Ă邩?
#define is_back()		bit_test(trans_data,1)
#define is_right()		bit_test(trans_data,2)
#define is_left()		bit_test(trans_data,3)
#define is_onejump()	bit_test(trans_data,4)
#define is_infjump()	bit_test(trans_data,5)
#define is_auto2j()		bit_test(trans_data,6)

#define pin_forward pin_a0
#define pin_back pin_a1
#define pin_right pin_a2
#define pin_left pin_a3
#define pin_onejump pin_a4
#define pin_infjump pin_a5
#define pin_auto2j pin_c3

#define out_forward pin_b2//getenv("BIT:RB2")
#define out_back pin_b3//getenv("BIT:RB3")
#define out_right pin_b4//getenv("BIT:RB4")
#define out_left pin_b5//getenv("BIT:RB5")
#define out_onejump pin_b1//getenv("BIT:RB6")
#define out_infjump pin_b6//getenv("BIT:RB1")



long inputpins[]	={pin_forward,pin_back,pin_right,pin_left,pin_onejump,pin_infjump,pin_auto2j};
int flags[] 		={          0,       0,        0,       0,          0,          0,         0};//���������񐔂𐔂���
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

#inline
void data_transport(void){//���݂̏󋵂𑗐M����
	int i;
	unsigned datasize temp_data = trans_data ^ before_data;
	if(temp_data==0){//�ω����Ȃ��Ƃ��͏I��
		return;
	}
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]) ; ++i){
		if(!bit_test(temp_data,i)){//�ω����Ă��Ȃ��ꍇ
			continue;//do nothing
		}
		switch(inputpins[i]){
			//*******************************************************
		case pin_forward:
			if(bit_test(trans_data,i)){//�O�i����
				if(is_right()){
					output_high(out_forward);
					output_high(out_right);
					output_low(out_left);
					output_low(out_back);
				}else if(is_left()){
					output_high(out_forward);
					output_low(out_right);
					output_high(out_left);
					output_low(out_back);
				}else{//�O�i�ł���i�ł��Ȃ��Ƃ�
					output_high(out_forward);
					output_low(out_right);
					output_low(out_left);
					output_low(out_back);
				}
			}else{
				output_low(out_forward);
			}
			break;
			//*******************************************************
		case pin_back:
			if(bit_test(trans_data,i)){
				if(is_right()){
					output_low(out_forward);
					output_low(out_right);
					output_high(out_left);
					output_high(out_back);
				}else if(is_left()){
					output_low(out_forward);
					output_high(out_right);
					output_low(out_left);
					output_high(out_back);
				}else{//�O�i�ł���i�ł��Ȃ��Ƃ�
					output_low(out_forward);
					output_low(out_right);
					output_low(out_left);
					output_high(out_back);
				}
			}else{
				output_low(out_back);
			}
			break;
			//*******************************************************
		case pin_right:
			if(bit_test(trans_data,i)){
				if(!is_forward()&&!is_back()){//�O�i�ł���i�ł��Ȃ��Ƃ�
					output_low(out_forward);
					output_high(out_right);
					output_low(out_left);
					output_low(out_back);
				}
			}else{
				output_low(out_right);
			}
			break;
			//*******************************************************
		case pin_left:
			if(bit_test(trans_data,i)){
				if(!is_forward()&&!is_back()){//�O�i�ł���i�ł��Ȃ��Ƃ�
					output_low(out_forward);
					output_low(out_right);
					output_high(out_left);
					output_low(out_back);
				}
			}else{
				output_low(out_left);
			}
			break;
			//*******************************************************
		case pin_onejump 	:
			if(bit_test(trans_data,i) && !bit_test(before_data,i)){
				output_high(out_onejump);
				output_low(out_infjump);
			}
			break;
			//*******************************************************
		case pin_infjump 	:
			if(bit_test(trans_data,i) && !bit_test(before_data,i)){
				output_low(out_onejump);
				output_high(out_infjump);
			}
			break;
			//*******************************************************
		case pin_auto2j 	:
			if(bit_test(trans_data,i) && !bit_test(before_data,i)){
				output_high(out_onejump);
				output_high(out_infjump);
			}
			break;
		}
	}
	
	if(! is_onejump() && ! is_infjump() && ! is_auto2j() ){
		output_low(out_onejump);
		output_low(out_infjump);
	}
	
	//�Ƃ܂��Ă�Ƃ�
	if(!is_right()&&!is_left()&&!is_forward()&&!is_back()){
		output_low(out_forward);
		output_low(out_right);
		output_low(out_left);
		output_low(out_back);
	}
	
	before_data = trans_data;
}


/*�����̑���
�Ԃ��l�͈̔�
0 �` SSW_DELAY					:���苗��
SSW_DELAY+1 �` SSW_DELAY+99		:����͈͊O
SSW_DELAY+100 �`				:�Z���T�[��������
*/
long check_ssw(long ssw_sig){
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
		if(get_timer3() > (800*2))//�K�i�ł�750us��ɒl��Ԃ��Ă���͂�
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
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)//���ׂ�OFF
	{
		flags[i]=0;//�t���O�S������
	}
	trans_data=0;//�o�͑Slow
	data_transport();//�o�͕ύX
	output_high(ERR_LED);
	flag_err=true;//�G���[����
}

#inline
void check_sensor(void){//1.5�b��1��Ă΂ꂽ��(��])
	int servivecounter=0;
	unsigned int i;
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i){
		if(SSW_DELAY+100 <= check_ssw(inputpins[i])){//���Ă���
			ExceptionDistanceERR();//i�ԉ��Ă��
		}else{
			servivecounter++;//�����Ă��
		}
	}
	
	if(servivecounter==(sizeof(inputpins)/sizeof(inputpins[0]) )){//�S�������Ă�����ۂ�������
		output_low (ERR_LED);
		flag_err=false;//�G���[�Ȃ�ĂȂ�����
	}
}

#inline
int1 check_continue(unsigned int num){
	
	//�ړ����̓W�����v�֌W�̃Z���T���~
	if(input(out_forward)||input(out_back)||input(out_right)||input(out_left)){
		if(inputpins[num]==pin_onejump
		 ||inputpins[num]==pin_infjump
		 ||inputpins[num]==pin_auto2j){//�W�����v�֌W
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	
	//���ׂẴZ���T�[���~
	if(input(SPECIALPIN)){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//�G���[�t���O�������Ă���Ƃ������ׂē��삵�Ȃ�
	if(flag_err){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//��i���Ă���Ƃ��͑O��
	if(inputpins[num]==pin_forward && was_back()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//�O�i���Ă���Ƃ��͌㒎
	if(inputpins[num]==pin_back && was_forward()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	//�E�܂���Ă�Ƃ��͍���
	if(inputpins[num]==pin_left && was_right()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	//���܂���Ă�Ƃ��͉E��
	if(inputpins[num]==pin_right && was_left()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//�O��ɓ����ĂȂ��Đ��񂵂Ă���Ƃ��O�i��i�𖳎�
	if(inputpins[num]==pin_forward || inputpins[num]==pin_back){
		if(!was_forward() && !was_back()&& (was_right() || was_left()) ){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//���ʓ��쒆�͂ق��̃s����
	if(input(out_onejump) && !input(out_infjump)){
		if(inputpins[num]==pin_infjump||inputpins[num]==pin_auto2j){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//���ʓ��쒆�͂ق��̃s����
	if(!input(out_onejump) && input(out_infjump)){
		if(inputpins[num]==pin_onejump||inputpins[num]==pin_auto2j){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//���ʓ��쒆�͂ق��̃s����
	if(input(out_onejump) && input(out_infjump)){
		if(inputpins[num]==pin_onejump||inputpins[num]==pin_infjump){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	
	
	return (0);
}

#INT_timer0
int interval(void)
{
	long distance;
	int i;
	static int checkcounter=0;//100���1��͑��`�F�b�N������
	long detectivedistance;
	
	if( 50 < checkcounter++ || flag_err)
	{
		checkcounter=0;//���Z�b�g
		check_sensor();
	}
	
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)
	{
		if(check_continue(i)){
			continue;
		}
		
		//�������Ƃ�
		distance=check_ssw(inputpins[i]);
		//���ĂȂ����_��
		if(distance < SSW_DELAY+100)//����͈͂̎�(over���܂�)
		{
			if(inputpins[i]==pin_auto2j){
				detectivedistance=AUTO2JDISTANCE;
			}else{
				detectivedistance=DETECTDISTANCE;
			}
			
			if(distance < detectivedistance)//�w�苗�����̎�
			{
				flags[i]=(flags[i]<3) ? flags[i]+1:flags[i];//�C���N�������g
			}else{
				flags[i]=(flags[i]>0) ? flags[i]-1:flags[i];//���炷
			}
			
			if(bit_test(trans_data,i)==0 && flags[i]==3){//���̃Z���T�����쒆�łȂ��A3�ɂȂ�����
				bit_set(trans_data,i);
			}
			if(bit_test(trans_data,i)==1 && flags[i]==0){//���쒆�̃Z���T��0�ɂȂ��Ă��܂�����
				bit_clear(trans_data,i);
			}
			
		}
		else if(SSW_DELAY+100 <= distance){
			ExceptionDistanceERR();
		}
	}
	
	data_transport();//�����Ńf�[�^�𑗐M����

	set_timer0(INTERVALTIME);//30ms�����Ɋ��荞��
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
