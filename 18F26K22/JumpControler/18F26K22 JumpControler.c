/************************************************
�W�����v���䕔
���[�^���[�G���R�[�_�[���̓s��1	:RA0 ->encoder RC0 //����]//��������]����
���[�^���[�G���R�[�_�[���̓s��2	:RA1 ->encoder RC1 //�E��]//�t
���[�^���[�G���R�[�_�[���̓s��3	:RA2 ->encoder RC2 //��~�M��
�X�C�b�`���̓s��				:RA3
�_�~�[���̓s��					:RA4
�t�H�g�C���^���v�^���̓s��		:RA5
�����񃂁[�h���̓s��			:RA6(RA7)
�^�N�g�X�C�b�`����			:RB0 
�G���[LED			:RB1 
1��W�����v����		:RB2 
�����W�����v���̓s��		:RB3 
���ʓ���O�i�o��				:RB4
����蒆�o�̓s��			:RB5 
���ʓ��쒆�o�̓s��			:RB6 
�ړ�������			:RB7 
���[�^�[����o�̓s��1			:RC0
���[�^�[����o�̓s��2			:RC1
���[�^�[����o�̓s��3			:RC2
���[�^�[����o�̓s��4			:RC3
�������������o�̓s��			:RC4 
�́[���I�o�̓s��				:RC5 
�V���A���ʐMTX1					:RC6
�V���A���ʐMRX1					:RC7
************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use RS232(baud=19200,xmit=pin_c6,rcv=pin_c7,INVERT)

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
#define DAMMYSWITCH		pin_a4//�_�~�[�X�C�b�`����
#define INTERRUPT	 	pin_a5//�t�H�g�C���^���v�^����
#define INFINITYMODE	pin_a6//������W�����v�̎���High
#define TACTSWITCH		pin_b0//�^�N�g�X�C�b�`�Ő���]�̔�����
#define ERRLED 			pin_b1//�G���[���N����ƌ���
#define ONEJUMP 		pin_b2//���W�����v�i�Z���T�[���䕔������́j
#define INFINITYJUMP 	pin_b3//������W�����v�i�Z���T�[���䕔������́j
#define SPFORWARD 		pin_b4//���ʃV�[�P���X�őO�i����Ƃ��ɏo�͂���
#define WINDING 		pin_b5//����蒆�ɏo��
#define SEQUENCEMODE 	pin_b6//���ʃV�[�P���X���s���Ă���Ƃ��ɏo�͂���
#define MOVING 			pin_b7//�ړ�������
#define JEJEJEOUT		pin_c4//�W�����v����Ƃ��ɂ�����������
#define HEYOUT			pin_c5//�́[���I
#define CLOCKWISE 		0b00001010
#define C_CLOCKWISE 	0b00000101 
#define WISESTOP 		0b00001001
#define DEADTIME		1	//ms
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
bool infinity_flag=0;



//input(SWWITCHINPT)	�X�C�b�`�������True
//input(INTERRUPT)		�؂�ڂ̎�True
//input(MOVING)			�ړ����̎�True
//input(�`�`TURN)		True�̂Ƃ��A���̕����ɉ�]���Ă�,STOP�͎~�܂��Ă�



void timing_bit(bool bits){
	if(bits){
		putc(SIGNALCHAR);
	}else{
		putc(ENDSIGNALCHAR);
	}
}

void yellow_blink(bool bits){
	if(bits){
		putc(YELLOWCHAR);
	}else{
		putc(ENDYELLOWCHAR);
	}
}


void initializing(void){
	//��ڰ��ݒ�
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	//TRIS�ݒ�
	output_a(0x00);
	output_b(0x00);
	output_c(WISESTOP);//��]��~�ŃX�^�[�g
	//a b��tris��ݒ肵�Ȃ�
	set_tris_a(0xff);//0b11111111
	set_tris_b(0x8d);//0b10001101
	set_tris_c(0x80);//0b10000000
	Setup_timer_0(T0_DIV_256);//1count 16us
	
	if(input(INFINITYMODE)){
		putc(INFCLEARCHAR);//�N���������񎞐M���N���A
		infinity_flag=true;
	}else{
		putc(CLEARCHAR);//�N�����M���N���A
		infinity_flag=false;
	}
	
	//�N���ҋ@����
	output_high(SEQUENCEMODE);//���[�^�[�Ƃ������Ȃ��悤��
	putc(WAITINGCHAR);
	while(!input(TACTSWITCH));//�����̑ҋ@
	putc(WAITEDCHAR);
	while( input(TACTSWITCH));//�����̑ҋ@
	delay_ms(300);
	output_low (SEQUENCEMODE);//�N���ҋ@�I��
	while(input(MOVING)){}//�܂��������Ă˂���ȁH
	//�N���ҋ@�����I��
}

void change_c(int port){
	output_c((input_c() & 0xf0) | 0);
	delay_ms(DEADTIME);
	output_c((input_c() & 0xf0) | port);
}
void Exception_ERR(long ms=1000){
	change_c(0);//�o�͂Ȃ�
	output_low (WINDING);
	output_low (SEQUENCEMODE);
	output_low (SPFORWARD);
	putc(ERRCHAR);
	while(true){
		output_toggle(ERRLED);//�������[�v
		delay_ms(ms);
	}
}


//�����グ�����A����ŃW�����v���s��
int Sequence_Winding(unsigned long num=1){//0�̎��͊����グ�̂ݍs��
	unsigned long jumpcounter =0;//�W�����v�̉񐔂𐔂���B
	unsigned int timecounter =0;//12500�J�E���g(0.2�b)��1�C���N�������g
	unsigned int timingcounter=0;//�^�C�~���O�J�E���^�Atimecounter�Ƃ̈Ⴂ�͂���������Ԃ��тɏ����������
	unsigned int stopcounter=0;//���[�^�[����~���Ă���񐔂�0.2�b�����ɐ�����
	bool edgeflag=false ;//�C���^���v�^�̃G�b�W������̂Ɏg��
	bool gapflag=false ;//�����グ�݂̂̎��̃t���O�̈����p��
	int duty=20;
	int nonduty=30;
	TRY{
		change_c( WISESTOP);
		while(!input(STOPTURN)){//�W�����v���[�^�[��~�҂�
		}
		if(num==0){
			gapflag=true;//�؂�ڂ�����
			num+=1;//�؂�ڂ̈ʒu�ɂ���̂łȂ����1��Ɠ�������
		}
		
		output_high(WINDING);
		if(gapflag&&input(INTERRUPT)){//�t���O������A���łɐ؂�ڂɂ���Ȃ�܂킳�Ȃ�
			THROW(END);//�֐������I��
		}
		
		
		edgeflag=input(INTERRUPT);
		change_c(CLOCKWISE);//�����グ���s��
		set_timer0(0);
		do{//while(jumpcounter<num);
			if(get_timer0()>=12500L){//�w��񐔂�
				set_timer0(0);//�^�C�}���Z�b�g����
				if(timecounter<uINT_MAX)timecounter++;//���E�s�����琔����̂�����߂�
				timingcounter++;//�C���N�������g
				//�X�g�[�����o
				if(input(STOPTURN)){
					stopcounter++;//�C���N�������g
					if(stopcounter>=5){//1.0�b�~�܂��Ă�����X�g�[���̋^��
						THROW(ERR);//ERR Throw
					}
				}else{
					stopcounter=0;//��~�M�������Ă��Ȃ���΃J�E���^���Z�b�g;
				}
				//PWM����
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
			
			//�^�C�~���O���Ƃ�
			if(timingcounter >= 3){
				timing_bit(true);
			}else{
				timing_bit(false);
			}

			if(edgeflag!=input(INTERRUPT)){//�C���^���v�^�ɕω�����������
				edgeflag=input(INTERRUPT);//0�Ȃ�؂�ڂ��甲���鎞�A1�Ȃ�؂�ڂɓ��鎞
				if(edgeflag){//�؂�ڂɓ�������
					if(gapflag){//�؂�ڒT���̂Ƃ��͂����܂�
						break;//�W�����v�����Ƃ݂Ȃ��ƃ��[�v�I��
					}
					if(timecounter >= 4){//�����o�������莞�Ԍo���Ă���
						jumpcounter++;//�W�����v�����Ƃ݂Ȃ�
						timingcounter=0;
						duty=0;//PWMreset
						nonduty=50;
					}
				}else{//�؂�ڂ��甲������
					
				}
			}
			
		}while(jumpcounter<num);//�����Ȃ��Ă���
		
	}CATCH(ERR){
		Exception_ERR();
		return 1;
	}CATCH(END){
		//do nothing
	}FINALLY{
		change_c( WISESTOP);//��~
		output_low(WINDING);//�����I���[
		timing_bit(false);//�^�C�~���O�Ƃ�̂͏���
		return 0;//����I��
	}
}

void Sequence_Onejump(){
	int i;
	putc(SPBEGINCHAR);
	output_high(SEQUENCEMODE);
	output_high(HEYOUT);
	
	for(i=0;i<2;i++){//2�b�ԓ_��
		yellow_blink(true);
		delay_ms(500);
		yellow_blink(false);
		delay_ms(500);
	}
	timing_bit(true);
	
		//�W�����v
		Sequence_Winding(1);
	
	//�X�y�V�������[�h�I��
	putc(SPENDCHAR);
	output_low (SEQUENCEMODE);
	output_low (HEYOUT);
}

void Sequence_Infinityjump(unsigned long cont = uLONG_MAX){
	int i;
	putc(SPBEGINCHAR);
	output_high(SEQUENCEMODE);
	
	for(i=0;i<2;i++){//2�b�ԓ_��
		yellow_blink(true);
		delay_ms(500);
		yellow_blink(false);
		delay_ms(500);
	}
	timing_bit(true);
	
		//�W�����v
		Sequence_Winding(cont);//�����̎���uLONG_MAX
	
	putc(SPENDCHAR);
	output_low (SEQUENCEMODE);
}

void Sequence_Twojump(){
	putc(SWITCHCHAR);
	output_high(SEQUENCEMODE);
	//�ꂪ����̑ҋ@
	delay_ms(200);
	timing_bit(false);
	delay_ms(400);
	timing_bit(true);
	delay_ms(200);//total=800
	
		//�W�����v
		Sequence_Winding(2);
	
	putc(ENDSWITCHCHAR);
	//���n�ҋ@
	delay_ms(400);
	output_low (SEQUENCEMODE);
}
int main(void)
{
	int duty;
	int nonduty;
	bool movingflag=true;//�G�b�W���o�݂̂Ɏg��
	int timecounter=0;
	delay_ms(100);
	initializing();

	//�N�����̊����グ
	Sequence_Winding(0);
	
	while(true)
	{	
		if(movingflag!=input(MOVING)){
			movingflag=input(MOVING);
			if(!movingflag){//�ړ����ł͂Ȃ��Ȃ����Ƃ�
				set_timer0(0);
				timecounter=0;
			}
		}
		
		if(!input(MOVING)){
			//�ړ����łȂ���
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
			movingflag=true;//�Ǌ��O�ɍs�����̂œ����Ă������Ƃɂ���
			timecounter=0;//�ꉞ���ԃ��Z�b�g
			continue;
		}
		
		if(input(INFINITYJUMP)){
			if(infinity_flag){//�����W�����v����ׂ����ۂ�
				Sequence_Infinityjump(uLONG_MAX);
			}else{
				Sequence_Infinityjump(6);
			}
			
			movingflag=true;//�Ǌ��O�ɍs�����̂œ����Ă������Ƃɂ���
			timecounter=0;//�ꉞ���ԃ��Z�b�g
			continue;
		}
		
		if(get_timer0() >= 62500L){
			set_timer0(0);
			if(timecounter!=uINT_MAX)timecounter++;
		}
		
		if(! movingflag && !input(MOVING)&& timecounter >= 1 && input(SWITCHINPUT) && !input(DAMMYSWITCH)){
			output_high(JEJEJEOUT);
			Sequence_Twojump();
			output_low (JEJEJEOUT);
			movingflag=true;//�Ǌ��O�ɍs�����̂œ����Ă������Ƃɂ���
			timecounter=0;//�ꉞ���ԃ��Z�b�g
			continue;
		}
	}
return 0;
}
