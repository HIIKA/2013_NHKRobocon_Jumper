/*************************************************
ロータリーエンコーダー制御部
ロータリーエンコーダー入力�@	:RB0 <-エンコーダー
ロータリーエンコーダー入力�A	:RB1 <-エンコーダー
確認入力						:RB2 ->ジャンプ制御部 RA0
カウント出力					:RB3 ->ジャンプ制御部 RA1
停止出力						:RB4 ->ジャンプ制御部 RA2
LED切り替え1					:RA0
LED切り替え2					:RA1
ルーレットピン					:PORTC

ルーレット実装テスト
LED16個？
400カウントで一周
LED一つ当たり25カウント
向きがおかしければマクロで変更
**************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR
#DEVICE HIGH_INTS=TRUE

#use delay(clock = 64000000)

#use fast_io(all)

#define ENCODER_A pin_b0
#define ENCODER_B pin_b1
#define CONFIRMIN	pin_b2
#define COUNTOUT	pin_b3
#define STOPTURN	pin_b4
#define SELECTPIN1 pin_a0
#define SELECTPIN2 pin_a1
#define COUNTS		300
#define ROULETTE_DIRECTION	false//ルーレットの回る方向

signed long g_count = 0;
signed long count = 0;

void initializing(void){
	setup_oscillator(OSC_NORMAL | OSC_64MHZ | OSC_PLL_ON);
	set_tris_a(0);
	set_tris_b(0);
	set_tris_c(0);
	output_float(ENCODER_A);
	output_float(ENCODER_B);
	output_float(CONFIRMIN);
	output_drive(COUNTOUT);
	output_drive(STOPTURN);
	output_a(0x00);
	output_b(0);
	setup_timer_0(RTCC_INTERNAL | RTCC_DIV_1);
	enable_interrupts(INT_EXT);
	enable_interrupts(INT_EXT1);	
	enable_interrupts(INT_TIMER0);
	enable_interrupts(GLOBAL);
	set_timer0(0xC600);
	output_c(1);
	output_high(SELECTPIN1);
	output_low (SELECTPIN2);
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
void ToggleSelectPin(void){
	output_toggle(SELECTPIN1);
	output_toggle(SELECTPIN2);
	if(input(SELECTPIN1)==input(SELECTPIN2)){//おかしいとき
		output_high(SELECTPIN1);
		output_low (SELECTPIN2);
	}
}
void RouletteTurnRight(void){
	if((input_c()>>1)!=0){
		output_c(input_c()>>1);
	}else{
		ToggleSelectPin();
		output_c(0x80);
	}
}
void RouletteTurnLeft(){
	if((input_c()<<1)!=0){
		output_c(input_c()<<1);
	}else{
		ToggleSelectPin();
		output_c(0x01);
	}
}

#inline
void Playroulette(void){
	static signed long before_count;
	if(g_count!=0&&abs(g_count)%25==0&&before_count!=g_count){//25の倍数の時ルーレット動かす
		#if ROULETTE_DIRECTION
		RouletteTurnRight();
		#else
		RouletteTurnLeft ();
		#endif
		before_count=g_count;
	}
	if(g_count==0){//0の時は初期位置に戻す
		output_c(1);
		output_high(SELECTPIN1);
		output_low (SELECTPIN2);
	}
}

#INT_TIMER0
void timer()
{
	static unsigned int stop_count = 0;
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
	
	if(COUNTS <= abs(g_count)){
		output_high(COUNTOUT);
	}
	
	if(input(CONFIRMIN)){//確認が来たら
		g_count=0;//リセット
		output_low(COUNTOUT);//カウンタリセットした信号
	}
	
	Playroulette();//ルーレット処理ルーチン
	
	set_timer0(0xC600);
}

//メイン関数/////////////////////////////////////////

void main(){
	initializing();
	
	//起動時割り込みエッジ設定
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
