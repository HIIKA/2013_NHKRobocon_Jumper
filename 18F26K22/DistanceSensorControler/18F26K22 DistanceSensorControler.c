/************************************************
距離センサー制御部
			RB7=>特別シーケンス中の動作停止を受け取る入力ピン
			RB0=>LED1//なんかつながってないセンサーあるよ！
		入力ピン	<=>	出力ピン
			RA0	<=>	RB6 
			RA1	<=>	RB5
			RA2	<=>	RB4
			RA3	<=>	RB3
			RA4	<=>	RB2
			RA5	<=>	RB1
//RC3 <=> RC4
//RC0 <=> RC7
//RC1 <=> RC6
//RC2 <=> RC5
************************************************/
#include <18f26K22.h>
#include <limits.h>
#fuses PLLEN, INTRC_IO,NOWDT,PUT,NOPROTECT,BROWNOUT,NOLVP,MCLR

#use delay(clock = 64000000)

#use fast_io(a)
#use fast_io(b)
#use fast_io(c)


#define SSW_DELAY 800				//求める最大の距離[mm]
#define INTERVALTIME 0xF8AD
#define DETECTDISTANCE 200 //反応距離
#define ERR_LED pin_b0//エラー出力用
#define SPECIALPIN pin_b7//特別動作
//出力ピンと入力ピン（グローバル変数）
long outputpins[]	={pin_b6,pin_b5,pin_b4,pin_b3,pin_b2,pin_b1};//
long inputpins[]	={pin_a0,pin_a1,pin_a2,pin_a3,pin_a4,pin_a5};//
int flags[] 		={     0,     0,     0,     0,     0,     0};//反応した回数を数える
int1 flag_ERR=false;

void initializing(void){
	//ｵｼﾚｰﾀ設定
	setup_oscillator(OSC_NORMAL | OSC_64MHZ);
	//TRIS設定
	set_tris_a(0);
	set_tris_b(0x80);
	set_tris_c(0);
	output_a(0x00);
	output_b(0x00);
	output_c(0x00);
	//タイマーセットアップ
	setup_timer_0(T0_INTERNAL|T0_DIV_256);//インターバル用
	set_timer0(INTERVALTIME);
	setup_timer_3(T3_INTERNAL|T3_DIV_BY_8);//センサーに使う
	
	//割り込み許可
	enable_interrupts(GLOBAL);
	enable_interrupts(INT_timer0);

	output_low (ERR_LED);
}


/*距離の測定
返す値の範囲
0 ～ SSW_DELAY					:測定距離
SSW_DELAY+1 ～ SSW_DELAY+99		:測定範囲外
SSW_DELAY+100 ～				:センサー反応せず
*/
long check_ssw(long ssw_sig){
	long nagasa;
	output_drive(ssw_sig);		//SSWをつないだピンを出力に設定
	//ssw_tris=ssw_tris&~ssw_port;
	output_low(ssw_sig);
	delay_us(2);
	output_high(ssw_sig);				//パルス出力命令の送信
	delay_us(5);
	output_low(ssw_sig);
	output_float(ssw_sig);//SSWをつないだピンを入力に設定
	//ssw_tris=ssw_tris|ssw_port;
	
	set_timer3(0);
	while(!input(ssw_sig))
	{
		if(get_timer3() > (800*2))//規格では750us後に値を返してくるはず
		{
			return SSW_DELAY + 200;//センサー反応しなかった
		}
	}			//パルスまで待つ
	
	set_timer3(0);
	while(get_timer3()<SSW_DELAY*12){
		if(!input(ssw_sig)){
			nagasa=get_timer3()/12;//長さを返す
			return nagasa;
		}
	}
	return SSW_DELAY+10;//overした
}

void ExceptionDistanceERR(void)
{
	int i;
	for(i=0; i < sizeof(outputpins)/sizeof(outputpins[0]); ++i)//すべてOFF
	{
		output_low(outputpins[i]);
		flags[i]=0;
	}
	output_high(ERR_LED);
	flag_err=true;//エラー発生
}

#INT_timer0
int interval(void)
{
	long distance;
	int i;
	static long operating=0;
	int servivecounter=0;
	
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)
	{
		//距離をとる
		distance=check_ssw(inputpins[i]);
		//壊れてないか点検
		if(distance < SSW_DELAY+100)//正常範囲の時(overを含む)
		{
			servivecounter++;//壊れていないものをカウントする
			
			//現在稼働中のpinではない時ここで正常なら次へ
			if(operating!=0 && operating!=inputpins[i])
			{
				output_low(outputpins[i]);
				flags[i]=0;
				continue;
			}
			
			//特別動作中はセンサーはすべて作動しない
			if(input(SPECIALPIN)){
				output_low(outputpins[i]);
				flags[i]=0;
				continue;
			}
			
			//エラーフラグが立っているときもすべて動作しない
			if(flag_err){
				output_low(outputpins[i]);
				flags[i]=0;
				continue;
			}
			
			
			if(distance < DETECTDISTANCE)//指定距離内の時
			{
				flags[i]=(flags[i]<3) ? flags[i]+1:flags[i];//インクリメント
			}else{
				flags[i]=(flags[i]>0) ? flags[i]-1:flags[i];//減らす
			}
			
			if(operating==0 && flags[i]==3){//動作中のセンサがなく、3になったら
				output_high(outputpins[i]);
				operating=inputpins[i];//現在稼働中を指定
			}
			if(operating==inputpins[i] && flags[i]==0){//動作中のセンサが0になってしまったら
				output_low(outputpins[i]);
				operating=0;//どれも稼働してない
			}
			
		}
		else if(SSW_DELAY+100 <= distance){
			ExceptionDistanceERR();
		}
	}
	
	if(servivecounter==(sizeof(inputpins)/sizeof(inputpins[0]) )){//全部生きているっぽかったら
		output_low (ERR_LED);
		flag_err=false;//エラーなんてなかった
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
