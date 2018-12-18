/************************************************
距離センサー制御部
			RB7=>特別シーケンス中の動作停止を受け取る入力ピン
			RB0=>LED1//なんかつながってないセンサーあるよ！
前進センサー			:RA0
後進センサー			:RA1
右旋回センサー			:RA2
左旋回センサー			:RA3
一回ジャンプセンサー	:RA4
無限回ジャンプセンサー	:RA5
自動2回ジャンプセンサー	:RC3
前進出力				:RB2
後進出力				:RB3
右旋回出力				:RB4
左旋回出力				:RB5
一回ジャンプ出力		:RB6
無限回ジャンプ出力		:RB1

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
#define AUTO2JDISTANCE 100 //auto2jの反応距離
#define ERR_LED pin_b0//エラー出力用
#define SPECIALPIN pin_b7//特別動作
//出力ピンと入力ピン（グローバル変数）


#define datasize int8//データサイズ(センサーが8個以上になったらint16)
unsigned datasize trans_data=0;//送信するデータ各ビットが動作状況を表す
unsigned datasize before_data=0;//前回送信したときの状況

/*
int1 output_low(out_forward);
int1 out_back;
int1 out_right;
int1 out_left;
int1 out_onejump;
int1 out_infjump;
*/

#define was_forward()	bit_test(before_data,0)//前進していたか?
#define was_back()		bit_test(before_data,1)
#define was_right()		bit_test(before_data,2)
#define was_left()		bit_test(before_data,3)
#define is_forward()	bit_test(trans_data,0)//前進してるか?
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
int flags[] 		={          0,       0,        0,       0,          0,          0,         0};//反応した回数を数える
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

#inline
void data_transport(void){//現在の状況を送信する
	int i;
	unsigned datasize temp_data = trans_data ^ before_data;
	if(temp_data==0){//変化がないときは終了
		return;
	}
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]) ; ++i){
		if(!bit_test(temp_data,i)){//変化していない場合
			continue;//do nothing
		}
		switch(inputpins[i]){
			//*******************************************************
		case pin_forward:
			if(bit_test(trans_data,i)){//前進時に
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
				}else{//前進でも後進でもないとき
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
				}else{//前進でも後進でもないとき
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
				if(!is_forward()&&!is_back()){//前進でも後進でもないとき
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
				if(!is_forward()&&!is_back()){//前進でも後進でもないとき
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
	
	//とまってるとき
	if(!is_right()&&!is_left()&&!is_forward()&&!is_back()){
		output_low(out_forward);
		output_low(out_right);
		output_low(out_left);
		output_low(out_back);
	}
	
	before_data = trans_data;
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
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)//すべてOFF
	{
		flags[i]=0;//フラグ全初期化
	}
	trans_data=0;//出力全low
	data_transport();//出力変更
	output_high(ERR_LED);
	flag_err=true;//エラー発生
}

#inline
void check_sensor(void){//1.5秒に1回呼ばれたい(願望)
	int servivecounter=0;
	unsigned int i;
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i){
		if(SSW_DELAY+100 <= check_ssw(inputpins[i])){//壊れてたら
			ExceptionDistanceERR();//i番壊れてるよ
		}else{
			servivecounter++;//生きてるよ
		}
	}
	
	if(servivecounter==(sizeof(inputpins)/sizeof(inputpins[0]) )){//全部生きているっぽかったら
		output_low (ERR_LED);
		flag_err=false;//エラーなんてなかった
	}
}

#inline
int1 check_continue(unsigned int num){
	
	//移動中はジャンプ関係のセンサを停止
	if(input(out_forward)||input(out_back)||input(out_right)||input(out_left)){
		if(inputpins[num]==pin_onejump
		 ||inputpins[num]==pin_infjump
		 ||inputpins[num]==pin_auto2j){//ジャンプ関係
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	
	//すべてのセンサーを停止
	if(input(SPECIALPIN)){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//エラーフラグが立っているときもすべて動作しない
	if(flag_err){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//後進しているときは前虫
	if(inputpins[num]==pin_forward && was_back()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//前進しているときは後虫
	if(inputpins[num]==pin_back && was_forward()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	//右まわってるときは左虫
	if(inputpins[num]==pin_left && was_right()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	//左まわってるときは右虫
	if(inputpins[num]==pin_right && was_left()){
		flags[num]=0;
		bit_clear(trans_data,num);
		return (1);
	}
	
	//前後に動いてなくて旋回しているとき前進後進を無視
	if(inputpins[num]==pin_forward || inputpins[num]==pin_back){
		if(!was_forward() && !was_back()&& (was_right() || was_left()) ){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//特別動作中はほかのピン虫
	if(input(out_onejump) && !input(out_infjump)){
		if(inputpins[num]==pin_infjump||inputpins[num]==pin_auto2j){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//特別動作中はほかのピン虫
	if(!input(out_onejump) && input(out_infjump)){
		if(inputpins[num]==pin_onejump||inputpins[num]==pin_auto2j){
			flags[num]=0;
			bit_clear(trans_data,num);
			return (1);
		}
	}
	//特別動作中はほかのピン虫
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
	static int checkcounter=0;//100回に1回は総チェックを入れる
	long detectivedistance;
	
	if( 50 < checkcounter++ || flag_err)
	{
		checkcounter=0;//リセット
		check_sensor();
	}
	
	for(i=0; i < sizeof(inputpins)/sizeof(inputpins[0]); ++i)
	{
		if(check_continue(i)){
			continue;
		}
		
		//距離をとる
		distance=check_ssw(inputpins[i]);
		//壊れてないか点検
		if(distance < SSW_DELAY+100)//正常範囲の時(overを含む)
		{
			if(inputpins[i]==pin_auto2j){
				detectivedistance=AUTO2JDISTANCE;
			}else{
				detectivedistance=DETECTDISTANCE;
			}
			
			if(distance < detectivedistance)//指定距離内の時
			{
				flags[i]=(flags[i]<3) ? flags[i]+1:flags[i];//インクリメント
			}else{
				flags[i]=(flags[i]>0) ? flags[i]-1:flags[i];//減らす
			}
			
			if(bit_test(trans_data,i)==0 && flags[i]==3){//そのセンサが動作中でなく、3になったら
				bit_set(trans_data,i);
			}
			if(bit_test(trans_data,i)==1 && flags[i]==0){//動作中のセンサが0になってしまったら
				bit_clear(trans_data,i);
			}
			
		}
		else if(SSW_DELAY+100 <= distance){
			ExceptionDistanceERR();
		}
	}
	
	data_transport();//ここでデータを送信する

	set_timer0(INTERVALTIME);//30msおきに割り込み
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
