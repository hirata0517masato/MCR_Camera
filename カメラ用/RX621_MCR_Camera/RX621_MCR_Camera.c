/****************************************************************************************************/
/* プロジェクト名：RX621_SAMPLE     	                　　　　　          			    */
/* モジュール名：  			                     					    */
/* 履    歴    ：										    */
/* 使用マイコン：RX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* 作成者      ：hirata0517masato               				                                            */
/* バージョン  ：0.00                                                                               */
/* 作成日      ：2017/02/10 									    */
/****************************************************************************************************/                  

//メモ：最適化オプションは無しにしたほうが良い

#include "iodefine.h"
#include <machine.h>
//#include "lowsrc.h"

#include "usb.h"

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init(void);

void CLK_init(void);
void IO_init(void);
void AD_init(void);
void CMT_init(void);

void cam_out(void);
void ImageCapture(int,int);
int  get_ad( void );
void expose( void );
void expose2( void );
void binarization(int,int);
void WhiteLineWide(int,int);

void AE_check(void);


/* 定数設定 */

//#define PRINT//データ出力時のみ有効にする

/* TAOS TSL1401CL */
#define	TAOS_SI_HIGH	PORTD.DR.BIT.B0 = 1	/* CN3-17 */
#define	TAOS_SI_LOW	PORTD.DR.BIT.B0 = 0	/* CN3-17 */
#define	TAOS_CLK_HIGH	PORTD.DR.BIT.B1 = 1	/* CN3-18 */
#define	TAOS_CLK_LOW	PORTD.DR.BIT.B1 = 0	/* CN3-18 */

#define MODE_HIGH_BIT	PORT3.PORT.BIT.B0 /* CN2-8 */
#define MODE_LOW_BIT	PORT3.PORT.BIT.B1 /* CN2-7 */

#define WIDE_OUT 	PORT2.DR.BYTE /* CN2-16-9 */
#define CENTER_OUT 	PORTC.DR.BYTE /* CN4-19-20 CN2-30-25 */

//AD0 /* CN3-9 P40 */

#define		Line_Max	760		/* ライン白色MAX値の設定 */
#define		Line_Max_Core	80		/* Line_Max - Line_Max_Core < 現在の明るさ < Line_Max + Line_Max_Core  がAE収束状態とする　*/
#define		Line_Max_Core2	250		/* 現在の明るさが大幅に外れていたら露光時間の変化量を増加させる＝早く目標値に到達させたい*/

#define 	LineStart 	27		/* カメラで見る範囲(通常モード) */
#define 	LineStop  	100

#define 	LineStartSaka 	50		/* カメラで見る範囲(坂モード) */
#define 	LineStopSaka  	77

#define 	LineStartNonR 	35		/* カメラで見る範囲(右無視) */
#define 	LineStopNonR  	64

#define 	LineStartNonL 	64		/* カメラで見る範囲(左無視) */
#define 	LineStopNonL  	92



/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   print_cnt =  0;
int    		print_cnt2=  0;

/* カメラ関連 */
unsigned long	EXPOSURE_timer = 15000;	/* 露光時間	20000				*/
int		ImageData[130];			/* カメラの値				*/
int 		BinarizationData[130];	/* ２値化					*/

int		Max = 0,Min,Ave;	/*カメラ読み取り最大値、最小値、平均値*/
int 	Ave_old = 0;

unsigned int 	Rsensor;				/* ラインの右端 */
unsigned int 	Lsensor;				/* ラインの左端 */
unsigned int 	Wide = 0;				/* ラインの幅 */
int		Center = 0;				/* ラインの重心 */

unsigned int 	Wide_old;				/* 過去の太いラインの幅 */
int		Center_old;				/* 過去の太いラインの重心 */
int		old_flag = 0;			/*0 = 通常出力, 1 = oldを出力*/
int		old_cnt = 0;			/* old出力用カウント	*/

int		Center_lasttime;		/* 前回のラインの重心 */

int             White;					/* 白の数	*/

int		mode;				/* 0 = 通常 1 = 坂 2 = 右無視 3 = 左無視	*/	

int		EXPOSURE_cnt = 0;		/* */		

char		AE_fin_fag = 0;			/* AEが完了 = 1 まだ = 0   明るさの最大値をチェックしているので真っ黒のときは０になるので注意 */

//printf用バッファ
unsigned int 	Rsensor_PBuf;				/* ラインの右端 */
unsigned int 	Lsensor_PBuf;				/* ラインの左端 */
unsigned int 	Wide_PBuf;				/* ラインの幅 */
int		Center_PBuf;				/* ラインの重心 */
int		Max_PBuf,Min_PBuf;
unsigned long	EXPOSURE_timer_PBuf;
char		AE_fin_fag_PBuf;
/***********************************************************************/
/* メインプログラム                                                    */
/***********************************************************************/
void main(void)
{
	int	i = 0;
 	
    	/* マイコン機能の初期化 */	
	CLK_init();  // クロックの初期化
 	IO_init();   // IOの初期化
 //	CMT_init();  // CMTの初期化
 	AD_init();   // A/Dの初期化
		
#ifdef PRINT
	USB_init();  //USB CDCの初期化
#endif //PRINT
	
	/* Data Initialization */
	for(i=0;i<130;i++){
		ImageData[i] = 0;			/* カメラの値				*/
		BinarizationData[i] = 0;	/* ２値化					*/
	}
	
	cam_out();//制御用へ出力
	
	
	//明るさの最大値が目標値に近づくまでループ && ラインを発見するまでループ
	do{
		
		#ifndef PRINT
		mode = (MODE_HIGH_BIT & 0x01) + ((MODE_LOW_BIT & 0x01) << 1);//モード判定に使用
		#endif
		
		expose2();				//露光時間（全白、全黒でも時間変更)
		
		ImageCapture(LineStart,LineStop);	//イメージキャプチャー
		
		AE_check();				//AE収束判定
		
		binarization(LineStart,LineStop); 	//２値化
		
		WhiteLineWide(LineStart,LineStop);	//白ラインの測定	
		
		//cam_out();//制御用へ出力
		Center_lasttime = Center;//過去の値を保存
		
	}while(!( (AE_fin_fag == 1) && (Wide != 0) && (Wide < 25)  ));// （明るさ調整OK　＆＆　ラインの幅がOK　）でなければ
	
	
	while( 1 ) {
	
		#ifndef PRINT
		mode = (MODE_HIGH_BIT & 0x01) + ((MODE_LOW_BIT & 0x01) << 1);//モード判定に使用
		#endif
		
		switch(mode){
			case 0://通常モード
				expose();				//露光時間
				
				ImageCapture(LineStart,LineStop);	//イメージキャプチャー
		
				AE_check();				//AE収束判定
				
				binarization(LineStart,LineStop); 	//２値化
		
				WhiteLineWide(LineStart,LineStop);	//白ラインの測定
				
				break;
				
			case 1://坂モード
				expose();					//露光時間
				
				ImageCapture(LineStartSaka,LineStopSaka);	//イメージキャプチャー
		
				AE_check();					//AE収束判定
				
				binarization(LineStartSaka,LineStopSaka); 	//２値化
		
				WhiteLineWide(LineStartSaka,LineStopSaka);	//白ラインの測定
				
				break;
				
			case 2://右無視
				expose();					//露光時間
				
				ImageCapture(LineStartNonR,LineStopNonR);	//イメージキャプチャー
		
				AE_check();					//AE収束判定
				
				binarization(LineStartNonR,LineStopNonR); 	//２値化
		
				WhiteLineWide(LineStartNonR,LineStopNonR);	//白ラインの測定
				
				break;
				
			case 3://左無視
				expose();					//露光時間
				
				ImageCapture(LineStartNonL,LineStopNonL);	//イメージキャプチャー
		
				AE_check();					//AE収束判定
				
				binarization(LineStartNonL,LineStopNonL); 	//２値化
		
				WhiteLineWide(LineStartNonL,LineStopNonL);	//白ラインの測定
			
				break;
		}
	
		
		cam_out();//制御用へ出力
		Center_lasttime = Center;//過去の値を保存
		
				
		#ifdef PRINT
			print_cnt++;
			
			if(print_cnt > 10){
				
				if(print_cnt2 == 0){
					//for(i = LineStart; i <= LineStop; i++)printf("%d",BinarizationData[i]);
					for(i = LineStart; i <= LineStop; i+=2)printf("%d",(char)BinarizationData[i]);
					
					Center_PBuf = Center;
					Wide_PBuf = Wide;
					Lsensor_PBuf = Lsensor;
					Rsensor_PBuf = Rsensor;
					Max_PBuf = Max;
					Min_PBuf = Min;
					EXPOSURE_timer_PBuf= EXPOSURE_timer;
					AE_fin_fag_PBuf = AE_fin_fag;
				
				}else{
					printf("Center = %d Wide = %d Lsensor = %d Rsensor = %d ",Center_PBuf,Wide_PBuf,Lsensor_PBuf,Rsensor_PBuf);
				
					printf("Max = %d Min = %d time = %d AE_fin = %d ",Max_PBuf,Min_PBuf,EXPOSURE_timer_PBuf,AE_fin_fag_PBuf);
				
					printf("\n");
				}
				print_cnt=0;
				print_cnt2++;
				if(print_cnt2 > 1)print_cnt2 = 0;
			}
		#endif
    }
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：クロックの初期化                                                                    */
/* 関 数 詳 細：システムクロック96MHz,周辺モジュールクロック24MHz                                   */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CLK_init(void)
{
   SYSTEM.SCKCR.BIT.ICK = 0;		// システムクロック(ICLK)       EXTAL×8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// 周辺モジュールクロック(PCLK)	EXTAL×2 (24MHz)
     
   //SYSTEM.SUBOSCCR = 1;              // 0：サブクロック発振器動作（0：デフォルトで動作 1:停止）
   //RTC.RCR2.BIT.RTCOE = 1;           // 1：RTCOUTを端子(P32)から出力する
   
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：I/Oの初期化                                                                    　　 */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void IO_init(void)
{
	//unsigned int uc;
	
 //  PORTC.PCR.BYTE   = 0x03;        // PC0,1をプルアップ指定
    
    PORT2.DDR.BYTE = 0xff;           // P2を出力に設定
    PORTC.DDR.BYTE = 0xff;           // PCを出力に設定
    PORTD.DDR.BYTE = 0xff;           // PDを出力に設定

    PORT3.DDR.BYTE = 0x00;           // P3を入力に設定    
    PORT4.DDR.BYTE = 0x00;           // P4を入力に設定   AN0  
 /*   
    // シリアル通信 
	MSTP(SCI1) = 0;			//ストップ解除
	SCI1.SCR.BYTE = 0x00;	// 内蔵クロック、送受信禁止 
	SCI1.SMR.BYTE = 0x00;	// PCLKクロック、STOP1bit、パリティ無し、8Bitデータ、調歩同期式 
	SCI1.BRR = 80;			// 77 = 19200bps 
	for(uc=0;uc<1000;uc++);
	SCI1.SSR.BYTE = 0x00;
	// 送信許可 
	SCI1.SCR.BYTE = 0x20;
	IEN(SCI1,RXI1) = 1;	// SCI1のRXI1割り込み要求許可 
	IPR(SCI1,RXI1) = 2;	// SCI1のRXI1割り込みレベル設定 
	IR(SCI1,RXI1) = 0;
*/	
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：A/Dの初期化                                                                         */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void AD_init(void)
{
   //A/Dの初期化
  // SYSTEM.MSTPCRA.BIT.MSTPA22 = 0; // AD1モジュールストップの解除
   SYSTEM.MSTPCRA.BIT.MSTPA23 = 0; // AD0モジュールストップの解除
   SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bitでA/D変換する
   
   MSTP(S12AD) = 0;//モジュールストップ状態を解除

   S12AD.ADCSR.BYTE = 0x0c;//動作モード、変換開始要件、クロックの設定
   S12AD.ADANS.WORD = 0x0001;//変換端子の設定
   S12AD.ADSTRGR.BYTE = 0x0000;//A/D 変換開始要件の設定
	
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：CMT0,CMT1 (コンペアマッチタイマー)の初期化                                          */
/* 関 数 詳 細：1ms割り込み周期                                                                     */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT_init(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT0.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 3000-1;           // 1ms Count： PCLK = 24MHz/8=3MHz 3M/1mS=3000 (得たいカウント数-1)
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
    	
    MSTP(CMT1) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT1 タイマースタンバイ解除 （0で解除）
    CMT1.CMCR.WORD = 0x0040;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT1.CMCOR = 3000-1;           // 1ms Count： PCLK = 24MHz/8=3MHz 3M/1mS=3000 (得たいカウント数-1)
    IPR(CMT1,CMI1) = 3;
    IEN(CMT1,CMI1) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0タイマースタート
    CMT.CMSTR0.BIT.STR1 = 1;       // CMT1タイマースタート

}
	


/************************************************************************/
/* 露光時間調整(全白　全黒の時は変更しない)                             */
/* 引数　 なし                                                          */
/* 戻り値 なし　　　　　                                                */
/************************************************************************/
/*
void expose( void )
{
	unsigned long i;
	int sa = Line_Max - Max;
	
	//if( Wide != 0 && White <= 60){//黒でなく白でもない
	if( Wide == 0 || White >= 70){//黒or白
		EXPOSURE_cnt++;
		if(EXPOSURE_cnt > 100)EXPOSURE_cnt = 100;
	}else{
		EXPOSURE_cnt = 0;
	}
		
	if(EXPOSURE_cnt < 5){
		if(AE_fin_fag == 1 ||  Wide == 0){//明るさが目標値に近い || 黒  //黒の時に露光量を変化させすぎるとラインの復帰ができなくなる
			
			if(-50 < sa && sa < 50){
				//EXPOSURE_timer += (long)sa * 1;
			}else{
				
				if((long)EXPOSURE_timer + (long)sa * 10 < 0)EXPOSURE_timer = 0; //オーバーフロー対策　
				else EXPOSURE_timer += (long)sa * 10;
			}
			
		}else{
			if((long)EXPOSURE_timer + (long)sa * 20 < 0)EXPOSURE_timer = 0; //オーバーフロー対策　
			else EXPOSURE_timer += (long)sa * 20;//早く目標値に近づけたいので変化量を大きくする
		
		}
	}
		
	
	if( EXPOSURE_timer > 100000) EXPOSURE_timer = 100000;
	else if( EXPOSURE_timer <= 1000 ) EXPOSURE_timer = 1000;

	for(i=0;i<EXPOSURE_timer;i++);

}
*/

void expose( void )
{
	unsigned long i;
	int sa = Line_Max - Max;
	
	//if( Wide != 0 && White <= 60){//黒でなく白でもない
	if( Wide == 0 || White >= 45){//黒or白
		EXPOSURE_cnt++;
		if(EXPOSURE_cnt > 100)EXPOSURE_cnt = 100;
	}else{
		EXPOSURE_cnt = 0;
	}
	
	if(EXPOSURE_cnt < 2 &&  Wide != 0 ){
		//if(-20 < sa && sa < 20)EXPOSURE_timer += (long)(sa*5);
		//else 
		
		//オーバーフロー対策 かつ　急激には変化しないようにする（急激な変化＝真っ黒or真っ白かも）
		if(sa > 100) sa = 100;
		if(sa < -100) sa = -100;
		
		EXPOSURE_timer += (long)sa * 10;
	}	
		
	
	if( EXPOSURE_timer > 100000) EXPOSURE_timer = 100000;
	else if( EXPOSURE_timer <= 1000 ) EXPOSURE_timer = 1000;

	for(i=0;i<EXPOSURE_timer;i++);

}

/************************************************************************/
/* 露光時間調整                                                         */
/* 引数　 なし                                                          */
/* 戻り値 なし　　　　　                                                */
/************************************************************************/
void expose2( void )
{
	unsigned long i;
	
	//EXPOSURE_timer += (long)((Line_Max - Max)*10);
	
	if(Line_Max - Max < 0){
		EXPOSURE_timer -= 100;
	}else{
		EXPOSURE_timer += 100;
	}
	if( EXPOSURE_timer > 100000) EXPOSURE_timer = 100000;
	else if( EXPOSURE_timer <= 1000 ) EXPOSURE_timer = 1000;
	
	for(i=0;i<EXPOSURE_timer;i++);

}
 
 /************************************************************************/
/* イメージキャプチャ                                                   */
/************************************************************************/
void ImageCapture(int linestart, int linestop){	 
	
	unsigned char i;

	Max = 0,Min = 4096;

	TAOS_SI_HIGH;
	TAOS_CLK_HIGH;
	TAOS_SI_LOW;
	ImageData[0] = 0;
	TAOS_CLK_LOW;
	for(i = 1; i < LineStart; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}
	for(i = LineStart; i < linestart; i++) {		
		TAOS_CLK_HIGH;	
		ImageData[i] = get_ad();
		TAOS_CLK_LOW;
	}
	
	for(i = linestart; i <= linestop; i++) {				
		 
		TAOS_CLK_HIGH;
		ImageData[i] = get_ad();	// inputs data from camera (one pixel each time through loop) 
		TAOS_CLK_LOW;
		
		if(Max < ImageData[i]){
			Max = ImageData[i];
		}			
		if(Min > ImageData[i]){
			Min = ImageData[i];
		}
		
	}
	
	for(i = linestop+1; i <= LineStop; i++) {		
		TAOS_CLK_HIGH;	
		ImageData[i] = get_ad();
		TAOS_CLK_LOW;
	}
	for(i = LineStop+1; i < 128; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}
	
	TAOS_CLK_HIGH;
	TAOS_CLK_LOW;
}
/************************************************************************/
/* A/D値読み込み(AN0)                                                 */
/* 引数　 なし                                                          */
/* 戻り値 A/D値 0〜4069                                                 */
/************************************************************************/
int get_ad(void)
{
/*	AD0.ADCSR.BIT.ADST = 1;  	// A/D 0の変換開始
   	while(AD0.ADCSR.BIT.ADST == 0); // 測定が終了するまで待つ
	return AD0.ADDRA;
*/	
	
	S12AD.ADCSR.BIT.ADST = 1;  	// A/D 0の変換開始
   	while(S12AD.ADCSR.BIT.ADST == 0); // 測定が終了するまで待つ
	return S12AD.ADDR0;	
}
/************************************************************************/
/* AE収束判定                                                            */
/************************************************************************/
void AE_check(){
	
	if(((-Line_Max_Core < (Line_Max - Max)) && ((Line_Max - Max) < Line_Max_Core))){
		AE_fin_fag = 1;
	}else{
		AE_fin_fag = 0;
	}
}

/************************************************************************/
/* ２値化                                                               */
/************************************************************************/
void binarization(int linestart, int linestop)
{
	int i,a;
	
	/* 最高値と最低値から間の値を求める */
		
	//if(mode == 1) Ave = Min + 130;
	//else Ave = ((Max + Min) >> 1) - 50;
	//else{
		a  = ((Max + Min) >> 1);
		Ave = ((a+Min) >> 1);
		Ave = ((a+Ave) >> 1);
		//Ave  = ((Max + Min) >> 1);
	//}
	
	/* 黒は０　白は１にする */
	White = 0;					/* 白の数を０にする */

	/*
	if(Max - Min > 50){//白黒の差がある程度あるなら２直化できる
		for(i = linestart ; i <= linestop; i++) {
			if(  ImageData[i] > Ave ){	
				White++;			
				BinarizationData[i] = 1;
			}else{
				BinarizationData[i] = 0;
			}	
		}
	}else{//白黒の差が無い＝真っ白 or 真っ黒
		if( Max > Line_Max - 400 ){//真っ白
			White = 127;
			for(i = linestart ; i <= linestop; i++) {
				BinarizationData[i] = 1;
			}
		}else{//真っ黒
			for(i = linestart ; i <= linestop; i++) {
				BinarizationData[i] = 0;
			}
		}
	}
*/	
	
	if( Max > Line_Max - 500 ){//320 -150  250
		// 白が一直線のとき 
		//if(Min > Line_Max - 80 ){//260
		if(Max - Min < 50){
			White = 127;
			for(i = linestart ; i <= linestop; i++) {
				BinarizationData[i] = 1;
			}
		}else{		
			for(i = linestart ; i <= linestop; i++) {
				if(  ImageData[i] > Ave ){	
					White++;			
					BinarizationData[i] = 1;
				}else{
					BinarizationData[i] = 0;
				}	
			}
		}
	// 黒が一面のとき 
	}else{
		for(i = linestart ; i <= linestop; i++) {
			BinarizationData[i] = 0;
		}
	}
	

	//範囲外は黒に
	for(i = 0; i < linestart; i++){
		BinarizationData[i] = 0;
	}
	for(i = linestop+1; i < 128; i++){
		BinarizationData[i] = 0;
	}
}


/************************************************************************/
/* 白線の幅を測定                                                       */
/************************************************************************/
void WhiteLineWide(int linestart, int linestop)
{
	int t,i;
		
	Lsensor = linestart;Rsensor = linestop;Wide = 0;t = 0;		
	
	if(Center_lasttime < 60){//ライン左寄り
		for(i = linestart ; i <= linestop; i++) {
			if(t==0){
				if( BinarizationData[i] ){					/* 左から最初の白 */
					Lsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* 左から最初の黒 */			
					Rsensor = i;
					t = 2;
				}
			}
		}
	}else{//ライン右寄り
		for(i = linestop; i >= linestart; i--) {
			if(t==0){
				if( BinarizationData[i] ){					/* 右から最初の白 */
					Rsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* 右から最初の黒 */			
					Lsensor = i;
					t = 2;
				}
			}
		}	
	}
		
	
	if(White > 70){//全白にする
		Wide = 127;Center = 64;						/* 白一面 */
		
	}else if((White > 6) && ((linestop - linestart) > 5)){//白が少なすぎない && ラインを探す範囲が狭すぎない
	
		Wide = Rsensor - Lsensor;					/* 幅を求める */	
		Center = (Lsensor + Rsensor) >> 1;		/* 重心を求める */	
			
			
		//ライン細すぎ || ( 前回、黒又は白一色ではない && ハーフラインなどではない &&  (急にラインが移動した))
		if((((mode == 1) && (Wide < 4)) || ((mode != 1) && (Wide < 6))) || ((Center_lasttime != 64) && (White < 20) && (((Center - Center_lasttime) > 15) || ((Center - Center_lasttime) < -15)))){
					
			if(Center_lasttime < 60){
						
				WhiteLineWide(Rsensor,linestop);//もう一度ラインを探す			
			}else{
					
				WhiteLineWide(linestart,Lsensor);//もう一度ラインを探す
			}
		}	
	}else{//全黒にする
		Wide = 0;Center = 64;						/* 黒一面 */
	}	
}

/**********************************************************************/
/*	R8CへCenterとWideの送信											  */
/*																	  */
/**********************************************************************/
void cam_out(){
	
	/*
	int wide,center;
	
	if(old_flag == 0){//通常出力
		wide = Wide;
		center = Center;
			
		if(Wide > 40){//幅が太い
			Center_old = Center;
			Wide_old = Wide;
			
			old_flag = 1;//old出力にする
			old_cnt = 0;
		}
	}else{//old出力
			
		if(Wide_old < Wide ){//さらに太い幅が見つかった
			Center_old = Center;
			Wide_old = Wide;
					
			old_cnt--;
		}
		
		wide = Wide_old;
		center = Center_old;		
			
		old_cnt++;
		if((old_cnt > 3) || (Center_old - Center < -5 || Center_old - Center > 5)){
			
			wide = Wide;
			center = Center;
		
			old_flag = 0;//通常出力に戻す
				
		}
	}
	
	WIDE_OUT  = wide;
	CENTER_OUT = center;
	*/
	
	WIDE_OUT  = (Wide << 1)&0xfe;//NEW
	//WIDE_OUT  = Wide; //手配線
	CENTER_OUT = Center;
	
	//WIDE_OUT  = Max/10;
	//CENTER_OUT = Min/10;
}

