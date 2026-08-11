/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
/********************************************************************************
*																				*
*	ファイル名	:Trace2.c												　　	*
*	概要		:RX621ライントレースプログラム									*
*																				*
********************************************************************************/
                  
#include "iodefine.h"
#include <machine.h>


void main(void);
void init_CMT(void);
void init(void);

 // グローバル変数の定義
 //**********************************************************//
 //   変数の定義                                             //
 //                                                          //
 //   型                    値の範囲       データサイズ      //
 //  char:                -128 ～ 127          1Byte         //
 //  unsigned char:          0 ～ 255          1Byte         //
 //  short:             -32768 ～ 32767        2Byte         //
 //  unsigned short          0 ～ 65535        2Byte         //
 //  int                -32768 ～ 32767        2Byte         //
 //  unsigned int            0 ～ 65535        2Byte         //
 //  long          -2147483648 ～ 2147483647   4byte         //
 //  unsigned long           0 ～ 4294967295   4Byte         //
 //                                                          //
 //**********************************************************//
#define	vshort	volatile short

vshort mode;             // メインモード選択
short  beep_pulse = 0;
short  beep_enable;	
short  beep_long;	
unsigned long n;
unsigned long goal_cnt = 0;    

//short i;
//short x;
//short y;

unsigned short GOAL;
unsigned short GOAL_LIM;
unsigned char  GOAL_FLG1 = 0;
unsigned short GOAL_FLG2 = 0;
unsigned short GOAL_REF; 

unsigned short SIDE_R;
unsigned short SIDE_R_LIM;
unsigned char  SIDE_R_FLG = 0;
unsigned short SIDE_R_REF;

unsigned short SIDE_L;
unsigned short SIDE_L_LIM;
unsigned char  SIDE_L_FLG = 0;
unsigned short SIDE_L_REF;

unsigned short CENTER_R;
unsigned short CENTER_R_LIM;
unsigned char  CENTER_R_FLG = 0;
unsigned short CENTER_R_REF;

unsigned short CENTER_L;
unsigned short CENTER_L_LIM;
unsigned char  CENTER_L_FLG = 0;
unsigned short CENTER_L_REF;

unsigned short MARKER;
unsigned short MARKER_LIM;
unsigned char  MARKER_FLG1 = 0;
unsigned short MARKER_FLG2 = 0;
unsigned short MARKER_FLG3 = 0;
unsigned short MARKER_REF;

unsigned short SEN_LIM;

short beep_cnt = 300;
short task;
short RUN_MODE;
unsigned short LOOP = 0;
short LED_speed;        // 7セグLEDサイクルの回転スピード
short dig;              // 7セグLED表示
short LED_cycle;        // 7セグLEDサーキット許可フラグ

char START_FLG = 0;
char SLOW_SPD  = 0;
 
unsigned short ct;
unsigned short cnt;
unsigned short cnt1;
unsigned long wait_timer;

short R_control_err;
short L_control_err;

char control_gain;

float STR_SPD;
float TURN_SPD;
float STR_SPD_NORMAL;
float STR_SPD_LOW;   
float TURN_SPD_HIGH;  
float TURN_SPD_LOW;  
  
float SPEED1;
float SPEED2;  


//**************************************************************************************
// 変数の定義
//**************************************************************************************
#define		START_SW	  PORTC.PORT.BIT.B0		// PC0: スタートスイッチ
#define		MODE_SW	  	  PORTC.PORT.BIT.B1		// PC1: モードスイッチ
#define		SW_ON	      0				        // タクトスイッチ"ＯＮ"   の論理			
#define		SW_OFF	      1				        // タクトスイッチ"ＯＦＦ" の論理
#define	    SEG_LED       PORTD.DR.BYTE         // PD0-7: 7セグLED の点灯
#define     SEN_DRV       PORT3.DR.BIT.B0       // P30: センサ駆動パルス
#define     BEEP          PORTE.DR.BIT.B0       // PE0: BEEP

#define     L_PWM		  PORT2.DR.BIT.B0       // P20: LモーターPWM出力  
#define     L_IN2		  PORT2.DR.BIT.B1       // P21: LモーターIN2
#define     L_IN1		  PORT2.DR.BIT.B2       // P22: LモーターIN1
#define     STBY		  PORT2.DR.BIT.B3       // P23: モーターSTBY
#define     R_IN1		  PORT2.DR.BIT.B4       // P24: RモーターIN1  
#define     R_IN2		  PORT2.DR.BIT.B5       // P25: RモーターIN2
#define     R_PWM		  PORT2.DR.BIT.B6       // P26: RモーターPWM出力

#define		PD0	          PORTD.DR.BIT.B0 		// PD0: 7セグLED: A          
#define		PD1 	      PORTD.DR.BIT.B1 		// PD1: 7セグLED: B          
#define		PD2	          PORTD.DR.BIT.B2 		// PD2: 7セグLED: C          
#define		PD3 	      PORTD.DR.BIT.B3 		// PD3: 7セグLED: D          
#define		PD4	          PORTD.DR.BIT.B4 		// PD4: 7セグLED: E          
#define		PD5 	      PORTD.DR.BIT.B5 		// PD5: 7セグLED: F          
#define		PD6	          PORTD.DR.BIT.B6 		// PD6: 7セグLED: G
#define		PD7	          PORTD.DR.BIT.B7 		// PD7: 7セグLED: DP


//**************************************************************************************
// I/Oの初期化
//**************************************************************************************
void init(void)
{

// クロックの設定 
   SYSTEM.SCKCR.BIT.ICK = 0;		// システムクロック(ICLK)       EXTAL×8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// 周辺モジュールクロック(PCLK)	EXTAL×2 (24MHz)
	
// IOポートの設定 
    PORTC.ICR.BIT.B0 = 0;               // PC0を入力に設定(START SW)
    PORTC.ICR.BIT.B1 = 0;	        // PC1を入力に設定(MODE SW)
    PORTC.PCR.BYTE   = 0x03;            // PC0,1をプルアップ指定

    PORTD.DDR.BYTE = 0xFF;	        // PD0～PE7を出力に設定(7セグ)
    PORT2.DDR.BYTE = 0xBE;	        // P20～P27を出力に設定(モーター)
    PORTE.DDR.BYTE = 0xFF;	        // PE0を出力に設定(BEEP)
    PORT3.DDR.BYTE = 0x01;	        // P30を出力に設定(センサードライブ)
    PORT0.DDR.BIT.B5 =  1;		// P05

//A/Dの初期化
   SYSTEM.MSTPCRA.BIT.MSTPA22 = 0; // AD1モジュールストップの解除
   SYSTEM.MSTPCRA.BIT.MSTPA23 = 0; // AD0モジュールストップの解除

}

//***************************************************************************
// beep data  "ドレミファソラシド  ドシレソファミレド”
//****************************************************************************
 
 unsigned char beep_data[17] = 
 
 {0,220, 196, 175, 165, 147, 131, 117, 110,          
  110, 117, 131, 147, 165, 175, 196, 220};
  
//**************************************************************************************
// CMT0,CMT1 (コンペアマッチタイマー)の初期化 
//**************************************************************************************
void init_CMT(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT0.CMCR.WORD = 0x0041;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 50;               // 設定 50;
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
    	
    MSTP(CMT1) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 タイマースタンバイ解除 （0で解除）
    CMT1.CMCR.WORD = 0x0041;       // 4:割り込み許可　0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT1.CMCOR = 400;              // 音程の設定 800;
    IPR(CMT1,CMI1) = 6;
    IEN(CMT1,CMI1) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0タイマースタート
    CMT.CMSTR0.BIT.STR1 = 1;       // CMT1タイマースタート
}
	
//------------------------------------------------------------------------
// 	ビープ発生モジュール     j:音程選択            		                            
//------------------------------------------------------------------------
void beep()
{
    beep_long = 0;
	 
} 
 
//**************************************************************************************
// MTU1(左モーター）,MTU2(右モーター） タイマーの初期化 
//**************************************************************************************
void mtu_init(void)
{
	volatile int C_cycle,duty;
	
	C_cycle = 24e6 / 1000;
	
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;  // MUTユニット０　モジュールストップ解除
	MTUA.TSTR.BYTE = 0x00;  // カウンタの停止
	
	MTU1.TCR.BIT.CCLR = 0x1; //
	MTU2.TCR.BIT.CCLR = 0x1; //
	
	MTU1.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU2.TMDR.BIT.MD = 0x2;  // PWM MODE1

	MTU1.TIOR.BIT.IOA = 0x6; //
	MTU1.TIOR.BIT.IOB = 0x5; //
	MTU2.TIOR.BIT.IOA = 0x6; //
	MTU2.TIOR.BIT.IOB = 0x5; //
		
	MTU1.TGRA = C_cycle;
	MTU2.TGRA = C_cycle;
	MTU1.TGRB = 0;
	MTU2.TGRB = 0;
	
	//MTUA.TOER.BIT.OE4A = 1;  //
	
	MTUA.TSTR.BYTE = 0x0F;	// 0000 1111
	
}
		
		
//**************************************************************************************
// SET PWM 	
//**************************************************************************************
void set_pwm (float duty_L, float duty_R)
{
	
    unsigned int dt_L, dt_R;
	
    if(duty_L >100.0) duty_L = 100.0; // duty_Lを100以上にしない
    if(duty_L <  0.0) duty_L =   0.0; // duty_Lを　0以下にしない
    if(duty_R >100.0) duty_R = 100.0;
    if(duty_R <  0.0) duty_R =   0.0;
	
    /* デューティ比の算出 */
    dt_L = MTU1.TGRA * duty_L / 100.0;//  dt_L = 0.9445*50/100 = 0.5
    dt_R = MTU2.TGRA * duty_R / 100.0;//
	
	/* デューティ比のオーバーフロー保護 */
    if(dt_L >= MTU1.TGRA)   dt_L = MTU1.TGRA - 1;  // 
    if(dt_R >= MTU2.TGRA)   dt_R = MTU2.TGRA - 1;
	
    /* デューティ比の設定 */
    MTU1.TGRB = dt_L;
    MTU2.TGRB = dt_R;
}

		
//***************************************************
// display data cycle
//***************************************************
 unsigned char disp_cycle_data[6] =
 
    { 0x01,0x02,0x04,0x08,0x10,0x20 };  // LED cycle
    


//========================================================================
//      ウェイト
//========================================================================
void wait (unsigned long n)        
{ 
	wait_timer = 0;
	while ( wait_timer < n  );  // cntがnになるまでループ
}                  


//**************************************************************************************
// display data 0 ～ F highで点灯
//**************************************************************************************
 unsigned char disp_data[16] =
   { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27,    // LED display data = "0"～"7" 
     0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};   // LED display data = "8"～"F" 


//**************************************************************************************
// 走行モジュール MODE0　ゴールで止まらない
//**************************************************************************************
void mode_0(void)
{

  STBY        = 1;       // モータースタンバイ解除
  SEG_LED     = 0x00;    // ７SEG LEDの表示クリア
  SEN_DRV     = 1;       // センサー用LEDの発光 ON
  
  STR_SPD_NORMAL = 22.0;    // 直進通常スピード 22.0
//STR_SPD_LOW    = 19.0;    // 直進減速スピード
//TURN_SPD_HIGH  = 21.0;    // ターンスピード高 20.0
//TURN_SPD_LOW   = 19.0;    // ターンスピード低 20.0
  
  GOAL_FLG2   = 0;
  MARKER_FLG2 = 0; 
  
  //SPEED1 = STR_SPD_NORMAL;      // 最初は通常スピード
  //SPEED2 = TURN_SPD_LOW;
  
  cnt = 0;
  while ( cnt < 1000 ) { LED_cycle == 1; beep(); }   //  暫く待つ
  LED_cycle = 0;
  RUN_MODE = 0;
  
  while(1)
	
    {        
	
		     if  (( GOAL_FLG2   == 1 ) && ( START_FLG == 0 ))                   // ①スタートラインを切った
			      { 
				    L_IN1 = 1; L_IN2 = 0; // 左前進
			        R_IN1 = 1; R_IN2 = 0; // 右前進
			        PD7 = 1;              // 7SEGドット点灯
			        while ( GOAL_FLG1  == 1 ){ set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL); } // スタートラインを通り過ぎるまで進む
					//PD7 = 0;  
				    RUN_MODE = 6;
				  }              
			 
			 
			
        else if  (( MARKER_FLG2 == 1 ) || ( GOAL_FLG2  == 1 ) && ( START_FLG == 1 ))                                // ②ライン検出 (どちらかを検出）ラインを通過するまで暫く進む
		       {    
				    goal_cnt = 0;
		            L_IN1 = 1; L_IN2 = 0; // 左前進
			        R_IN1 = 1; R_IN2 = 0; // 右前進
					
                          if ( MARKER_FLG2 == 1 ) { while  ( MARKER_FLG1 == 1 )          { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // マーカーライン(クロスライン）を通り過ぎるまで進む
                                                    cnt = 0;
                                                    while  ( cnt < 40 )                  { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ちょっと進む
                                                  }	
													
								   
				    else  if ( GOAL_FLG2   == 1 ) { while  ( GOAL_FLG1   == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ゴールーライン(クロスライン）を通り過ぎるまで進む
					                                cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ラインを完全に通り過ぎるまで進む
					                              }	              
					 	 
						     				   
                    if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 1 ))                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0; RUN_MODE = 0;        } // クロスラインと判定   ⇒ そのまま前進
               else if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 0 ))                       {                   MARKER_FLG2 = 0; RUN_MODE = 0; beep();  // マーカーラインと判定 ⇒ 
			   
                                                                                          //       if (MARKER_FLG3 == 0 ) { SPEED1 = STR_SPD_LOW ;   SPEED2 = TURN_SPD_HIGH; MARKER_FLG3 = 1; } // 曲率変化の開始
			                                                                              //  else if (MARKER_FLG3 == 1 ) { SPEED1 = STR_SPD_NORMAL; SPEED2 = TURN_SPD_LOW;  MARKER_FLG3 = 0; } // 曲率変化の終了
                                                                                                                                                                                            }

               else if (( MARKER_FLG2 == 0 ) && ( GOAL_FLG2 == 1 ) && ( START_FLG == 1 )) {  GOAL_FLG2   = 0;                  RUN_MODE = 0;} // ゴールーラインと判定 ⇒ 停止
			   else                                                                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0;              } // フラグクリア                                                       
               } 
	         
			 
			                
	   	     if  (( CENTER_L_FLG == 0 ) && ( CENTER_R_FLG == 1 ))     { RUN_MODE = 1; } // ③右旋回 L 0
	    else if  (( CENTER_R_FLG == 0 ) && ( CENTER_L_FLG == 1 ))     { RUN_MODE = 2; } // ④左旋回 R 0
		
        else if  ( SIDE_L_FLG   == 1 )                                { RUN_MODE = 4; } // ⑤左急旋回
	    else if  ( SIDE_R_FLG   == 1 )                                { RUN_MODE = 3; } // ⑥右急旋回
		else                                                          { RUN_MODE = 0; } // ⑦前進 
		
		
	      			
	switch (RUN_MODE)
		{
			case 0: // 前進(通常）
			        L_IN1 = 1; L_IN2 = 0;   // 左前進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (STR_SPD_NORMAL, STR_SPD_NORMAL);  // 直進 22.0 22.0
		 	        break;
		
		 	case 1:	// R_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左前進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (20.0 , 0.0);   // 右旋回 20.0 0.0
		 	        break;
		
		 	case 2: // L_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左後進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (0.0 , 20.0);   // 左旋回 0.0 20.0
		 	        break;
		
			case 3: // R_QTURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左前進
			        R_IN1 = 0; R_IN2 = 1;   // 右後退
			        set_pwm (35.0 ,25.0);   // 右急旋回 30.0 20.0
					break;
		
 		 	case 4: // L_QTURN:
			        L_IN1 = 0; L_IN2 = 1;   // 左後進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (25.0 , 35.0);  // 左急旋回 20.0 30.0
					break;
			
			
		    case 5: // STOP:ゴール
			        if (( goal_cnt > 50 ) && ( START_FLG  == 1 ))
					 { 
					   cnt = 0;                                         // カウントクリア
					   while ( cnt < 500) {set_pwm (18.0 , 18.0);}      // ゴールしたので減速して進む
			           set_pwm (0.0 , 0.0);                             // モーターPWMを0にする
					   PD7 = PD0 = PD3 = 0;                             // 7SEGドット消灯
					 		 
                       while(1) SEG_LED  = disp_data[15];               // 停止 "F"の表示
					 }
					 		 
				    break;
			
				
            case 6: // スタートライン通過処理
			        START_FLG  = 1;                                    // スタートフラグを立てる
					GOAL_FLG2  = 0;                                    // フラグクリア
					
			        break;
			

		}
		
  
    }
	
}

//**************************************************************************************
// 走行モジュール MODE1 ゴールで止まる
//**************************************************************************************
void mode_1(void)
{

  STBY        = 1;       // モータースタンバイ解除
  SEG_LED     = 0x00;    // ７SEG LEDの表示クリア
  SEN_DRV     = 1;       // センサー用LEDの発光 ON
  
  STR_SPD_NORMAL = 22.0;    // 直進通常スピード 22.0
//STR_SPD_LOW    = 19.0;    // 直進減速スピード
//TURN_SPD_HIGH  = 21.0;    // ターンスピード高 20.0
//TURN_SPD_LOW   = 19.0;    // ターンスピード低 20.0
  
  GOAL_FLG2   = 0;
  MARKER_FLG2 = 0; 
  
  //SPEED1 = STR_SPD_NORMAL;      // 最初は通常スピード
  //SPEED2 = TURN_SPD_LOW;
  
  cnt = 0;
  while ( cnt < 1000 ) { LED_cycle == 1; beep(); }   //  暫く待つ
  LED_cycle = 0;
  RUN_MODE = 0;
  
  while(1)
	
    {        
	
		     if  (( GOAL_FLG2   == 1 ) && ( START_FLG == 0 ))                   // ①スタートラインを切った
			      { 
				    L_IN1 = 1; L_IN2 = 0; // 左前進
			        R_IN1 = 1; R_IN2 = 0; // 右前進
			        PD7 = 1;              // 7SEGドット点灯
			        while ( GOAL_FLG1  == 1 ){ set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL); } // スタートラインを通り過ぎるまで進む
					//PD7 = 0;  
				    RUN_MODE = 6;
				  }              
			 
			 
			
        else if  (( MARKER_FLG2 == 1 ) || ( GOAL_FLG2  == 1 ) && ( START_FLG == 1 ))                                // ②ライン検出 (どちらかを検出）ラインを通過するまで暫く進む
		       {    
				    goal_cnt = 0;
		            L_IN1 = 1; L_IN2 = 0; // 左前進
			        R_IN1 = 1; R_IN2 = 0; // 右前進
					
			              if ( MARKER_FLG2 == 1 ) { while  ( MARKER_FLG1 == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // マーカーライン(クロスライン）を通り過ぎるまで進む
                                                    cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ちょっと進む
                                                  }	
													
								   
				    else  if ( GOAL_FLG2   == 1 ) { while  ( GOAL_FLG1   == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ゴールーライン(クロスライン）を通り過ぎるまで進む
					                                cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ラインを完全に通り過ぎるまで進む
					                              }	              
					 	 
						     				   
                        if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 1 ))                   {  GOAL_FLG2   = 0; MARKER_FLG2 = 0; RUN_MODE = 0;        } // クロスラインと判定   ⇒ そのまま前進
                   else if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 0 ))                   {                   MARKER_FLG2 = 0; RUN_MODE = 0; beep();  // マーカーラインと判定 ⇒ 
			   
                                                                                          //       if (MARKER_FLG3 == 0 ) { SPEED1 = STR_SPD_LOW ;   SPEED2 = TURN_SPD_HIGH; MARKER_FLG3 = 1; } // 曲率変化の開始
			                                                                              //  else if (MARKER_FLG3 == 1 ) { SPEED1 = STR_SPD_NORMAL; SPEED2 = TURN_SPD_LOW;  MARKER_FLG3 = 0; } // 曲率変化の終了
                                                                                                                                                                                            }

                   else if (( MARKER_FLG2 == 0 ) && ( GOAL_FLG2 == 1 ) && ( START_FLG == 1 )) {  GOAL_FLG2   = 0;                  RUN_MODE = 5;} // ゴールーラインと判定 ⇒ 停止
			       else                                                                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0;              } // フラグクリア                                                       
               } 
	                     
						 
						 
						    
	   	else if  (( CENTER_L_FLG == 0 ) && ( CENTER_R_FLG == 1 ))                         { RUN_MODE = 1; } // ③右旋回 L 0
		else if  (( CENTER_R_FLG == 0 ) && ( CENTER_L_FLG == 1 ))                         { RUN_MODE = 2; } // ④左旋回 R 0
		
        else if  (( SIDE_L_FLG   == 1 ) && ( SIDE_R_FLG   == 0 ))                         { RUN_MODE = 4; } // ⑤左急旋回
	    else if  (( SIDE_R_FLG   == 1 ) && ( SIDE_L_FLG   == 0 ))                         { RUN_MODE = 3; } // ⑥右急旋回
		else                                                                              { RUN_MODE = 0; } // ⑦前進 
		
		
	      			
	switch (RUN_MODE)
		{
			case 0: // 前進(通常）
			        L_IN1 = 1; L_IN2 = 0;   // 左前進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (STR_SPD_NORMAL, STR_SPD_NORMAL);  // 直進 22.0 22.0
		 	        break;
		
		 	case 1:	// R_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左前進 L_IN1 = 1; L_IN2 = 0;
			        R_IN1 = 1; R_IN2 = 0;   // 右前進 R_IN1 = 1; R_IN2 = 0;
			        set_pwm (20.0 , 0.0);   // 右旋回 20.0 0.0
		 	        break;
		
		 	case 2: // L_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左後進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (0.0 , 20.0);   // 左旋回 0.0 20.0
		 	        break;
		
			case 3: // R_QTURN:
			        L_IN1 = 1; L_IN2 = 0;   // 左前進
			        R_IN1 = 0; R_IN2 = 1;   // 右後退
			        set_pwm (35.0 , 25.0);
					break;
		
 		 	case 4: // L_QTURN:
			        L_IN1 = 0; L_IN2 = 1;   // 左後進
			        R_IN1 = 1; R_IN2 = 0;   // 右前進
			        set_pwm (25.0 , 35.0);  // 左急旋回 20.0 30.0
					break;
			
			
		    case 5: // STOP:ゴール
			        if (( goal_cnt > 50 ) && ( START_FLG  == 1 ))
					 { 
					   cnt = 0;                                         // カウントクリア
					   while ( cnt < 500) {set_pwm (18.0 , 18.0);}      // ゴールしたので減速して進む
			           set_pwm (0.0 , 0.0);                             // モーターPWMを0にする
					   PD7 = PD0 = PD3 = 0;                             // 7SEGドット消灯
					 		 
                       while(1) SEG_LED  = disp_data[15];               // 停止 "F"の表示
					 }
					 		 
				    break;
			
				
            case 6: // スタートライン通過処理
			        START_FLG  = 1;                                    // スタートフラグを立てる
					GOAL_FLG2  = 0;                                    // フラグクリア
					
			        break;
			

		}
		
  
    }
	
}

 

//------------------------------------------------------------------------------------------------------
//    各モードでの実行する
//------------------------------------------------------------------------------------------------------
void start_exe()

{
     wait(1000);                                   // チャタリング対策
     while(START_SW == SW_OFF);                    // 実行スイッチが放されるまで待つ
     //beep(4);                                    // スタートスイッチの合図
                
                                                   // 各モードでの実行
     if      (mode ==  0) mode_0(); // モード0   
     else if (mode ==  1) mode_1(); // モード1   
     else if (mode ==  2) ;// mode_2(); // モード2   
     else if (mode ==  3) ;// mode_3(); // モード3   
     else if (mode ==  4) ;// mode_4(); // モード4  
     else if (mode ==  5) ;// mode_5(); // モード5   
     else if (mode ==  6) ;// モード6  
     else if (mode ==  7) ;// mode_7(); // モード7  
     else if (mode ==  8) ;// mode_8(); // モード8  
     else if (mode ==  9) ;// mode_9(); // モード9  
     else if (mode == 10) ;// mode_10();// モード10 A
     else if (mode == 11) ;// mode_11();// モード11 B
     else if (mode == 12) ;// mode_12();// モード12 C
     else if (mode == 13) ;// mode_13();// モード13 D
     else if (mode == 14) ;// mode_14();// モード14 E
     else if (mode == 15) ;// mode_15();// モード15 F
}

	 
//--------------------------------------------------------------------------
//		メインプログラム 												
//--------------------------------------------------------------------------
void  main(void)
{
  
  init();      // 初期化
  init_CMT();  // CMTの初期化
  mtu_init();  // MTU0,MTU1の初期化
  
  beep();   
      
  mode = 0;
  
 
  // モード切替処理 ----------------------------------------------------------------------------
  while(1)
  { 
           if( MODE_SW == SW_ON)                    // モードスイッチは押されている  
              {                 
                      mode ++;                      // モードを1つあげる
                      
                      beep();                       // モード切替時ビープ音の発生
                      wait(500000);                 // チャタリング対策
                      while(!MODE_SW == SW_OFF);    // モードスイッチが放されるまで待つ
              }
                
               SEG_LED  = disp_data[mode];          // 7segへモード表示  
               if( mode == 16 )   mode = 0;         // モード16まで来たら0に戻す
               if( START_SW == SW_ON ) start_exe(); // 実行モードへ
   }                    
     
}    


//--------------------------------------------------------------------------
//		割り込みプログラム CMT0 (SENSOR, LED Cycle, Switch用）										
//--------------------------------------------------------------------------
#pragma interrupt (Excep_CMTU0_CMT0(vect=28))
void Excep_CMTU0_CMT0(void)
{
		
	      task ++;                                            // タスクの更新						
      if (task == 9) task = 0;                                // 9まで来たら０にクリア
      
   switch(task)                                               //タスクポインタに従って処理を行う			
   {
	
	
	case 0: // 1msecタイマーの確認用
            PORTE.DR.BIT.B2 ^= 1;                    // TEST PE2へ1mSのパルスを発生
            break;                                   // 正確にパルスが出ているかオシロで確認用
	
	// センサーの処理
	
	case 1: // GOAL 
	        //SEN_DRV = 1;                            // センサー用LEDの発光 ON
			AD0.ADCSR.BYTE =0x60;   			    // A/D変換開始
			while(AD0.ADCSR.BIT.ADST == 0);		    // 測定が終了するまで待つ
			//SEN_DRV = 0;                            // センサー用LEDの発光 OFF
			GOAL = AD0.ADDRA;				        // データを取得
			GOAL_LIM = 0x2F;	                    // GOALセンサー判定の閾値   2F
			    if(GOAL > GOAL_LIM) 
	             { GOAL_FLG1 = 1; GOAL_FLG2 = 1; PD0 = 1;} // GOALラインを検出したらフラグを立てる
	        else { GOAL_FLG1 = 0;                PD0 = 0;}
	        break;
			
			
	case 2: // SIDE_R
	        //SEN_DRV = 1;                            // センサー用LEDの発光 ON
	        AD0.ADCSR.BYTE =0x61;   			    // A/D変換開始
			while(AD0.ADCSR.BIT.ADST == 0);		    // 測定が終了するまで待つ
			//SEN_DRV = 0;                            // センサー用LEDの発光 OFF
			SIDE_R = AD0.ADDRB;				        // データを取得
			SIDE_R_LIM = 0x2D;	                    // SIDE_Rセンサー判定の閾値 2D
			    if(SIDE_R > SIDE_R_LIM)
			     { SIDE_R_FLG = 1; PD2 = 1;}        // SIDE_R ラインを検出したらフラグを立てる
			else { SIDE_R_FLG = 0; PD2 = 0;}
			break;
	
	case 3: // CENTER_R 
	        //SEN_DRV = 1;                             // センサー用LEDの発光 ON
            AD0.ADCSR.BYTE =0x62;   			     // A/D変換開始
			while(AD0.ADCSR.BIT.ADST == 0);		     // 測定が終了するまで待つ
			//SEN_DRV = 0;                             // センサー用LEDの発光 OFF
			CENTER_R = AD0.ADDRC;				     // データを取得
			CENTER_R_LIM = 0x2D;	                 // CENTER_Rセンサー判定の閾値 2D
			    if(CENTER_R > CENTER_R_LIM) 
			     { CENTER_R_FLG = 1; PD1 = 1;}       // SIDE_R ラインを検出したらフラグを立てる 
			else { CENTER_R_FLG = 0; PD1 = 0;}
            break;
			
	case 4: // CENTER_L
	        //SEN_DRV = 1;                            // センサー用LEDの発光 ON
            AD0.ADCSR.BYTE =0x63;   			      // A/D変換開始
			while(AD0.ADCSR.BIT.ADST == 0);		      // 測定が終了するまで待つ
			//SEN_DRV = 0;                              // センサー用LEDの発光 OFF
			CENTER_L = AD0.ADDRD;				      // データを取得
			CENTER_L_LIM = 0x2D;	                  // CENTER_Lセンサー判定の閾値 2D
		         if(CENTER_L > CENTER_L_LIM)
			      { CENTER_L_FLG = 1; PD5 = 1;}       // SIDE_R ラインを検出したらフラグを立てる
			else  { CENTER_L_FLG = 0; PD5 = 0;}       // 
			break;
				
	case 5: // SIDE_L
	        //SEN_DRV = 1;                            // センサー用LEDの発光 ON
            AD1.ADCSR.BYTE =0x60;   			      // A/D変換開始
	        while(AD1.ADCSR.BIT.ADST == 0);		      // 測定が終了するまで待つ
			//SEN_DRV = 0;                            // センサー用LEDの発光 OFF
	        SIDE_L = AD1.ADDRA;				          // データを取得
			SIDE_L_LIM = 0x2D;	                      // SIDE_Lセンサー判定の閾値 2D
			     if(SIDE_L > SIDE_L_LIM)
			      { SIDE_L_FLG = 1; PD4 = 1;}         // SIDE_R ラインを検出したらフラグを立てる 
			else  { SIDE_L_FLG = 0; PD4 = 0;} 
			break;
				
	case 6: // MARKER 
	        //SEN_DRV = 1;                            // センサー用LEDの発光 ON
            AD1.ADCSR.BYTE =0x61;   			      // A/D変換開始
			while(AD1.ADCSR.BIT.ADST == 0);		      // 測定が終了するまで待つ
			//SEN_DRV = 0;                            // センサー用LEDの発光 OFF
			MARKER = AD1.ADDRB;				          // データを取得
			MARKER_LIM = 0x2F;	                      // MERKERセンサー判定の閾値 2F
			    if(MARKER > MARKER_LIM)
			     { MARKER_FLG1 = 1; MARKER_FLG2 = 1; PD3 = 1;} // SIDE_R ラインを検出したらフラグを立てる 
	        else { MARKER_FLG1 = 0;                  PD3 = 0;} // しばらくしたらマーカーフラグクリア
	        break;
			
			
	case 7: // LEDサイクル点灯処理 --------------------------------------------------
  
          /*  if(LED_cycle == 1)
            {
                if (LED_speed >= 200)                           // 点灯回転の速度
                  {
                      LED_speed = 0;
                      SEG_LED = disp_cycle_data[dig];
                      dig ++;
                      if (dig == 6) dig = 0;                    // ６まで行ったら０に戻す
                  }

              else{  LED_speed ++;  } 
            } 
			*/
            break;
			
			
	case 8: // カウント
	        cnt ++ ;
			cnt1 ++ ;
			wait_timer++;
			goal_cnt ++;
			
			break;
	
    default: break;
	
	  }
}


//--------------------------------------------------------------------------
//		割り込みプログラム CMT1 (BEEP専用）										
//--------------------------------------------------------------------------
#pragma interrupt (Excep_CMTU0_CMT0(vect=29))
void Excep_CMTU0_CMT1(void)
{
	
	if ( beep_long < 100 )    // beep_long BEEP音の長さ
 	   {
	  		beep_long ++;     //
			BEEP ^= 1;        // 反転してパルスの発生
       }


  if(LED_cycle == 1)
        {
                if (LED_speed >= 200)                           // 点灯回転の速度
                  {
                      LED_speed = 0;
                      SEG_LED = disp_cycle_data[dig];
                      dig ++;
                      if (dig == 6) dig = 0;                    // ６まで行ったら０に戻す
                  }

              else{  LED_speed ++;  } 
         } 
 

}











//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


#ifdef __cplusplus
void abort(void)
{

}
#endif
