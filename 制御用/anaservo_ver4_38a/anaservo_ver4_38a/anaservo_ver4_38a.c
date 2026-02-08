/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     モータドライブ基板TypeD Ver.1.0・							*/
/*              を使用したマイコンカートレースプログラム                    */
/* バージョン   Ver.0.00                                                    */
/* Date         2017.02.10                                                  */
/* Copyright                            */
/****************************************************************************/

/*
本プログラムは、
●モータドライブ基板TypeTypeD Ver.1.0
●TSL1401
を使用したマイコンカーを動作させるプログラムです。
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "printf_lib.h"                 /* printf使用ライブラリ         */
#include "types3_beep.h"                /* ブザー追加                   */
#include "microsd_lib.h"                /* microSD制御ライブラリ        */
#include "data_flash_lib.h"

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* 定数設定 */
#define     TRC_MOTOR_CYCLE     20000   /* 左前,右前モータPWMの周期     */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* 左後,右後,ｻｰﾎﾞﾓｰﾀPWMの周期   */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     FREE                1       /* モータモード　フリー         */
#define     BRAKE               0       /* モータモード　ブレーキ       */

#define 	SERVO_MAX 			125	  	/* ハンドル最大位置 115           */

#define 	MAXTIME 			1600 //1100	  	/* 最大走行時間 (0.01秒)  1200 = 12s     1250     */


/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char cn6_get( void );
void led_out( unsigned char led );
void motor_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
unsigned char check_startgate( void );
unsigned char check_crossline( void );
unsigned char check_wideline( void );
unsigned char check_halfline( void );//1 = 左, 2 = 右
unsigned char check_halfline_forC( void );//1 = 左, 2 = 右

int getServoAngle( void );
void servoControl( void );
void servoControl2( void );
void cam_in(void);
void wait(int);
int camera(int, int);
void mode_out(void);
void get_angle_y(void);
void get_angle_x(void);
int angle_check(void);
int date_f_make(int, int , int,int);
int IR_L(void);
int IR_R(void);
void Get_Center_IR(void);
void IRcalibration( void );
int max(int,int);


/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
int             i_pattern;                /* マイコンカー動作パターン     */
unsigned long   ul_cnt_1ms = 0;            /* タイマ用                     */
unsigned long   ul_cnt_running_1ms = 0;    /* 最大走行時間用               */
unsigned long   ul_cnt_for_wait = 0;       /* wait 関数用                  */
unsigned long   ul_cnt_saka = 0;           /* 坂センサーチェック用         */
unsigned long   ul_cnt_straight_time_1ms = 0;/* 直線とカーブのカウント用   */
unsigned long   ul_cnt_curve_time_1ms = 0;   /* カーブ加速用のカウント用   */
char 			c_running_flag = 0;			/* 1 = 走行中					*/
char			c_mode = 0;				/* 0 = 通常 1 = 坂	2 = 右無視 3 = 左無視	*/
char			c_saka_cnt = 0;				/* 偶数 = 無視しない 奇数 = 無視する*/
int				i_out_cnt = 0;			//脱線カウント
char			c_out_flag = 0;			//脱線flag 1=コースアウト
char 			c_black_flag = 0;			//
char			c_Cu_flag = 0;			//0 = 直線, 1 = カーブ
char			c_c_short_mode = 0; //クランク　0:long 1:short
char			c_c_cut;	//0= 再生しない 1= 再生する 編集無意味
char			c_c_cut_short;	// 1= 距離が短いショートカット

/* microSD関連変数 */
signed char     c_msdBuff[ 512 ];         /* 一時保存バッファ             */
int             i_msdBuffAddress;         /* 一時記録バッファ書込アドレス */
int             i_msdFlag = 0;                /* 1:データ記録 0:記録しない    */
unsigned long   ul_msdStartAddress;        /* 記録開始アドレス             */
unsigned long   ul_msdEndAddress;          /* 記録終了アドレス             */
unsigned long   ul_msdWorkAddress;         /* 作業用アドレス               */
int             i_msdError = 0;               /* エラー番号記録               */

signed char     c_msdBuff_ch[ 512 ];         /* 一時保存バッファ             */
unsigned long   ul_msdStartAddress_ch = 0x0200;        /* 記録開始アドレス             */



char c_logfin = 0;

/* 現在の状態保存用 */
int             i_handleBuff;             /* 現在のハンドル角度記録       */
int             i_FleftMotorBuff = 0;          /* 現在の前左モータPWM値記録      */
int             i_FrightMotorBuff = 0;         /* 現在の前右モータPWM値記録      */
int             i_RleftMotorBuff = 0;          /* 現在の後左モータPWM値記録      */
int             i_RrightMotorBuff= 0;         /* 現在の後右モータPWM値記録      */


/* マイコン内フラッシュメモリ関連 */
int				i_date_f_mode = 0;		//0=なし 1=IN 2=OUT
signed char 	c_date_f_buff[32] ={0};
int			 	i_date_f_buff_int[16] ={0};
int				i_date_f_num = 0;
int				i_Cu_Angle	=		20;		//カーブ判定に使用 正数限定
int				i_Cu_Angle_saka	=	55;		//カーブ判定に使用 正数限定 坂用
signed char 	c_date_f_buff_ch[32] ={0};
int			 	i_date_f_buff_ch_int[32] ={0};//偶数＝パターン　奇数＝距離
int				i_date_f_num_ch = 0;

/*クランク、ハーフ距離計測用*/
int			 	i_date_buff_ch_int[32] ={0};//偶数＝パターン　奇数＝距離
int				i_date_num_ch = 0;
long            l_EncoderTotal_ch = 0;          /* 積算値保存用                 */

/* エンコーダ関連 */
int             i_Timer10 = 0;               /* 10msカウント用               */
long            l_EncoderTotal = 0;          /* 積算値保存用                 */
int             i_Encoder10 = 0;               /* 10ms毎の最新値               */
int             i_Encoder5  = 0;               /*  5ms毎の最新値               */
unsigned int    ui_EncoderBuff  = 0;           /* 計算用　割り込み内で使用     */
long			l_startPoint = 0;			/* 距離計測用スタート地点 */
long			l_startPoint_saka = 0;		/* 距離計測用スタート地点 坂？ */
long			l_startPoint_curve = 0;		/* 距離計測用スタート地点 カーブ終了位置*/
long            l_straight_EncoderTotal = 0;   /* 直線積算値保存用                 */

/*  サーボ関連 */
int             i_SensorBefore = 0;          /* 前回のセンサ値保存           */
int             i_SensorBeforeIR = 0;          /* 前回のセンサ値保存           */
int             i_AngleBefore2 = 0;
int             i_ServoPwm = 0;              /* サーボＰＷＭ値               */
int             i_ServoPwm2 = 0;
int             i_Angle0 = 0;                /* 中心時のA/D値保存            */
int 			i_SetAngle = 0;
int				i_AngleBefore = 0;

/* センサ関連 */
int             i_SensorPattern;         /* センサ状態保持用             */
int  			i_Center;					/*カメラセンター	0~127				*/
int			    i_Wide;					/*白線幅			0~127			*/
int  			i_Center_old;					/*カメラセンター				*/
int			    i_Wide_old;					/*白線幅						*/
int  			i_Center_offset = 0;		/*カメラセンターを移動＝寄せる				*/
int 			i_angle_y = 0;				/* ジャイロセンサーのY軸の値	*/
int 			i_angle_x = 0;				/* ジャイロセンサーのX軸の値	*/
int 			i_pre_angle_y[2] = {0};		/* ジャイロセンサーのY軸の過去の値	*/
int 			i_pre_angle_x[2] = {0};		/* ジャイロセンサーのX軸の過去の値	*/
int 			i_Center_IR = 0;
char 			c_IR_flag = 0;
int 			i_IR_max[2] = {0};
int				i_IR_min[2] = {1024,1024};
int 			i_IR_cnt = 0;
int 			i_IR_old = 0;


/* TRCレジスタのバッファ */
unsigned int    ui_trcgrb_buff;            /* TRCGRBのバッファ             */
unsigned int    ui_trcgrd_buff;            /* TRCGRDのバッファ             */

/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
unsigned char   uc_types_led;              /* LED値設定                    */
unsigned char   uc_types_dipsw;            /* ディップスイッチ値保存       */



/*	パラメータ	*/
/*************************************************************************************************

モータの出力を上げることがタイムの向上に直結するわけではない

極端な高出力設定は予選（フリー走行）のときだけにすること
安定しないのでトーナメントでコースアウトする

カーブで滑らないことが最重要

**************************************************************************************************/
//オフセット
int  		i_Center_offset_MAX = 10;		/*カーブ時カメラセンターを移動＝寄せる 最小値 0 	*/
int  		i_Center_offset_Angle = -3;	/*この値につき１ＩＮ側に寄せる	正：IN　負：OUT		*/

int			i_KASOKU = 15;


#define		MOTOR_OUT_BASE			90		//カーブ前半用　外側モーター用パラメーター 

#define		MOTOR_OUT_BASE_N		100		//カーブ後半用　外側モーター用パラメーター 

#define		MAX_TOPSPEED	99	//ブースト時でもこの速度以上は出ないように制限する JMCR指定モータ相当の最大速度と同等に設定する

int		    i_TOPSPEED	=		50;		//直線 

/////////////////////////////////////////////////////////////////////////////////////// 0:禁止 1と-1は同じ
//前半
int			i_SPEED_DOWN	=		5;//5		//角度によりi_TOPSPEEDを減速 カーブ前半 8 6
int			i_MOTOR_out_R	=	 	1;//1		//外側モーター用パラメーター 1	-2
int			i_MOTOR_in_F	=		4;//4		//内側モーター用パラメーター 	2 	1
int			i_MOTOR_in_R	=		-2;//-2		//内側モーター用パラメーター -2	-3
	
//後半
int			i_SPEED_DOWN_N=		6;//7		//角度によりi_TOPSPEEDを減速  カーブ後半 11 10
int			i_MOTOR_out_R_N=	4;//5		//外側モーター用パラメーター 後半	5	5
int			i_MOTOR_in_F_N=		7;//8		//内側モーター用パラメーター　後半	6	6
int			i_MOTOR_in_R_N=		4;//6		//内側モーター用パラメーター　後半	3	3


#define		date_f_brake		400	//再生走行時 通常走行と同様の速度制限をする距離 400
#define		date_f_brake2		65	//再生走行時　残り距離/date_f_brake2 だけ速度上限を上げる 数値を大きくした方が遅くなる(0にはしないこと）

#define		Cu_FREE_time  		15		//カーブ終了時の後輪フリーの時間(msec）

#define		Cu_BRAKE_time  		10		//カーブ進入時のブレーキ時間 (msec)
#define		Cu_BRAKE_SP 		30		//カーブ進入時にこの速度以上ならブレーキ
#define		Cu_BRAKE			-90		//カーブ進入時のブレーキ（後輪） 
#define		Cu_BRAKE_out		 0		//カーブ進入時のブレーキ(前輪OUT） 
#define		Cu_BRAKE_Fin		-35		//カーブ進入時のブレーキ(前輪IN） 


#define		Cu_N_time			200	//Cu_N_time ms カーブを走行すると後半になる 	

#define		Cu_BRAKE_N			50	//カーブ前半中のブレーキの前半時間

//////////////////////////////////////////////////////////////////////////////////////////////
//坂
int			i_S_flag = 2;				//坂道　遇数回を　1 = 無視しない  2 = 無視する
int			i_saka_max	  =		  1;	//認識可能な坂の数
#define 	KASA_Encoder1  	100	//坂開始	50
#define 	KASA_Encoder2  	400	//上り途中 終わり 300
#define 	KASA_Encoder3  	1500	//上り終わり 

#define		KASA_Encoder4  	3800	//坂上終わり  2500
#define		KASA_Encoder5  	4400	//下り終わり 通常にもどる 

#define		KASA_Encoder4_2  3000	//坂上終わり(最後の坂道)2500 3000
#define		KASA_Encoder5_2  3600	//下り終わり 通常にもどる(最後の坂道) 3000 3200 3500 4500


//斜面(上り)
#define		    TOPSPEED2			44		//直線(坂）30 33
#define			SPEED_DOWN2			20		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN2_N		20		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out2_R		20 		//外側モーター用パラメーター(坂）
#define			MOTOR_in2_F			20		//内側モーター用パラメーター(坂）
#define			MOTOR_in2_R			20		//内側モーター用パラメーター(坂）


//斜面(上り,頂上付近　飛び跳ね防止)
#define		    TOPSPEED3			31		//直線(坂）30 33
#define			SPEED_DOWN3			6		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN3_N		6		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out3_R		2	//外側モーター用パラメーター(坂）
#define			MOTOR_in3_F			4		//内側モーター用パラメーター(坂）
#define			MOTOR_in3_R			1		//内側モーター用パラメーター(坂）


//上
#define		    TOPSPEED4			50		//直線(坂上）30 33
#define			SPEED_DOWN4			6		//角度によりTOPSPEEDを減速(坂上）カーブ前半
#define			SPEED_DOWN4_N		6		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out4_R		1		//外側モーター用パラメーター(坂上)
#define			MOTOR_in4_F			4		//内側モーター用パラメーター(坂上）
#define			MOTOR_in4_R			-2		//内側モーター用パラメーター(坂上）


//斜面(下り)
#define		    TOPSPEED5			50		//直線(坂）30 33
#define			SPEED_DOWN5			20		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN5_N		20		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out5_R		20	//外側モーター用パラメーター(坂）
#define			MOTOR_in5_F			20		//内側モーター用パラメーター(坂）
#define			MOTOR_in5_R			20		//内側モーター用パラメーター(坂）


//////////////////////////////////////////////////////////////////////////////////////////////

//クランク、ハーフ直後の設定値	カーブのパラメータ変更がクランク、ハーフに影響しないようにするため
#define		    TOPSPEED_CH_Len		700		//クランク、ハーフ直後のこの距離未満は以下の設定値で走る　注意：カーブ前半のみ有効

#define			MOTOR_OUT_BASE_CH	85		//カーブ前半用　外側モーター用パラメーター クランク、ハーフの直後

#define		    TOPSPEED_CH			50		//直線
#define			SPEED_DOWN_CH		 6		//角度によりTOPSPEEDを減速  カーブ前半
#define			SPEED_DOWN_CH_N		 14		//角度によりTOPSPEEDを減速  カーブ後半（実質無効）

#define			MOTOR_out_CH_R		 1		//外側モーター用パラメーター
#define			MOTOR_in_CH_F		 4		//内側モーター用パラメーター
#define			MOTOR_in_CH_R		-2		//内側モーター用パラメーター

//コース記憶用
char c_ch_boost_on = 0; //このフラグが１のときのみ　_Boost を使用する　クランクのみ有効か中

#define			MOTOR_OUT_BASE_CH_Boost	 70//45		//カーブ前半用　外側モーター用パラメーター クランク、ハーフの直後

#define		    TOPSPEED_CH_Boost		50		//直線
#define			SPEED_DOWN_CH_Boost		20		//角度によりTOPSPEEDを減速  カーブ前半
#define			SPEED_DOWN_CH_N_Boost	20		//角度によりTOPSPEEDを減速  カーブ後半（実質無効）

#define			MOTOR_out_CH_R_Boost	 -2		//外側モーター用パラメーター
#define			MOTOR_in_CH_F_Boost		 2		//内側モーター用パラメーター
#define			MOTOR_in_CH_R_Boost		-3		//内側モーター用パラメーター

#define			MOTOR_out_CH_R_Boost_min	 -20//-20		//外側モーター用パラメーター 最小PWM	
#define			MOTOR_in_CH_F_Boost_min		 -10 //-10		//内側モーター用パラメーター　最小PWM
#define			MOTOR_in_CH_R_Boost_min		 -20//-20		//内側モーター用パラメーター　最小PWM

//クランク
int		    i_C_TOPSPEED	=		29;		//クランク(入)  25 33
int		    i_C_TOPSPEED2	=		50;		//クランク(出)	40

int 		i_C_TOPSPEED4 = 		48;		//再生走行時のブレーキ前
int		    i_C_TOPSPEED3	=		40;		//再生走行用クランク(入)  25 33 

int			i_C_short_len =		600;	//この距離未満はショート、以上はロング
#define		C_TOPSPEED_SHORT	2		//(i_Encoder10 > C_TOPSPEED + C_TOPSPEED_SHORT )のとき　＝減速できていない場合はショート
#define		C_TOPSPEED_SHORT_NG	1		//(i_Encoder10 < C_TOPSPEED - C_TOPSPEED_SHORT_NG )のとき　＝距離はショートでも速度が遅いときはロング

int			i_date_f_brake_c	=	600;	//再生走行時のブレーキ使用可能距離(mm) クランク用 600
int			i_date_f_shortcat_c=	280;	//再生走行時のショートカット距離(mm) クランク用 210

char		c_c_cut_master  	 =	  1;	//再生走行時であっても 0= 再生しない 1= 再生する 	
int			i_c_cut_encoder	 =	540;  	//この距離未満の場合は再生しない 540


//ハーフ 
//#define HWall 							//壁ありの時に有効化すること

#ifdef  HWall //壁あり
int		    i_H_TOPSPEED	=		45;		//ハーフ（侵入）
int		    i_H_TOPSPEED2	=		42;		//ハーフ(斜め)
#else //壁なし
int		    i_H_TOPSPEED	=		50;		//ハーフ（侵入）
int		    i_H_TOPSPEED2	=		47;		//ハーフ(斜め) ブレーキがかからないように値を高くする
#endif
  
int		    i_H_TOPSPEED2_S=		50;		//ハーフ(斜め)  ショートカット用
int			i_date_f_brake_h	=	700;	//再生走行時のブレーキ使用可能距離(mm)　ハーフ用 
int			i_date_f_shortcat_h=	400;		//再生走行時のショートカット距離(mm)　ハーフ用

int			i_date_f_plus_h	=	300;		//再生走行時の直後のストレート距離補正(mm)　ハーフ用  

#ifdef  HWall
char		c_h_cut 			 =	  0;	//壁ありの時はショートカットしない
#else
char		c_h_cut 			 =	  1;	//再生走行時であっても 0= 再生しない 1= 再生する
#endif
//////////////////////////////////////////////////////////////////////////////////////////////

#define			BRAKE_MAX			-100	//ブレーキの最大パワー 
#define			BRAKE_MAX_R			-90	//ブレーキの最大パワー リア用


int				i_kp = -18;//- 8  3 -16 -19 -23  -13
int				i_kd = -105;//-80 20 -130 -190	-110
int 			i_ki = 0;//-2

int				i_kp_ir = -20;
int				i_kd_ir = -110;
int 			i_ki_ir = 0;


int				i_kp2 = -10;//角度指定用
int				i_kd2 = -50;//角度指定用


/*	その他	*/
int			i_topspeed;		
int			i_speed_down;
int			i_speed_down_n;
int			i_motor2_out_R;
int			i_motor2_in_F;
int			i_motor2_in_R;
int			i_MOTOR_out_base = MOTOR_OUT_BASE;

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
	int             i, j,old_i;
    unsigned int    u;
    char            c;
	int x,f,r,or,jj = 0;
	int     ret;
	
    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    initBeepS();                        /* ブザー関連処理               */
	init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
	setMicroSDLedPort( &p6, &pd6, 0 );  /* microSD モニタLED設定        */
   
	asm(" fset I ");                    /* 全体の割り込み許可           */
	    
	// microSD 書き込み開始アドレス// 512の倍数に設定する
    ul_msdStartAddress = 5120;

    // microSD 書き込み終了アドレス
    // 書き込みしたい時間[ms] : x = 10[ms] : 64バイト
    // 60000msなら、x = 60000 * 64 / 10 = 384000
    // 結果は512の倍数になるように繰り上げする。
    ul_msdEndAddress  = 768000;//384000;
    ul_msdEndAddress += ul_msdStartAddress;   /* スタート分足す               */                  

	 /* microSD初期化 */
    ret = initMicroSD();
    if( ret != 0x00 ) {
        i_msdError = 1;
		printf("%d\n",ret);
        /* 初期化できなければ3秒間、LEDの点灯方法を変える */
        ul_cnt_1ms = 0;
        while( ul_cnt_1ms < 3000 ) {
            if( ul_cnt_1ms % 200 < 100 ) {
                led_out( 0x3 );
            } else {
                led_out( 0x0 );
            }
        }
    }
	
    /* マイコンカーの状態初期化 */
    motor_mode_f( FREE, FREE );
 	//motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( FREE, FREE );
	//motor_mode_r( BRAKE, BRAKE );
	
    motor_f( 0, 0 );
    motor_r( 0, 0 );
    servoPwmOut( 0 );
    setBeepPatternS( 0x8000 );
	wait(10);
	
	i_topspeed = i_TOPSPEED;		//初期値
	i_speed_down = i_SPEED_DOWN;
	i_speed_down_n = i_SPEED_DOWN_N;
	i_motor2_in_F = i_MOTOR_in_F;
	i_motor2_in_R = i_MOTOR_in_R;
	i_motor2_out_R = i_MOTOR_out_R;

	wait(10);
	while(~p8 == 0xff);//起動直後は数値がおかしい
	wait(10);
	
	i_Angle0 = 0;
	i_Angle0 = getServoAngle();  /* 0度の位置記憶                */
	uc_types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
	

	//加速度センサー値の初期化
	i_angle_y = ad0;
	i_angle_x = ad1;
	
	
	/* スタート時、マイコンディップスイッチ 1 = ON   再生モード　IN */
	if( (dipsw_get() & 0x01) == 0x01 ) {
		i_date_f_mode = 1;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
	
	/* スタート時、マイコンディップスイッチ 2 = ON   再生モード　OUT */
	}else if( (dipsw_get() & 0x02) == 0x02 ) {
		i_date_f_mode = 2;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);	
	}else{
		wait(500);	
	}
	
	
	/* スタート時、マイコンディップスイッチ 4 = ON　データ転送モード*/
    if( (dipsw_get() & 0x08) == 0x08 ) {
        i_pattern = 101;
        ul_cnt_1ms = 0;	
			
	// スタート時、ディップスイッチ 4 = ON   遇数回を　1 = 無視しない  2 = 無視する
/*	}else if( dipsw_get2() == 0x10 ) {
		i_S_flag = 1;
				
	// スタート時、ディップスイッチ 3 = ON   RXとの通信チェックモード
	}else if( dipsw_get2() == 0x20) {
		i_pattern = 500;
*/	
	}
 


    while( 1 ) {
		
//	I2CEepromProcess();                 /* I2C EEP-ROM保存処理          */

	mode_out();//坂フラグを出力
	
	//値の保存
	i_Center_old = i_Center;
	i_Wide_old = i_Wide;
	
	Get_Center_IR();//赤外線センサー
	cam_in();//値の取得

	
	if(i_Wide == 0 && c_mode != 1){//黒だったら && 坂でない
		if(c_black_flag == 1){//前回も黒
			
		}else{
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		c_black_flag = 1;
	}else{
		c_black_flag = 0;
	}
	
	if(i_pattern >= 10){
		led_out(0);
	}else if(i_pattern > 1){
		led_out(camera(i_Center,i_Wide));
	}
		
	//脱線チェック
	if(10 <= i_pattern && i_pattern < 100){
		if(((check_wideline()) || (i_Encoder10 < 3) || (c_mode != 1 && i_Wide == 0 && i_Center == 0  && i_pattern != 53 && i_pattern != 63 && i_pattern != 31 && i_pattern != 41 && i_pattern != 32 && i_pattern != 42)) && (i_pattern >= 10) && (i_pattern != 70)){
			i_out_cnt ++;		
		}else{
			if((check_wideline() ||(i_Wide == 0 && i_Center == 0) )&& ((i_pattern == 53 || i_pattern != 63) && ((l_EncoderTotal - l_startPoint ) >= 1000)) ){
				i_out_cnt ++;
			}else{
				i_out_cnt = 0;
			}
		}
		
		if((l_EncoderTotal > 500) && ((c_mode == 1 && i_out_cnt > 2000) || (c_mode != 1 && i_pattern != 22 && i_out_cnt > 1000) || (i_pattern == 22 && i_out_cnt > 2000 ) )){
			i_pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			c_out_flag = 1;
		}
	}
	
    switch( i_pattern ) {	
    case 0:
        
		/* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			i_Angle0 = 0;
			i_Angle0 = getServoAngle();  /* 0度の位置記憶                */
			
			IRcalibration( );
			
			/*
 			while(1){
				printf("%4d  %4d  %4d -- \t",IR_L(), IR_R(),IR_R() - IR_L());
				printf("%4d  %4d \n",ad6 ,ad7 );				
			}*/
			
            setBeepPatternS( 0xcc00 );
            ul_cnt_1ms = 0;
            i_pattern = 5;
            break;
        }
		
        led_out(camera(i_Center,i_Wide));
		
        break;
	case 5:
		/* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			i_Angle0 = 0;
			i_Angle0 = getServoAngle();  /* 0度の位置記憶                */
			
			
			ret = eraseMicroSD( ul_msdStartAddress, ul_msdEndAddress-1 );
            if( ret != 0x00 ) {
                /* イレーズできず */
                i_msdError = 2;
            }
            /* microSDProcess開始処理 */
            ret = microSDProcessStart( ul_msdStartAddress );
            if( ret != 0x00 ) {
                /* 開始処理できず */
                i_msdError = 3;
            }
			
			if(i_date_f_mode == 1){
				readDataFlash( 0x3000, c_date_f_buff, 32 );
				readDataFlash( 0x3400, c_date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					i_date_f_buff_int[j] = (int)(c_date_f_buff[i])*100 + (int)c_date_f_buff[i+1];
					j++;	
				}
				
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					i_date_f_buff_ch_int[j] =(int)c_date_f_buff_ch[i];
					j++;
					
					i_date_f_buff_ch_int[j] = (int)(c_date_f_buff_ch[i+1])*100 + (int)c_date_f_buff_ch[i+2];
					j++;
				}
				
				
				
			
				
			/*	
					//31：右クランク 41：左クランク 53:左ハーフ 63:右ハーフ
					//クランク　金具　近い：200　遠い：450
					//ハーフ　金具　350
					 
				i_date_f_buff_ch_int[0] = 63;
				i_date_f_buff_ch_int[2] = 31;
				i_date_f_buff_ch_int[4] = 41;
				i_date_f_buff_ch_int[6] = 31;
				i_date_f_buff_ch_int[8] = 53;
			
				i_date_f_buff_ch_int[1] = 1200;
				i_date_f_buff_ch_int[3] = 580;
				i_date_f_buff_ch_int[5] = 500;
				i_date_f_buff_ch_int[7] = 880;
				i_date_f_buff_ch_int[9] = 700;
				
				//直線
				i_date_f_buff_int[0] = 1800;
				i_date_f_buff_int[1] = 1000;
				i_date_f_buff_int[2] = 1500;
				i_date_f_buff_int[3] = 1300;
				i_date_f_buff_int[4] = 3200;
				i_date_f_buff_int[5] = 2800;
				i_date_f_buff_int[6] = 1600;
				i_date_f_buff_int[7] = 1600;
				i_date_f_buff_int[8] = 2000;
				i_date_f_buff_int[9] = 4500;
				i_date_f_buff_int[10] = 1000;
				
			
				
				*/
			}else if(i_date_f_mode == 2){
				readDataFlash( 0x3800, c_date_f_buff, 32 );
				readDataFlash( 0x3c00, c_date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					i_date_f_buff_int[j] = (int)(c_date_f_buff[i])*100 + (int)c_date_f_buff[i+1];
					j++;	
				}
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					i_date_f_buff_ch_int[j] =(int)c_date_f_buff_ch[i];
					j++;
					
					i_date_f_buff_ch_int[j] = (int)(c_date_f_buff_ch[i+1])*100 + (int)c_date_f_buff_ch[i+2];
					j++;
				}
		

				
				
				/*
					//31：右クランク 41：左クランク 53:左ハーフ 63:右ハーフ
					//クランク　金具　近い：200　遠い：450
					//ハーフ　金具　350
					 
				i_date_f_buff_ch_int[0] = 41;
				i_date_f_buff_ch_int[2] = 31;
				i_date_f_buff_ch_int[4] = 53;
				i_date_f_buff_ch_int[6] = 63;
			    i_date_f_buff_ch_int[8] = 31;
			
				i_date_f_buff_ch_int[1] = 500;
				i_date_f_buff_ch_int[3] = 880;
				i_date_f_buff_ch_int[5] = 700;
				i_date_f_buff_ch_int[7] = 1200;
				i_date_f_buff_ch_int[9] = 580;
				
				//直線
				i_date_f_buff_int[0] = 1600;
				i_date_f_buff_int[1] = 1600;
				i_date_f_buff_int[2] = 1600;
				i_date_f_buff_int[3] = 2000;
				i_date_f_buff_int[4] = 4500;
				i_date_f_buff_int[5] = 1000;
				i_date_f_buff_int[6] = 1500;
				i_date_f_buff_int[7] = 1300;
				i_date_f_buff_int[8] = 3200;
				i_date_f_buff_int[9] = 2800;
				i_date_f_buff_int[10] = 1600;
				*/
			}	
			
			
			i_date_f_num_ch = 0;
			
            setBeepPatternS( 0xcc00 );
            ul_cnt_1ms = 0;
            i_pattern = 1;
			
			wait(1500);////////////////////////////////////////////////////////////////////////////
			
            break;
        }
/*		
		if(angle_check() != 1){ //坂センサーチェック
			setBeepPatternS( 0x8000 );
			
		}
*/		
        
        if( check_startgate() ) {
             if(((ul_cnt_1ms/100) % 8)%2 == 0)led_out( 0x81);
			 else led_out(camera(i_Center,i_Wide));
			 
        }else{
			led_out(camera(i_Center,i_Wide));
		}
		break;
    case 1:
        /* スタートバー開待ち */
		servoPwmOut( 0 );
		
		motor_f( 0, 0 );
        motor_r( 0, 0 );
			
		old_i = i_Angle0;
		i_Angle0 = 0;
        i_Angle0 = getServoAngle();  /* 0度の位置記憶                */
		i_Angle0 = (i_Angle0 + old_i) >> 1;
		
        if(  (!check_startgate()) && (i_Wide != 0)) {//ゲートが消えた　&& ラインが見えた
		
            ul_cnt_1ms = 0;
			ul_cnt_running_1ms = 0;
			ul_cnt_straight_time_1ms = 0;
			c_running_flag = 1;//走行開始
			l_EncoderTotal = 0;  
			l_startPoint = 0;
			l_startPoint_saka = 0;
			
			c_saka_cnt = 0;//坂道の回数
			
			i_msdBuffAddress = 0;
            ul_msdWorkAddress = ul_msdStartAddress;
            i_msdFlag = 1;                /* データ記録開始               */
		
            i_pattern = 10;
            break;
			
        }
		
        led_out( 1 << (ul_cnt_1ms/50) % 8 );
        break;

	case 10://スタート直後
		
		if(i_Center < -10)i_Center = -10;
		if(i_Center > 10)i_Center = 10;
		
		old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;
		

		if(l_EncoderTotal < 200){// && i_Wide > 28){//走行開始直後は直線 && ゲート見えてる
			c_mode = 1;//視野を狭くする
			
			i_SetAngle = 0;
			servoPwmOut( i_ServoPwm2 );		
		}else{//通常
			c_mode = 0;	
			servoPwmOut( i_ServoPwm / 2 );
		}
		
		if( i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)) ) {// エンコーダによりスピード制御 
			x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;
				
	
			motor_f( x, x );
            motor_r( x, x );
	
		}else{
			motor_f(100 , 100 );
         	motor_r(100 , 100 );
		}
		
		
		if(l_EncoderTotal > 700){
			
			c_mode = 0;//元に戻す
			
			i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
		}
		break;
		
    case 11:
        /* 通常トレース */
		 
		old_i = i;//前回の角度を記憶
        i = getServoAngle();//ハンドル角度取得
		i = (i +old_i) >> 1;
		
		if(c_mode != 1 && i_Wide == 0){//坂中ではない　＆＆　黒だったら
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		
		if(((i_Center - i_Center_old) < -15 || 15 < (i_Center - i_Center_old)) && ( i_Wide < 30)){//急にラインが変化したら
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		
		if(c_mode == 1
		 && i_Wide > 28){//坂中 && 幅が太い
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;	
		}
		
	
		if(c_mode == 1){
			if(c_IR_flag == 0){//赤外線に切り替え
				if(((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (-5 < i_Center_IR  && 	i_Center_IR < 5) || (KASA_Encoder2 <= (l_EncoderTotal-l_startPoint_saka)) )){
					//if(KASA_Encoder1 <= (l_EncoderTotal-l_startPoint_saka) && (l_EncoderTotal-l_startPoint_saka) < KASA_Encoder2 + 300){
					if(KASA_Encoder1 <= (l_EncoderTotal-l_startPoint_saka)){
						i_IR_cnt++;
						if(i_IR_cnt > 10){
							c_IR_flag = 1;
							i_IR_cnt = 0;
						}	
					}
				}
			}else{//カメラに切り替え
				if((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (	-5 < i_Center_IR  && 	i_Center_IR < 5)){
				//	if(KASA_Encoder4 <= (l_EncoderTotal-l_startPoint_saka)){
					
					if(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4)) || 
						((i_saka_max == 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4_2))  ){
					
						i_IR_cnt++;
						if(i_IR_cnt > 10){
							c_IR_flag = 0;
							i_IR_cnt = 0;
						}	
					}
				}
			}
		}else{
			if(c_IR_flag == 1){//坂以外で赤外線モードならカメラに切り替え
				if((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (	-5 < i_Center_IR  && 	i_Center_IR < 5)){
					
					i_IR_cnt++;
					if(i_IR_cnt > 20){
						c_IR_flag = 0;
						i_IR_cnt = 0;
						c_mode = 0;
					}	
				}
			}
		}	
		
		
		if(ul_cnt_running_1ms >=  MAXTIME * 10){//走行時間終了
			i_pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			break;
		}
		
		
//if(c_saka_cnt == 0){//２回目は絶対に検出しない　一時的な対策。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。		
		if(-15 < i && i < 15){
			if(c_mode == 0){//通常
				//if(angle_check() == 2 && ( (c_saka_cnt%2 == 1) || ((l_EncoderTotal-l_startPoint_saka) >= 1000) && ((l_EncoderTotal-l_startPoint ) >= 1000) )){//坂センサーチェック l_startPoint =クランク終了位置
				//if(angle_check() == 2 && ( ((l_EncoderTotal-l_startPoint_saka) >= 600) && ((l_EncoderTotal-l_startPoint ) >= 1000) )){//坂センサーチェック l_startPoint =クランク終了位置
				if(angle_check() == 2 &&  (((l_EncoderTotal-l_startPoint_saka) >= 300) || ((c_saka_cnt%2 == 1) && ((l_EncoderTotal-l_startPoint_saka) >= 300))) && ((l_EncoderTotal-l_startPoint ) >= 500) ){//坂センサーチェック l_startPoint =クランク終了位置
					ul_cnt_saka++;
				}else{
					ul_cnt_saka = 0;
				}
			}else{
				ul_cnt_saka = 0;
			}
		}
//}		
		//if(-20 < i && i < 20){
		//if(-50 < i && i < 50){
		if(-25 < i && i < 25){
			if(c_mode == 0){//坂中でなければ
				if(l_EncoderTotal > 200 && (l_startPoint_saka == 0 || (l_EncoderTotal-l_startPoint_saka) >= 500) && (l_startPoint == 0 || (l_EncoderTotal-l_startPoint ) >= 150) && (l_startPoint_curve == 0 || (l_EncoderTotal-l_startPoint_curve) >= 0)){//ゲートに反応しないように && 坂終了から少しの間は無視 && クランク、ハーフ終了後少し無視 && カーブ直後は無視
				
					if(i_date_f_mode == 0){
						if( check_crossline() ) {       // クロスラインチェック         
            				ul_cnt_1ms = 0;
            				i_pattern = 21;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
				
        				}else if(check_halfline() == 1){//左レーンチェンジチェック
							ul_cnt_1ms = 0;
            				i_pattern = 51;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
				
						}else if(check_halfline() == 2){//右レーンチェンジチェック
							ul_cnt_1ms = 0;
        	    			i_pattern = 61;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
						}
					}else{
						//if( check_wideline() == 1) {       // 線幅が太くなったら      
						//if( (check_crossline() || check_halfline() != 0 ||  check_wideline() == 1) && i_Encoder10 < 60){ 		
						//if( (check_crossline() || check_halfline() != 0 ) && i_Encoder10 < 53){ 
						if( (check_crossline() || check_halfline() != 0 )){ 
							
							if(i_date_f_buff_ch_int[i_date_f_num_ch] == 31 || i_date_f_buff_ch_int[i_date_f_num_ch] == 41) {       // クロスラインチェック         
            					ul_cnt_1ms = 0;
            					i_pattern = 21;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								
								
								if(c_c_cut_master == 0){
									c_c_cut = 0;
								}else{
									if(i_date_f_buff_ch_int[i_date_f_num_ch] < i_c_cut_encoder){
										//c_c_cut = 0;
										c_c_cut = 1;
										c_c_cut_short = 1;
									}else c_c_cut = 1;
								}
						
								break;
				
        					}else if(i_date_f_buff_ch_int[i_date_f_num_ch] == 53){//左レーンチェンジチェック
								ul_cnt_1ms = 0;
            					i_pattern = 51;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								break;
				
							}else if(i_date_f_buff_ch_int[i_date_f_num_ch] == 63){//右レーンチェンジチェック
								ul_cnt_1ms = 0;
        		    			i_pattern = 61;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								break;
							}
						}
					}
				}
			}
		}
		
		if(c_mode == 0){//通常
		
			if(ul_cnt_saka >= 5){//坂
			
				if(i_saka_max <= 0){
					ul_cnt_saka = 0;
					
				}else{
					if(c_saka_cnt % i_S_flag == 0 && i_saka_max > 0){

						i_saka_max--;
						c_mode = 1;//坂モードに
						l_startPoint_saka = l_EncoderTotal;

					}
					c_saka_cnt++;
					ul_cnt_saka = 0;
				
					l_startPoint_saka = l_EncoderTotal;//チャタリング防止
				
				}
				
			}
		
		}else if(c_mode == 1){//坂

			if( (-10 < i && i < 10) &&
				(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder5)) || 
				((i_saka_max <= 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder5_2)) ) ){//通常に戻す
			
				c_mode = 0;
				
				l_startPoint_saka = l_EncoderTotal;//チャタリング防止
				
				i_TOPSPEED = i_topspeed;
				i_SPEED_DOWN = i_speed_down;
				i_SPEED_DOWN_N = i_speed_down_n;
				i_MOTOR_out_R = i_motor2_out_R;
				i_MOTOR_in_F = i_motor2_in_F;
				i_MOTOR_in_R = i_motor2_in_R;
				
				
			}else if(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4)) || 
				((i_saka_max <= 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4_2))  ){// 下り開始
				
				
				i_TOPSPEED = TOPSPEED5;
				i_SPEED_DOWN = SPEED_DOWN5;
				i_SPEED_DOWN_N = SPEED_DOWN5_N;
				i_MOTOR_out_R = MOTOR_out5_R;
				i_MOTOR_in_F = MOTOR_in5_F;
				i_MOTOR_in_R = MOTOR_in5_R;
				
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder3){// 坂上
				i_TOPSPEED = TOPSPEED4;
				i_SPEED_DOWN = SPEED_DOWN4;
				i_SPEED_DOWN_N = SPEED_DOWN4_N;
				i_MOTOR_out_R = MOTOR_out4_R;
				i_MOTOR_in_F = MOTOR_in4_F;
				i_MOTOR_in_R = MOTOR_in4_R;
			
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2){//速度をさらに遅く 坂上ジャンプ防止
				i_TOPSPEED = TOPSPEED3;
				i_SPEED_DOWN = SPEED_DOWN3;
				i_SPEED_DOWN_N = SPEED_DOWN3_N;
				i_MOTOR_out_R = MOTOR_out3_R;
				i_MOTOR_in_F = MOTOR_in3_F;
				i_MOTOR_in_R = MOTOR_in3_R;
			
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1){//速度を遅く 上り開始
				i_TOPSPEED = TOPSPEED2;
				i_SPEED_DOWN = SPEED_DOWN2;
				i_SPEED_DOWN_N = SPEED_DOWN2_N;
				i_MOTOR_out_R = MOTOR_out2_R;
				i_MOTOR_in_F = MOTOR_in2_F;
				i_MOTOR_in_R = MOTOR_in2_R;
			
				
				//if( (i < -20 || 20 < i) && (l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1+500){//上っている途中でカーブはありえない
		/*		if( i < -20 || 20 < i){//上っている途中でカーブはありえない
					i_TOPSPEED = i_topspeed;
					SPEED_DOWN = i_speed_down;
					SPEED_DOWN_N = i_speed_down_n;
					i_MOTOR_out_R = i_motor2_out_R;
					i_MOTOR_in_F = i_motor2_in_F;
					i_MOTOR_in_R = i_motor2_in_R;

					c_mode = 0;
					
					c_saka_cnt--;//今回の分を無かったことに
					i_saka_max++;
					l_startPoint_saka = l_EncoderTotal;//チャタリング防止
				}*/
			}	
		}
		
		 
		
		if(c_mode == 1){
			if(((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1) && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder2)servoPwmOut( i_ServoPwm /2);
			else servoPwmOut( i_ServoPwm );
		}else{
			servoPwmOut( i_ServoPwm );

		}
		
		if(c_mode != 1){//坂中の設定が上書きされないようにする
			if(l_EncoderTotal > 1000 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後は設定値を変える
			
				if(c_ch_boost_on == 1){
					i_MOTOR_out_base = MOTOR_OUT_BASE_CH_Boost;
			
					i_TOPSPEED = TOPSPEED_CH_Boost;
					i_SPEED_DOWN = SPEED_DOWN_CH_Boost;
					i_SPEED_DOWN_N = SPEED_DOWN_CH_N_Boost;
					i_MOTOR_out_R = MOTOR_out_CH_R_Boost;
					i_MOTOR_in_F = MOTOR_in_CH_F_Boost;
					i_MOTOR_in_R = MOTOR_in_CH_R_Boost;
				
				}else{
					i_MOTOR_out_base = MOTOR_OUT_BASE_CH;
			
					i_TOPSPEED = TOPSPEED_CH;
					i_SPEED_DOWN = SPEED_DOWN_CH;
					i_SPEED_DOWN_N = SPEED_DOWN_CH_N;
					i_MOTOR_out_R = MOTOR_out_CH_R;
					i_MOTOR_in_F = MOTOR_in_CH_F;
					i_MOTOR_in_R = MOTOR_in_CH_R;
				}

				
			}else{//クランク、ハーフ直後でなければ通常設定にする
				i_MOTOR_out_base = MOTOR_OUT_BASE;
			
				i_TOPSPEED = i_topspeed;
				i_SPEED_DOWN = i_speed_down;
				i_SPEED_DOWN_N = i_speed_down_n;
				i_MOTOR_out_R = i_motor2_out_R;
				i_MOTOR_in_F = i_motor2_in_F;
				i_MOTOR_in_R = i_motor2_in_R;
				
				c_ch_boost_on = 0;

			}
		}
		
        if(  i > 9 ){//ハンドル右 //3行したの数値と合わせること
		
			if(c_mode != 1){//坂中でなければ
				i_Center_offset = (i-9) / i_Center_offset_Angle ;//カーブで寄せる
				if(i_Center_offset > i_Center_offset_MAX )i_Center_offset = i_Center_offset_MAX;
				if(i_Center_offset < -i_Center_offset_MAX )i_Center_offset = -i_Center_offset_MAX;
			}else{
				i_Center_offset = 0;
			}
			
			
			if((i - old_i > 0) && (c_Cu_flag == 0)){//直線からカーブへ 
			
				if(ul_cnt_straight_time_1ms >= 30 && (l_EncoderTotal-l_startPoint ) >= 100  && (i_Encoder10 > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					if(l_EncoderTotal > 500 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					}else{
						ul_cnt_straight_time_1ms = 0;
					}
				}
				c_Cu_flag = 1;
			
			}
			
			l_startPoint_curve = l_EncoderTotal;//カーブ終了位置用
				
			if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (l_EncoderTotal-l_startPoint ) >= 100 && (l_EncoderTotal-l_startPoint_saka) >= 100){//カーブ進入時のブレーキ
				
				servoPwmOut( i_ServoPwm * 2 );
				
				motor_f( Cu_BRAKE_out , Cu_BRAKE_Fin );
            	motor_r( Cu_BRAKE , Cu_BRAKE );
				
			}else if(ul_cnt_curve_time_1ms <= Cu_N_time && i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)) && (l_EncoderTotal-l_startPoint ) >= 200) {// エンコーダによりスピード制御  カーブ前半
			
				if(c_mode == 1 && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder3 - 300){//坂登り中のブレーキは全輪均等にする
					x = (i_TOPSPEED -i_Encoder10)*10;
				
					r = x;
					f = x;
				}else if(ul_cnt_curve_time_1ms <= Cu_BRAKE_N && c_mode != 1){//ブレーキ前半 && 坂モードでなければ
				
					x = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*5;	
					r = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*40;
					f = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*20;
				
					if(x < -10) x = -10;
					if(r < -80) r = -80;
					if(f < -35) f = -35;
				}else{
					x = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;	
					r = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*5;
					f = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;
				
					if(x < -10) x = -10;
					if(r < -20) r = -20;
					if(f < -20) f = -20;	
				}
				motor_f( x, f );
            	motor_r( r, r );			
			
			}else if(i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN_N)) && (l_EncoderTotal-l_startPoint ) >= 200 ) {// エンコーダによりスピード制御  カーブ後半
			
				x=((i_TOPSPEED -(i / i_SPEED_DOWN_N))-i_Encoder10)*2;	
				r = x;
				f = x;

				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( x, r );
            	motor_r( f, r );	
					
        	}else{
				
				if((ul_cnt_curve_time_1ms <= Cu_N_time) || (c_mode == 1) || (i > 95) ){//カーブ前半 || 坂モード || 曲げすぎ || 
					if(i_MOTOR_in_F > 0)f= (i_MOTOR_out_base - (i / i_MOTOR_in_F));
					else f= (i_MOTOR_out_base - (i * -i_MOTOR_in_F));
				
					if(i_MOTOR_in_R > 0)r= (i_MOTOR_out_base - (i / i_MOTOR_in_R));
					else r= (i_MOTOR_out_base - (i * -i_MOTOR_in_R));


					if(i_MOTOR_out_R > 0)or = (i_MOTOR_out_base - (i / i_MOTOR_out_R));
					else or = (i_MOTOR_out_base - (i * -i_MOTOR_out_R));
					
				

					if(c_ch_boost_on == 1){//ショートカットクランク直後
						if(f < MOTOR_in_CH_F_Boost_min) f = MOTOR_in_CH_F_Boost_min;
						if(r < MOTOR_in_CH_R_Boost_min) r = MOTOR_in_CH_R_Boost_min;
						if(or < MOTOR_out_CH_R_Boost_min) or = MOTOR_out_CH_R_Boost_min;
					}else{
						if(f < 0) f = 0;
						if(r < -10) r = -10;
						if(or < 0) or = 0;
					}
					 
					if(f > i_MOTOR_out_base) f = i_MOTOR_out_base;
					if(r > i_MOTOR_out_base) r = i_MOTOR_out_base;
					if(or > i_MOTOR_out_base) or = i_MOTOR_out_base;
					
					
				
				
					motor2_f( i_MOTOR_out_base , f);
           			motor2_r( or, r);
					
				}else{//カーブ後半

					if(i_MOTOR_in_F_N > 0)f= (MOTOR_OUT_BASE_N - (i / i_MOTOR_in_F_N));
					else f= (MOTOR_OUT_BASE_N - (i * -i_MOTOR_in_F_N));
				
					if(i_MOTOR_in_R_N > 0)r= (MOTOR_OUT_BASE_N - (i / i_MOTOR_in_R_N));
					else r= (MOTOR_OUT_BASE_N - (i * -i_MOTOR_in_R_N));
				
					if(i_MOTOR_out_R_N > 0)or = (MOTOR_OUT_BASE_N - (i / i_MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (i * -i_MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > MOTOR_OUT_BASE_N) f = MOTOR_OUT_BASE_N;
					if(r > MOTOR_OUT_BASE_N) r = MOTOR_OUT_BASE_N;
					if(or > MOTOR_OUT_BASE_N) or = MOTOR_OUT_BASE_N;
				
					motor2_f( MOTOR_OUT_BASE_N, f);
           			motor2_r( or, r);
					
				}
							
			}
			 		 	 
		}else if( i < -9 ){//ハンドル左 //3行したの数値と合わせること
			
		
			if(c_mode != 1){
				i_Center_offset = (i+9) / i_Center_offset_Angle ;//カーブで寄せる
				if(i_Center_offset > i_Center_offset_MAX )i_Center_offset = i_Center_offset_MAX;
				if(i_Center_offset < -i_Center_offset_MAX )i_Center_offset = -i_Center_offset_MAX;
			}else{
				i_Center_offset = 0;
			}
			
			
			if( (i - old_i < 0) && (c_Cu_flag == 0)){//直線からカーブへ 
				
				if(ul_cnt_straight_time_1ms >= 30 && (l_EncoderTotal-l_startPoint ) >= 100  && (i_Encoder10 > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					if(l_EncoderTotal > 500 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はブレーキしない
						ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//クランク直後のカーブはまだクランクの抜けなのでブレーキ不要
					}else{
						ul_cnt_straight_time_1ms = 0;
					}
				}
				
				c_Cu_flag = 1;
		
			}
			
			l_startPoint_curve = l_EncoderTotal;//カーブ終了位置用

			if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (l_EncoderTotal-l_startPoint ) >= 100 && (l_EncoderTotal-l_startPoint_saka) >= 100){//カーブ進入時のブレーキ
				
				servoPwmOut( i_ServoPwm * 2);
				
				motor_f( Cu_BRAKE_Fin, Cu_BRAKE_out );
            	motor_r( Cu_BRAKE, Cu_BRAKE );
				
			}else if(ul_cnt_curve_time_1ms <= Cu_N_time &&  i_Encoder10 >= (i_TOPSPEED -(-i / i_SPEED_DOWN)) && (l_EncoderTotal-l_startPoint ) >= 200 ) {  // エンコーダによりスピード制御 カーブ前半
				
				if(c_mode == 1 && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder3 - 300){//坂登り中のブレーキは全輪均等にする
					x = (i_TOPSPEED -i_Encoder10)*10;
	
					r = x;
					f = x;
				}else if(ul_cnt_curve_time_1ms <= Cu_BRAKE_N && c_mode != 1){//ブレーキ前半 && 坂モードでなければ
					x = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*5;
					r = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*40;
					f = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*20;
							
		
					if(x < -10) x = -10;
					if(r < -80) r = -80;
					if(f < -35) f = -35;
				}else{
					x = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*2;
					r = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*5;
					f = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*2;
							

					if(x < -10) x = -10;
					if(r < -20) r = -20;
					if(f < -20) f = -20;
				}	
				motor_f( f, x );
            	motor_r( r, r );
				
			}else if(i_Encoder10 >= (i_TOPSPEED -(-i / i_SPEED_DOWN_N)) && (l_EncoderTotal-l_startPoint ) >= 200) {  // エンコーダによりスピード制御 カーブ後半
			
				x=((i_TOPSPEED -(-i / i_SPEED_DOWN_N))-i_Encoder10)*2;
				r = x;
				f = x;	
	
				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( r, x );
            	motor_r( r, f );
				
        	}else{
				
				if((ul_cnt_curve_time_1ms <= Cu_N_time) || (c_mode == 1) || (i < -95) ){//カーブ前半 || 坂モード || 大曲
					if(i_MOTOR_in_F >0)f = (i_MOTOR_out_base - (-i / i_MOTOR_in_F));
					else f = (i_MOTOR_out_base - (-i * -i_MOTOR_in_F)); 
				
					if(i_MOTOR_in_R >0)r = (i_MOTOR_out_base - (-i / i_MOTOR_in_R));
					else r = (i_MOTOR_out_base - (-i * -i_MOTOR_in_R));
				
					if(i_MOTOR_out_R >0)or = (i_MOTOR_out_base - (-i / i_MOTOR_out_R));
					else or = (i_MOTOR_out_base - (-i * -i_MOTOR_out_R));
					
					
					if(c_ch_boost_on == 1){//ショートカットクランク直後
						if(f < MOTOR_in_CH_F_Boost_min) f = MOTOR_in_CH_F_Boost_min;
						if(r < MOTOR_in_CH_R_Boost_min) r = MOTOR_in_CH_R_Boost_min;
						if(or < MOTOR_out_CH_R_Boost_min) or = MOTOR_out_CH_R_Boost_min;
					}else{
						if(f < 0) f = 0;
						if(r < -10) r = -10;
						if(or < 0) or = 0;	
					}
					
					if(f > i_MOTOR_out_base) f = i_MOTOR_out_base;
					if(r > i_MOTOR_out_base) r = i_MOTOR_out_base;
					if(or > i_MOTOR_out_base) or = i_MOTOR_out_base;
					
					motor2_f(f, i_MOTOR_out_base);
           			motor2_r(r, or );
					
				}else{//カーブ後半
				
					if(i_MOTOR_in_F_N >0)f = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_in_F_N));
					else f = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_in_F_N)); 
				
					if(i_MOTOR_in_R_N >0)r = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_in_R_N));
					else r = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_in_R_N));
				
					if(i_MOTOR_out_R_N >0)or = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > MOTOR_OUT_BASE_N) f = MOTOR_OUT_BASE_N;
					if(r > MOTOR_OUT_BASE_N) r = MOTOR_OUT_BASE_N;
					if(or > MOTOR_OUT_BASE_N) or = MOTOR_OUT_BASE_N;
				
					motor2_f(f, MOTOR_OUT_BASE_N );
           			motor2_r(r, or );
					
				}	
				
				
			}
			 	 
		}else{//直線
			i_Center_offset = 0;
			
			ul_cnt_curve_time_1ms = 0;
						
			if((c_Cu_flag == 1)&&(c_mode == 0)){//カーブから直線へ && 坂中ではない
			
				if(ul_cnt_straight_time_1ms >= 60){//カーブを一定時間走っていたら＝あまりカーブを走っていない時（後輪滑り後）はフリーにしたくない
					ul_cnt_straight_time_1ms = 0;
				}
				c_Cu_flag = 0;
			}
			
			if(ul_cnt_straight_time_1ms <= Cu_FREE_time && (l_EncoderTotal-l_startPoint ) >= 700){//カーブ終わりから一定時間内　＆＆　クランク、ハーフからの復帰直後ではない
				
				if(c_mode == 0 &&  i_Center < -10) {//車体左寄り
				
					motor_f(98 , 100 );
				
				}else if(c_mode == 0 &&  i_Center > 10) {//車体右寄り
				
					motor_f(100 , 98 );
					
				}else{
			
					motor_f(100 , 100 );
				}
				
            	motor_r( 0, 0 );
			
			}else if( (i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN))) && (i_date_f_mode == 0 || c_mode == 1) ) {// エンコーダによりスピード制御 
				
				if(c_mode == 1){//坂
					x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*10;
					r=x;
			
					motor_f( x, x );
            		motor_r( r, r );
				}else{
					//通常ブレーキ
					motor_f( 0, 0 );
            		motor_r( 0, 0 );
				}
		//再生走行時　残り距離がdate_f_brake以下になったら通常走行の制限速度までブレーキ
			}else if(c_mode == 0 && i_date_f_mode != 0 && ((i_date_f_buff_int[i_date_f_num] - date_f_brake)< l_straight_EncoderTotal) && (i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)))  ) {// エンコーダによりスピード制御 
					 
				//再生走行時　最終ブレーキ
				x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*20;
				r=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*10;
			
				motor_f( x, x );
            	motor_r( r, r );
					
			//再生走行時　残り距離に応じて速度制限を変更する
			}else if(c_mode == 0 && i_date_f_mode != 0 && (i_Encoder10 >= max(i_TOPSPEED,i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) ) {// エンコーダによりスピード制御 
				
				//再生走行時　ブースト中
				x=(max(i_TOPSPEED,(i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) -i_Encoder10)*10;
				r=(max(i_TOPSPEED,(i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) -i_Encoder10)*5;
				
				//if(x < -40) x = -40;
				//r = x;
				 
				motor_f( x, x );
            	motor_r( r, r );
			
			}else if(i_Encoder10 >= MAX_TOPSPEED){//ブースト時でもMAX_TOPSPEED以上は出ないように制限する
				
				motor_f( 0, 0 );
            	motor_r( 0, 0 );
					
		
			}else if(c_mode == 0 && i_Center < -10) {//車体左寄り
				
				motor_f(95 , 100 );
				motor_r(100 , 100 );
			}else if(c_mode == 0 &&  i_Center > 10) {//車体右寄り
				
				motor_f(100 , 95 );
				motor_r(100 , 100 );
				
			}else{
				
				//if((c_mode == 1) &&  (i_Wide == 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2) && ((l_EncoderTotal-l_startPoint_saka) < KASA_Encoder3)){//坂頂上付近のとき
				if((c_mode == 1) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2) && ((l_EncoderTotal-l_startPoint_saka) < KASA_Encoder3)){//坂頂上付近のとき
					motor_f(50 , 50 );
           			motor_r(0 , 0 );		//飛び跳ね軽減のため後輪はフリー
				}else{
					motor_f(100 , 100 );
           			motor_r(100 , 100 );
				}	
			}
		}       
        break;
		
    case 21:
	
        /* クロスライン通過処理 */
      	//setBeepPatternS( 0x8000 );

		if(i_Wide > 20 || i_Center < -20 || i_Center > 20 ){//クロスラインが見えているとき
			i_SetAngle = 0;
			servoPwmOut( i_ServoPwm2 );
		}else{
			servoPwmOut( i_ServoPwm  );
		}
		
		//再生走行時　　ブレーキ
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4)){
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			x=(i_C_TOPSPEED4-i_Encoder10)*5;
	
			motor_f( x, x );
            motor_r( x, x );
			
		//クランク　ブレーキ
        }else if( (i_date_f_mode == 0 && i_Encoder10 >= i_C_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED)   ) {          // エンコーダによりスピード制御 
          
			x = (i_C_TOPSPEED-i_Encoder10)*2;
			r = (i_C_TOPSPEED-i_Encoder10)*1;
			
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
			
			motor_f( x, x );
            motor_r( r, r );
		 
        }else{
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }	

        if( (l_EncoderTotal-l_startPoint ) >= 70 ) {	
           	ul_cnt_1ms = 0;
            i_pattern = 22;
			l_startPoint = l_EncoderTotal;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
        }
        break;

    case 22:	
        /* クロスライン後のトレース、直角検出処理 */
		

		servoPwmOut( i_ServoPwm * 15 / 10   );
		
		if( (l_EncoderTotal-l_startPoint ) >= 250 ) {
			//if((( c_c_cut == 0 || i_date_f_mode == 0) && check_halfline() == 2) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31  && (check_wideline() == 1 || ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch))))){//右クランク
            
			if((i_date_f_mode == 0 && check_halfline_forC() == 2) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31  && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch)))
						|| ((c_c_cut == 0 && i_date_f_mode != 0) && (i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31) && (check_wideline() == 1))){//右クランク
            	ul_cnt_1ms = 0;
				l_startPoint = l_EncoderTotal;
				
				if(i_date_f_mode == 0){//距離計測
					i_date_buff_ch_int[i_date_num_ch++] = 31;
					i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
				}
				
				//if((i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch )|| (i_date_f_mode == 0 && (i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT))){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき
				if( (i_date_f_mode != 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch - max(0,i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c) )  ||  (i_date_f_mode == 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch ) 
					 || ( i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT) ){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき
					
					//if((i_date_f_mode == 0 )&& (i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//距離はショートでも進入速度が遅いのでロングとする
					if(i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG ){//距離はショートでも進入速度が遅いのでロングとする
						c_c_short_mode = 0;
					}else{
						c_c_short_mode = 1;	
					}
				}else{
					c_c_short_mode = 0;
				}
				
          		i_pattern = 31;//右クランク
				i_date_f_num_ch++;
			
				i_Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(i_date_f_mode == 0 || c_c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_c_cut == 1 && i_date_f_mode != 0){//ショートカット時に
					if(i_Encoder10 >= i_C_TOPSPEED3){//指定速度を超えていた場合は
						l_startPoint -= (i_Encoder10 -  i_C_TOPSPEED3) * 5;//距離を補正し、より速く曲がるようにする
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;
				
        	}
        	
			//if(((c_c_cut == 0 || i_date_f_mode == 0) && check_halfline() == 1) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41  && (check_wideline() == 1 || ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch))))){//左クランク
            
			if((i_date_f_mode == 0 && check_halfline_forC() == 1) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41  && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch)))
						|| ((c_c_cut == 0 && i_date_f_mode != 0) && (i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41) && (check_wideline() == 1))){//左クランク
            	ul_cnt_1ms = 0;
				l_startPoint = l_EncoderTotal;
				
				if(i_date_f_mode == 0){//距離計測
					i_date_buff_ch_int[i_date_num_ch++] = 41;
					i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
				}
				
				//if((i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch) || (i_date_f_mode == 0 && (i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT))){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき){//距離が短いと減速ができていない可能性がある
				if( (i_date_f_mode != 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch - max(0,i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c) )  ||  (i_date_f_mode == 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch ) 
					 || ( i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT) ){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき
				
					//if((i_date_f_mode == 0) && (i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//距離はショートでも進入速度が遅いのでロングとする
					if(i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG ){//距離はショートでも進入速度が遅いのでロングとする
						c_c_short_mode = 0;
					}else{
						c_c_short_mode = 1;	
					}
				}else{
					c_c_short_mode = 0;
				}
				
          		i_pattern = 41;//左クランク
				i_date_f_num_ch++;
			
				i_Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(i_date_f_mode == 0 || c_c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_c_cut == 1 && i_date_f_mode != 0){//ショートカット時に
					if(i_Encoder10 >= i_C_TOPSPEED3){//指定速度を超えていた場合は
						l_startPoint -= (i_Encoder10 -  i_C_TOPSPEED3) * 5;//距離を補正し、より速く曲がるようにする
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;	
        	}
		}
		
		old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;
		
		//再生走行時 　ブレーキ
		//if((c_c_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4){
		if((i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4){ //再生走行時　ブレーキ可能可能距離に到達していないときも少し減速する
			
			x=(i_C_TOPSPEED4-i_Encoder10)*2;
			r=(i_C_TOPSPEED4-i_Encoder10)*2;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
			motor_f( x, x );
            motor_r( r, r );
		
      	
 		}else if( ((i_date_f_mode == 0) && i_Encoder10 >= i_C_TOPSPEED) //通常モード　指定速度を超えたとき
				|| ((c_c_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED3) //再生走行モード ブレーキ可能距離で速度を超えたとき
				|| ((c_c_cut == 0 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED)   ) {  //再生走行モード　クランクは再生しないモード　ブレーキ可能距離で速度を超えたとき        // エンコーダによりスピード制御 
              
		
			//通常
			if(c_c_cut == 0 || i_date_f_mode == 0)x=(i_C_TOPSPEED -i_Encoder10)*20;
			else x=(i_C_TOPSPEED3 -i_Encoder10)*30;
			
			if(c_c_cut == 0 || i_date_f_mode == 0)r=(i_C_TOPSPEED -i_Encoder10)*20;
			else r=(i_C_TOPSPEED3 -i_Encoder10)*30;
		
		/*		
			if(i < -10 || 10 < i){//abs
		
				if(r < -30)r = -30;
			}
		*/	 
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
				
			motor_f( x, x );
            motor_r( r, r );
			
	
        }else{
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }
		
		
		if( l_EncoderTotal-l_startPoint >= 2000 ) {//誤動作チェック
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			i_Center_offset = 0;
			i_date_f_num_ch--;
            i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
        }

        break;

    case 31:
        /* 右クランク処理 */
		//setBeepPatternS( 0x8000 );
		
		old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;     
			
        if(i_date_f_mode == 0 || c_c_cut == 0){//通常
			c_mode = 3;//左無視
		
			if(c_c_short_mode == 1){//short
				if((l_EncoderTotal-l_startPoint ) >= 130){
					if(i < 95)i_SetAngle = 125;
					else i_SetAngle = 115;
				
				}else i_SetAngle = 110;
			
				motor_f( 15, 0 );  //10,0        /* この部分は「角度計算(4WD時).xls」 85 -40*/
        		motor_r( -40, -40 ); //-15.-15         /* で計算                        */
			
			}else{//long
				if((l_EncoderTotal-l_startPoint ) >= 200){
					if(i < 95)i_SetAngle = 130;
					else i_SetAngle = 100;
				
				}else i_SetAngle = 95;
			
				motor_f( 85,  -5 );          /* この部分は「角度計算(4WD時).xls」 85 -40*/
        		motor_r( -30,  -30 );          /* で計算                        */
			}
			
		
		}else{
			c_mode = 1;//見る範囲を狭く
			c_ch_boost_on = 1; //ショートカットON後のカーブ用パラメータを変更するフラグ
				
			if(c_c_cut_short == 1){//距離が短いとき
				
				//if(0 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 150  || (i_Wide != 0 && -6 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 200)){
				if(0 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 150){
			
					i_SetAngle = -40;
					motor2_f( -20,   20 );  //0 80    
	        		motor2_r( 0,   0 );   //0 0
				
				//	l_startPoint += 2; //手前で曲がりすぎているため距離を少しのばす
				
		
				}else if((l_EncoderTotal-l_startPoint ) >= 350){
				
					i_SetAngle = 100;
					motor2_f( 40,   5 );  //85 5       
	        		motor2_r( 8,   -5 );  //25 0
				
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i < 70)i_SetAngle = 120;
					else i_SetAngle = 98;
				
					motor2_f( 20,   -10 );  //80 0       
	        		motor2_r( -20,   -20 );   //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
				
					if(i < 45)i_SetAngle = 100;
					else i_SetAngle = 80;
				
					motor2_f( 30,   -10 );  //85 5      
	        		motor2_r( -10,   -20 );  //15 0
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
				
					if(i < 30)i_SetAngle = 80;
					else i_SetAngle = 50;
				
					motor2_f( 35,  -10 ); //85 30        
	        		motor2_r( 0,   -20 );  //25 0
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
				
					if(i < 20)i_SetAngle = 50;
					else i_SetAngle = 35;
				
					motor2_f( 40, -10 ); //90 35        
	        		motor2_r( 0,  -20 );   //35 0
					
				}else{
					i_SetAngle = 25;
					motor2_f( 40,   -5 );  //90 40       
	        		motor2_r( 0,   -20 );   //40  0	
				} 
				
				
				 
			}else{	//通常距離
			
				//if(0 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 150  || (i_Wide != 0 && -6 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 200)){
				if(0 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 150){
			
					i_SetAngle = -30;
					motor2_f( -20,   50 );  //0 80    
	        		motor2_r( 0,   0 );   //0 0
				
				//	l_startPoint += 2; //手前で曲がりすぎているため距離を少しのばす
				
				}else if((l_EncoderTotal-l_startPoint ) >= 350){
				
					i_SetAngle = 100;
					motor2_f( 40,   5 );  //85 5       
        			motor2_r( 8,   -5 );  //25 0
				
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i < 70)i_SetAngle = 120;
					else i_SetAngle = 98;
				
					motor2_f( 30,   0 );  //80 0       
        			motor2_r( -10,   -10 );   //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
				
					if(i < 45)i_SetAngle = 100;
					else i_SetAngle = 80;
				
					motor2_f( 40,   0 );  //85 5      
        			motor2_r( 0,   -10 );  //15 0
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
				
					if(i < 30)i_SetAngle = 80;
					else i_SetAngle = 50;
				
					motor2_f( 45,   2 ); //85 30        
        			motor2_r( 10,   -10 );  //25 0
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
				
					if(i < 20)i_SetAngle = 50;
					else i_SetAngle = 35;
				
					motor2_f( 50,   5 ); //90 35        
        			motor2_r( 15,  -5 );   //35 0
					
				}else{
					i_SetAngle = 25;
					motor2_f( 50,   10 );  //90 40       
        			motor2_r( 20,   -5 );   //40  0	
				} 
/*		
				}else if((l_EncoderTotal-l_startPoint ) >= 350){
				
					i_SetAngle = 100;
					motor2_f( 40,   5 );  //85 5       
	        		motor2_r( 8,   -5 );  //25 0
				
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i < 70)i_SetAngle = 120;
					else i_SetAngle = 98;
				
					motor2_f( 25,   -5 );  //80 0       
	        		motor2_r( -15,   -15 );   //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
				
					if(i < 45)i_SetAngle = 100;
					else i_SetAngle = 80;
				
					motor2_f( 35,   -5 );  //85 5      
	        		motor2_r( -5,   -15 );  //15 0
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
				
					if(i < 30)i_SetAngle = 80;
					else i_SetAngle = 50;
				
					motor2_f( 40,   0 ); //85 30        
	        		motor2_r( 5,   -15 );  //25 0
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
				
					if(i < 20)i_SetAngle = 50;
					else i_SetAngle = 35;
				
					motor2_f( 45,   0 ); //90 35        
	        		motor2_r( 10,  -10 );   //35 0
					
				}else{
					i_SetAngle = 25;
					motor2_f( 45,   5 );  //90 40       
	        		motor2_r( 15,   -10 );   //40  0	
				}
	*/
				
			}
			
       	}
  
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */
        
		//180 -15 < 25
        if((( c_c_cut == 0 || i_date_f_mode == 0) && (l_EncoderTotal-l_startPoint ) >= 150) || ((c_c_cut == 1 && i_date_f_mode != 0) && (l_EncoderTotal-l_startPoint ) >= 240 )  ){
			if(i_Wide != 0){
			//if (((20 < i_Center)&&(i_Center < 40)) || ((-15 < i_Center)&&(i_Center < 15))) {    /* 曲げ終わりチェック           */
			if ( (( c_c_cut == 0 || i_date_f_mode == 0) && 18 < i_Center && i_Center < 35 && (i_Wide != 0 && i_Wide < 12) ) 
				|| ((c_c_cut == 1 && i_date_f_mode != 0) && -20 < i_Center && i_Center < 0 && (i_Wide_old == 0 || i_Wide_old == 127 || i_Wide > i_Wide_old))
			//	|| ((c_c_cut == 1 && i_date_f_mode != 0) && -20 < i_Center && i_Center < 0 && (i_Wide != 0 && i_Wide < 30))
				  || ((c_c_cut == 1 && i_date_f_mode != 0) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 750)
				  || ((c_c_cut == 1 && i_date_f_mode != 0 && (c_c_cut_short == 1)) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 600) ){    /* 曲げ終わりチェック           */
				
            	ul_cnt_1ms = 0;
            	i_SensorPattern = 0;
            	
				l_startPoint = l_EncoderTotal;
            	i_pattern = 32;
				
			//	c_mode = 0;//通常
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				
				if(i_date_f_mode != 0 && c_c_cut == 1){	
					ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
					c_mode = 0;//見る範囲を元に戻す
            		i_pattern = 11;//通常トレースへ
					ul_cnt_curve_time_1ms = 0;
				}	
        	}
			}
		}
        break;

    case 32:
        /* 安定するまで (ショーカットは32には来ないよ)*/
		
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			c_mode = 0;//見る範囲を元に戻す
		}
		
		if(c_c_short_mode == 1){//short
        	if((l_EncoderTotal-l_startPoint ) >= 110)i_SetAngle = 90;
			else i_SetAngle = 95;
		}else{//long
			if((l_EncoderTotal-l_startPoint ) >= 100)i_SetAngle = 90;
			else i_SetAngle = 95;
		}
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if( i_Encoder10 >= i_C_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           
			x=(i_C_TOPSPEED2-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_c_short_mode == 1){//short
				motor2_f( 50, 40 );
           		motor2_r( 0, 0 );
		
		
			}else{//long
				motor2_f( 80, 40 );
           		motor2_r( 40, 10 );
			}
		}
         
		
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			if(i_Wide != 0){                       
        		if(((-15 < i_Center)&&(i_Center < 15) && getServoAngle() < 120) 
					|| ((-5 < i_Center)&&(i_Center < 5) && getServoAngle() < 128)) {    /*  直線になるまで          */
            		ul_cnt_1ms = 0;
            		i_SensorPattern = 0;
            	
           			i_pattern = 33;
					c_mode = 0;//通常
					l_startPoint = l_EncoderTotal;
        		}
			}
		}
	
        break;
	
	case 33://少し待つ	
		//c_mode = 1;//見る範囲を狭くする
		c_mode = 0;//見る範囲を元に戻す
		
		servoPwmOut( i_ServoPwm );

		if( i_Encoder10 >= i_TOPSPEED ) {          // エンコーダによりスピード制御 
          
			x=(i_TOPSPEED-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if( ((l_EncoderTotal-l_startPoint ) >= 100 && -30 < getServoAngle() && getServoAngle() < 30 ) || ((l_EncoderTotal-l_startPoint ) >= 300)  ) {
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
			c_mode = 0;//見る範囲を元に戻す
            i_pattern = 11;//通常トレースへ
			ul_cnt_curve_time_1ms = 0;
        }
        break;
		

    case 41:
        /* 左クランク処理 */
        //setBeepPatternS( 0x8000 );
			
        if(i_date_f_mode == 0 || c_c_cut == 0){//通常
			c_mode = 2;//右無視
			
			if(c_c_short_mode == 1){//short
				if((l_EncoderTotal-l_startPoint ) >= 130){
				
					if(i > -95)i_SetAngle = -125;
					else i_SetAngle = -115;
				
				}else i_SetAngle = -110;
			
				motor_f( 0, 15);    //0,10      /* この部分は「角度計算(4WD時).xls」*/
        		motor_r( -40, -40 );   //-15.-15       /* で計算                        */
				
			}else{//long
				
				if((l_EncoderTotal-l_startPoint ) >= 200){
				
					if(i > -95)i_SetAngle = -130;
					else i_SetAngle = -100;
				
				}else i_SetAngle = -95;
			
				motor_f(  -5, 85 );          /* この部分は「角度計算(4WD時).xls」*/
        		motor_r(  -30, -30 );          /* で計算                        */
			}
		}else{
			c_mode = 1;//見る範囲を狭く
			c_ch_boost_on = 1; //ショートカットON後のカーブ用パラメータを変更するフラグ
			
			 
			if(c_c_cut_short == 1){//距離が短いとき
			
				//if((i_Center < 0 && (l_EncoderTotal-l_startPoint ) >= 150) || (i_Wide != 0 && i_Center < 6 && (l_EncoderTotal-l_startPoint ) >= 200)){
				if((i_Center < 0 && (l_EncoderTotal-l_startPoint ) >= 150)){
					i_SetAngle = 40;

					motor2_f( 20,   -20 );  //80 0       
	        		motor2_r( 0,   0 );  //0 0
		
					//l_startPoint += 2; //手前で曲がりすぎているため距離を少しのばす

				
				}else if((l_EncoderTotal-l_startPoint ) >= 350){
					i_SetAngle = -100;

					motor2_f( 5,   40 ); //5 85        
	        		motor2_r( -5,   8 );  //0 25
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i > -70)i_SetAngle = -120;
					else i_SetAngle = -98;

					motor2_f( -10,   20 ); //0 80        
	        		motor2_r( -20,   -20 );  //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
					if(i > -45)i_SetAngle = -100;
					else i_SetAngle = -80;
				
					motor2_f( -10,   30 ); //5 85        
	        		motor2_r( -20,   -10 ); //0 15
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
					if(i > -30)i_SetAngle = -80;
					else i_SetAngle = -50;
				
					motor2_f( -5,   35 ); //30 85        
	        		motor2_r( -20,   0 ); //0 25
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
					if(i > -20)i_SetAngle = -50;
					else i_SetAngle = -35;
				
					motor2_f( -5,   40 ); //35 90       
	        		motor2_r( -20,  0 );   //0 35
					
				}else{
					i_SetAngle = -25;
					motor2_f( -5,   40 );  //40 90    
	        		motor2_r( -20,   0 );   // 0 40
					
				}
			  
			}else{//通常距離
				//if((i_Center < 0 && (l_EncoderTotal-l_startPoint ) >= 150) || (i_Wide != 0 && i_Center < 6 && (l_EncoderTotal-l_startPoint ) >= 200)){
				if((i_Center < 0 && (l_EncoderTotal-l_startPoint ) >= 150) ){
					i_SetAngle = 30;

					motor2_f( 50,   -20 );  //80 0       
	        		motor2_r( 0,   0 );  //0 0
		
					//l_startPoint += 2; //手前で曲がりすぎているため距離を少しのばす

				}else if((l_EncoderTotal-l_startPoint ) >= 350){
					i_SetAngle = -100;

					motor2_f( 5,   40 ); //5 85        
        			motor2_r( -5,   8 );  //0 25
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i > -70)i_SetAngle = -120;
					else i_SetAngle = -98;

					motor2_f( 0,   30 ); //0 80        
        			motor2_r( -10,   -10 );  //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
					if(i > -45)i_SetAngle = -100;
					else i_SetAngle = -80;
				
					motor2_f( 0,   40 ); //5 85        
        			motor2_r( -10,   0 ); //0 15
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
					if(i > -30)i_SetAngle = -80;
					else i_SetAngle = -50;
				
					motor2_f( 2,   45 ); //30 85        
        			motor2_r( -10,   10 ); //0 25
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
					if(i > -20)i_SetAngle = -50;
					else i_SetAngle = -35;
				
					motor2_f( 5,   50 ); //35 90       
        			motor2_r( -5,  15 );   //0 35
					
				}else{
					i_SetAngle = -25;
					motor2_f( 10,   50 );  //40 90    
        			motor2_r( -5,   20 );   // 0 40
				}  
			
			
/*				
				}else if((l_EncoderTotal-l_startPoint ) >= 350){
					i_SetAngle = -100;

					motor2_f( 5,   40 ); //5 85        
	        		motor2_r( -5,   8 );  //0 25
				
				}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
					if(i > -70)i_SetAngle = -120;
					else i_SetAngle = -98;

					motor2_f( -5,   25 ); //0 80        
	        		motor2_r( -15,   -15 );  //0 0
			
				
				}else if((l_EncoderTotal-l_startPoint ) >= 150){
					if(i > -45)i_SetAngle = -100;
					else i_SetAngle = -80;
				
					motor2_f( -5,   35 ); //5 85        
	        		motor2_r( -15,   -5 ); //0 15
				
				}else if((l_EncoderTotal-l_startPoint ) >= 100){
					if(i > -30)i_SetAngle = -80;
					else i_SetAngle = -50;
				
					motor2_f( 0,   40 ); //30 85        
	        		motor2_r( -15,   5 ); //0 25
			
				}else if((l_EncoderTotal-l_startPoint ) >= 50){
					if(i > -20)i_SetAngle = -50;
					else i_SetAngle = -35;
				
					motor2_f( 0,   45 ); //35 90       
	        		motor2_r( -10,  10 );   //0 35
					
				}else{
					i_SetAngle = -25;
					motor2_f( 5,   45 );  //40 90    
	        		motor2_r( -10,   15 );   // 0 40
				}  
*/
			}
       	}
		
		servoPwmOut( i_ServoPwm2 );        /* 振りが弱いときは大きくする       */
        
		if(((c_c_cut == 0 || i_date_f_mode == 0) && (l_EncoderTotal-l_startPoint ) >= 150) || ((c_c_cut == 1 && i_date_f_mode != 0) && (l_EncoderTotal-l_startPoint ) >= 240 ) ){
			if(i_Wide != 0){ 
			//if( ((-40 < i_Center)&&(i_Center < -20)) || ((-15 < i_Center)&&(i_Center < 15))) {    /* 曲げ終わりチェック           */
	 		if(( (c_c_cut == 0 || i_date_f_mode == 0) && -35 < i_Center && i_Center < -18 && (i_Wide != 0 && i_Wide < 10)) 
				|| ( (c_c_cut == 1 && i_date_f_mode != 0) && 0 < i_Center && i_Center < 20 && (i_Wide_old == 0 || i_Wide_old == 127 ||  i_Wide < i_Wide_old)) 
				//|| ( (c_c_cut == 1 && i_date_f_mode != 0) && 0 < i_Center && i_Center < 20 && (i_Wide != 0 && i_Wide < 30)) 
					|| ((c_c_cut == 1 && i_date_f_mode != 0) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 750)
					|| ((c_c_cut == 1 && i_date_f_mode != 0 && (c_c_cut_short == 1)) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 600)){    /* 曲げ終わりチェック           */
	 	
            	ul_cnt_1ms = 0;
            	i_SensorPattern = 0;
            
            	l_startPoint = l_EncoderTotal;
            	i_pattern = 42;
				
				//c_mode = 0;//通常
				
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				if(i_date_f_mode != 0 && c_c_cut == 1){	
					ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
					c_mode = 0;//見る範囲を元に戻す
            		i_pattern = 11;//通常トレースへ
					ul_cnt_curve_time_1ms = 0;
				}
			}
			}
		}
        break;

    case 42:
	
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			c_mode = 0;//見る範囲を元に戻す
		}
		
		/* 安定するまで (ショーカットは42には来ないよ)*/
		if(c_c_short_mode == 1){//short
			if((l_EncoderTotal-l_startPoint ) >= 110)i_SetAngle = -90;
			else i_SetAngle = -95;
		}else{//long
			if((l_EncoderTotal-l_startPoint ) >= 100)i_SetAngle = -90;
			else i_SetAngle = -95;
		}
		
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if( i_Encoder10 >= i_C_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           
			x=(i_C_TOPSPEED2-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_c_short_mode == 1){//short
				motor2_f( 40, 50);
           		motor2_r( 0, 0 );
			}else{//long
				motor2_f( 40, 80);
           		motor2_r( 10, 40 );
			}
		}
         	  
		
		if((l_EncoderTotal-l_startPoint ) >= 50){   
			if(i_Wide != 0){                      
				if(((-15 < i_Center)&&(i_Center < 15) && getServoAngle() > -120) 
					|| ((-5 < i_Center)&&(i_Center < 5) && getServoAngle() > -128)) {    /*  直線になるまで          */
            		
            		ul_cnt_1ms = 0;
      			    i_SensorPattern = 0;
            	
           			i_pattern = 43;
					c_mode = 0;//通常
					l_startPoint = l_EncoderTotal;
        		}
			}
		}
		
        break;
		
	case 43://少し待つ	
		//c_mode = 1;//見る範囲を狭くする
		c_mode = 0;//見る範囲を元に戻す
		
		servoPwmOut( i_ServoPwm );

		if( i_Encoder10 >= i_TOPSPEED ) {          // エンコーダによりスピード制御 
			
			x=(i_TOPSPEED-i_Encoder10)*10;
	
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if(( (l_EncoderTotal-l_startPoint ) >= 100 && -30 < getServoAngle() && getServoAngle() < 30) || ((l_EncoderTotal-l_startPoint ) >= 300) ) {
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
			c_mode = 0;//見る範囲を元に戻す
            i_pattern = 11;//通常トレースへ
			ul_cnt_curve_time_1ms = 0;
        }
        break;
		
	case 51://左ハーフ
		//setBeepPatternS( 0x8000 );
	    i_SetAngle = 0;
		servoPwmOut( i_ServoPwm2 );

		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // エンコーダによりスピード制御 
       
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(i_date_f_mode == 0 && (check_crossline() || check_halfline() == 2 )) {       // クロスラインチェック         
            ul_cnt_1ms = 0;
            i_pattern = 21;	
        }
  
        if( (l_EncoderTotal - l_startPoint ) >= 100 ) {
            ul_cnt_1ms = 0;
            i_pattern = 52;
			l_startPoint = l_EncoderTotal;
        }
        break;

	case 52://ハーフライン後
	//	if( i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 100 )c_mode = 3;//左無視
	
#ifdef HWall //壁あり	 
		if(i_Center > 11){//寄りすぎ
			i_Center_offset = -6;//左に寄る
			servoPwmOut( i_ServoPwm );
		}else{
			i_Center_offset = -9;//左に寄る
			servoPwmOut( i_ServoPwm  );
		}
#else //壁無し
		if(i_Center > 12){//寄りすぎ
			i_Center_offset = -7;//左に寄る
			servoPwmOut( i_ServoPwm );
		}else{
			i_Center_offset = -10;//左に寄る
			servoPwmOut( i_ServoPwm  );
		}
#endif		
		
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
			
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}

	
	//	if((l_EncoderTotal-l_startPoint ) >= 300 ){
	//		if((( c_h_cut == 0 || i_date_f_mode == 0)  && i_Wide == 0) || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //脱線チェック
        
		if(((c_h_cut == 0 || i_date_f_mode == 0)  && (i_Wide == 0 || check_crossline()) && (l_EncoderTotal-l_startPoint ) >= 250 )  || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //脱線チェック
            
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
				
			if(i_date_f_mode == 0){//距離計測
				i_date_buff_ch_int[i_date_num_ch++] = 53;
				i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
			}
				
           	 i_pattern = 53;
			i_date_f_num_ch++;
			c_mode = 0;//通常
			i_Center_offset = 0;//オフセットを戻す
            break;
		}
		
		if(i_date_f_mode == 0 && (l_EncoderTotal-l_startPoint ) < 30  && ( check_crossline() || check_halfline() == 2 )) {       // クロスラインチェック         
            ul_cnt_1ms = 0;
			i_Center_offset = 0;//オフセットを戻す
            i_pattern = 21;	
        }
  
		if( (l_EncoderTotal-l_startPoint ) >= 1500 ) {//誤動作チェック
			
			i_date_f_num_ch--;
			c_mode = 0;//通常
            i_pattern = 11;
			//ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
			i_Center_offset = 0;//オフセットを戻す
        }	

		break;

	case 53:
		c_mode = 2;//右無視
		
		if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //壁あり    	
			i_SetAngle = -65;
#else //壁無し
			i_SetAngle = -40;
#endif

		}else{
			i_SetAngle = -16;
		}
		
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
				
				motor_f( x, 10 );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
           		motor_f( 0, 100 );
           		motor_r( 10, 20 );
#else //壁無し

           		motor_f( 10, 90 );
           		motor_r( 70, 70 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
         
		if( ((i_date_f_mode == 0 || c_h_cut == 0) && (l_EncoderTotal - l_startPoint ) >= 200 ) || (i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 350 )) {  
	        if((-50 < i_Center)&&(i_Center < -10) && i_Wide != 0 ) {    /* 曲げ終わりチェック           */
			//	if(i_Center < -30) {    /* 曲げ終わりチェック           */
		
	            	ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
	            	i_pattern = 54;
				
					c_mode = 0;//通常
					motor_mode_r( BRAKE, BRAKE );
	        	}
			}
        break;

	case 54:
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
			
			if((i_Center == 0)&&(i_Wide == 0)){ //インに落ちそう
				i_SetAngle = -50;
			}else{
#ifdef HWall  //壁あり    	
				i_SetAngle = -30;
#else //壁無し
				i_SetAngle = 0; // -20
#endif
			}
		}else{
			i_SetAngle = -5;//5 7
		}
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
	
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
           		motor_f( 100, 100 );
           		motor_r( 60,  60 );
#else //壁無し
           		motor_f( 90, 80 );
           		motor_r( 60,  50 );
#endif


			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
	
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 90,  50 );
			}
		}
        
		
        //if( (15 < i_Center)&&(i_Center < 50) ) {    /* 曲げ終わりチェック           */
		//if(-10 < i_Center && i_Wide != 0) {    /* 曲げ終わりチェック           *///-7
		
#ifdef HWall  //壁あり    	
		if(10 < i_Center && i_Wide != 0) {    /* 曲げ終わりチェック           *///-7
#else //壁無し
		if(0 < i_Center && i_Wide != 0) {    /* 曲げ終わりチェック           *///-7
#endif
	//	if(20 < i_Center) {    /* 曲げ終わりチェック           */
		
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
            i_pattern = 55;
			
			motor_mode_r( FREE, FREE );
        }
		
        break;

	case 55://安定するまで
	
		if(i_Wide != 0 && -10 < i_Center && i_Center < 10 ){
			servoPwmOut( i_ServoPwm  );	
		}else if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //壁あり    	
			i_SetAngle = 20;
#else //壁無し
			i_SetAngle = 20;
#endif
			servoPwmOut( i_ServoPwm2 );          
		
			//servoPwmOut( i_ServoPwm  );
		}else{
			i_SetAngle = 20;//40 37
			servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		}
		
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
           		motor_f( 100, 30 );
           		motor_r( 90, 0 );
#else //壁無し
				if(i_Center > 10){//外に落ちそう
           			motor_f( 90, 0 );
				}else{
					motor_f( 90, 30 );
				}
				motor_r( 80, 0 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;

				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 50 );
           		motor_r( 90, 0 );
			}
		}
          
		if( (l_EncoderTotal - l_startPoint ) >= 50 ) {  

#ifdef HWall  //壁あり  
	        if((-35 < i_Center)&&(i_Center < 35)&&(i_Wide != 0)) {    /*  直線になるまで          */
#else //壁無し
			//if((-23 < i_Center)&&(i_Center < 23)&&(i_Wide != 0)) {    /*  直線になるまで          */
			if((-18 < i_Center)&&(i_Center < 18)&&(i_Wide != 0)) {    /*  直線になるまで          */
#endif	
            ul_cnt_1ms = 0;
            i_SensorPattern = 0;
          
			i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
			
			l_startPoint = l_EncoderTotal;
        }
		}
        break;
		
	
			
	case 61://右ハーフ
		
		//setBeepPatternS( 0x8000 );
		i_SetAngle = 0;
		servoPwmOut( i_ServoPwm2 );

		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(i_date_f_mode == 0 && (check_crossline() || check_halfline() == 1)) {       // クロスラインチェック         
            ul_cnt_1ms = 0;
           	
            i_pattern = 21;			
        }

        if( (l_EncoderTotal-l_startPoint ) >= 100 ) {
            ul_cnt_1ms = 0;
            i_pattern = 62;
			l_startPoint =l_EncoderTotal;
        }
        break;

	case 62://ハーフライン後
		
	//	if( i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 100 )c_mode = 2;//右無視

#ifdef HWall //壁あり
		if(i_Center < -11){//
			i_Center_offset = 6;//右に寄る
			servoPwmOut( i_ServoPwm );
				
		}else{
			i_Center_offset = 9;//右に寄る
			servoPwmOut( i_ServoPwm );
		}
#else //壁無し		
		if(i_Center < -12){//
			i_Center_offset = 7;//右に寄る
			servoPwmOut( i_ServoPwm );
				
		}else{
			i_Center_offset = 10;//右に寄る
			servoPwmOut( i_ServoPwm );
		}
#endif		
		
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
	
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
		
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(((c_h_cut == 0 || i_date_f_mode == 0)  && (i_Wide == 0 || check_crossline()) && (l_EncoderTotal-l_startPoint ) >= 250 )  || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //脱線チェック
            
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
				
			if(i_date_f_mode == 0){//距離計測
				i_date_buff_ch_int[i_date_num_ch++] = 63;
				i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
			}
				
           	i_pattern = 63;
			i_date_f_num_ch++;
			c_mode = 0;//通常
			i_Center_offset = 0;//オフセットを戻す
				
            break;
        }
			

		if(i_date_f_mode == 0 && (l_EncoderTotal-l_startPoint ) < 30  && ( check_crossline() || check_halfline() == 1 )) {       // クロスラインチェック         
            ul_cnt_1ms = 0;
			i_Center_offset = 0;//オフセットを戻す
            i_pattern = 21;	
        }
  
		if( (l_EncoderTotal-l_startPoint ) >= 1500 ) {//誤動作チェック
          	
			i_date_f_num_ch--;
			c_mode = 0;//通常
            i_pattern = 11;
			//ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
			i_Center_offset = 0;//オフセットを戻す
        }

		break;

	case 63:
		//setBeepPatternS( 0x8000 );

		c_mode = 3;//左無視
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
 
#ifdef HWall  //壁あり    	
			i_SetAngle = 68;
#else //壁無し
			i_SetAngle = 50;
#endif			
		}else{
			i_SetAngle = 16;//48 47
		}
		
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
          
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
			
				motor_f( 10, x );
    	       	motor_r( x, x );
       		}else{
#ifdef HWall  //壁あり    	
				motor_f( 100, 0 );
           		motor_r( 20, 10 );
#else //壁無し
				motor_f( 90, 10 );
           		motor_r( 70, 70 );
#endif	

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
    	       	motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
                
		if( ((i_date_f_mode == 0 || c_h_cut == 0) && (l_EncoderTotal - l_startPoint ) >= 200 ) || (i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 350 )) {             
	        if((10 < i_Center)&&(i_Center < 50) && i_Wide != 0) {    /* 曲げ終わりチェック           */
			//if( 30 < i_Center) {    /* 曲げ終わりチェック           */
		
	            ul_cnt_1ms = 0;
	            l_startPoint = l_EncoderTotal;
	            i_pattern = 64;
			
				c_mode = 0;//通常
			
				motor_mode_r( BRAKE, BRAKE );
	        }
		}
        break;

	case 64:

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if((i_Center == 0)&&(i_Wide == 0)){ //インに落ちそう
				i_SetAngle = 50;
			}else{
#ifdef HWall  //壁あり    	
				i_SetAngle = 30;//-3
#else //壁無し
				i_SetAngle = 0;//-3 20
#endif
			}
		}else{
			i_SetAngle = 5;//-3
		}
		
		servoPwmOut( i_ServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
          
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
				motor_f( 100, 100 );
           		motor_r( 60, 60 );
#else //壁無し
				motor_f( 80, 90 );
           		motor_r( 50, 60 );
#endif
           		
			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 50, 90 );
			}
		}
             
		//if( (l_EncoderTotal - l_startPoint ) >= 30 ) {                  
        //if((-50 < i_Center)&&(i_Center < -15)) {    /* 曲げ終わりチェック           */

#ifdef HWall  //壁あり    	
		if(i_Center < -10 && i_Wide != 0) {    /* 曲げ終わりチェック           *///-5
#else //壁無し
		if(i_Center < 0 && i_Wide != 0) {    /* 曲げ終わりチェック           *///-5
#endif				
		//if(i_Center < -20) {    /* 曲げ終わりチェック           */
	
            ul_cnt_1ms = 0;
            l_startPoint = l_EncoderTotal;
            i_pattern = 65;
			
			motor_mode_r( FREE, FREE );
        }
		//}
        break;

	case 65://安定するまで
	
		if(i_Wide != 0 && -10 < i_Center && i_Center < 10 ){
			servoPwmOut( i_ServoPwm  );	
			
		}else if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //壁あり    	
			i_SetAngle = -20;
#else //壁無し
			i_SetAngle = -10;
			
#endif

			servoPwmOut( i_ServoPwm2 );
			
		//	servoPwmOut( i_ServoPwm  );
		}else{
			i_SetAngle = -20;//-40 -35
			
			servoPwmOut( i_ServoPwm2 );
		}
		       

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;

				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
				motor_f( 30, 100 );
           		motor_r( 0, 90 );
#else //壁無し
				if(i_Center < -10){//外に落ちそう
					motor_f( 0, 90 );
				}else{
					motor_f( 30, 90 );					
				}
           		motor_r( 0, 80 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
				
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 50, 100 );
           		motor_r( 0, 90 );
			}
		}
                      
	    if( (l_EncoderTotal - l_startPoint ) >= 50 ) {  
#ifdef HWall  //壁あり  
	        if((-35 < i_Center)&&(i_Center < 35)&&(i_Wide != 0)) {    /*  直線になるまで          */
#else //壁無し
			//if((-23 < i_Center)&&(i_Center < 23)&&(i_Wide != 0)) {    /*  直線になるまで          */
			if((-18 < i_Center)&&(i_Center < 18)&&(i_Wide != 0)) {    /*  直線になるまで          */
#endif		
	            ul_cnt_1ms = 0;
	            i_SensorPattern = 0;
        
				i_pattern = 11;
				ul_cnt_curve_time_1ms = 0;
		
				l_startPoint = l_EncoderTotal;
	        }
		}
        break;
	case 101:
        /* 停止 */
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        setBeepPatternS( 0xc000 );
		
	
		i_msdFlag = 0;
        if( i_msdError != 0 ) {
            /* microSDに不具合があったなら終了 */
            printf( "microSD Initialize Error!!\n" );
            i_pattern = 109;
        } else {
            i_pattern = 102;
            ul_cnt_1ms = 0;
        }
        break;

    case 102:
		/* 最後のデータ書き込むまで待つ*/
        if( checkMicroSDProcess() == 0 ) {
            i_pattern = 103;               /* データ転送処理へ             */
            break;
        }
        if( checkMicroSDProcess() == 11 ) {
            microSDProcessEnd();        /* microSDProcess終了処理       */
            while( checkMicroSDProcess() );
            i_pattern = 103;               /* データ転送処理へ             */
        }
		
        break;

   case 103:
		 
        /* 0.5s待ち && プッシュスイッチが離されたかチェック*/
        if( ul_cnt_1ms >= 500 && !pushsw_get()) {
            i_pattern = 104;
            ul_cnt_1ms = 0;
        }
        break;

    case 104:
        /* プッシュスイッチが押されたかチェック */
        led_out( ul_cnt_1ms / 200 % 2 ? 0x6 : 0x9  );
        if( pushsw_get() ) {
            i_pattern = 105;
            ul_cnt_1ms = 0;
        }
        break;

    case 105:
        /* タイトル転送、転送準備 */
        printf( "\n" );
        printf( "CarName Data Out\n" ); /* 自分のカーネームを入れてください */
        printf( "i_pattern, i_Center, i_Wide ,角度, サーボPWM, " );
        printf( "左前PWM, 右前PWM, 左後PWM, 右後PWM, エンコーダ5*2,モード,坂道回数,ジャイロY/10,ジャイロX/10,赤外線\n" );
		
		ul_msdWorkAddress = ul_msdStartAddress;   /* 読み込み開始アドレス     */
        i_pattern = 106;
        break;
	case 106:
        /* microSDよりデータ読み込み */
        if( ul_msdWorkAddress >= ul_msdEndAddress ) {
            /* 書き込み終了アドレスになったら、終わり */
            i_pattern = 109;
            break;
        }
        ret = readMicroSD( ul_msdWorkAddress , c_msdBuff );
        if( ret != 0x00 ) {
            /* 読み込みエラー */
            printf( "\nmicroSD Read Error!!\n" );
            i_pattern = 109;
            break;
        } else {
            /* エラーなし */
            ul_msdWorkAddress += 512;
            i_msdBuffAddress = 0;
            i_pattern = 107;
			i = 0;
        }
        break;
	case 107:
        /* データ転送 */
        led_out( 1 << (ul_cnt_1ms/100) % 8 );

		if(c_msdBuff[i_msdBuffAddress+0] <= 0 ){ /* パターンが0以下なら終了 */
			date_f_make(-1,0,0,0);
            printf( "End.\n" );
            i_pattern = 108;
			ul_cnt_1ms  = 0 ;
			setBeepPatternS( 0xff00 );
            break;
        }

	//	if( (dipsw_get() & 0x04) == 0x00 ) {//ログ出力なし、早くコース記憶したい時用
			/* データの転送 */
	        printf( "%d,%4d,%4d,%5d,%5d,%5d,%5d,%5d,%5d,%4d,%4d,%4d,%4d,%4d,%4d\n",
	            (int)c_msdBuff[i_msdBuffAddress+0],                  /* パターン     */
	            (int)c_msdBuff[i_msdBuffAddress+1],					/* センター*/
	            (unsigned char)c_msdBuff[i_msdBuffAddress+2],                  /* ワイド */
				(int)((unsigned char)c_msdBuff[i_msdBuffAddress+3]*0x100 +
		             (unsigned char)c_msdBuff[i_msdBuffAddress+4] ),			/* 角度 */
	            /* サーボPWM */
		            c_msdBuff[i_msdBuffAddress+6],
		            /* 左前PWM */
		            c_msdBuff[i_msdBuffAddress+7],
		            /* 右前PWM */
		            c_msdBuff[i_msdBuffAddress+8],
		            /* 左後PWM */
		            c_msdBuff[i_msdBuffAddress+9],
		            /* 右後PWM */
		            c_msdBuff[i_msdBuffAddress+10],
		            /* エンコーダ */
		            c_msdBuff[i_msdBuffAddress+11] * 2,
					/* モード */
		            c_msdBuff[i_msdBuffAddress+12],
					/* 坂道回数 */
		            c_msdBuff[i_msdBuffAddress+13],
					/* 加速度センサーY */
		            c_msdBuff[i_msdBuffAddress+14],
					/* 加速度センサーX */
		            c_msdBuff[i_msdBuffAddress+15],
					/* 赤外線センサーの差 */
		            c_msdBuff[i_msdBuffAddress+5]
        );		
	//	}
        
		if(i_date_f_mode != 0){
			date_f_make((int)c_msdBuff[i_msdBuffAddress+0],(int)((unsigned char)c_msdBuff[i_msdBuffAddress+3]*0x100 +
	             (unsigned char)c_msdBuff[i_msdBuffAddress+4] ) ,c_msdBuff[i_msdBuffAddress+11] ,(int)c_msdBuff[i_msdBuffAddress+12]);
		}
		
		 i_msdBuffAddress += 64;  /* 次の送信準備                 */

        if( i_msdBuffAddress >= 512 ) {
            i_pattern = 106;
        }
		
        break;

    case 108:
        /* 転送終了 */
		if(i_date_f_mode == 1){
			/* ブロックA イレーズします */
			blockEraseDataFlash( 0x3000 );
			/* ブロックB イレーズします */
			blockEraseDataFlash( 0x3400 );
			
			/* ブロックA 書き込み */
			writeDataFlash( 0x3000, c_date_f_buff, 32 );
			
			
			if( readMicroSD( ul_msdStartAddress_ch , c_msdBuff_ch ) != 0x00 ) {
				// 読み込みエラー 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)c_date_f_buff_ch[i] = c_msdBuff_ch[i];
			}
			
			/* ブロックB 書き込み */
			writeDataFlash( 0x3400, c_date_f_buff_ch, 32 );
			printf("i_date_f_mode == 1\n");
			
		}else if(i_date_f_mode == 2){
			/* ブロックC イレーズします */
			blockEraseDataFlash( 0x3800 );
			/* ブロックD イレーズします */
			blockEraseDataFlash( 0x3c00 );
			
			/* ブロックC 書き込み */
			writeDataFlash( 0x3800, c_date_f_buff, 32 );
			
			if( readMicroSD( ul_msdStartAddress_ch , c_msdBuff_ch ) != 0x00 ) {
				// 読み込みエラー 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)c_date_f_buff_ch[i] = c_msdBuff_ch[i];
			}
		
			/* ブロックD 書き込み */
			writeDataFlash( 0x3c00, c_date_f_buff_ch, 32 );
		
			printf("i_date_f_mode == 2\n");
		}
		
		j=0;
		for(i = 0; i < 16; i++){
			//printf("%d%d",c_date_f_buff[j],c_date_f_buff[j+1]);
			printf("%d\n",i_date_f_buff_int[i]);
			
			j+=2;
		}
		
		printf("\n");
		for(i = 0; i < 32; i+=3){
			printf("%d %d%02d\n",c_date_f_buff_ch[i],c_date_f_buff_ch[i+1],c_date_f_buff_ch[i+2]);
		}
		
		
		i_pattern = 109;
        led_out( 0xff );
        break;
		
	case 109:
		led_out( 0xff );
		break;
	case 200://走行終了
		setBeepPatternS( 0xff00 );
		
		while(1){
			cam_in();
			if(c_logfin == 0)led_out(camera(i_Center,i_Wide));
			else led_out(0);
			
			if(i_Encoder10 < 5 ){
				if(i_msdFlag == 1)i_msdFlag = 2;                /* データ記録終了               */
				servoPwmOut( 0 );
			}else{
				if(c_out_flag == 1){//コースアウト時
					servoPwmOut( 0 );//衝突時の破損を防ぐためサーボに力を入れない
				}else{
					servoPwmOut( i_ServoPwm );					
				}
			}
		
		/*	if( pushsw_get() ) {
				while(pushsw_get());
		 		for(i = 0; i < 32; i+=2){
					printf("%d %d\n",i_date_buff_ch_int[i],i_date_buff_ch_int[i+1]);
				}
			}*/
			motor_f( 0, 0 );
            motor_r( 0, 0 );
			if(c_logfin == 0){
				if(i_msdFlag == 0){
					//setBeepPatternS( 0xff00 );
					
					servoPwmOut( 0 );
					
					j = 0;
					for(i = 0; i < 32; i+=2){
						//printf("%d %d\n",i_date_buff_ch_int[i],i_date_buff_ch_int[i+1]);
						c_msdBuff_ch[j++] = (signed char)i_date_buff_ch_int[i];
						c_msdBuff_ch[j++] = (signed char)(i_date_buff_ch_int[i+1]/100);
						c_msdBuff_ch[j++] = (signed char)(i_date_buff_ch_int[i+1]%100);
					}
					
					while( checkMicroSDProcess() != 11 )wait(2); /* 書き込みが終わるまで待つ */
					// 書き込み処理が終わるまで繰り返す
					while( microSDProcessEnd() != 0 )wait(2);
					
					for(i = 0; i < 3; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					/* microSDProcess開始処理 */
					while(microSDProcessStart( ul_msdStartAddress_ch) != 0x00)wait(2);
            		
					setMicroSDdata( c_msdBuff_ch );
					
					while( checkMicroSDProcess() != 11 )wait(2); /* 書き込みが終わるまで待つ */
					// 書き込み処理が終わるまで繰り返す
					while( microSDProcessEnd() != 0 )wait(2);
			
					
					for(i = 0; i < 5; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					c_logfin = 1;
				}
			}
		}
		break;

		
	case 500:
		
		if( pushsw_get() ) {
			c_mode++;
			if(c_mode >= 4)c_mode = 0;
			while(pushsw_get());
		}
		
		
		cam_in();
		led_out(camera(i_Center,i_Wide));
	
		printf("c_mode = %d  %4d   %4d\n",c_mode,i_Center,i_Wide);
		break;
	
    default:
        break;
    }
    }
}


int date_f_make(int i_pattern, int i_angle, int i_encoder, int i_rmode){

	static int si_mode = 0; //S = 0, L = 1, R = 2, C = 3, H = 4, F = 5
	static long sl_Encoder = 0;
	static int si_buff_num = 0;
	static int si_buff_num_int = 0;
//	static int si_buff_num_ch = 0;
//	static int si_buff_num_int_ch = 0;
	
	sl_Encoder += i_encoder;
	
	switch(i_pattern){
		case -1:
			//if(	sl_Encoder >= 500){//この距離以下は無視（滑り？）
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//この距離以下は無効
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
					si_mode = 5;//F
				}
		//	}
			break;
		case 10:
		case 11:
		//if(	sl_Encoder >= 500){//この距離以下は無視（滑り？）
			if(si_mode == 0){//S
				if(((i_rmode == 0 ) && (i_angle < -i_Cu_Angle)) || ((i_rmode != 0 ) && (i_angle < -(i_Cu_Angle_saka)))){
					if(sl_Encoder > 1000){//この距離以下は無効
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
						
					si_mode = 1;//L
						
				}else if(((i_rmode == 0) &&(i_Cu_Angle < i_angle)) || ((i_rmode != 0) &&(i_Cu_Angle_saka < i_angle))){
					if(sl_Encoder > 1000){//この距離以下は無効
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
						
					si_mode = 2;//R
				}
			}else if(si_mode == 1){//Lカーブ
				//if(((i_rmode == 0) && (-i_Cu_Angle < i_angle)) || ((i_rmode != 0) && (-(i_Cu_Angle_saka) < i_angle)) ){
				if(-i_Cu_Angle < i_angle){
					sl_Encoder =0;
											
					si_mode = 0;//S
				}
			}else if(si_mode == 2){//Rカーブ
				//if(((i_rmode ==0) && (i_angle < i_Cu_Angle) || ((i_rmode !=0) && (i_angle < i_Cu_Angle_saka)))){
				if(i_angle < i_Cu_Angle){
				
					sl_Encoder =0;
						
					si_mode = 0;//S
				}
			}else{
				if(((i_rmode == 0) &&(-i_Cu_Angle < i_angle && i_angle < i_Cu_Angle)) || ((i_rmode != 0) &&(-(i_Cu_Angle_saka) < i_angle && i_angle < i_Cu_Angle_saka))){
					sl_Encoder =0;
					si_mode = 0;//S
				}
			}

		//}
		break;

		case 21:
		case 22:
			if(si_mode != 3){//C
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//この距離以下は無効
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
				}
				sl_Encoder =0;
			}
			si_mode = 3;//C
				
			break;
		case 31:
			if(si_mode != 2){//R
				//このときのsl_Encoderが曲がるまでの距離
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/			
				sl_Encoder =0;
			}
			si_mode = 2;//R
			break;
		case 41:
			if(si_mode != 1){//L
				//このときのsl_Encoderが曲がるまでの距離
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/
				sl_Encoder =0;	
			}
			si_mode = 1;//L
			break;

		case 51:
		case 61:
		case 52:
		case 62:
			if(si_mode != 4){//H
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//この距離以下は無効
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = (sl_Encoder/100);
						si_buff_num++;
						c_date_f_buff[si_buff_num] = (sl_Encoder - ((sl_Encoder/100)*100));
						si_buff_num++;
					}
				}
				sl_Encoder =0;
			}
			si_mode = 4;//H
			break;
		case 53:
			if(si_mode != 1){//L
				//このときのsl_Encoderが曲がるまでの距離
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/	
				sl_Encoder =0;
			}
			si_mode = 1;//L
			break;
		
		case 55:
			sl_Encoder =0;
			si_mode = 0;//S
			break;
			
		case 63:
			if(si_mode != 2){//R
				//このときのsl_Encoderが曲がるまでの距離
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/				
				sl_Encoder =0;
			}
			si_mode = 2;//R
			break;
		
		case 65:
			sl_Encoder =0;
			si_mode = 0;//S
			break;	
			
		default:
			break;
	}
	
	return 0;
}

/************************************************************************/
/* R8C/38A スペシャルファンクションレジスタ(SFR)の初期化                */
/************************************************************************/
void init( void )
{
    int     i;

    /* クロックをXINクロック(20MHz)に変更 */
    prc0  = 1;                          /* プロテクト解除               */
    cm13  = 1;                          /* P4_6,P4_7をXIN-XOUT端子にする*/
    cm05  = 0;                          /* XINクロック発振              */
    for(i=0; i<50; i++ );               /* 安定するまで少し待つ(約10ms) */
    ocd2  = 0;                          /* システムクロックをXINにする  */
    prc0  = 0;                          /* プロテクトON                 */

    /* ポートの入出力設定 */

    /*  PWM(予備)       左前M_PMW       右前M_PWM       ブザー
        センサ左端      センサ左中      センサ右中      センサ右端  */
    p0   = 0x00;
    prc2 = 1;                           /* PD0のプロテクト解除          */
    pd0  = 0xf0;

    /*  センサ中心      ｽﾀｰﾄﾊﾞｰ         RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0          */
    pur0 |= 0x04;                       /* P1_3～P1_0のプルアップON     */
    p1  = 0x00;
    pd1 = 0x10;

    /*  右前M_方向      ステアM_方向    ステアM_PWM     右後M_PWM
        右後M_方向      左後M_PWM       左後M_方向      左前M_方向      */
    p2  = 0x00;
    pd2 = 0xff;

	
	/*  7:モード出力1bit           6:幅6           5:モード出力0bit            4:幅5
        3:幅4            2:エンコーダB相      	   1:幅3            0:エンコーダA相   */
    p3  = 0x00;
    pd3 = 0xa0;	
	
    /*  XOUT            XIN             ボード上のLED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;

	
    /*      センター値   P5_6～P5_0       */
	pur1 |= 0x0c;   /* P5_7～P5_0のプルアップON     */
    p5  = 0x00;
    pd5 = 0x00;

    /*      SDカード 0,1,2,3,4 
			幅(2,1,0bit) 7,6,5          */
	pur1 |= 0x30;   /* P6_7～P6_0のプルアップON     */
	//pur1_addr.bit.b5;	/* P6_4 to P6_7 pull-up */
    p6  = 0x00;
    pd6 = 0x1f;
	
    /*  CN6.2入力       CN6.3入力       CN6.4入力       CN6.5入力
        none(ｱﾅﾛｸﾞ予備) 角度VR          センサ_左ｱﾅﾛｸﾞ  センサ_右ｱﾅﾛｸﾞ  */
    p7  = 0x00;
    pd7 = 0x00;

    /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
        DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
    pur2 |= 0x03;                       /* P8_7～P8_0のプルアップON     */
    p8  = 0x00;
    pd8 = 0x00;

    /*  -               -               ﾌﾟｯｼｭｽｲｯﾁ       P8制御(LEDorSW)
        右前M_Free      左前M_Free      右後M_Free      左後M_Free      */
    p9  = 0x00;
    pd9 = 0x1f;
    pu23 = 1;   // P9_4,P9_5をプルアップする
    /* タイマRBの設定 */
    /* 割り込み周期 = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* 動作モード、分周比設定       */
    trbpre = 200-1;                     /* プリスケーラレジスタ         */
    trbpr  = 100-1;                     /* プライマリレジスタ           */
    trbic  = 0x06;                      /* 割り込み優先レベル設定       */
    trbcr  = 0x01;                      /* カウント開始                 */

    /* A/Dコンバータの設定 */
    admod   = 0x33;                     /* 繰り返し掃引モードに設定     */
  //  adinsel = 0x90;                     /* 入力端子P7の4端子を選択      */
	adinsel = 0xb0;                     /* 入力端子P7の8端子を選択      */
    
    adcon1  = 0x30;                     /* A/D動作可能                  */
    asm(" nop ");                       /* φADの1サイクルウエイト入れる*/
    adcon0  = 0x01;                     /* A/D変換スタート              */

 /* タイマRG(位相計数モード)の設定 */
    timsr = 0xc0;                       /* TRGCLKA,TRGCLKB端子割り当て  */
    trgcntc = 0xff;                     /* 位相計数ﾓｰﾄﾞのｶｳﾝﾄ方法指定   */
    trgmr = 0x82;                       /* TRGのカウント開始            */
	
	/* タイマRG タイマモード(両エッジでカウント)の設定 */
    //timsr = 0x40;                       /* TRGCLKA端子 P3_0に割り当てる */
    //trgcr = 0x15;                       /* TRGCLKA端子の両エッジでカウント*/
	//trgcr = 0x05;                       /* TRGCLKA端子の立ち上がりエッジでカウント*/
    //trgmr = 0x80;                       /* TRGのカウント開始            */
   

    /* タイマRC PWMモード設定(左前モータ、右前モータ) */
    trcpsr0 = 0x40;                     /* TRCIOA,B端子の設定           */
    trcpsr1 = 0x33;                     /* TRCIOC,D端子の設定           */
    trcmr   = 0x0f;                     /* PWMモード選択ビット設定      */
    trccr1  = 0x8e;                     /* ｿｰｽｶｳﾝﾄ:f1,初期出力の設定    */
    trccr2  = 0x00;                     /* 出力レベルの設定             */
    trcgra  = TRC_MOTOR_CYCLE - 1;      /* 周期設定                     */
    trcgrb  = ui_trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
    trcgrc  = trcgra;                   /* P0_7端子のON幅(予備)         */
    trcgrd  = ui_trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
    trcic   = 0x07;                     /* 割り込み優先レベル設定       */
    trcier  = 0x01;                     /* IMIAを許可                   */
    trcoer  = 0x01;                     /* 出力端子の選択               */
    trcmr  |= 0x80;                     /* TRCカウント開始              */

    /* タイマRD リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ、ｻｰﾎﾞﾓｰﾀ) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0端子設定        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1端子設定     */
    trdmr   = 0xf0;                     /* バッファレジスタ設定         */
    trdfcr  = 0x01;                     /* リセット同期PWMモードに設定  */
    trdcr0  = 0x20;                     /* ソースカウントの選択:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* 周期設定             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2端子のON幅(左後モータ)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4端子のON幅(右後モータ)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5端子のON幅(サーボモータ) */
    trdoer1 = 0xcd;                     /* 出力端子の選択               */
    trdstr  = 0x0d;                     /* TRD0カウント開始             */
	
	
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
    unsigned int i;
	int k;
	//static unsigned int sui_EncoderMod = 0;
	//static int si_Encoder5_old = 0; 
	static int si_Encoder1_buf[10] = {0}; 
	int i_Encoder_buf = 0; 
	int a;
	static int si_flag = 0,si_flag20 = 0,si_flag56 = 0;
	signed char *p;
	
    asm(" fset I ");                    /* タイマRB以上の割り込み許可   */
	
    ul_cnt_1ms++;
	ul_cnt_running_1ms++;
	ul_cnt_for_wait++;
	ul_cnt_straight_time_1ms++;
	ul_cnt_curve_time_1ms++;
	
	
	/* microSD間欠書き込み処理(1msごとに実行)   */
	microSDProcess();

	
	//加速度センサー
	get_angle_y();
	get_angle_x();

	
    /* サーボモータ制御 */
    servoControl();
	servoControl2();
	
    /* ブザー処理 */
    beepProcessS();
	
	////////////////////////////////////////////////////////
	

	//33*3.14  // 1パルス*1.03 = 1mm  //1回転 100パルス
	i = trg;
	si_Encoder1_buf[i_Timer10] = (i - ui_EncoderBuff)/2;
	l_EncoderTotal += si_Encoder1_buf[i_Timer10];
	if(l_EncoderTotal < 0)l_EncoderTotal = 0;
	
	//ui_EncoderBuff = i;
	if((i - ui_EncoderBuff)%2 == 0){
    	ui_EncoderBuff = i;
  	}else{
    	ui_EncoderBuff = i - 1;
  	}
  
	i_Encoder_buf = 0;
	for(k = 0; k < 10; k++)i_Encoder_buf += si_Encoder1_buf[k];
	i_Encoder10 = i_Encoder_buf;
	

    /* 10回中1回実行する処理 */
    i_Timer10++;
    switch( i_Timer10 ) {

    case 1:
        break;

    case 2:
        /* スイッチ読み込み準備 */
        p9_4 = 0;                       /* LED出力OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* スイッチ読み込み、LED出力 */
        uc_types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
        p8  = uc_types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のLEDへ出力*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED出力ON                    */
        break;

    case 4:
	case 9:
	/*	si_Encoder5_old = i_Encoder5 ;
		
		//21*3.14=65.94 200パルス 200/65.94=3パルスで1mm
		i = trg;
        i_Encoder5       = (i - ui_EncoderBuff)/3;	
		sui_EncoderMod     = (i - ui_EncoderBuff)%3;	
        l_EncoderTotal += i_Encoder5;
		//ui_EncoderBuff = i ;
        ui_EncoderBuff = i - sui_EncoderMod ;
		
		i_Encoder10 = i_Encoder5 + si_Encoder5_old ; 
*/		

		i_Encoder_buf = 0;
		for(k = 0; k < 5; k++)i_Encoder_buf += si_Encoder1_buf[(10 + i_Timer10 - k) % 10];
		i_Encoder5 = i_Encoder_buf;
	
		if(i_date_f_mode != 0 && ( i_msdFlag == 1 || i_msdFlag == 2 )){//再生走行モード
			a = getServoAngle();
			//直線 
			if((i_pattern == 11 || i_pattern == 10) && (((c_mode == 0) &&(-i_Cu_Angle < a && a < i_Cu_Angle)) || (((c_mode != 0) || ( (l_EncoderTotal - l_startPoint_saka) < 1500 ) ) &&(-(i_Cu_Angle_saka) < a && a < i_Cu_Angle_saka)) ) ){
				l_straight_EncoderTotal += i_Encoder5;//距離計測
				
				if(si_flag56 == 1){//ハーフ後の距離補正
					i_date_f_buff_int[i_date_f_num] -= i_date_f_plus_h;
					si_flag56 = 0;	
				}
				
				if(c_mode == 0){
					//if(i_date_f_buff_int[i_date_f_num] - date_f_brake < l_straight_EncoderTotal)si_flag = 1;//記録した直線を走った
					if(i_date_f_buff_int[i_date_f_num] - 500 < l_straight_EncoderTotal)si_flag = 1;//記録した直線を走った
				}else{
					//if(i_date_f_buff_int[i_date_f_num] - date_f_brake - 500 < l_straight_EncoderTotal)si_flag = 1;//記録した直線を走った
					if(i_date_f_buff_int[i_date_f_num] - 500 - 600 < l_straight_EncoderTotal)si_flag = 1;//記録した直線を走った
				}
				si_flag20 = 0;
			
			}else if(i_pattern == 21 || i_pattern == 22  ){
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//次の直線待ち状態
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//記録済み直線終了
				}
				
				if(si_flag20 == 0){
					si_flag20 = 1;
					si_flag56 = 0;
					//l_straight_EncoderTotal = 0;
					l_straight_EncoderTotal = i_Encoder5;
				}else{
					l_straight_EncoderTotal += i_Encoder5;//クロスラインなどからの距離計測
				}
				
			}else if( i_pattern == 51 || i_pattern == 52 || i_pattern == 61 || i_pattern == 62 ){
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//次の直線待ち状態
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//記録済み直線終了
				}
				
				if(si_flag20 == 0){
					si_flag20 = 1;
					si_flag56 = 1;
					//l_straight_EncoderTotal = 0;
					l_straight_EncoderTotal = i_Encoder5;
				}else{
					l_straight_EncoderTotal += i_Encoder5;//クロスラインなどからの距離計測
				}
				
			}else{//カーブ or クランク,ハーフ曲がり中 
			
				l_straight_EncoderTotal = 0;
				
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//次の直線待ち状態
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//記録済み直線終了
				}
			}
		
			if(si_flag20 == 99){//記録済み直線終了
				//クランク、ハーフも終了
				if(i_date_f_buff_ch_int[i_date_f_num_ch] == 0 && (i_pattern == 11 || i_pattern == 10))i_date_f_mode = 0;//再生走行終了
			}
		}
		
		
		
		/* microSD記録処理 */
    	if( i_msdFlag == 1 || i_msdFlag == 2 ) {
			p = c_msdBuff + i_msdBuffAddress;

            /* バッファに記録 ここから */
            *p++ = i_pattern;             /* パターン                     */
            *p++ = (char)i_Center;    
            *p++ = (char)i_Wide;
			i = getServoAngle();        /* 角度                         */
			*p++ = i >> 8;
            *p++ = i & 0xff;
			
			*p++ = (char)i_Center_IR;//赤外線センサー
			
			*p++ = i_handleBuff;  /* サーボPWM保存        */
            *p++ = i_FleftMotorBuff;       /* 前左モータPWM値                */
            *p++ = i_FrightMotorBuff;      /* 前右モータPWM値                */
			*p++ = i_RleftMotorBuff;       /* 後左モータPWM値                */
            *p++ = i_RrightMotorBuff;      /* 後右モータPWM値                */
			*p++ = i_Encoder5;    /* エンコーダ                   */
            *p++ = c_mode;/*モード*/
            *p++ = c_saka_cnt;/*坂道回数*/
            *p++ = (char)(i_angle_y/10);
            *p++ = (char)(i_angle_x/10);
			
		//	*p++ = (char)(IR_L());
        //    *p++ = (char)(IR_R());
            
			
            /* バッファに記録 ここまで */

            i_msdBuffAddress += 64;       /* RAMの記録アドレスを次へ      */


			if( i_msdBuffAddress >= 512) {	
                /* 512個になったら、microSDに記録する */
                i_msdBuffAddress = 0;
                setMicroSDdata( c_msdBuff );
                ul_msdWorkAddress += 512;
				
                if( ul_msdWorkAddress >= ul_msdEndAddress || i_msdFlag == 2) {
                    /* 記録処理終了 */
    				i_msdFlag = 0;
                }
            }
		}
        break;

    case 5:
        break;

    case 6:
        break;

    case 7:
        break;

    case 8:
        break;

  //  case 9:
    //    break;

    case 10:
        /* i_Timer10変数の処理 */
        i_Timer10 = 0;
        break;
    }
}

/************************************************************************/
/* タイマRC 割り込み処理                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
    trcsr &= 0xfe;

    /* タイマRC　デューティ比の設定 */
    trcgrb = ui_trcgrb_buff;
    trcgrd = ui_trcgrd_buff;
}


/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char uc_sw;

    uc_sw = p1 & 0x0f;                     /* P1_3～P1_0読み込み           */

    return uc_sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return uc_types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char uc_sw;

    uc_sw = ~p9_5 & 0x01;

    return uc_sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のCN6の状態読み込み                     */
/* 引数　 なし                                                          */
/* 戻り値 0～15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char uc_data;

    uc_data = p7 >> 4;

    return uc_data;
}


/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char uc_led )
{
	/*
	int i;
	unsigned char LED = 0,l[8];
	
	//	8bit表示用に変換	
	l[0] = led & 0x01;
	l[4] = (led & 0x02) >> 1;
	l[1] = (led & 0x04) >> 2;
	l[5] = (led & 0x08) >> 3;
	l[2] = (led & 0x10) >> 4;
	l[6] = (led & 0x20) >> 5;
	l[3] = (led & 0x40) >> 6;
	l[7] = (led & 0x80) >> 7;
	
	for(i = 7; i > 0; i--){
		LED += l[i];
		LED <<= 1;
	}
	LED += l[0];
	
	// 実際の出力はタイマRB割り込み処理で実施 
    uc_types_led = LED;
	*/
	uc_types_led = uc_led;
	
}


/************************************************************************/
/* 後輪の速度制御 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
		
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < BRAKE_MAX_R){
		accele_l = BRAKE_MAX_R;
	}
	
	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < BRAKE_MAX_R){
		accele_r = BRAKE_MAX_R;
	}
	
	
	i_RleftMotorBuff  = accele_l;          /* バッファに保存               */
    i_RrightMotorBuff = accele_r;         /* バッファに保存               */
	

    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
	/*	if(accele_l == 100){
			p2_2 = 1;
			//trdpsr0 = 0x08;                     // TRDIOB0,C0,D0端子設定        
    		//trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1端子設定     
		}else{
			trdpsr0 = 0x08;                     // TRDIOB0,C0,D0端子設定        
    		trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1端子設定     
		}*/
		
		if(accele_l == 100)trdgrd0 = (long)( TRD_MOTOR_CYCLE +2);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
		if(accele_l == -100)trdgrd0 = (long)( TRD_MOTOR_CYCLE + 2);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
		if(accele_r == 100)trdgrc1 = (long)( TRD_MOTOR_CYCLE  +2);
        else trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
		
		//trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
		if(accele_r == -100)trdgrc1 = (long)( TRD_MOTOR_CYCLE  +2);
        else trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
		
		//trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}


/************************************************************************/
/* 前輪の速度制御 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < BRAKE_MAX){
		accele_l = BRAKE_MAX;
	}
	

	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < BRAKE_MAX){
		accele_r = BRAKE_MAX;
	}
	
	
	i_FleftMotorBuff  = accele_l;          /* バッファに保存               */
    i_FrightMotorBuff = accele_r;         /* バッファに保存               */
	
	
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = ui_trcgrb_buff = trcgra;
    } else {
		if(accele_l == 100)ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
		
		//ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = ui_trcgrd_buff = trcgra;
    } else {
		if(accele_r == 100)ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
		
		//ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}


/************************************************************************/
/* 後輪の速度制御3 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < BRAKE_MAX_R)accele_l = BRAKE_MAX_R;
	
	
	if(accele_l == 0 || (i_RleftMotorBuff > 0 && accele_l < 0) || (i_RleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - i_RleftMotorBuff  > i_KASOKU)accele_l = i_RleftMotorBuff + i_KASOKU;
	else if(accele_l - i_RleftMotorBuff < -i_KASOKU)accele_l = i_RleftMotorBuff - i_KASOKU;
	
	
	if(accele_r > 100)accele_r = 100;
	else if(accele_r < BRAKE_MAX_R)accele_r = BRAKE_MAX_R;
	
	
	if(accele_r == 0 || (i_RrightMotorBuff > 0 && accele_r < 0) || (i_RrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - i_RrightMotorBuff > i_KASOKU)accele_r = i_RrightMotorBuff + i_KASOKU;
	else if(accele_r - i_RrightMotorBuff < -i_KASOKU)accele_r = i_RrightMotorBuff- i_KASOKU;
	
	
	i_RleftMotorBuff  = accele_l;          /* バッファに保存               */
    i_RrightMotorBuff = accele_r;          /* バッファに保存               */

    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
	/*	if(accele_l == 100){
			p2_2 = 1;
			//trdpsr0 = 0x08;                     // TRDIOB0,C0,D0端子設定        
    		//trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1端子設定     
		}else{
			trdpsr0 = 0x08;                     // TRDIOB0,C0,D0端子設定        
    		trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1端子設定     
		}*/
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
		//if(accele_l == -100)p2_2 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
		//if(accele_r == 100)p2_4 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
		//if(accele_r == -100)p2_4 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}


/************************************************************************/
/* 前輪の速度制御3 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < BRAKE_MAX)accele_l = BRAKE_MAX;
	
	
	if(accele_l == 0  || (i_FleftMotorBuff > 0 && accele_l < 0) || (i_FleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - i_FleftMotorBuff  > i_KASOKU)accele_l = i_FleftMotorBuff + i_KASOKU;
	else if(accele_l - i_FleftMotorBuff < -i_KASOKU)accele_l = i_FleftMotorBuff - i_KASOKU;
	

	if(accele_r > 100)accele_r = 100;
	else if(accele_r < BRAKE_MAX)accele_r = BRAKE_MAX;
	
	
	if(accele_r == 0 || (i_FrightMotorBuff > 0 && accele_r < 0) || (i_FrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - i_FrightMotorBuff > i_KASOKU)accele_r = i_FrightMotorBuff + i_KASOKU;
	else if(accele_r - i_FrightMotorBuff < -i_KASOKU)accele_r = i_FrightMotorBuff- i_KASOKU;
	

	i_FleftMotorBuff  = accele_l;          /* バッファに保存               */
    i_FrightMotorBuff = accele_r;          /* バッファに保存               */
		
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = ui_trcgrb_buff = trcgra;
    } else {
		//if(accele_l == 100)p0_5 = 1;
        ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = ui_trcgrd_buff = trcgra;
    } else {
		//if(accele_r == 100)p0_6 = 1;
        ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* 後モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if( mode_r ) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}

/************************************************************************/
/* 前モータ停止動作（ブレーキ、フリー）                                 */
/* 引数　 左モータ:FREE or BRAKE , 右モータ:FREE or BRAKE               */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if( mode_r ) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int i_pwm )
{
	int i_angle = getServoAngle();
	
	if((i_angle < -SERVO_MAX && i_pwm  > 0) || ( SERVO_MAX < i_angle && i_pwm  < 0)){//ハンドル曲げすぎ
		i_pwm = -(i_pwm / 8);//逆転				
	}
	
	i_pwm = -i_pwm;//
	
	if( i_pwm >  100 ) i_pwm =  100;        /* マイコンカーが安定したら     */
    if( i_pwm < -100 ) i_pwm = -100;        /* 上限を90くらいにしてください */
    
	i_handleBuff = i_pwm;                 /* バッファに保存               */


    if( i_pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * i_pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -i_pwm ) / 100;
    }
}


/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( ad2 - i_Angle0 );
}

/************************************************************************/
/* スタートゲート検出処理												*/
/* 戻り値 0:なし 1:あり													*/
/************************************************************************/
unsigned char check_startgate( void )
{
	unsigned char uc_ret;
	
	uc_ret = 0;
	
	//if(IR_R() > 140){//37
	if(850 < ad7){ 
		uc_ret = 1;
	}
	/*
	if((i_Wide >= 27) && (i_Center > -20) && (i_Center < 20)){
	
		uc_ret = 1;			// ゲート発見 
	}*/
	return uc_ret;
}
 

/************************************************************************/
/* クロスライン検出処理													*/
/* 戻り値 0:クロスラインなし 1:あり										*/
/************************************************************************/
unsigned char check_crossline( void )
{
	unsigned char uc_ret;
	
	//cam_in();//値の取得

	uc_ret = 0;
	if( (i_Wide > 60) || ((i_Wide >= 40) && (-8 < i_Center ) && (i_Center < 8))   || ((i_Wide >= 28) && (i_Center > -5) && (i_Center < 5)) ){
	
		uc_ret = 1;			/* クロスライン発見 */

	}
	return uc_ret;
}
 

/************************************************************************/
/* ハーフライン検出処理                                                 */
/* 戻り値 0:ハーフラインなし 1:左 2:右 3:クランク                       */
/************************************************************************/
unsigned char check_halfline( void )
{
    unsigned char uc_ret;

	uc_ret = 0;
	if(i_Wide > 44 && i_Wide < 50){
		if(i_Center < -10){//センター左寄り
			uc_ret = 1;
			
		}else if(i_Center > 10){//センター右寄り
			uc_ret = 2;
			
		}
	}else if(i_Wide > 34){
		if(i_Center < -8){//センター左寄り
			uc_ret = 1;
			
		}else if(i_Center > 8){//センター右寄り
			uc_ret = 2;
			
		}
	
	}else if(i_Wide > 23){
		if(i_Center < -5){//センター左寄り
			uc_ret = 1;
			
		}else if(i_Center > 5){//センター右寄り
			uc_ret = 2;
			
		}
	}
	
	return uc_ret;
}

/************************************************************************/
/* ハーフライン検出処理  クランク用                                     */
/* 戻り値 0:ハーフラインなし 1:左 2:右 3:クランク                       */
/************************************************************************/
unsigned char check_halfline_forC( void ) //クランク時の直角確認用
{
    unsigned char uc_ret;
	
	uc_ret = 0;
	if(i_Wide > 23){
		if(i_Center < -1){//センター左寄り
			uc_ret = 1;
			
		}else if(i_Center > 1){//センター右寄り
			uc_ret = 2;
			
		}
	}
	
	return uc_ret;
}

/************************************************************************/
/* wideline検出処理													*/
/* 戻り値 0:なし 1:あり										*/
/************************************************************************/
unsigned char check_wideline( void )
{
	unsigned char uc_ret;
	
	//cam_in();//値の取得
	
	uc_ret = 0;
	if(i_Wide > 23){
	
		uc_ret = 1;			/* wideライン発見 */
	}
	return uc_ret;
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 i_ServoPwm に代入                               */
/************************************************************************/
void servoControl( void )
{
	long     l_Ret, l_P, l_D;
	static int si_I = 0;

  
    /* サーボモータ用PWM値計算 */
	if(c_IR_flag == 0){	
		l_P = i_kp * (i_Center + i_Center_offset);      /* 比例                         */
		si_I = si_I + (i_SensorBefore - (i_Center + i_Center_offset) );
    	l_D = i_kd * (i_SensorBefore - (i_Center + i_Center_offset));     /* 微分(目安はPの5～10倍)       */
		l_Ret = l_P - l_D - si_I * i_ki;
	}else{
		l_P = i_kp_ir * (i_Center_IR);      /* 比例                         */
		si_I = si_I + (i_SensorBeforeIR - (i_Center_IR) );
    	l_D = i_kd_ir * (i_SensorBeforeIR - (i_Center_IR));     /* 微分(目安はPの5～10倍)       */
		l_Ret = l_P - l_D - si_I * i_ki_ir;
	}
    l_Ret /= 2;//256 128

    /* PWMの上限の設定 */
    if( l_Ret >  100 ) l_Ret =  100;        /* マイコンカーが安定したら     */
    if( l_Ret < -100) l_Ret = -100;        /* 上限を90くらいにしてください */
    i_ServoPwm = l_Ret;

    i_SensorBefore = (i_Center + i_Center_offset );                  /* 次回はこの値が1ms前の値となる*/
	i_SensorBeforeIR = i_Center_IR;
}

/************************************************************************/
/* モジュール名 servoControl2                                            */
/* 処理概要     サーボモータ制御                                        */
/* 引数         なし                                                    */
/* 戻り値       グローバル変数 i_ServoPwm2 に代入                         */
/************************************************************************/
void servoControl2( void )
{
    int      i,j, i_Ret, i_P, i_D;


    i = i_SetAngle;              	/* 目標角度             */
    j = getServoAngle();              	/* 目標角度センサ値             */

    /* サーボモータ用PWM値計算 */
    i_P = - i_kp2 * (j - i);                        /* 比例                         */
    i_D = - i_kd2 * (i_AngleBefore2 - j);     /* 微分(目安はPの5～10倍)       */
    i_Ret = i_P - i_D;
    i_Ret /= 2;

    /* PWMの上限の設定 */
    if( i_Ret >  100 ) i_Ret =  100;        /* マイコンカーが安定したら     */
    if( i_Ret < -100 ) i_Ret = -100;        /* 上限を90くらいにしてください */

    i_ServoPwm2 = i_Ret;

    i_AngleBefore2 = j;                  /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/*	wait																*/
/************************************************************************/
void wait(int t)
{
	ul_cnt_for_wait = 0;
	while (ul_cnt_for_wait <= t);
}

/**************************************************************************/
/*	RX62Gからのカメラの値を取得											  */
/**************************************************************************/
void cam_in(){
	int i_wide = 0;
	
	/*	
	//手配線
	i_wide += ((p6 >> 5 ) & 0x07);//012
	i_wide += ((p3 << 2 ) & 0x08);//3
	i_wide += ((p3 << 1 ) & 0x30);//45
	i_wide += ( p3 & 0x40);//6
	*/
	
	//NEW
	i_wide += ((p3 >> 6 ) & 0x01);
	i_wide += ((p3 >> 3 ) & 0x02);
	i_wide += ((p3 >> 1 ) & 0x04);
	i_wide += ((p3 << 2 ) & 0x08);
	i_wide += ((p6 >> 3 ) & 0x10);
	i_wide += ((p6 >> 1 ) & 0x20);
	i_wide += ((p6 << 1 ) & 0x40);
	
	
	//カメラソフトの設定範囲
	//LineStart 	35		/* カメラで見る範囲(通常モード) */
	//LineStop  	92
	if(i_wide <= 57 || i_wide == 127){ //92 - 35 = 57 この幅を超える場合は全白の127しかありえない
		i_Wide = i_wide;
	}else{
		//前回の値をそのまま使用する	
	}
	
	

	//i_Center = (p5 & 0x7f);
	
	if(1 < i_Wide  && i_Wide < 100){//カメラの取り付けで少しずれていすので
		i_Center = (p5 & 0x7f) - 64 + 3;	
	}else{
		i_Center = (p5 & 0x7f) - 64;
	}
}

/**************************************************************************/
/*	RX62Gへモードの値を出力											  */
/**************************************************************************/
void mode_out(){
	
	p3_5 = c_mode & 0x01;//LOW
	p3_7 = (c_mode & 0x02) >> 1;//HIGH
}

/**************************************************************************/
/*	LED表示用の値を計算													  */
/**************************************************************************/
int camera(int i_center, int i_wide){
	
	int i_start = 17,i_end = 111,i,i_led = 0,i_cnt = 1;
	// (end - start)/7 = 13
	int i_led_bit[8] = {999,48,55,61,999,67,74,81};//4は不調
	
	i_center += 64;
	i_wide /= 2;
	
	for(i = 0; i < 8; i++){
		if(i_center - i_wide < i_led_bit[i] && i_led_bit[i] < i_center + i_wide){
			i_led += i_cnt;
		}
		i_cnt *= 2;
	}

	return i_led;
}


/**************************************************************************/
/*	ジャイロセンサーのY軸の値を取得										  */
/**************************************************************************/
void get_angle_y(){
	
	i_pre_angle_y[0] = i_pre_angle_y[1];
	i_pre_angle_y[1] = i_angle_y;
	
	i_angle_y = (i_angle_y * 9/10) + (ad0 * 1/10);
	
	if(((i_pre_angle_y[0] < i_pre_angle_y[1]) && (i_pre_angle_y[1] < i_angle_y))  || ((i_angle_y < i_pre_angle_y[1]) && (i_pre_angle_y[1] < i_pre_angle_y[0])))
		i_angle_y = i_pre_angle_y[1];
		
	else if(((i_pre_angle_y[1] < i_pre_angle_y[0]) && (i_pre_angle_y[0] < i_angle_y))  || ((i_angle_y < i_pre_angle_y[0]) && (i_pre_angle_y[0] < i_pre_angle_y[1])))
		i_angle_y = i_pre_angle_y[0];
}

/**************************************************************************/
/*	ジャイロセンサーのX軸の値を取得													  */
/**************************************************************************/
void get_angle_x(){
	i_pre_angle_x[0] = i_pre_angle_x[1];
	i_pre_angle_x[1] = i_angle_x;
	
	i_angle_x = (i_angle_x * 85/100) + (ad1 * 15/100);
	
	if(((i_pre_angle_x[0] < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_pre_angle_x[0])))
		i_angle_x = i_pre_angle_x[1];
		
	else if(((i_pre_angle_x[1] < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_pre_angle_x[1])))
		i_angle_x = i_pre_angle_x[0];
}

/**************************************************************************/
/*	ジャイロセンサーのX軸の値をチェック									  */
/**************************************************************************/
int angle_check(){

	if(200 < i_angle_y && i_angle_y < 400){
		if(i_angle_x < 100)return 2;//上 170
		if(i_angle_x > 440)return 0;//下
	}
	return 1;//変化無し
}



/**************************************************************************/
/*	IR_L												  */
/**************************************************************************/
int IR_L(){
	
	return 100 * (ad6 - i_IR_min[0]) / (i_IR_max[0] - i_IR_min[0]);
}
/**************************************************************************/
/*	IR_R												  */
/**************************************************************************/
int IR_R(){
	
	return 100 * (ad7 - i_IR_min[1]) / (i_IR_max[1] - i_IR_min[1]);
}
/**************************************************************************/
/*	Get_Center_IR											  */
/**************************************************************************/
void Get_Center_IR(){
	int i_l = IR_L(),i_r = IR_R();
	i_IR_old = i_Center_IR;
	
	i_Center_IR = (i_r - i_l)/5;
	
	if(i_l < 15 && i_r < 15){//赤外線も遠くなったら前回値から
		if(	i_IR_old < 0)i_Center_IR = -30;
		else i_Center_IR = 30;
	}
	
}
/**************************************************************************/
/*	赤外線センサーのキャリブレーション												  */
/**************************************************************************/
void IRcalibration( ){
	int i ,j,l,r,p = 20,a = 60;

	i_Angle0 = 0;
	i_Angle0 = getServoAngle();  /* 0度の位置記憶                */
	i = getServoAngle();
	
	for(j = 0; j < 2;j++){
		while(-a < i){
			i = getServoAngle();
			servoPwmOut(p);
			r = ad7;
			if(i_IR_max[1] < r)i_IR_max[1] = r;
			if(r < i_IR_min[1])i_IR_min[1] = r;
			l = ad6;
			if(i_IR_max[0] < l)i_IR_max[0] = l;
			if(l < i_IR_min[0])i_IR_min[0] = l;
		}
	
		while(i < a){
			i = getServoAngle();
			servoPwmOut(-p);
			r = ad7;
			if(i_IR_max[1] < r)i_IR_max[1] = r;
			if(r < i_IR_min[1])i_IR_min[1] = r;
			l = ad6;
			if(i_IR_max[0] < l)i_IR_max[0] = l;
			if(l < i_IR_min[0])i_IR_min[0] = l;
		}
	}
	while(0 < i){
		i = getServoAngle();
		servoPwmOut(p);
		r = ad7;
		if(i_IR_max[1] < r)i_IR_max[1] = r;
		if(r < i_IR_min[1])i_IR_min[1] = r;
		l = ad6;
		if(i_IR_max[0] < l)i_IR_max[0] = l;
		if(l < i_IR_min[0])i_IR_min[0] = l;
	}
	servoPwmOut( 0 );
}

int max(int i_a,int i_b){
	if(i_a < i_b)return i_b;
	return i_a;	
}
/************************************************************************/
/* end of file                                                          */
/************************************************************************/



/*
改訂経歴

2011.06.01 Ver.1.00 作成
2012.02.23 Ver.1.01 モータドライブ基板TypeS Ver.3と
                    アナログセンサ基板TypeS Ver.2のコメント変更
2013.05.10 Ver.2.00 モータドライブ基板TypeS Ver.4に対応
*/
