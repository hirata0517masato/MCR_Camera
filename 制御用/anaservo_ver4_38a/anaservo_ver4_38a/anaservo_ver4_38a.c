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

#define 	MAXTIME 			1100 //1100	  	/* 最大走行時間 (0.01秒)  1200 = 12s     1250     */


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
int             pattern;                /* マイコンカー動作パターン     */
//int             crank_mode;             /* 1:クランクモード 0:通常      */
unsigned long   cnt1 = 0;               /* タイマ用                     */
unsigned long   cnt2 = 0;               /* 最大走行時間用               */
unsigned long   cnt3 = 0;               /* wait 関数用                  */
unsigned long   cnt4 = 0;               /* 脱線チェック用               */
unsigned long   cnt5 = 0;               /* 坂センサーチェック用         */
unsigned long   cnt6 = 0;               /* 脱線チェック用(エンコーダ)   */
unsigned long   cnt7 = 0;               /* 直線とカーブのカウント用   */
unsigned long   cnt8 = 0;               /* カーブ加速用のカウント用   */
unsigned long   cnt9 = 0;               /* 直線のカウント用   */
char 			run = 0;				/* 1 = 走行中					*/
char			mode = 0;				/* 0 = 通常 1 = 坂	2 = 右無視 3 = 左無視	*/
char			flag2 = 0;				/* 偶数 = 無視しない 奇数 = 無視する*/
int				out_cnt = 0;			//脱線カウント
char 			black_flag = 0;			//
char			Cu_flag = 0;			//0 = 直線, 1 = カーブ
char			gate = 0;
char			c_short_mode = 0; //クランク　0:long 1:short
char			c_cut;	//0= 再生しない 1= 再生する 編集無意味
char			c_cut_in_flag = 0;		//クランクショートカット時にINに入りすぎた時のフラグ

/* microSD関連変数 */
signed char     msdBuff[ 512 ];         /* 一時保存バッファ             */
int             msdBuffAddress;         /* 一時記録バッファ書込アドレス */
int             msdFlag = 0;                /* 1:データ記録 0:記録しない    */
int             msdTimer;               /* 取得間隔計算用               */
unsigned long   msdStartAddress;        /* 記録開始アドレス             */
unsigned long   msdEndAddress;          /* 記録終了アドレス             */
unsigned long   msdWorkAddress;         /* 作業用アドレス               */
int             msdError = 0;               /* エラー番号記録               */

signed char     msdBuff_ch[ 512 ];         /* 一時保存バッファ             */
unsigned long   msdStartAddress_ch = 0x0200;        /* 記録開始アドレス             */


char logfin = 0;

/* 現在の状態保存用 */
int             handleBuff;             /* 現在のハンドル角度記録       */
int             FleftMotorBuff = 0;          /* 現在の前左モータPWM値記録      */
int             FrightMotorBuff = 0;         /* 現在の前右モータPWM値記録      */
int             RleftMotorBuff = 0;          /* 現在の後左モータPWM値記録      */
int             RrightMotorBuff= 0;         /* 現在の後右モータPWM値記録      */


/* マイコン内フラッシュメモリ関連 */
int				date_f_mode = 0;		//0=なし 1=IN 2=OUT
signed char 	date_f_buff[32] ={0};
int			 	date_f_buff_int[16] ={0};
int				date_f_num = 0;
int				Cu_Angle	=		20;		//カーブ判定に使用 正数限定
int				Cu_Angle_saka	=	45;		//カーブ判定に使用 正数限定 坂用
signed char 	date_f_buff_ch[32] ={0};
int			 	date_f_buff_ch_int[32] ={0};//偶数＝パターン　奇数＝距離
int				date_f_num_ch = 0;

/*クランク、ハーフ距離計測用*/
int			 	date_buff_ch_int[32] ={0};//偶数＝パターン　奇数＝距離
int				date_num_ch = 0;
long            lEncoderTotal_ch = 0;          /* 積算値保存用                 */

/* エンコーダ関連 */
int             iTimer10 = 0;               /* 10msカウント用               */
long            lEncoderTotal = 0;          /* 積算値保存用                 */
unsigned long   lEncoderTotal2 = 0;          /* 積算値保存用                 */
int             iEncoder10 = 0;               /* 10ms毎の最新値               */
int             iEncoder5  = 0;               /*  5ms毎の最新値               */
unsigned int    uEncoderBuff  = 0;           /* 計算用　割り込み内で使用     */
long			sp = 0;						/* 距離計測用スタート地点 */
long			sp2 = 0;						/* 距離計測用スタート地点 坂？ */
long			sp3 = 0;						/* 距離計測用スタート地点 カーブ終了位置*/
long            SEncoderTotal = 0;          /* 直線積算値保存用                 */

/*  サーボ関連 */
int             iSensorBefore = 0;          /* 前回のセンサ値保存           */
int             iSensorBeforeIR = 0;          /* 前回のセンサ値保存           */
int             iAngleBefore2 = 0;
int             iServoPwm = 0;              /* サーボＰＷＭ値               */
int             iServoPwm2 = 0;
int             iAngle0 = 0;                /* 中心時のA/D値保存            */
int 			iSetAngle = 0;
int				iAngleBefore = 0;

/* センサ関連 */
int             iSensorPattern;         /* センサ状態保持用             */
int  			Center;					/*カメラセンター	0~127				*/
int			    Wide;					/*白線幅			0~127			*/
int  			Center_old;					/*カメラセンター				*/
int			    Wide_old;					/*白線幅						*/
int  			Center_offset = 0;		/*カメラセンターを移動＝寄せる				*/
int 			angle_y = 0;				/* ジャイロセンサーのY軸の値	*/
int 			angle_x = 0;				/* ジャイロセンサーのX軸の値	*/
int 			pre_angle_y[2] = {0};		/* ジャイロセンサーのY軸の過去の値	*/
int 			pre_angle_x[2] = {0};		/* ジャイロセンサーのX軸の過去の値	*/
int 			Center_IR = 0;
char 			IR_flag = 0;
int 			IR_max[2] = {0};
int				IR_min[2] = {1024,1024};
int 			IR_cnt = 0;
int 			IR_old = 0;


/* TRCレジスタのバッファ */
unsigned int    trcgrb_buff;            /* TRCGRBのバッファ             */
unsigned int    trcgrd_buff;            /* TRCGRDのバッファ             */

/* モータドライブ基板TypeS Ver.3上のLED、ディップスイッチ制御 */
unsigned char   types_led;              /* LED値設定                    */
unsigned char   types_dipsw;            /* ディップスイッチ値保存       */

/*	パラメータ	*/
//オフセット
int  		Center_offset_MAX = 5;		/*カーブ時カメラセンターを移動＝寄せる 最小値 0 	*/
int  		Center_offset_Angle = -3;	/*この値につき１ＩＮ側に寄せる	正：IN　負：OUT		*/

int 		Cu_Max_Angle = 200;		//カーブの最大角度　この値以上は曲げない

int			KASOKU = 15;

//通常

#define		MOTOR_OUT_BASE			90		//カーブ前半用　外側モーター用パラメーター 
#define		MOTOR_OUT_BASE_CH		75		//カーブ前半用　外側モーター用パラメーター クランク、ハーフの直後

#define		MOTOR_OUT_BASE_N		100		//カーブ後半用　外側モーター用パラメーター 

//////////////////////////////////////////// 0:禁止 1と-1は同じ

int		    TOPSPEED	=		50;		//直線 

//前半
int			SPEED_DOWN	=		5;		//角度によりTOPSPEEDを減速 カーブ前半 8 6
int			MOTOR_out_R	=		 1;		//外側モーター用パラメーター 1	-2
int			MOTOR_in_F	=		 4;		//内側モーター用パラメーター 	2 	1
int			MOTOR_in_R	=		 -2;		//内側モーター用パラメーター -2	-3
	
//後半
int			SPEED_DOWN_N=		8;		//角度によりTOPSPEEDを減速  カーブ後半 11 10
int			MOTOR_out_R_N=		7;		//外側モーター用パラメーター 後半	5	5
int			MOTOR_in_F_N=		8;		//内側モーター用パラメーター　後半	6	6
int			MOTOR_in_R_N=		6;		//内側モーター用パラメーター　後半	3	3


int			S_para		=		1;		//S字きりかえし用パラメータ
int			OUT_M_DOWN	=		2;		//カーブ外寄りブレーキ用倍率

#define		date_f_brake		400	//再生走行時 通常走行と同様の速度制限をする距離 400
#define		date_f_brake2		65	//再生走行時　残り距離/date_f_brake2 だけ速度上限を上げる 数値を大きくした方が遅くなる(0にはしないこと）

#define		Cu_FREE_time  		15		//カーブ終了時の後輪フリーの時間(msec）

#define		Cu_BRAKE_time  		10		//カーブ進入時のブレーキ時間 (msec)
#define		Cu_BRAKE_SP 		30		//カーブ進入時にこの速度以上ならブレーキ
#define		Cu_BRAKE			-80		//カーブ進入時のブレーキ（後輪） 
#define		Cu_BRAKE_out		 0		//カーブ進入時のブレーキ(前輪OUT） 
#define		Cu_BRAKE_Fin		-25		//カーブ進入時のブレーキ(前輪IN） 


#define		Cu_N_time			200	//Cu_N_time ms カーブを走行すると後半になる 	
//「使うな」#define		Cu_N_time_safe		80	//この時間未満の直線直後ならカーブ後半を維持する //S字で加速したいときに有効かする  「使うな」
#define		Cu_BRAKE_N			50	//カーブ前半中のブレーキの前半時間

//坂
int			S_flag = 2;				//坂道　遇数回を　1 = 無視しない  2 = 無視する
int			saka_max	  =		  1;	//認識可能な坂の数
#define 	KASA_Encoder1  	100	//坂開始	50
#define 	KASA_Encoder2  	400	//上り途中 終わり 300
#define 	KASA_Encoder3  	1500	//上り終わり 

#define		KASA_Encoder4  	2500	//坂上終わり  
#define		KASA_Encoder5  	3200	//下り終わり 通常にもどる 

#define		KASA_Encoder4_2  2500	//坂上終わり(最後の坂道)2500 3000
#define		KASA_Encoder5_2  2700	//下り終わり 通常にもどる(最後の坂道) 3000 3500


//斜面(上り)
#define		    TOPSPEED2			44		//直線(坂）30 33
#define			SPEED_DOWN2			10		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN2_N		10		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out2_R		10 		//外側モーター用パラメーター(坂）
#define			MOTOR_in2_F			10		//内側モーター用パラメーター(坂）
#define			MOTOR_in2_R			10		//内側モーター用パラメーター(坂）

#define			S_para2				2		//S字きりかえし用パラメータ(坂）
#define			OUT_M_DOWN2			2		//カーブ外寄りブレーキ用倍率(坂）

//斜面(上り,頂上付近　飛び跳ね防止)
#define		    TOPSPEED3			31		//直線(坂）30 33
#define			SPEED_DOWN3			6		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN3_N		6		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out3_R		2	//外側モーター用パラメーター(坂）
#define			MOTOR_in3_F			4		//内側モーター用パラメーター(坂）
#define			MOTOR_in3_R			1		//内側モーター用パラメーター(坂）

#define			S_para3				2		//S字きりかえし用パラメータ(坂）
#define			OUT_M_DOWN3			4		//カーブ外寄りブレーキ用倍率(坂）

//上
#define		    TOPSPEED4			50		//直線(坂上）30 33
#define			SPEED_DOWN4			6		//角度によりTOPSPEEDを減速(坂上）カーブ前半
#define			SPEED_DOWN4_N		6		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out4_R		1		//外側モーター用パラメーター(坂上)
#define			MOTOR_in4_F			4		//内側モーター用パラメーター(坂上）
#define			MOTOR_in4_R			-2		//内側モーター用パラメーター(坂上）

#define			S_para4				2		//S字きりかえし用パラメータ(坂上）
#define			OUT_M_DOWN4			4		//カーブ外寄りブレーキ用倍率(坂上）

//斜面(下り)
#define		    TOPSPEED5			50		//直線(坂）30 33
#define			SPEED_DOWN5			8		//角度によりTOPSPEEDを減速(坂）カーブ前半
#define			SPEED_DOWN5_N		9		//角度によりTOPSPEEDを減速  カーブ後半
#define			MOTOR_out5_R		1	//外側モーター用パラメーター(坂）
#define			MOTOR_in5_F			4		//内側モーター用パラメーター(坂）
#define			MOTOR_in5_R			-2		//内側モーター用パラメーター(坂）

#define			S_para5				2		//S字きりかえし用パラメータ(坂）
#define			OUT_M_DOWN5			2		//カーブ外寄りブレーキ用倍率(坂）


//クランク、ハーフ直後の設定値	カーブのパラメータ変更がクランク、ハーフに影響しないようにするため
#define		    TOPSPEED_CH_Len		600		//クランク、ハーフ直後のこの距離未満は以下の設定値で走る

#define		    TOPSPEED_CH			50		//直線
#define			SPEED_DOWN_CH		 6		//角度によりTOPSPEEDを減速  カーブ前半
#define			SPEED_DOWN_CH_N		 14		//角度によりTOPSPEEDを減速  カーブ後半

#define			MOTOR_out_CH_R		 1		//外側モーター用パラメーター
#define			MOTOR_in_CH_F		 4		//内側モーター用パラメーター
#define			MOTOR_in_CH_R		-2		//内側モーター用パラメーター

#define			S_para_CH			1		//S字きりかえし用パラメータ
#define			OUT_M_DOWN_CH		2		//カーブ外寄りブレーキ用倍率

//クランク
int		    C_TOPSPEED	=		30;		//クランク(入)  25 33
int		    C_TOPSPEED2	=		50;		//クランク(出)	40

int 		C_TOPSPEED4 = 		47;		//再生走行時のブレーキ前
int		    C_TOPSPEED3	=		40;		//再生走行用クランク(入)  25 33 
int 		C_TOPSPEED5 = 		40;		//再生走行時のブレーキ前 距離短い時用

int 		C_short_len_boost = 600;  //再生走行　距離短い時用

int			C_short_len =		590;	//この距離未満はショート、以上はロング
#define		C_TOPSPEED_SHORT	2		//(iEncoder10 > C_TOPSPEED + C_TOPSPEED_SHORT )のとき　＝減速できていない場合はショート
#define		C_TOPSPEED_SHORT_NG	1		//(iEncoder10 < C_TOPSPEED - C_TOPSPEED_SHORT_NG )のとき　＝距離はショートでも速度が遅いときはロング

int			date_f_brake_c	=	580;	//再生走行時のブレーキ使用可能距離(mm) クランク用
int			date_f_shortcat_c=	260;	//再生走行時のショートカット距離(mm) クランク用 210

char		c_cut_master  	 =	  1;	//再生走行時であっても 0= 再生しない 1= 再生する 			
int			c_cut_encoder	 =	540;  	//この距離未満の場合は再生しない


//char saka_c_flag = 0;

//ハーフ 
//#define HWall 							//壁ありの時に有効化すること

#ifdef  HWall //壁あり
int		    H_TOPSPEED	=		45;		//ハーフ（侵入）
int		    H_TOPSPEED2	=		42;		//ハーフ(斜め)
#else //壁なし
int		    H_TOPSPEED	=		50;		//ハーフ（侵入）
int		    H_TOPSPEED2	=		45;		//ハーフ(斜め) ブレーキがかからないように値を高くする
#endif
  
int		    H_TOPSPEED2_S=		50;		//ハーフ(斜め)  ショートカット用
int			date_f_brake_h	=	700;	//再生走行時のブレーキ使用可能距離(mm)　ハーフ用 
int			date_f_shortcat_h=	450;		//再生走行時のショートカット距離(mm)　ハーフ用

int			date_f_plus_h	=	300;		//再生走行時の直後のストレート距離補正(mm)　ハーフ用  

#ifdef  HWall
char		h_cut 			 =	  0;	//壁ありの時はショートカットしない
#else
char		h_cut 			 =	  1;	//再生走行時であっても 0= 再生しない 1= 再生する
#endif
///////////////////////////////////

#define			BRAKE_MAX			-100	//ブレーキの最大パワー 


int				kp = -18;//- 8  3 -16 -19 -23  -13
int				kd = -105;//-80 20 -130 -190	-110
int 			ki = 0;//-2

int				kp_ir = -20;
int				kd_ir = -110;
int 			ki_ir = 0;


int				kp2 = -10;//角度指定用
int				kd2 = -50;//角度指定用


/*	その他	*/
int			topspeed;		
int			speed_down;
int			speed_down_n;
int			motor2_out_R;
int			motor2_in_F;
int			motor2_in_R;
int			s_para;
int			out_m_down;
int			MOTOR_out_base = MOTOR_OUT_BASE;

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
    msdStartAddress = 5120;

    // microSD 書き込み終了アドレス
    // 書き込みしたい時間[ms] : x = 10[ms] : 64バイト
    // 60000msなら、x = 60000 * 64 / 10 = 384000
    // 結果は512の倍数になるように繰り上げする。
    msdEndAddress  = 768000;//384000;
    msdEndAddress += msdStartAddress;   /* スタート分足す               */                  

	 /* microSD初期化 */
    ret = initMicroSD();
    if( ret != 0x00 ) {
        msdError = 1;
		printf("%d\n",ret);
        /* 初期化できなければ3秒間、LEDの点灯方法を変える */
        cnt1 = 0;
        while( cnt1 < 3000 ) {
            if( cnt1 % 200 < 100 ) {
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
	
	topspeed = TOPSPEED;		//初期値
	speed_down = SPEED_DOWN;
	speed_down_n = SPEED_DOWN_N;
	motor2_in_F = MOTOR_in_F;
	motor2_in_R = MOTOR_in_R;
	motor2_out_R = MOTOR_out_R;
	s_para = S_para;
	out_m_down = OUT_M_DOWN;
	
	wait(10);
	while(~p8 == 0xff);//起動直後は数値がおかしい
	wait(10);
	
	iAngle0 = 0;
	iAngle0 = getServoAngle();  /* 0度の位置記憶                */
	types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
	

	//加速度センサー値の初期化
	angle_y = ad0;
	angle_x = ad1;
	
	
	/* スタート時、マイコンディップスイッチ 1 = ON   再生モード　IN */
	if( (dipsw_get() & 0x01) == 0x01 ) {
		date_f_mode = 1;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
	
	/* スタート時、マイコンディップスイッチ 2 = ON   再生モード　OUT */
	}else if( (dipsw_get() & 0x02) == 0x02 ) {
		date_f_mode = 2;
		
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
        pattern = 101;
        cnt1 = 0;	
			
	// スタート時、ディップスイッチ 4 = ON   遇数回を　1 = 無視しない  2 = 無視する
/*	}else if( dipsw_get2() == 0x10 ) {
		S_flag = 1;
				
	// スタート時、ディップスイッチ 3 = ON   RXとの通信チェックモード
	}else if( dipsw_get2() == 0x20) {
		pattern = 500;
*/	
	}
 


    while( 1 ) {
		
//	I2CEepromProcess();                 /* I2C EEP-ROM保存処理          */

	mode_out();//坂フラグを出力
	
	//値の保存
	Center_old = Center;
	Wide_old = Wide;
	
	Get_Center_IR();//赤外線センサー
	cam_in();//値の取得

	
	if(Wide == 0 && mode != 1){//黒だったら && 坂でない
		if(black_flag == 1){//前回も黒
			
		}else{
			Center = Center_old;
			Wide = Wide_old;
		}
		black_flag = 1;
	}else{
		black_flag = 0;
	}
	
	if(pattern >= 10){
		led_out(0);
	}else if(pattern > 1){
		led_out(camera(Center,Wide));
	}
	
	//脱線チェック
	if(10 <= pattern && pattern < 100){
		if(((check_wideline()) || (iEncoder10 < 3) || (mode != 1 && Wide == 0 && Center == 0  && pattern != 53 && pattern != 63 && pattern != 31 && pattern != 41 && pattern != 32 && pattern != 42)) && (pattern >= 10) && (pattern != 70)){
			out_cnt ++;		
		}else{
			out_cnt = 0;
		}
		
		if((lEncoderTotal > 500) && ((pattern != 22 && out_cnt > 500) || (pattern != 22 && out_cnt > 2000 ) )){
			pattern = 200;	
		}
	}
	
    switch( pattern ) {	
    case 0:
        
		/* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			iAngle0 = 0;
			iAngle0 = getServoAngle();  /* 0度の位置記憶                */
			
			IRcalibration( );
			
			/*
 			while(1){
				printf("%4d  %4d  %4d -- \t",IR_L(), IR_R(),IR_R() - IR_L());
				printf("%4d  %4d \n",ad6 ,ad7 );				
			}*/
			
            setBeepPatternS( 0xcc00 );
            cnt1 = 0;
            pattern = 5;
            break;
        }
		
        led_out(camera(Center,Wide));
		
        break;
	case 5:
		/* プッシュスイッチ押下待ち */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			iAngle0 = 0;
			iAngle0 = getServoAngle();  /* 0度の位置記憶                */
			
			
			ret = eraseMicroSD( msdStartAddress, msdEndAddress-1 );
            if( ret != 0x00 ) {
                /* イレーズできず */
                msdError = 2;
            }
            /* microSDProcess開始処理 */
            ret = microSDProcessStart( msdStartAddress );
            if( ret != 0x00 ) {
                /* 開始処理できず */
                msdError = 3;
            }
			
			if(date_f_mode == 1){
				readDataFlash( 0x3000, date_f_buff, 32 );
				readDataFlash( 0x3400, date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					date_f_buff_int[j] = (int)(date_f_buff[i])*100 + (int)date_f_buff[i+1];
					j++;	
				}
				
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					date_f_buff_ch_int[j] =(int)date_f_buff_ch[i];
					j++;
					
					date_f_buff_ch_int[j] = (int)(date_f_buff_ch[i+1])*100 + (int)date_f_buff_ch[i+2];
					j++;
				}

			
			/*	
					//31：右クランク 41：左クランク 53:左ハーフ 63:右ハーフ
					//クランク　金具　近い：200　遠い：450
					//ハーフ　金具　350
					 
				date_f_buff_ch_int[0] = 63;
				date_f_buff_ch_int[2] = 31;
				date_f_buff_ch_int[4] = 41;
				date_f_buff_ch_int[6] = 31;
				date_f_buff_ch_int[8] = 53;
			
				date_f_buff_ch_int[1] = 1200;
				date_f_buff_ch_int[3] = 580;
				date_f_buff_ch_int[5] = 500;
				date_f_buff_ch_int[7] = 880;
				date_f_buff_ch_int[9] = 700;
				
				//直線
				date_f_buff_int[0] = 1800;
				date_f_buff_int[1] = 1000;
				date_f_buff_int[2] = 1500;
				date_f_buff_int[3] = 1300;
				date_f_buff_int[4] = 3200;
				date_f_buff_int[5] = 2800;
				date_f_buff_int[6] = 1600;
				date_f_buff_int[7] = 1600;
				date_f_buff_int[8] = 2000;
				date_f_buff_int[9] = 4500;
				date_f_buff_int[10] = 1000;
				
			
				
				*/
			}else if(date_f_mode == 2){
				readDataFlash( 0x3800, date_f_buff, 32 );
				readDataFlash( 0x3c00, date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					date_f_buff_int[j] = (int)(date_f_buff[i])*100 + (int)date_f_buff[i+1];
					j++;	
				}
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					date_f_buff_ch_int[j] =(int)date_f_buff_ch[i];
					j++;
					
					date_f_buff_ch_int[j] = (int)(date_f_buff_ch[i+1])*100 + (int)date_f_buff_ch[i+2];
					j++;
				}
		
				
				/*
					//31：右クランク 41：左クランク 53:左ハーフ 63:右ハーフ
					//クランク　金具　近い：200　遠い：450
					//ハーフ　金具　350
					 
				date_f_buff_ch_int[0] = 41;
				date_f_buff_ch_int[2] = 31;
				date_f_buff_ch_int[4] = 53;
				date_f_buff_ch_int[6] = 63;
			    date_f_buff_ch_int[8] = 31;
			
				date_f_buff_ch_int[1] = 500;
				date_f_buff_ch_int[3] = 880;
				date_f_buff_ch_int[5] = 700;
				date_f_buff_ch_int[7] = 1200;
				date_f_buff_ch_int[9] = 580;
				
				//直線
				date_f_buff_int[0] = 1600;
				date_f_buff_int[1] = 1600;
				date_f_buff_int[2] = 1600;
				date_f_buff_int[3] = 2000;
				date_f_buff_int[4] = 4500;
				date_f_buff_int[5] = 1000;
				date_f_buff_int[6] = 1500;
				date_f_buff_int[7] = 1300;
				date_f_buff_int[8] = 3200;
				date_f_buff_int[9] = 2800;
				date_f_buff_int[10] = 1600;
				*/
			}	
			
			
			date_f_num_ch = 0;
			
            setBeepPatternS( 0xcc00 );
            cnt1 = 0;
            pattern = 1;
			
			wait(1500);////////////////////////////////////////////////////////////////////////////
			
            break;
        }
/*		
		if(angle_check() != 1){ //坂センサーチェック
			setBeepPatternS( 0x8000 );
			
		}
*/		
        
        if( check_startgate() ) {
             if(((cnt1/100) % 8)%2 == 0)led_out( 0x81);
			 else led_out(camera(Center,Wide));
			 
        }else{
			led_out(camera(Center,Wide));
		}
		break;
    case 1:
        /* スタートバー開待ち */
		servoPwmOut( 0 );
		
		motor_f( 0, 0 );
        motor_r( 0, 0 );
			
		old_i = iAngle0;
		iAngle0 = 0;
        iAngle0 = getServoAngle();  /* 0度の位置記憶                */
		iAngle0 = (iAngle0 + old_i) >> 1;
		
        if(  (!check_startgate()) && (Wide != 0)) {//ゲートが消えた　&& ラインが見えた
		//	iAngle0 = 0;
        //    iAngle0 = getServoAngle();  /* 0度の位置記憶                */
			//if(cnt1 > 1000)gate = 1;
			gate = 1;
            cnt1 = 0;
			cnt2 = 0;
			cnt7 = 0;
			cnt9 = 0;
			run = 1;//走行開始
			lEncoderTotal = 0; 
			lEncoderTotal2 = 0; 
			sp = 0;
			sp2 = 0;
			
			flag2 = 0;//坂道の回数
			
			msdBuffAddress = 0;
            msdWorkAddress = msdStartAddress;
            msdFlag = 1;                /* データ記録開始               */
		
            pattern = 10;
            break;
			
        }
		
		/*if( pushsw_get() ) {//ボタンが押された ゲートに近づいてスタートするモード
			setBeepPatternS( 0x8000 );
			wait(1000);
			pattern = 2;
            break;
		}*/
        led_out( 1 << (cnt1/50) % 8 );
        break;
	/*
	case 2:
		if(  (!check_startgate()) && (Wide != 0)) {//ゲートが消えた　&& ラインが見えた
			setBeepPatternS( 0x8000 );
			wait(1000);
			pattern = 3;
            break;
		}
		break;
		
	case 3:

		old_i = iAngle0;
		iAngle0 = 0;
        iAngle0 = getServoAngle();  // 0度の位置記憶                
		iAngle0 = (iAngle0 + old_i) >> 1;
		
		//if(Wide_old != 0 && Wide - Wide_old > 6){//ラインが太くなった＝ゲートが見えた
		//if(Wide > 20){//ラインが太くなった＝ゲートが見えた
		 if( check_startgate() ) {
            pattern = 4;
		}
		break;
	
	case 4:

		
		if(Wide < 22){//ラインが消えた
			
            cnt1 = 0;
			cnt2 = 0;
			cnt7 = 0;
			run = 1;//走行開始
			lEncoderTotal = 0; 
			lEncoderTotal2 = 0; 
			sp = 0;
			sp2 = 0;
			
			flag2 = 0;//坂道の回数
			
			msdBuffAddress = 0;
            msdWorkAddress = msdStartAddress;
            msdFlag = 1;                // データ記録開始               
			gate = 1;
            pattern = 10;
		}
		break;
*/
	case 10://スタート直後
		
		if(Center < -10)Center = -10;
		if(Center > 10)Center = 10;
		
		old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;
		
		//if(lEncoderTotal < 200 && Wide > 20){//走行開始直後は直線 && ゲート見えてる
		if(lEncoderTotal < 200){// && gate == 1 ){// && Wide > 28){//走行開始直後は直線 && ゲート見えてる
			mode = 1;//視野を狭くする
			
			iSetAngle = 0;
			servoPwmOut( iServoPwm2 );		
		}else{//通常
			mode = 0;	
			servoPwmOut( iServoPwm / 2 );
		}
		
		if( iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)) ) {// エンコーダによりスピード制御 
			x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*2;
				
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
	
		}else{
			motor_f(100 , 100 );
         	motor_r(100 , 100 );
		}
		
		
		if(lEncoderTotal > 700){
			
			mode = 0;//元に戻す
			
			pattern = 11;
			cnt8 = 0;
		}
		break;
		
    case 11:
        /* 通常トレース */
		 
		 old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;
		
		if(mode != 1 && Wide == 0){//黒だったら
			Center = Center_old;
			Wide = Wide_old;
		}
		
		if(((Center - Center_old) < -15 || 15 < (Center - Center_old)) && ( Wide < 30)){//急にラインが変化したら
			Center = Center_old;
			Wide = Wide_old;
		}
		
		if(mode == 1
		 && Wide > 28){//坂中 && 幅が太い
			Center = Center_old;
			Wide = Wide_old;	
		}
		
	
		if(mode == 1){
			if(IR_flag == 0){//赤外線に切り替え
				if(((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (-5 < Center_IR  && 	Center_IR < 5) || (KASA_Encoder2 <= (lEncoderTotal-sp2)) )){
					//if(KASA_Encoder1 <= (lEncoderTotal-sp2) && (lEncoderTotal-sp2) < KASA_Encoder2 + 300){
					if(KASA_Encoder1 <= (lEncoderTotal-sp2)){
						IR_cnt++;
						if(IR_cnt > 10){
							IR_flag = 1;
							IR_cnt = 0;
						}	
					}
				}
			}else{//カメラに切り替え
				if((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (	-5 < Center_IR  && 	Center_IR < 5)){
				//	if(KASA_Encoder4 <= (lEncoderTotal-sp2)){
					
					if(((saka_max > 0) && ((lEncoderTotal-sp2) >= KASA_Encoder4)) || 
						((saka_max == 0) && ((lEncoderTotal-sp2) >= KASA_Encoder4_2))  ){
					
						IR_cnt++;
						if(IR_cnt > 10){
							IR_flag = 0;
							IR_cnt = 0;
						}	
					}
				}
			}
		}else{
			if(IR_flag == 1){//坂以外で赤外線モードならカメラに切り替え
				if((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (	-5 < Center_IR  && 	Center_IR < 5)){
					
					IR_cnt++;
					if(IR_cnt > 20){
						IR_flag = 0;
						IR_cnt = 0;
						mode = 0;
					}	
				}
			}
		}	
		
		
		if(cnt2 >=  MAXTIME * 10){//走行時間終了
			pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			break;
		}
		
		
//if(flag2 == 0){//２回目は絶対に検出しない　一時的な対策。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。。		
		if(-15 < i && i < 15){
			if(mode == 0){//通常
				//if(angle_check() == 2 && ( (flag2%2 == 1) || ((lEncoderTotal-sp2) >= 1000) && ((lEncoderTotal-sp) >= 1000) )){//坂センサーチェック sp=クランク終了位置
				//if(angle_check() == 2 && ( ((lEncoderTotal-sp2) >= 600) && ((lEncoderTotal-sp) >= 1000) )){//坂センサーチェック sp=クランク終了位置
				if(angle_check() == 2 &&  (((lEncoderTotal-sp2) >= 600) || ((flag2%2 == 1) && ((lEncoderTotal-sp2) >= 300))) && ((lEncoderTotal-sp) >= 1000) ){//坂センサーチェック sp=クランク終了位置
					cnt5++;
				}else{
					cnt5 = 0;
				}
			}else{
				cnt5 = 0;
			}
		}
//}		
		if(-25 < i && i < 25){
			if(mode == 0){//坂中でなければ
				if(lEncoderTotal > 200 && (sp2 == 0 || (lEncoderTotal-sp2) >= 0) && (sp == 0 || (lEncoderTotal-sp) >= 0) && (sp3 == 0 || (lEncoderTotal-sp3) >= 0)){//ゲートに反応しないように && 坂終了から少しの間は無視 && クランク、ハーフ終了後少し無視 && カーブ直後は無視
				
				//if(lEncoderTotal > 200 && (sp2 == 0 || (lEncoderTotal-sp2) >= 1500) && (sp == 0 || (lEncoderTotal-sp) >= 150) && (sp3 == 0 || (lEncoderTotal-sp3) >= 0)){//ゲートに反応しないように && 坂終了から少しの間は無視 && クランク、ハーフ終了後少し無視 && カーブ直後は無視
				
				//if(lEncoderTotal > 200 ){//ゲートに反応しないように 
				
					if(date_f_mode == 0){
						if( check_crossline() ) {       // クロスラインチェック         
            				cnt1 = 0;
            				pattern = 21;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
				
        				}else if(check_halfline() == 1){//左レーンチェンジチェック
							cnt1 = 0;
            				pattern = 51;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
				
						}else if(check_halfline() == 2){//右レーンチェンジチェック
							cnt1 = 0;
        	    			pattern = 61;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
						}
					}else{
						//if( check_wideline() == 1) {       // 線幅が太くなったら      
						if( (check_crossline() || check_halfline() != 0 ||  check_wideline() == 1) && iEncoder10 < 60){ 
							if(date_f_buff_ch_int[date_f_num_ch] == 31 || date_f_buff_ch_int[date_f_num_ch] == 41) {       // クロスラインチェック         
            					cnt1 = 0;
            					pattern = 21;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								
								
								if(c_cut_master == 0){
									c_cut = 0;
								}else{
									if(date_f_buff_ch_int[date_f_num_ch] < c_cut_encoder)c_cut = 0;
									else c_cut = 1;
								}
						
								break;
				
        					}else if(date_f_buff_ch_int[date_f_num_ch] == 53){//左レーンチェンジチェック
								cnt1 = 0;
            					pattern = 51;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								break;
				
							}else if(date_f_buff_ch_int[date_f_num_ch] == 63){//右レーンチェンジチェック
								cnt1 = 0;
        		    			pattern = 61;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								break;
							}
						}
					}
				}
			}
		}
		
		if(mode == 0){//通常
		
			if(cnt5 >= 5){//坂
			
				if(flag2 % S_flag == 0 && saka_max > 0){

					saka_max--;
					mode = 1;//坂モードに
					sp2 = lEncoderTotal;

				}
				flag2++;
				cnt5 = 0;
				
				sp2 = lEncoderTotal;//チャタリング防止
				
			}
		
		}else if(mode == 1){//坂

			if(((saka_max > 0) && ((lEncoderTotal-sp2) >= KASA_Encoder5)) || 
				((saka_max <= 0) && ((lEncoderTotal-sp2) >= KASA_Encoder5_2))  ){//通常に戻す
			
				mode = 0;
				
				sp2 = lEncoderTotal;//チャタリング防止
				
				TOPSPEED = topspeed;
				SPEED_DOWN = speed_down;
				SPEED_DOWN_N = speed_down_n;
				MOTOR_out_R = motor2_out_R;
				MOTOR_in_F = motor2_in_F;
				MOTOR_in_R = motor2_in_R;
				S_para = s_para;
				OUT_M_DOWN = out_m_down;
				
			}else if(((saka_max > 0) && ((lEncoderTotal-sp2) >= KASA_Encoder4)) || 
				((saka_max <= 0) && ((lEncoderTotal-sp2) >= KASA_Encoder4_2))  ){// 下り開始
				
				
				TOPSPEED = TOPSPEED5;
				SPEED_DOWN = SPEED_DOWN5;
				SPEED_DOWN_N = SPEED_DOWN5_N;
				MOTOR_out_R = MOTOR_out5_R;
				MOTOR_in_F = MOTOR_in5_F;
				MOTOR_in_R = MOTOR_in5_R;
				S_para = S_para5;
				OUT_M_DOWN = OUT_M_DOWN5;
				
			}else if((lEncoderTotal-sp2) >= KASA_Encoder3){// 坂上
				TOPSPEED = TOPSPEED4;
				SPEED_DOWN = SPEED_DOWN4;
				SPEED_DOWN_N = SPEED_DOWN4_N;
				MOTOR_out_R = MOTOR_out4_R;
				MOTOR_in_F = MOTOR_in4_F;
				MOTOR_in_R = MOTOR_in4_R;
				S_para = S_para4;
				OUT_M_DOWN = OUT_M_DOWN4;
			
			}else if((lEncoderTotal-sp2) >= KASA_Encoder2){//速度をさらに遅く 坂上ジャンプ防止
				TOPSPEED = TOPSPEED3;
				SPEED_DOWN = SPEED_DOWN3;
				SPEED_DOWN_N = SPEED_DOWN3_N;
				MOTOR_out_R = MOTOR_out3_R;
				MOTOR_in_F = MOTOR_in3_F;
				MOTOR_in_R = MOTOR_in3_R;
				S_para = S_para3;
				OUT_M_DOWN = OUT_M_DOWN3;
			
			
			}else if((lEncoderTotal-sp2) >= KASA_Encoder1){//速度を遅く 上り開始
				TOPSPEED = TOPSPEED2;
				SPEED_DOWN = SPEED_DOWN2;
				SPEED_DOWN_N = SPEED_DOWN2_N;
				MOTOR_out_R = MOTOR_out2_R;
				MOTOR_in_F = MOTOR_in2_F;
				MOTOR_in_R = MOTOR_in2_R;
				S_para = S_para2;
				OUT_M_DOWN = OUT_M_DOWN2;
				
				//if( (i < -20 || 20 < i) && (lEncoderTotal-sp2) >= KASA_Encoder1+500){//上っている途中でカーブはありえない
		/*		if( i < -20 || 20 < i){//上っている途中でカーブはありえない
					TOPSPEED = topspeed;
					SPEED_DOWN = speed_down;
					SPEED_DOWN_N = speed_down_n;
					MOTOR_out_R = motor2_out_R;
					MOTOR_in_F = motor2_in_F;
					MOTOR_in_R = motor2_in_R;
					S_para = s_para;
					OUT_M_DOWN = out_m_down;
					mode = 0;
					
					flag2--;//今回の分を無かったことに
					saka_max++;
					sp2 = lEncoderTotal;//チャタリング防止
				}*/
			}	
		}
		
		 
		
		if(mode == 1){
			if(((lEncoderTotal-sp2) >= KASA_Encoder1) && (lEncoderTotal-sp2) <= KASA_Encoder2)servoPwmOut( iServoPwm /2);
			else servoPwmOut( iServoPwm );
		}else{
			if(lEncoderTotal > 1000 && (lEncoderTotal-sp) < TOPSPEED_CH_Len ){//クランク、ハーフ直後はカーブの最大角度は設定しない
				servoPwmOut( iServoPwm );
				
			}else{
				if(Cu_Max_Angle < i && (Center + Center_offset) > 0){ //カーブの最大角度を超えている
					iSetAngle = Cu_Max_Angle;
					servoPwmOut( iServoPwm2 );
				
				}else if( i < -Cu_Max_Angle && (Center + Center_offset) < 0){//カーブの最大角度を超えている
				
					iSetAngle = -Cu_Max_Angle;
					servoPwmOut( iServoPwm2 );
					
				}else{
					servoPwmOut( iServoPwm );
				}
			}
		}
		
		if(mode != 1){//坂中の設定が上書きされないようにする
			if(lEncoderTotal > 1000 && (lEncoderTotal-sp) < TOPSPEED_CH_Len ){//クランク、ハーフ直後は設定値を変える
				MOTOR_out_base = MOTOR_OUT_BASE_CH;
			
				TOPSPEED = TOPSPEED_CH;
				SPEED_DOWN = SPEED_DOWN_CH;
				SPEED_DOWN_N = SPEED_DOWN_CH_N;
				MOTOR_out_R = MOTOR_out_CH_R;
				MOTOR_in_F = MOTOR_in_CH_F;
				MOTOR_in_R = MOTOR_in_CH_R;
				S_para = S_para_CH;
				OUT_M_DOWN = OUT_M_DOWN_CH;
				
			}else{
				MOTOR_out_base = MOTOR_OUT_BASE;
			
				TOPSPEED = topspeed;
				SPEED_DOWN = speed_down;
				SPEED_DOWN_N = speed_down_n;
				MOTOR_out_R = motor2_out_R;
				MOTOR_in_F = motor2_in_F;
				MOTOR_in_R = motor2_in_R;
				S_para = s_para;
				OUT_M_DOWN = out_m_down;
			}
		}
		
        if(  i > 12 ){//ハンドル右
		
			if(mode != 1){
				Center_offset = (i-12) / Center_offset_Angle ;//カーブで寄せる
				if(Center_offset > Center_offset_MAX )Center_offset = Center_offset_MAX;
				if(Center_offset < -Center_offset_MAX )Center_offset = -Center_offset_MAX;
			}else{
				Center_offset = 0;
			}
			
			
			if((i > 8)&& (i - old_i > 0) && (Cu_flag == 0)){//直線からカーブへ 
			
				if(cnt7 >= 30 && (lEncoderTotal-sp) >= 100  && (iEncoder10 > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					cnt7 = 0;
				}
				Cu_flag = 1;
				
#ifdef Cu_N_time_safe
				if(cnt9 <= Cu_N_time_safe && (lEncoderTotal-sp) >= 1000){//直線距離が短かった && クランク、ハーフ直後は無し
					cnt8 += Cu_N_time;//強制的にカーブ後半へ	
				}
#endif
			
			}
			
			sp3 = lEncoderTotal;//カーブ終了位置用
			cnt9 = 0;
				
			if(cnt7 <= Cu_BRAKE_time && (lEncoderTotal-sp) >= 100 && (lEncoderTotal-sp2) >= 100){//カーブ進入時のブレーキ
				
				servoPwmOut( iServoPwm * 2 );
				
				motor_f( Cu_BRAKE_out , Cu_BRAKE_Fin );
            	motor_r( Cu_BRAKE , Cu_BRAKE );
				
			}else if(cnt8 <= Cu_N_time && iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)) && (lEncoderTotal-sp) >= 200) {// エンコーダによりスピード制御  カーブ前半
			
				if(mode == 1 && (lEncoderTotal-sp2) <= KASA_Encoder3 - 300){//坂登り中のブレーキは全輪均等にする
					x = (TOPSPEED -iEncoder10)*10;
					if(x < BRAKE_MAX) x = BRAKE_MAX;
					r = x;
					f = x;
				}else if(cnt8 <= Cu_BRAKE_N && mode != 1){//ブレーキ前半 && 坂モードでなければ
				
					x = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*5;	
					r = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*40;
					f = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*20;
				
					//if(x < BRAKE_MAX) x = BRAKE_MAX;
					if(x < -10) x = -10;
					if(r < -80) r = -80;
					if(f < -25) f = -25;
				}else{
					x = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*2;	
					r = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*5;
					f = ((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*2;
				
					//if(x < BRAKE_MAX) x = BRAKE_MAX;
					if(x < -5) x = -5;
					if(r < -15) r = -15;
					if(f < -15) f = -15;	
				}
				motor_f( x, f );
            	motor_r( r, r );			
			
			}else if(iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN_N)) && (lEncoderTotal-sp) >= 200 ) {// エンコーダによりスピード制御  カーブ後半
			
				x=((TOPSPEED -(i / SPEED_DOWN_N))-iEncoder10)*2;	
				r = x;
				f = x;
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( x, r );
            	motor_r( f, r );	
/*					
			}else if(Center + Center_offset < - 10 ) {//きりかえし
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((i / MOTOR_in_F) * S_para));
				else f = (MOTOR_out_base - ((i * -MOTOR_in_F) * S_para));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R > 0 )r = (MOTOR_out_base - ((i / MOTOR_in_R) * S_para));
				else r = (MOTOR_out_base - ((i * -MOTOR_in_R) * S_para));
				if(r < 0) r = 0;
				
				
				if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
				else or = (MOTOR_out_base - (i * -MOTOR_out_R));
				
				if(or < 0) or = 0;
				
				motor2_f(f,MOTOR_out_base);
				//motor_r(x,99);
				
				motor2_r(or,r);
					
			}else if((Center  + Center_offset > 10) || (i >= 75 && cnt8 <= Cu_N_time) || (i >= 118)) {//外寄り
				
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((i / MOTOR_in_F) * OUT_M_DOWN));
				else f = (MOTOR_out_base - ((i * -MOTOR_in_F) * OUT_M_DOWN));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R > 0)r = (MOTOR_out_base - ((i / MOTOR_in_R) * OUT_M_DOWN));
				else r = (MOTOR_out_base - ((i * -MOTOR_in_R) * OUT_M_DOWN));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
				else or = (MOTOR_out_base - (i * -MOTOR_out_R));
				
				if(or < 0) or = 0;
					
				motor2_f( MOTOR_out_base-10, f);
				motor2_r( or, r);
*/					
        	}else{
				
				if((cnt8 <= Cu_N_time) || (mode == 1) || (i > 95) ){//カーブ前半 || 坂モード || 曲げすぎ || 
					if(MOTOR_in_F > 0)f= (MOTOR_out_base - (i / MOTOR_in_F));
					else f= (MOTOR_out_base - (i * -MOTOR_in_F));
				
					if(MOTOR_in_R > 0)r= (MOTOR_out_base - (i / MOTOR_in_R));
					else r= (MOTOR_out_base - (i * -MOTOR_in_R));


					if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
					else or = (MOTOR_out_base - (i * -MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -10) r = -10;
					if(or < 0) or = 0;
					if(f > MOTOR_out_base) f = MOTOR_out_base;
					if(r > MOTOR_out_base) r = MOTOR_out_base;
					if(or > MOTOR_out_base) or = MOTOR_out_base;
				
					motor2_f( MOTOR_out_base , f);
           			motor2_r( or, r);
					
				}else{//カーブ後半

					if(MOTOR_in_F_N > 0)f= (MOTOR_OUT_BASE_N - (i / MOTOR_in_F_N));
					else f= (MOTOR_OUT_BASE_N - (i * -MOTOR_in_F_N));
				
					if(MOTOR_in_R_N > 0)r= (MOTOR_OUT_BASE_N - (i / MOTOR_in_R_N));
					else r= (MOTOR_OUT_BASE_N - (i * -MOTOR_in_R_N));
				
					if(MOTOR_out_R_N > 0)or = (MOTOR_OUT_BASE_N - (i / MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (i * -MOTOR_out_R_N));
					
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
			 		 	 
		}else if( i < -12 ){//ハンドル左
			
		
			if(mode != 1){
				Center_offset = (i+12) / Center_offset_Angle ;//カーブで寄せる
				if(Center_offset > Center_offset_MAX )Center_offset = Center_offset_MAX;
				if(Center_offset < -Center_offset_MAX )Center_offset = -Center_offset_MAX;
			}else{
				Center_offset = 0;
			}
			
			
			if(( i < -8) && (i - old_i < 0) && (Cu_flag == 0)){//直線からカーブへ 
				
				if(cnt7 >= 30 && (lEncoderTotal-sp) >= 100  && (iEncoder10 > Cu_BRAKE_SP)){//あまり直線を走っていない時はブレーキしないように && クランクなどの直後は無視
					cnt7 = 0;
				}
				
				Cu_flag = 1;

#ifdef Cu_N_time_safe			
				if(cnt9 <= Cu_N_time_safe && (lEncoderTotal-sp) >= 1000){//直線距離が短かった && クランク、ハーフ直後は無し
					cnt8 += Cu_N_time;//強制的にカーブ後半へ	
				}
#endif			
			}
			
			sp3 = lEncoderTotal;//カーブ終了位置用
			cnt9 = 0;
			
			if(cnt7 <= Cu_BRAKE_time && (lEncoderTotal-sp) >= 100 && (lEncoderTotal-sp2) >= 100){//カーブ進入時のブレーキ
				
				servoPwmOut( iServoPwm * 2);
				
				motor_f( Cu_BRAKE_Fin, Cu_BRAKE_out );
            	motor_r( Cu_BRAKE, Cu_BRAKE );
				
			}else if(cnt8 <= Cu_N_time &&  iEncoder10 >= (TOPSPEED -(-i / SPEED_DOWN)) && (lEncoderTotal-sp) >= 200 ) {  // エンコーダによりスピード制御 カーブ前半
				
				if(mode == 1 && (lEncoderTotal-sp2) <= KASA_Encoder3 - 300){//坂登り中のブレーキは全輪均等にする
					x = (TOPSPEED -iEncoder10)*10;
					if(x < BRAKE_MAX) x = BRAKE_MAX;
					r = x;
					f = x;
				}else if(cnt8 <= Cu_BRAKE_N && mode != 1){//ブレーキ前半 && 坂モードでなければ
					x = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*5;
					r = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*40;
					f = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*20;
							
					//if(x < BRAKE_MAX) x = BRAKE_MAX;
					if(x < -10) x = -10;
					if(r < -80) r = -80;
					if(f < -25) f = -25;
				}else{
					x = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*2;
					r = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*5;
					f = ((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*2;
							
					//if(x < BRAKE_MAX) x = BRAKE_MAX;
					if(x < -5) x = -5;
					if(r < -15) r = -15;
					if(f < -15) f = -15;
				}	
				motor_f( f, x );
            	motor_r( r, r );
				
			}else if(iEncoder10 >= (TOPSPEED -(-i / SPEED_DOWN_N)) && (lEncoderTotal-sp) >= 200) {  // エンコーダによりスピード制御 カーブ後半
			
				x=((TOPSPEED -(-i / SPEED_DOWN_N))-iEncoder10)*2;
				r = x;
				f = x;	
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( r, x );
            	motor_r( r, f );
/*				
			}else if((Center  + Center_offset < -10) || (i <= -75 && cnt8 <= Cu_N_time)|| (i <= -118) ) {//外寄り
				
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((-i / MOTOR_in_F) * OUT_M_DOWN));
				else f = (MOTOR_out_base - ((-i * -MOTOR_in_F) * OUT_M_DOWN));
				if(f < 0) f = 0;
					
				if(MOTOR_in_R >0)r = (MOTOR_out_base - ((-i / MOTOR_in_R) * OUT_M_DOWN));
				else r = (MOTOR_out_base - ((-i * -MOTOR_in_R) * OUT_M_DOWN));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
				else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
				if(or < 0) or = 0;
				
				motor2_f(f, MOTOR_out_base-10);
				motor2_r(r, or);
					
			}else if(Center  + Center_offset > 10) {//きりかえし
			
				if(MOTOR_in_F >0)f = (MOTOR_out_base - ((-i / MOTOR_in_F) * S_para));
				else f = (MOTOR_out_base - ((-i * -MOTOR_in_F) * S_para));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R >0)r = (MOTOR_out_base - ((-i / MOTOR_in_R) * S_para));
				else r = (MOTOR_out_base - ((-i * -MOTOR_in_R) * S_para));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
				else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
				if(or < 0) or = 0;
				
				motor2_f(MOTOR_out_base, f);
			//	motor_r(99, x);
			
				motor2_r(r,or);
*/					
        	}else{
				
				if((cnt8 <= Cu_N_time) || (mode == 1) || (i < -95) ){//カーブ前半 || 坂モード || 大曲
					if(MOTOR_in_F >0)f = (MOTOR_out_base - (-i / MOTOR_in_F));
					else f = (MOTOR_out_base - (-i * -MOTOR_in_F)); 
				
					if(MOTOR_in_R >0)r = (MOTOR_out_base - (-i / MOTOR_in_R));
					else r = (MOTOR_out_base - (-i * -MOTOR_in_R));
				
					if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
					else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -10) r = -10;
					if(or < 0) or = 0;
					if(f > MOTOR_out_base) f = MOTOR_out_base;
					if(r > MOTOR_out_base) r = MOTOR_out_base;
					if(or > MOTOR_out_base) or = MOTOR_out_base;
				
					motor2_f(f, MOTOR_out_base);
           			motor2_r(r, or );
					
				}else{//カーブ後半
				
					if(MOTOR_in_F_N >0)f = (MOTOR_OUT_BASE_N - (-i / MOTOR_in_F_N));
					else f = (MOTOR_OUT_BASE_N - (-i * -MOTOR_in_F_N)); 
				
					if(MOTOR_in_R_N >0)r = (MOTOR_OUT_BASE_N - (-i / MOTOR_in_R_N));
					else r = (MOTOR_OUT_BASE_N - (-i * -MOTOR_in_R_N));
				
					if(MOTOR_out_R_N >0)or = (MOTOR_OUT_BASE_N - (-i / MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (-i * -MOTOR_out_R_N));
					
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
			Center_offset = 0;
			
			cnt8 = 0;
						
			if((Cu_flag == 1)&&(mode == 0)){//カーブから直線へ && 坂中ではない
			
				if(cnt7 >= 60){//カーブを一定時間走っていたら＝あまりカーブを走っていない時（後輪滑り後）はフリーにしたくない
					cnt7 = 0;
				}
				Cu_flag = 0;
			}
			
			if(cnt7 <= Cu_FREE_time && (lEncoderTotal-sp) >= 700){//カーブ終わりから一定時間内　＆＆　クランク、ハーフからの復帰直後ではない
				
				if(mode == 0 &&  Center < -10) {//車体左寄り
				
					motor_f(98 , 100 );
				
				}else if(mode == 0 &&  Center > 10) {//車体右寄り
				
					motor_f(100 , 98 );
					
				}else{
			
					motor_f(100 , 100 );
				}
				
            	motor_r( 0, 0 );
			
			}else if( (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))) && (date_f_mode == 0 || mode == 1) ) {// エンコーダによりスピード制御 
				
				if(mode == 1){//坂
					x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*10;
				
					if(x < BRAKE_MAX) x = BRAKE_MAX;
			
					motor_f( x, x );
            		motor_r( x, x );
				}else{
					//通常ブレーキ
					motor_f( 0, 0 );
            		motor_r( 0, 0 );
				}
		//再生走行時　残り距離がdate_f_brake以下になったら通常走行の制限速度までブレーキ
			}else if(mode == 0 && date_f_mode != 0 && ((date_f_buff_int[date_f_num] - date_f_brake)< SEncoderTotal) && (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)))  ) {// エンコーダによりスピード制御 
					 
				//再生走行時　最終ブレーキ
				x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*20;
				
				if(x < BRAKE_MAX) x = BRAKE_MAX;
			
				motor_f( x, x );
            	motor_r( x, x );
					
			//再生走行時　残り距離に応じて速度制限を変更する
			}else if(mode == 0 && date_f_mode != 0 && (iEncoder10 >= max(TOPSPEED,TOPSPEED+(date_f_buff_int[date_f_num] - SEncoderTotal - date_f_brake)/date_f_brake2)) ) {// エンコーダによりスピード制御 
				
				//再生走行時　ブースト中
				x=(max(TOPSPEED,(TOPSPEED+(date_f_buff_int[date_f_num] - SEncoderTotal - date_f_brake)/date_f_brake2)) -iEncoder10)*10;
				
				if(x < -40) x = -40;
				 
				motor_f( x, x );
            	motor_r( x, x );
			
		
			}else if(mode == 0 && Center < -10) {//車体左寄り
				
				motor_f(95 , 100 );
				motor_r(100 , 100 );
			}else if(mode == 0 &&  Center > 10) {//車体右寄り
				
				motor_f(100 , 95 );
				motor_r(100 , 100 );
				
	/*		}else if(mode == 0 && date_f_mode != 0 && date_f_buff_int[date_f_num] - date_f_brake > SEncoderTotal){//ブースト
				
				motor_f(100 , 100 );
				motor_r(100 , 100 );
	*/
			}else{
				
				//if((mode == 1) &&  (Wide == 0) && ((lEncoderTotal-sp2) >= KASA_Encoder2) && ((lEncoderTotal-sp2) < KASA_Encoder3)){//坂頂上付近のとき
				if((mode == 1) && ((lEncoderTotal-sp2) >= KASA_Encoder2) && ((lEncoderTotal-sp2) < KASA_Encoder3)){//坂頂上付近のとき
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
	/*
		if((IR_L() > 120 ||  IR_R() > 120)  && saka_c_flag == 1){//坂をクランクと読み間違えた
			pattern = 11;
			sp = lEncoderTotal-5000;
			
			date_f_num_ch--;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
		}
	*/	
        /* クロスライン通過処理 */
      	//setBeepPatternS( 0x8000 );

		if(Wide > 20 || Center < -20 || Center > 20 ){
			iSetAngle = 0;
			servoPwmOut( iServoPwm2 );
		}else{
			servoPwmOut( iServoPwm  );
		}
		
		//再生走行時　　ブレーキ
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4)){
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			x=(C_TOPSPEED4-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		//クランク　ブレーキ
        }else if( (date_f_mode == 0 && iEncoder10 >= C_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED)   ) {          // エンコーダによりスピード制御 
          
			x=(C_TOPSPEED-iEncoder10)*10;
			r = x;
			
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			
			if(r < -50 && (c_cut == 0 || date_f_mode == 0)) r = -50;
			else if(r < BRAKE_MAX) r = BRAKE_MAX;
				
			
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

        if( (lEncoderTotal-sp) >= 70 ) {	
           	cnt1 = 0;
            pattern = 22;
			sp = lEncoderTotal;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
        }
        break;

    case 22:
	/*
		if( (lEncoderTotal-sp) <= 100  && saka_c_flag == 1) {
			if(IR_L() > 120 ||  IR_R() > 120){//坂をクランクと読み間違えた
				pattern = 11;
				sp = lEncoderTotal-5000;
			
				date_f_num_ch--;
			
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
			}
		}
	*/	
        /* クロスライン後のトレース、直角検出処理 */
		
		//if(c_cut == 1 && date_f_mode != 0){
		/*if(date_f_mode != 0){
			if(date_f_buff_ch_int[date_f_num_ch-1] == 31){
				
				if(Center < -14){//寄りすぎ
					Center_offset = 9;//右に寄る
					servoPwmOut( iServoPwm );
				
				}else{
					Center_offset = 12;//右に寄る
					servoPwmOut( iServoPwm );
				}
			}else{
				if(Center > 14){//寄りすぎ
					Center_offset = -9;//左に寄る
					servoPwmOut( iServoPwm );
				}else{
					Center_offset = -12;//左に寄る
					servoPwmOut( iServoPwm  );
				}
			}
			
		}else{
			servoPwmOut( iServoPwm * 1.5  );
		}*/
		servoPwmOut( iServoPwm * 1.5  );
		
		if( (lEncoderTotal-sp) >= 250 ) {
			//if((( c_cut == 0 || date_f_mode == 0) && check_halfline() == 2) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 31  && (check_wideline() == 1 || ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch))))){//右クランク
            
			if((date_f_mode == 0 && check_halfline_forC() == 2) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 31  && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch)))
						|| ((c_cut == 0 && date_f_mode != 0) && (date_f_buff_ch_int[date_f_num_ch-1] == 31) && (check_wideline() == 1))){//右クランク
            	cnt1 = 0;
				sp = lEncoderTotal;
				
				if(date_f_mode == 0){//距離計測
					date_buff_ch_int[date_num_ch++] = 31;
					date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
				}
				
				if((C_short_len > lEncoderTotal - lEncoderTotal_ch )|| (date_f_mode == 0 && (iEncoder10 > C_TOPSPEED + C_TOPSPEED_SHORT))){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき
					
					if((date_f_mode == 0 )&& (iEncoder10 < C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//距離はショートでも進入速度が遅いのでロングとする
						c_short_mode = 0;
					}else{
						c_short_mode = 1;	
					}
				}else{
					c_short_mode = 0;
				}
				
          		pattern = 31;//右クランク
				date_f_num_ch++;
				c_cut_in_flag = 0;
			
				Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(date_f_mode == 0 || c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_cut == 1 && date_f_mode != 0){
					if(iEncoder10 >= C_TOPSPEED3){
						sp -= (iEncoder10 -  C_TOPSPEED3) * 5;
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;
				
        	}
        	
			//if(((c_cut == 0 || date_f_mode == 0) && check_halfline() == 1) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 41  && (check_wideline() == 1 || ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch))))){//左クランク
            
			if((date_f_mode == 0 && check_halfline_forC() == 1) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 41  && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch)))
						|| ((c_cut == 0 && date_f_mode != 0) && (date_f_buff_ch_int[date_f_num_ch-1] == 41) && (check_wideline() == 1))){//左クランク
            	cnt1 = 0;
				sp = lEncoderTotal;
				
				if(date_f_mode == 0){//距離計測
					date_buff_ch_int[date_num_ch++] = 41;
					date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
				}
				
				if((C_short_len > lEncoderTotal - lEncoderTotal_ch) || (date_f_mode == 0 && (iEncoder10 > C_TOPSPEED + C_TOPSPEED_SHORT))){//距離が短いと減速ができていない可能性がある || 設定速度まで減速できていないとき){//距離が短いと減速ができていない可能性がある
					if((date_f_mode == 0) && (iEncoder10 < C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//距離はショートでも進入速度が遅いのでロングとする
						c_short_mode = 0;
					}else{
						c_short_mode = 1;	
					}
				}else{
					c_short_mode = 0;
				}
				
          		pattern = 41;//左クランク
				date_f_num_ch++;
				c_cut_in_flag = 0;
			
				Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(date_f_mode == 0 || c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_cut == 1 && date_f_mode != 0){
					if(iEncoder10 >= C_TOPSPEED3){
						sp -= (iEncoder10 -  C_TOPSPEED3) * 5;
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
		//if((c_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4){
		if((date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4){
			
			x=(C_TOPSPEED4-iEncoder10)*2;
			
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
			motor_f( x, x );
            motor_r( x, x );
		
		}else if( (date_f_buff_ch_int[date_f_num_ch] < C_short_len_boost) && 
				( ((date_f_mode == 0) && iEncoder10 >= C_TOPSPEED) 
				|| ((c_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED5) 
				|| ((c_cut == 0 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED))   ) {          // エンコーダによりスピード制御 
   
				if(c_cut == 0 || date_f_mode == 0)x=(C_TOPSPEED -iEncoder10)*10;
				else x=(C_TOPSPEED5 -iEncoder10)*30;
			
				r = x;
				if(r < -100 && (c_cut == 0 || date_f_mode == 0)) r = -100;
				else if(r < BRAKE_MAX) r = BRAKE_MAX;
				
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				
				motor_f( x, x );
            	motor_r( r, r );
		//	}
	
      	
 		}else if( ((date_f_mode == 0) && iEncoder10 >= C_TOPSPEED) 
				|| ((c_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED3) 
				|| ((c_cut == 0 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED)   ) {          // エンコーダによりスピード制御 
            /* 
			if((i < -20 || 20 < i) && 
				((date_f_mode != 0 && ( date_f_buff_ch_int[date_f_num_ch-1] == 31 || date_f_buff_ch_int[date_f_num_ch-1] == 41) && (lEncoderTotal-sp) >= 300) 
				|| (date_f_mode == 0))){//abs
				//motor_f( 0, 0 );
            	//motor_r( 0, 0 );
				
				if(c_cut == 0 || date_f_mode == 0)x=(C_TOPSPEED -iEncoder10)*20;
				else x=(C_TOPSPEED3 -iEncoder10)*20;
			
				r = x;
				if(r < -30) r = -30;
				
				if(x < BRAKE_MAX+20) x = BRAKE_MAX+20;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				
				motor_f( x, x );
            	motor_r( r, r );
			
			}else{*/
				if(c_cut == 0 || date_f_mode == 0)x=(C_TOPSPEED -iEncoder10)*10;
				else x=(C_TOPSPEED3 -iEncoder10)*30;
			
				r = x;
				if(r < -100 && (c_cut == 0 || date_f_mode == 0)) r = -100;
				else if(r < BRAKE_MAX) r = BRAKE_MAX;
				
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				
				motor_f( x, x );
            	motor_r( r, r );
		//	}
	
        }else{
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }
		
/*		
		if(angle_check() == 2){ //坂センサーチェック
			pattern = 11;
			cnt8 = 0;
			cnt1 = 0;
			mode = 1;//坂モードに
			sp2 = lEncoderTotal;
			flag2++;
		}
*/		
		if( lEncoderTotal-sp >= 2000 ) {//誤動作チェック
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			Center_offset = 0;
			date_f_num_ch--;
            pattern = 11;
			cnt8 = 0;
			cnt1 = 0;
        }

        break;

    case 31:
        /* 右クランク処理 */
		//setBeepPatternS( 0x8000 );
		
		old_i = i;//前回の角度を記憶
		
        i = getServoAngle();//ハンドル角度取得
		
		i = (i +old_i) >> 1;     
			
        if(date_f_mode == 0 || c_cut == 0){//通常
			mode = 3;//左無視
		
			if(c_short_mode == 1){//short
				if((lEncoderTotal-sp) >= 130){
					if(i < 95)iSetAngle = 120;
					else iSetAngle = 105;
				
				}else iSetAngle = 100;
			
				motor_f( 10, 10 );  //0,-5        /* この部分は「角度計算(4WD時).xls」 85 -40*/
        		motor_r( -5, -5 );          /* で計算                        */
			
			}else{//long
				if((lEncoderTotal-sp) >= 200){
					if(i < 95)iSetAngle = 130;
					else iSetAngle = 100;
				
				}else iSetAngle = 95;
			
				motor_f( 75,  -5 );          /* この部分は「角度計算(4WD時).xls」 85 -40*/
        		motor_r( -20,  -20 );          /* で計算                        */
			}
			
		
		}else{
			mode = 1;//見る範囲を狭く
				
			if(0 == Center && Wide == 0){
				c_cut_in_flag = 0;	
			}
				
			if(0 < Center  && (lEncoderTotal-sp) >= 150  || (Wide != 0 && -6 < Center  && (lEncoderTotal-sp) >= 200)){
			
				iSetAngle = 50;
				motor2_f( -10,   50 );  //0 80    
        		motor2_r( 0,   0 );   //0 0
				
			//	sp += 2; //手前で曲がりすぎているため距離を少しのばす
				
				c_cut_in_flag = 1;
		
			}else if((lEncoderTotal-sp) >= 350){
				
				iSetAngle = 100;
				motor2_f( 40,   5 );  //85 5       
        		motor2_r( 8,   -5 );  //25 0
				
				
			}else if((lEncoderTotal-sp) >= 250){
				
				if(i < 70)iSetAngle = 120;
				else iSetAngle = 98;
				
				motor2_f( 30,   0 );  //80 0       
        		motor2_r( -10,   -10 );   //0 0
			
				
			}else if((lEncoderTotal-sp) >= 150){
				
				if(i < 45)iSetAngle = 100;
				else iSetAngle = 90;
				
				motor2_f( 40,   0 );  //85 5      
        		motor2_r( 0,   -10 );  //15 0
				
			}else if((lEncoderTotal-sp) >= 100){
				
				if(i < 30)iSetAngle = 80;
				else iSetAngle = 60;
				
				motor2_f( 45,   2 ); //85 30        
        		motor2_r( 10,   -10 );  //25 0
			
			}else if((lEncoderTotal-sp) >= 50){
				
				if(i < 20)iSetAngle = 50;
				else iSetAngle = 35;
				
				motor2_f( 50,   5 ); //90 35        
        		motor2_r( 15,  -5 );   //35 0
					
			}else{
				iSetAngle = 25;
				motor2_f( 50,   10 );  //90 40       
        		motor2_r( 20,   -5 );   //40  0	
			} 
			
       	}
  
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */
        
		//180 -15 < 25
        if((( c_cut == 0 || date_f_mode == 0) && (lEncoderTotal-sp) >= 150) || ((c_cut == 1 && date_f_mode != 0) && (lEncoderTotal-sp) >= 240 && c_cut_in_flag ==0)  ){
			if(Wide != 0){
			//if (((20 < Center)&&(Center < 40)) || ((-15 < Center)&&(Center < 15))) {    /* 曲げ終わりチェック           */
			if ( (( c_cut == 0 || date_f_mode == 0) && 15 < Center && Center < 35 && (Wide != 0 && Wide < 30) ) 
				|| ((c_cut == 1 && date_f_mode != 0) && -20 < Center && Center < 0 && (Wide_old == 0 || Wide > Wide_old))
				  || ((c_cut == 1 && date_f_mode != 0) && -25 < Center && Center < 25 && (Wide_old != 0) && (lEncoderTotal-sp) >= 750)) {    /* 曲げ終わりチェック           */
				
            	cnt1 = 0;
            	iSensorPattern = 0;
            	
				sp = lEncoderTotal;
            	pattern = 32;
				
				mode = 0;//通常
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				
				if(date_f_mode != 0 && c_cut == 1){	
					cnt1 = 0;
					sp = lEncoderTotal;
					mode = 0;//見る範囲を元に戻す
            		pattern = 11;//通常トレースへ
					//cnt8 = 0;
				}	
        	}
			}
		}
        break;

    case 32:
        /* 安定するまで (ショーカットは32には来ないよ)*/
		
		if(c_short_mode == 1){//short
        	if((lEncoderTotal-sp) >= 110)iSetAngle = 90;
			else iSetAngle = 95;
		}else{//long
			if((lEncoderTotal-sp) >= 100)iSetAngle = 90;
			else iSetAngle = 95;
		}
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if( iEncoder10 >= C_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           
			x=(C_TOPSPEED2-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_short_mode == 1){//short
				motor2_f( 50, 40 );
           		motor2_r( 0, 0 );
		
		
			}else{//long
				motor2_f( 80, 40 );
           		motor2_r( 40, 10 );
			}
		}
         
		
		if((lEncoderTotal-sp) >= 90){ 
			if(Wide != 0){                       
        		if(((-15 < Center)&&(Center < 20) && getServoAngle() < 120) 
					|| ((-5 < Center)&&(Center < 5) && getServoAngle() < 128)) {    /*  直線になるまで          */
            		cnt1 = 0;
            		iSensorPattern = 0;
            	
           			pattern = 33;
				//	mode = 0;//通常
					sp = lEncoderTotal;
        		}
			}
		}
	
        break;
	
	case 33://少し待つ	
		//mode = 1;//見る範囲を狭くする
		mode = 0;//見る範囲を元に戻す
		
		servoPwmOut( iServoPwm );

		if( iEncoder10 >= TOPSPEED ) {          // エンコーダによりスピード制御 
          
			x=(TOPSPEED-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if( ((lEncoderTotal-sp) >= 100 && -30 < getServoAngle() && getServoAngle() < 30 ) || ((lEncoderTotal-sp) >= 300)  ) {
            cnt1 = 0;
			sp = lEncoderTotal;
			mode = 0;//見る範囲を元に戻す
            pattern = 11;//通常トレースへ
			//cnt8 = 0;
        }
        break;
		

    case 41:
        /* 左クランク処理 */
        //setBeepPatternS( 0x8000 );
			
        if(date_f_mode == 0 || c_cut == 0){//通常
			mode = 2;//右無視
			
			if(c_short_mode == 1){//short
				if((lEncoderTotal-sp) >= 130){
				
					if(i > -95)iSetAngle = -120;
					else iSetAngle = -105;
				
				}else iSetAngle = -100;
			
				motor_f( 10, 10 );          /* この部分は「角度計算(4WD時).xls」*/
        		motor_r( -5, -5 );          /* で計算                        */
				
			}else{//long
				
				if((lEncoderTotal-sp) >= 200){
				
					if(i > -95)iSetAngle = -130;
					else iSetAngle = -100;
				
				}else iSetAngle = -95;
			
				motor_f(  -5, 75 );          /* この部分は「角度計算(4WD時).xls」*/
        		motor_r(  -20, -20 );          /* で計算                        */
			}
		}else{
			mode = 1;//見る範囲を狭く
			
  			if(0 == Center && Wide == 0){
				c_cut_in_flag = 0;	
			}
			 
			if((Center < 0 && (lEncoderTotal-sp) >= 150) || (Wide != 0 && Center < 6 && (lEncoderTotal-sp) >= 200)){
				iSetAngle = -50;

				motor2_f( 50,   -10 );  //80 0       
        		motor2_r( 0,   0 );  //0 0
		
				//sp += 2; //手前で曲がりすぎているため距離を少しのばす
				
				c_cut_in_flag = 1;
				
			}else if((lEncoderTotal-sp) >= 350){
				iSetAngle = -100;

				motor2_f( 5,   40 ); //5 85        
        		motor2_r( -5,   8 );  //0 25
				
			}else if((lEncoderTotal-sp) >= 250){
				
				if(i > -70)iSetAngle = -120;
				else iSetAngle = -98;

				motor2_f( 0,   30 ); //0 80        
        		motor2_r( -10,   -10 );  //0 0
			
				
			}else if((lEncoderTotal-sp) >= 150){
				if(i > -45)iSetAngle = -100;
				else iSetAngle = -90;
				
				motor2_f( 0,   40 ); //5 85        
        		motor2_r( -10,   0 ); //0 15
				
			}else if((lEncoderTotal-sp) >= 100){
				if(i > -30)iSetAngle = -80;
				else iSetAngle = -60;
				
				motor2_f( 2,   45 ); //30 85        
        		motor2_r( -10,   10 ); //0 25
			
			}else if((lEncoderTotal-sp) >= 50){
				if(i > -20)iSetAngle = -50;
				else iSetAngle = -35;
				
				motor2_f( 5,   50 ); //35 90       
        		motor2_r( -5,  15 );   //0 35
					
			}else{
				iSetAngle = -25;
				motor2_f( 10,   50 );  //40 90    
        		motor2_r( -5,   20 );   // 0 40
			}  
       	}
		
		servoPwmOut( iServoPwm2 );        /* 振りが弱いときは大きくする       */
        
		if(((c_cut == 0 || date_f_mode == 0) && (lEncoderTotal-sp) >= 150) || ((c_cut == 1 && date_f_mode != 0) && (lEncoderTotal-sp) >= 240 && c_cut_in_flag == 0) ){
			if(Wide != 0){ 
			//if( ((-40 < Center)&&(Center < -20)) || ((-15 < Center)&&(Center < 15))) {    /* 曲げ終わりチェック           */
	 		if(( (c_cut == 0 || date_f_mode == 0) && -35 < Center && Center < -15 && (Wide != 0 && Wide < 30)) 
				|| ( (c_cut == 1 && date_f_mode != 0) && 0 < Center && Center < 20 && (Wide_old == 0 || Wide < Wide_old)) 
					|| ((c_cut == 1 && date_f_mode != 0) && -25 < Center && Center < 25 && (Wide_old != 0) && (lEncoderTotal-sp) >= 750)){    /* 曲げ終わりチェック           */
	 	
            	cnt1 = 0;
            	iSensorPattern = 0;
            
            	sp = lEncoderTotal;
            	pattern = 42;
				
				mode = 0;//通常
				
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				if(date_f_mode != 0 && c_cut == 1){	
					cnt1 = 0;
					sp = lEncoderTotal;
					mode = 0;//見る範囲を元に戻す
            		pattern = 11;//通常トレースへ
					//cnt8 = 0;
				}
			}
			}
		}
        break;

    case 42:
		/* 安定するまで (ショーカットは42には来ないよ)*/
		if(c_short_mode == 1){//short
			if((lEncoderTotal-sp) >= 110)iSetAngle = -90;
			else iSetAngle = -95;
		}else{//long
			if((lEncoderTotal-sp) >= 100)iSetAngle = -90;
			else iSetAngle = -95;
		}
		
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if( iEncoder10 >= C_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           
			x=(C_TOPSPEED2-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_short_mode == 1){//short
				motor2_f( 40, 50);
           		motor2_r( 0, 0 );
			}else{//long
				motor2_f( 40, 80);
           		motor2_r( 10, 40 );
			}
		}
         	  
		
		if((lEncoderTotal-sp) >= 50){   
			if(Wide != 0){                      
				if(((-30 < Center)&&(Center < 15) && getServoAngle() > -120) 
					|| ((-5 < Center)&&(Center < 5) && getServoAngle() > -128)) {    /*  直線になるまで          */
            		
            		cnt1 = 0;
      			    iSensorPattern = 0;
            	
           			pattern = 43;
				//	mode = 0;//通常
					sp = lEncoderTotal;
        		}
			}
		}
		
        break;
		
	case 43://少し待つ	
	//	mode = 1;//見る範囲を狭くする
		mode = 0;//見る範囲を元に戻す
		
		servoPwmOut( iServoPwm );

		if( iEncoder10 >= TOPSPEED ) {          // エンコーダによりスピード制御 
			
			x=(TOPSPEED-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if(( (lEncoderTotal-sp) >= 100 && -30 < getServoAngle() && getServoAngle() < 30) || ((lEncoderTotal-sp) >= 300) ) {
            cnt1 = 0;
			sp = lEncoderTotal;
			mode = 0;//見る範囲を元に戻す
            pattern = 11;//通常トレースへ
			//cnt8 = 0;
        }
        break;
		
	case 51://左ハーフ
		//setBeepPatternS( 0x8000 );
	    iSetAngle = 0;
		servoPwmOut( iServoPwm2 );

		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // エンコーダによりスピード制御 
       
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(date_f_mode == 0 && (check_crossline() || check_halfline() == 2 )) {       // クロスラインチェック         
            cnt1 = 0;
            pattern = 21;	
        }
  
        if( (lEncoderTotal - sp) >= 100 ) {
            cnt1 = 0;
            pattern = 52;
			sp = lEncoderTotal;
        }
        break;

	case 52://ハーフライン後
	//	if( date_f_mode != 0 && (lEncoderTotal - sp) >= 100 )mode = 3;//左無視
	
#ifdef HWall //壁あり	 
		if(Center > 11){//寄りすぎ
			Center_offset = -6;//左に寄る
			servoPwmOut( iServoPwm );
		}else{
			Center_offset = -9;//左に寄る
			servoPwmOut( iServoPwm  );
		}
#else //壁無し
		if(Center > 12){//寄りすぎ
			Center_offset = -7;//左に寄る
			servoPwmOut( iServoPwm );
		}else{
			Center_offset = -10;//左に寄る
			servoPwmOut( iServoPwm  );
		}
#endif		
		
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(H_TOPSPEED -iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}

	
	//	if((lEncoderTotal-sp) >= 300 ){
	//		if((( h_cut == 0 || date_f_mode == 0)  && Wide == 0) || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //脱線チェック
        
		if(((h_cut == 0 || date_f_mode == 0)  && (Wide == 0 || check_crossline()) && (lEncoderTotal-sp) >= 250 )  || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //脱線チェック
            
            cnt1 = 0;
			sp = lEncoderTotal;
				
			if(date_f_mode == 0){//距離計測
				date_buff_ch_int[date_num_ch++] = 53;
				date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
			}
				
           	 pattern = 53;
			date_f_num_ch++;
			mode = 0;//通常
			Center_offset = 0;//オフセットを戻す
            break;
		}
		
		if(date_f_mode == 0 && (lEncoderTotal-sp) < 30  && ( check_crossline() || check_halfline() == 2 )) {       // クロスラインチェック         
            cnt1 = 0;
			Center_offset = 0;//オフセットを戻す
            pattern = 21;	
        }
  
		if( (lEncoderTotal-sp) >= 1500 ) {//誤動作チェック
			
			date_f_num_ch--;
			mode = 0;//通常
            pattern = 11;
			//cnt8 = 0;
			cnt1 = 0;
			Center_offset = 0;//オフセットを戻す
        }	

		break;

	case 53:
		mode = 2;//右無視
		
		if(date_f_mode == 0 || h_cut == 0){

#ifdef HWall  //壁あり    	
			iSetAngle = -65;
#else //壁無し
			iSetAngle = -50;
#endif

		}else{
			iSetAngle = -15;
		}
		
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				
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
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
         
		if( ((date_f_mode == 0 || h_cut == 0) && (lEncoderTotal - sp) >= 200 ) || (date_f_mode != 0 && (lEncoderTotal - sp) >= 350 )) {  
	        if((-50 < Center)&&(Center < -10) && Wide != 0 ) {    /* 曲げ終わりチェック           */
			//	if(Center < -30) {    /* 曲げ終わりチェック           */
		
	            	cnt1 = 0;
					sp = lEncoderTotal;
	            	pattern = 54;
				
					mode = 0;//通常
					motor_mode_r( BRAKE, BRAKE );
	        	}
			}
        break;

	case 54:
		
		if(date_f_mode == 0 || h_cut == 0){
			
			if((Center == 0)&&(Wide == 0)){ //インに落ちそう
				iSetAngle = -50;
			}else{
#ifdef HWall  //壁あり    	
				iSetAngle = -30;
#else //壁無し
				iSetAngle = -20;
#endif
			}
		}else{
			iSetAngle = -5;//5 7
		}
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
           		motor_f( 100, 100 );
           		motor_r( 60,  60 );
#else //壁無し
           		motor_f( 90, 80 );
           		motor_r( 80,  50 );
#endif


			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 90,  50 );
			}
		}
        
		
        //if( (15 < Center)&&(Center < 50) ) {    /* 曲げ終わりチェック           */
		//if(-10 < Center && Wide != 0) {    /* 曲げ終わりチェック           *///-7
		
#ifdef HWall  //壁あり    	
		if(10 < Center && Wide != 0) {    /* 曲げ終わりチェック           *///-7
#else //壁無し
		if(0 < Center && Wide != 0) {    /* 曲げ終わりチェック           *///-7
#endif
	//	if(20 < Center) {    /* 曲げ終わりチェック           */
		
            cnt1 = 0;
			sp = lEncoderTotal;
            pattern = 55;
			
			motor_mode_r( FREE, FREE );
        }
		
        break;

	case 55://安定するまで
	
		if(date_f_mode == 0 || h_cut == 0){

#ifdef HWall  //壁あり    	
			iSetAngle = 20;
#else //壁無し
			iSetAngle = 15;
#endif
			servoPwmOut( iServoPwm2 );          
		
			//servoPwmOut( iServoPwm  );
		}else{
			iSetAngle = 20;//40 37
			servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		}
		
		
		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
           		motor_f( 100, 30 );
           		motor_r( 90, 0 );
#else //壁無し
				if(Center > 10){//外に落ちそう
           			motor_f( 90, 0 );
				}else{
					motor_f( 90, 30 );
				}
				motor_r( 80, 0 );
#endif

			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 50 );
           		motor_r( 90, 0 );
			}
		}
          
		if( (lEncoderTotal - sp) >= 100 ) {  

#ifdef HWall  //壁あり  
	        if((-35 < Center)&&(Center < 35)&&(Wide != 0)) {    /*  直線になるまで          */
#else //壁無し
			if((-23 < Center)&&(Center < 23)&&(Wide != 0)) {    /*  直線になるまで          */
#endif	
            cnt1 = 0;
            iSensorPattern = 0;
          
			pattern = 11;
			//cnt8 = 0;
			
			sp = lEncoderTotal;
        }
		}
        break;
		
	
			
	case 61://右ハーフ
		
		//setBeepPatternS( 0x8000 );
		iSetAngle = 0;
		servoPwmOut( iServoPwm2 );

		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(date_f_mode == 0 && (check_crossline() || check_halfline() == 1)) {       // クロスラインチェック         
            cnt1 = 0;
           	
            pattern = 21;			
        }

        if( (lEncoderTotal-sp) >= 100 ) {
            cnt1 = 0;
            pattern = 62;
			sp=lEncoderTotal;
        }
        break;

	case 62://ハーフライン後
		
	//	if( date_f_mode != 0 && (lEncoderTotal - sp) >= 100 )mode = 2;//右無視

#ifdef HWall //壁あり
		if(Center < -11){//
			Center_offset = 6;//右に寄る
			servoPwmOut( iServoPwm );
				
		}else{
			Center_offset = 9;//右に寄る
			servoPwmOut( iServoPwm );
		}
#else //壁無し		
		if(Center < -12){//
			Center_offset = 7;//右に寄る
			servoPwmOut( iServoPwm );
				
		}else{
			Center_offset = 10;//右に寄る
			servoPwmOut( iServoPwm );
		}
#endif		
		
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // エンコーダによりスピード制御 
    
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(((h_cut == 0 || date_f_mode == 0)  && (Wide == 0 || check_crossline()) && (lEncoderTotal-sp) >= 250 )  || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //脱線チェック
            
            cnt1 = 0;
			sp = lEncoderTotal;
				
			if(date_f_mode == 0){//距離計測
				date_buff_ch_int[date_num_ch++] = 63;
				date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
			}
				
           	pattern = 63;
			date_f_num_ch++;
			mode = 0;//通常
			Center_offset = 0;//オフセットを戻す
				
            break;
        }
			

		if(date_f_mode == 0 && (lEncoderTotal-sp) < 30  && ( check_crossline() || check_halfline() == 1 )) {       // クロスラインチェック         
            cnt1 = 0;
			Center_offset = 0;//オフセットを戻す
            pattern = 21;	
        }
  
		if( (lEncoderTotal-sp) >= 1500 ) {//誤動作チェック
          	
			date_f_num_ch--;
			mode = 0;//通常
            pattern = 11;
			//cnt8 = 0;
			cnt1 = 0;
			Center_offset = 0;//オフセットを戻す
        }

		break;

	case 63:
		//setBeepPatternS( 0x8000 );

		mode = 3;//左無視
		
		if(date_f_mode == 0 || h_cut == 0){
 
#ifdef HWall  //壁あり    	
			iSetAngle = 68;
#else //壁無し
			iSetAngle = 50;
#endif			
		}else{
			iSetAngle = 20;//48 47
		}
		
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
          
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
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
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
    	       	motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
                
		if( ((date_f_mode == 0 || h_cut == 0) && (lEncoderTotal - sp) >= 200 ) || (date_f_mode != 0 && (lEncoderTotal - sp) >= 350 )) {             
	        if((10 < Center)&&(Center < 50) && Wide != 0) {    /* 曲げ終わりチェック           */
			//if( 30 < Center) {    /* 曲げ終わりチェック           */
		
	            cnt1 = 0;
	            sp = lEncoderTotal;
	            pattern = 64;
			
				mode = 0;//通常
			
				motor_mode_r( BRAKE, BRAKE );
	        }
		}
        break;

	case 64:

		if(date_f_mode == 0 || h_cut == 0){
			if((Center == 0)&&(Wide == 0)){ //インに落ちそう
				iSetAngle = 50;
			}else{
#ifdef HWall  //壁あり    	
				iSetAngle = 30;//-3
#else //壁無し
				iSetAngle = 20;//-3
#endif
			}
		}else{
			iSetAngle = 5;//-3
		}
		
		servoPwmOut( iServoPwm2 );          /* 振りが弱いときは大きくする       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
          
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
				motor_f( 100, 100 );
           		motor_r( 60, 60 );
#else //壁無し
				motor_f( 80, 90 );
           		motor_r( 50, 80 );
#endif
           		
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 50, 90 );
			}
		}
             
		//if( (lEncoderTotal - sp) >= 30 ) {                  
        //if((-50 < Center)&&(Center < -15)) {    /* 曲げ終わりチェック           */

#ifdef HWall  //壁あり    	
		if(Center < -10 && Wide != 0) {    /* 曲げ終わりチェック           *///-5
#else //壁無し
		if(Center < 0 && Wide != 0) {    /* 曲げ終わりチェック           *///-5
#endif				
		//if(Center < -20) {    /* 曲げ終わりチェック           */
	
            cnt1 = 0;
            sp = lEncoderTotal;
            pattern = 65;
			
			motor_mode_r( FREE, FREE );
        }
		//}
        break;

	case 65://安定するまで
	
		if(date_f_mode == 0 || h_cut == 0){

#ifdef HWall  //壁あり    	
			iSetAngle = -20;
#else //壁無し
			iSetAngle = -15;
			
#endif

			servoPwmOut( iServoPwm2 );
			
		//	servoPwmOut( iServoPwm  );
		}else{
			iSetAngle = -20;//-40 -35
			
			servoPwmOut( iServoPwm2 );
		}
		       

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* エンコーダによりスピード制御 */
           	
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //壁あり    	
				motor_f( 30, 100 );
           		motor_r( 0, 90 );
#else //壁無し
				if(Center < -10){//外に落ちそう
					motor_f( 0, 90 );
				}else{
					motor_f( 30, 90 );					
				}
           		motor_r( 0, 80 );
#endif

			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* エンコーダによりスピード制御 */
           
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 50, 100 );
           		motor_r( 0, 90 );
			}
		}
                      
	    if( (lEncoderTotal - sp) >= 100 ) {  
#ifdef HWall  //壁あり  
	        if((-35 < Center)&&(Center < 35)&&(Wide != 0)) {    /*  直線になるまで          */
#else //壁無し
			if((-23 < Center)&&(Center < 23)&&(Wide != 0)) {    /*  直線になるまで          */
#endif		
	            cnt1 = 0;
	            iSensorPattern = 0;
        
				pattern = 11;
				//cnt8 = 0;
		
				sp = lEncoderTotal;
	        }
		}
        break;
	case 101:
        /* 停止 */
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        setBeepPatternS( 0xc000 );
		
	
		msdFlag = 0;
        if( msdError != 0 ) {
            /* microSDに不具合があったなら終了 */
            printf( "microSD Initialize Error!!\n" );
            pattern = 109;
        } else {
            pattern = 102;
            cnt1 = 0;
        }
        break;

    case 102:
		/* 最後のデータ書き込むまで待つ*/
        if( checkMicroSDProcess() == 0 ) {
            pattern = 103;               /* データ転送処理へ             */
            break;
        }
        if( checkMicroSDProcess() == 11 ) {
            microSDProcessEnd();        /* microSDProcess終了処理       */
            while( checkMicroSDProcess() );
            pattern = 103;               /* データ転送処理へ             */
        }
		
        break;

   case 103:
		 
        /* 0.5s待ち && プッシュスイッチが離されたかチェック*/
        if( cnt1 >= 500 && !pushsw_get()) {
            pattern = 104;
            cnt1 = 0;
        }
        break;

    case 104:
        /* プッシュスイッチが押されたかチェック */
        led_out( cnt1 / 200 % 2 ? 0x6 : 0x9  );
        if( pushsw_get() ) {
            pattern = 105;
            cnt1 = 0;
        }
        break;

    case 105:
        /* タイトル転送、転送準備 */
        printf( "\n" );
        printf( "CarName Data Out\n" ); /* 自分のカーネームを入れてください */
        printf( "Pattern, Center, Wide ,角度, サーボPWM, " );
        printf( "左前PWM, 右前PWM, 左後PWM, 右後PWM, エンコーダ5*2,モード,坂道回数,ジャイロY/10,ジャイロX/10,赤外線\n" );
		
		msdWorkAddress = msdStartAddress;   /* 読み込み開始アドレス     */
        pattern = 106;
        break;
	case 106:
        /* microSDよりデータ読み込み */
        if( msdWorkAddress >= msdEndAddress ) {
            /* 書き込み終了アドレスになったら、終わり */
            pattern = 109;
            break;
        }
        ret = readMicroSD( msdWorkAddress , msdBuff );
        if( ret != 0x00 ) {
            /* 読み込みエラー */
            printf( "\nmicroSD Read Error!!\n" );
            pattern = 109;
            break;
        } else {
            /* エラーなし */
            msdWorkAddress += 512;
            msdBuffAddress = 0;
            pattern = 107;
			i = 0;
        }
        break;
	case 107:
        /* データ転送 */
        led_out( 1 << (cnt1/100) % 8 );

		if(msdBuff[msdBuffAddress+0] <= 0 ){ /* パターンが0以下なら終了 */
			date_f_make(-1,0,0,0);
            printf( "End.\n" );
            pattern = 108;
			cnt1  = 0 ;
			setBeepPatternS( 0xff00 );
            break;
        }

	//	if( (dipsw_get() & 0x04) == 0x00 ) {//ログ出力なし、早くコース記憶したい時用
			/* データの転送 */
	        printf( "%d,%4d,%4d,%5d,%5d,%5d,%5d,%5d,%5d,%4d,%4d,%4d,%4d,%4d,%4d\n",
	            (int)msdBuff[msdBuffAddress+0],                  /* パターン     */
	            (int)msdBuff[msdBuffAddress+1],					/* センター*/
	            (unsigned char)msdBuff[msdBuffAddress+2],                  /* ワイド */
				(int)((unsigned char)msdBuff[msdBuffAddress+3]*0x100 +
		             (unsigned char)msdBuff[msdBuffAddress+4] ),			/* 角度 */
	            /* サーボPWM */
		            msdBuff[msdBuffAddress+6],
		            /* 左前PWM */
		            msdBuff[msdBuffAddress+7],
		            /* 右前PWM */
		            msdBuff[msdBuffAddress+8],
		            /* 左後PWM */
		            msdBuff[msdBuffAddress+9],
		            /* 右後PWM */
		            msdBuff[msdBuffAddress+10],
		            /* エンコーダ */
		            msdBuff[msdBuffAddress+11] * 2,
					/* モード */
		            msdBuff[msdBuffAddress+12],
					/* 坂道回数 */
		            msdBuff[msdBuffAddress+13],
					/* 加速度センサーY */
		            msdBuff[msdBuffAddress+14],
					/* 加速度センサーX */
		            msdBuff[msdBuffAddress+15],
					/* 赤外線センサーの差 */
		            msdBuff[msdBuffAddress+5]
        );		
	//	}
        
		if(date_f_mode != 0){
			date_f_make((int)msdBuff[msdBuffAddress+0],(int)((unsigned char)msdBuff[msdBuffAddress+3]*0x100 +
	             (unsigned char)msdBuff[msdBuffAddress+4] ) ,msdBuff[msdBuffAddress+11] ,(int)msdBuff[msdBuffAddress+12]);
		}
		
		 msdBuffAddress += 64;  /* 次の送信準備                 */

        if( msdBuffAddress >= 512 ) {
            pattern = 106;
        }
		
        break;

    case 108:
        /* 転送終了 */
		if(date_f_mode == 1){
			/* ブロックA イレーズします */
			blockEraseDataFlash( 0x3000 );
			/* ブロックB イレーズします */
			blockEraseDataFlash( 0x3400 );
			
			/* ブロックA 書き込み */
			writeDataFlash( 0x3000, date_f_buff, 32 );
			
			
			if( readMicroSD( msdStartAddress_ch , msdBuff_ch ) != 0x00 ) {
				// 読み込みエラー 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)date_f_buff_ch[i] = msdBuff_ch[i];
			}
			
			/* ブロックB 書き込み */
			writeDataFlash( 0x3400, date_f_buff_ch, 32 );
			printf("date_f_mode == 1\n");
			
		}else if(date_f_mode == 2){
			/* ブロックC イレーズします */
			blockEraseDataFlash( 0x3800 );
			/* ブロックD イレーズします */
			blockEraseDataFlash( 0x3c00 );
			
			/* ブロックC 書き込み */
			writeDataFlash( 0x3800, date_f_buff, 32 );
			
			if( readMicroSD( msdStartAddress_ch , msdBuff_ch ) != 0x00 ) {
				// 読み込みエラー 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)date_f_buff_ch[i] = msdBuff_ch[i];
			}
		
			/* ブロックD 書き込み */
			writeDataFlash( 0x3c00, date_f_buff_ch, 32 );
		
			printf("date_f_mode == 2\n");
		}
		
		j=0;
		for(i = 0; i < 16; i++){
			//printf("%d%d",date_f_buff[j],date_f_buff[j+1]);
			printf("%d\n",date_f_buff_int[i]);
			
			j+=2;
		}
		
		printf("\n");
		for(i = 0; i < 32; i+=3){
			printf("%d %d%02d\n",date_f_buff_ch[i],date_f_buff_ch[i+1],date_f_buff_ch[i+2]);
		}
		
		
		pattern = 109;
        led_out( 0xff );
        break;
		
	case 109:
		led_out( 0xff );
		break;
	case 200://走行終了
		setBeepPatternS( 0xff00 );
		
		while(1){
			cam_in();
			if(logfin == 0)led_out(camera(Center,Wide));
			else led_out(0);
			
			if(iEncoder10 < 5 ){
				if(msdFlag == 1)msdFlag = 2;                /* データ記録終了               */
				servoPwmOut( 0 );
			}else{
				servoPwmOut( iServoPwm );
			}
		
		/*	if( pushsw_get() ) {
				while(pushsw_get());
		 		for(i = 0; i < 32; i+=2){
					printf("%d %d\n",date_buff_ch_int[i],date_buff_ch_int[i+1]);
				}
			}*/
			motor_f( 0, 0 );
            motor_r( 0, 0 );
			if(logfin == 0){
				if(msdFlag == 0){
					//setBeepPatternS( 0xff00 );
					
					servoPwmOut( 0 );
					
					j = 0;
					for(i = 0; i < 32; i+=2){
						//printf("%d %d\n",date_buff_ch_int[i],date_buff_ch_int[i+1]);
						msdBuff_ch[j++] = (signed char)date_buff_ch_int[i];
						msdBuff_ch[j++] = (signed char)(date_buff_ch_int[i+1]/100);
						msdBuff_ch[j++] = (signed char)(date_buff_ch_int[i+1]%100);
					}
					
					while( checkMicroSDProcess() != 11 )wait(2); /* 書き込みが終わるまで待つ */
					// 書き込み処理が終わるまで繰り返す
					while( microSDProcessEnd() != 0 )wait(2);
					
					for(i = 0; i < 3; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					/* microSDProcess開始処理 */
					while(microSDProcessStart( msdStartAddress_ch) != 0x00)wait(2);
            		
					setMicroSDdata( msdBuff_ch );
					
					while( checkMicroSDProcess() != 11 )wait(2); /* 書き込みが終わるまで待つ */
					// 書き込み処理が終わるまで繰り返す
					while( microSDProcessEnd() != 0 )wait(2);
			
					
					for(i = 0; i < 5; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					logfin = 1;
				}
			}
		}
		break;

		
	case 500:
		
		if( pushsw_get() ) {
			mode++;
			if(mode >= 4)mode = 0;
			while(pushsw_get());
		}
		
		
		cam_in();
		led_out(camera(Center,Wide));
	
		printf("mode = %d  %4d   %4d\n",mode,Center,Wide);
		break;
	
    default:
        break;
    }
    }
}


int date_f_make(int pattern, int angle, int encoder, int rmode){

	static int mode = 0; //S = 0, L = 1, R = 2, C = 3, H = 4, F = 5
	static long Encoder = 0;
	static int buff_num = 0;
	static int buff_num_int = 0;
//	static int buff_num_ch = 0;
//	static int buff_num_int_ch = 0;
	
	Encoder += encoder;
	
	switch(pattern){
		case -1:
			//if(	Encoder >= 500){//この距離以下は無視（滑り？）
				if(mode == 0){//S
					if(Encoder > 1000){//この距離以下は無効
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
					mode = 5;//F
				}
		//	}
			break;
		case 10:
		case 11:
		//if(	Encoder >= 500){//この距離以下は無視（滑り？）
			if(mode == 0){//S
				if(((rmode == 0 ) && (angle < -Cu_Angle)) || ((rmode != 0 ) && (angle < -(Cu_Angle_saka)))){
					if(Encoder > 1000){//この距離以下は無効
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
						
					mode = 1;//L
						
				}else if(((rmode == 0) &&(Cu_Angle < angle)) || ((rmode != 0) &&(Cu_Angle_saka < angle))){
					if(Encoder > 1000){//この距離以下は無効
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
						
					mode = 2;//R
				}
			}else if(mode == 1){//Lカーブ
				//if(((rmode == 0) && (-Cu_Angle < angle)) || ((rmode != 0) && (-(Cu_Angle_saka) < angle)) ){
				if(-Cu_Angle < angle){
					Encoder =0;
											
					mode = 0;//S
				}
			}else if(mode == 2){//Rカーブ
				//if(((rmode ==0) && (angle < Cu_Angle) || ((rmode !=0) && (angle < Cu_Angle_saka)))){
				if(angle < Cu_Angle){
				
					Encoder =0;
						
					mode = 0;//S
				}
			}else{
				if(((rmode == 0) &&(-Cu_Angle < angle && angle < Cu_Angle)) || ((rmode != 0) &&(-(Cu_Angle_saka) < angle && angle < Cu_Angle_saka))){
					Encoder =0;
					mode = 0;//S
				}
			}

		//}
		break;

		case 21:
		case 22:
			if(mode != 3){//C
				if(mode == 0){//S
					if(Encoder > 1000){//この距離以下は無効
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
				}
				Encoder =0;
			}
			mode = 3;//C
				
			break;
		case 31:
			if(mode != 2){//R
				//このときのEncoderが曲がるまでの距離
			/*	date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
			*/			
				Encoder =0;
			}
			mode = 2;//R
			break;
		case 41:
			if(mode != 1){//L
				//このときのEncoderが曲がるまでの距離
			/*	date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
			*/
				Encoder =0;	
			}
			mode = 1;//L
			break;

		case 51:
		case 61:
		case 52:
		case 62:
			if(mode != 4){//H
				if(mode == 0){//S
					if(Encoder > 1000){//この距離以下は無効
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = (Encoder/100);
						buff_num++;
						date_f_buff[buff_num] = (Encoder - ((Encoder/100)*100));
						buff_num++;
					}
				}
				Encoder =0;
			}
			mode = 4;//H
			break;
		case 53:
			if(mode != 1){//L
				//このときのEncoderが曲がるまでの距離
			/*	date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
			*/	
				Encoder =0;
			}
			mode = 1;//L
			break;
		
		case 55:
			Encoder =0;
			mode = 0;//S
			break;
			
		case 63:
			if(mode != 2){//R
				//このときのEncoderが曲がるまでの距離
			/*	date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
			*/				
				Encoder =0;
			}
			mode = 2;//R
			break;
		
		case 65:
			Encoder =0;
			mode = 0;//S
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
    pur0 |= 0x04;                       /* P1_3〜P1_0のプルアップON     */
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

	
    /*      センター値   P5_6〜P5_0       */
	pur1 |= 0x0c;   /* P5_7〜P5_0のプルアップON     */
    p5  = 0x00;
    pd5 = 0x00;

    /*      SDカード 0,1,2,3,4 
			幅(2,1,0bit) 7,6,5          */
	pur1 |= 0x30;   /* P6_7〜P6_0のプルアップON     */
	//pur1_addr.bit.b5;	/* P6_4 to P6_7 pull-up */
    p6  = 0x00;
    pd6 = 0x1f;
	
    /*  CN6.2入力       CN6.3入力       CN6.4入力       CN6.5入力
        none(ｱﾅﾛｸﾞ予備) 角度VR          センサ_左ｱﾅﾛｸﾞ  センサ_右ｱﾅﾛｸﾞ  */
    p7  = 0x00;
    pd7 = 0x00;

    /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
        DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
    pur2 |= 0x03;                       /* P8_7〜P8_0のプルアップON     */
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
    trcgrb  = trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
    trcgrc  = trcgra;                   /* P0_7端子のON幅(予備)         */
    trcgrd  = trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
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
	//static unsigned int EncoderMod = 0;
	//static int iEncoder5_old = 0; 
	static int iEncoder1_buf[10] = {0}; 
	//static unsigned long  SEncoderTotal3 = 0;          /* 積算値保存用                 */
	int iEncoder_buf = 0; 
	int a;
	static int flag = 0,flag20 = 0,flag56 = 0;
	signed char *p;
	
    asm(" fset I ");                    /* タイマRB以上の割り込み許可   */
	
    cnt1++;
	cnt2++;
	cnt3++;
	cnt7++;
	cnt8++;
	if( pattern == 11){//通常トレースのみ有効
		cnt9++;
	}else{
		cnt9 = 0;	
	}
	
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
	iEncoder1_buf[iTimer10] = (i - uEncoderBuff)/2;
	lEncoderTotal += iEncoder1_buf[iTimer10];
	if(lEncoderTotal < 0)lEncoderTotal = 0;
	uEncoderBuff = i;
	
	iEncoder_buf = 0;
	for(k = 0; k < 10; k++)iEncoder_buf += iEncoder1_buf[k];
	iEncoder10 = iEncoder_buf;
	

    /* 10回中1回実行する処理 */
    iTimer10++;
    switch( iTimer10 ) {

    case 1:
        break;

    case 2:
        /* スイッチ読み込み準備 */
        p9_4 = 0;                       /* LED出力OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* スイッチ読み込み、LED出力 */
        types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のSW読み込み*/
        p8  = types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.3のLEDへ出力*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED出力ON                    */
        break;

    case 4:
	case 9:
	/*	iEncoder5_old = iEncoder5 ;
		
		//21*3.14=65.94 200パルス 200/65.94=3パルスで1mm
		i = trg;
        iEncoder5       = (i - uEncoderBuff)/3;	
		EncoderMod     = (i - uEncoderBuff)%3;	
        lEncoderTotal += iEncoder5;
		//uEncoderBuff = i ;
        uEncoderBuff = i - EncoderMod ;
		
		iEncoder10 = iEncoder5 + iEncoder5_old ; 
*/		

		iEncoder_buf = 0;
		for(k = 0; k < 5; k++)iEncoder_buf += iEncoder1_buf[(10 + iTimer10 - k) % 10];
		iEncoder5 = iEncoder_buf;
	
		if(date_f_mode != 0 && ( msdFlag == 1 || msdFlag == 2 )){//再生走行モード
			a = getServoAngle();
			//直線 
			if((pattern == 11 || pattern == 10) && (((mode == 0) &&(-Cu_Angle < a && a < Cu_Angle)) || ((mode != 0) &&(-(Cu_Angle_saka) < a && a < Cu_Angle_saka)) ) ){
				SEncoderTotal += iEncoder5;//距離計測
				
				if(flag56 == 1){//ハーフ後の距離補正
					date_f_buff_int[date_f_num] -= date_f_plus_h;
					flag56 = 0;	
				}
				
				if(mode == 0){
					//if(date_f_buff_int[date_f_num] - date_f_brake < SEncoderTotal)flag = 1;//記録した直線を走った
					if(date_f_buff_int[date_f_num] - 500 < SEncoderTotal)flag = 1;//記録した直線を走った
				}else{
					//if(date_f_buff_int[date_f_num] - date_f_brake - 500 < SEncoderTotal)flag = 1;//記録した直線を走った
					if(date_f_buff_int[date_f_num] - 500 - 600 < SEncoderTotal)flag = 1;//記録した直線を走った
				}
				flag20 = 0;
			
			}else if(pattern == 21 || pattern == 22  ){
				if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//次の直線待ち状態
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//記録済み直線終了
				}
				
				if(flag20 == 0){
					flag20 = 1;
					flag56 = 0;
					//SEncoderTotal = 0;
					SEncoderTotal = iEncoder5;
				}else{
					SEncoderTotal += iEncoder5;//クロスラインなどからの距離計測
				}
				
			}else if( pattern == 51 || pattern == 52 || pattern == 61 || pattern == 62 ){
				if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//次の直線待ち状態
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//記録済み直線終了
				}
				
				if(flag20 == 0){
					flag20 = 1;
					flag56 = 1;
					//SEncoderTotal = 0;
					SEncoderTotal = iEncoder5;
				}else{
					SEncoderTotal += iEncoder5;//クロスラインなどからの距離計測
				}
				
			}else{//カーブ or クランク,ハーフ曲がり中 
			
				SEncoderTotal = 0;
				
				if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//次の直線待ち状態
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//記録済み直線終了
				}
			}
		
			if(flag20 == 99){//記録済み直線終了
				//クランク、ハーフも終了
				if(date_f_buff_ch_int[date_f_num_ch] == 0 && (pattern == 11 || pattern == 10))date_f_mode = 0;//再生走行終了
			}
		}
		
		
		
		/* microSD記録処理 */
    	if( msdFlag == 1 || msdFlag == 2 ) {
			p = msdBuff + msdBuffAddress;

            /* バッファに記録 ここから */
            *p++ = pattern;             /* パターン                     */
            *p++ = (char)Center;    
            *p++ = (char)Wide;
			i = getServoAngle();        /* 角度                         */
			*p++ = i >> 8;
            *p++ = i & 0xff;
			
			*p++ = (char)Center_IR;//赤外線センサー
			
			*p++ = handleBuff;  /* サーボPWM保存        */
            *p++ = FleftMotorBuff;       /* 前左モータPWM値                */
            *p++ = FrightMotorBuff;      /* 前右モータPWM値                */
			*p++ = RleftMotorBuff;       /* 後左モータPWM値                */
            *p++ = RrightMotorBuff;      /* 後右モータPWM値                */
			*p++ = iEncoder5;    /* エンコーダ                   */
            *p++ = mode;/*モード*/
            *p++ = flag2;/*坂道回数*/
            *p++ = (char)(angle_y/10);
            *p++ = (char)(angle_x/10);
			
		//	*p++ = (char)(IR_L());
        //    *p++ = (char)(IR_R());
            
			
            /* バッファに記録 ここまで */

            msdBuffAddress += 64;       /* RAMの記録アドレスを次へ      */


			if( msdBuffAddress >= 512) {	
                /* 512個になったら、microSDに記録する */
                msdBuffAddress = 0;
                setMicroSDdata( msdBuff );
                msdWorkAddress += 512;
				
                if( msdWorkAddress >= msdEndAddress || msdFlag == 2) {
                    /* 記録処理終了 */
    				msdFlag = 0;
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
        /* iTimer10変数の処理 */
        iTimer10 = 0;
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
    trcgrb = trcgrb_buff;
    trcgrd = trcgrd_buff;
}


/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0〜15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3〜P1_0読み込み           */

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0〜255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3上のプッシュスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.3のCN6の状態読み込み                     */
/* 引数　 なし                                                          */
/* 戻り値 0〜15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}


/************************************************************************/
/* モータドライブ基板TypeS Ver.3のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char led )
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
    types_led = LED;
	*/
	types_led = led;
	
}


/************************************************************************/
/* 後輪の速度制御 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100〜100 , 右モータ:-100〜100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < -100){
		accele_l = -100;
	}
	
	
	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < -100){
		accele_r = -100;
	}
	
	
	RleftMotorBuff  = accele_l;          /* バッファに保存               */
    RrightMotorBuff = accele_r;         /* バッファに保存               */
	

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
/* 引数　 左モータ:-100〜100 , 右モータ:-100〜100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < -100){
		accele_l = -100;
	}
	

	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < -100){
		accele_r = -100;
	}
	
	
	FleftMotorBuff  = accele_l;          /* バッファに保存               */
    FrightMotorBuff = accele_r;         /* バッファに保存               */
	
	
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = trcgrb_buff = trcgra;
    } else {
		if(accele_l == 100)trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
		
		//trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = trcgrd_buff = trcgra;
    } else {
		if(accele_r == 100)trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
		
		//trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}


/************************************************************************/
/* 後輪の速度制御3 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100〜100 , 右モータ:-100〜100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < -100)accele_l = -100;
	
	
	if(accele_l == 0 || (RleftMotorBuff > 0 && accele_l < 0) || (RleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - RleftMotorBuff  > KASOKU)accele_l = RleftMotorBuff + KASOKU;
	else if(accele_l - RleftMotorBuff < -KASOKU)accele_l = RleftMotorBuff - KASOKU;
	
	
	if(accele_r > 100)accele_r = 100;
	else if(accele_r < -100)accele_r = -100;
	
	
	if(accele_r == 0 || (RrightMotorBuff > 0 && accele_r < 0) || (RrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - RrightMotorBuff > KASOKU)accele_r = RrightMotorBuff + KASOKU;
	else if(accele_r - RrightMotorBuff < -KASOKU)accele_r = RrightMotorBuff- KASOKU;
	
	
	RleftMotorBuff  = accele_l;          /* バッファに保存               */
    RrightMotorBuff = accele_r;          /* バッファに保存               */

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
/* 引数　 左モータ:-100〜100 , 右モータ:-100〜100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < -100)accele_l = -100;
	
	
	if(accele_l == 0  || (FleftMotorBuff > 0 && accele_l < 0) || (FleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - FleftMotorBuff  > KASOKU)accele_l = FleftMotorBuff + KASOKU;
	else if(accele_l - FleftMotorBuff < -KASOKU)accele_l = FleftMotorBuff - KASOKU;
	

	if(accele_r > 100)accele_r = 100;
	else if(accele_r < -100)accele_r = -100;
	
	
	if(accele_r == 0 || (FrightMotorBuff > 0 && accele_r < 0) || (FrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - FrightMotorBuff > KASOKU)accele_r = FrightMotorBuff + KASOKU;
	else if(accele_r - FrightMotorBuff < -KASOKU)accele_r = FrightMotorBuff- KASOKU;
	

	FleftMotorBuff  = accele_l;          /* バッファに保存               */
    FrightMotorBuff = accele_r;          /* バッファに保存               */
		
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = trcgrb_buff = trcgra;
    } else {
		//if(accele_l == 100)p0_5 = 1;
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = trcgrd_buff = trcgra;
    } else {
		//if(accele_r == 100)p0_6 = 1;
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
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
/* 引数　 サーボモータPWM：-100〜100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
	int i = getServoAngle();
	
	if((i < -SERVO_MAX && pwm  > 0) || ( SERVO_MAX < i && pwm  < 0)){//ハンドル曲げすぎ
		pwm = -(pwm / 8);//逆転				
	}
	
	pwm = -pwm;//
	
	if( pwm >  100 ) pwm =  100;        /* マイコンカーが安定したら     */
    if( pwm < -100 ) pwm = -100;        /* 上限を90くらいにしてください */
    
	handleBuff = pwm;                 /* バッファに保存               */


    if( pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -pwm ) / 100;
    }
}


/************************************************************************/
/* サーボ角度取得                                                       */
/* 引数　 なし                                                          */
/* 戻り値 入れ替え後の値                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( ad2 - iAngle0 );
}

/************************************************************************/
/* スタートゲート検出処理												*/
/* 戻り値 0:なし 1:あり													*/
/************************************************************************/
unsigned char check_startgate( void )
{
	unsigned char ret;
	
	ret = 0;
	
	//if(IR_R() > 140){//37
	if(850 < ad7){ 
		ret = 1;
	}
	/*
	if((Wide >= 27) && (Center > -20) && (Center < 20)){
	
		ret = 1;			// ゲート発見 
	}*/
	return ret;
}
 

/************************************************************************/
/* クロスライン検出処理													*/
/* 戻り値 0:クロスラインなし 1:あり										*/
/************************************************************************/
unsigned char check_crossline( void )
{
	unsigned char ret;
	
	//cam_in();//値の取得
//	saka_c_flag = 0;
	ret = 0;
	if( (Wide > 60) || ((Wide >= 40) && (-10 < Center ) && (Center < 10))   || ((Wide >= 32) && (Center > -5) && (Center < 5)) ){
	
		ret = 1;			/* クロスライン発見 */
/*		
	}else if(((IR_L() < 110 &&  IR_R() < 110)&& (Wide >= 22) && (Center > -3) && (Center < 3))){
		saka_c_flag = 1; 	
		ret = 1;			
*/
	}
	return ret;
}
 

/************************************************************************/
/* ハーフライン検出処理                                                 */
/* 戻り値 0:ハーフラインなし 1:左 2:右 3:クランク                       */
/************************************************************************/
unsigned char check_halfline( void )
{
    unsigned char ret;
	int center;
	
	ret = 0;
	if(Wide > 44 && Wide < 50){
		if(Center < -12){//センター左寄り
			ret = 1;
			
		}else if(Center > 12){//センター右寄り
			ret = 2;
			
		}
	}else if(Wide > 34){
		if(Center < -10){//センター左寄り
			ret = 1;
			
		}else if(Center > 10){//センター右寄り
			ret = 2;
			
		}
		
	}else if(Wide > 25){
		if(Center < -7){//センター左寄り
			ret = 1;
			
		}else if(Center > 7){//センター右寄り
			ret = 2;
			
		}
	}
	
	return ret;
}

unsigned char check_halfline_forC( void ) //クランク時の直角確認用
{
    unsigned char ret;
	int center;
	
	ret = 0;
	if(Wide > 28){
		if(Center < -1){//センター左寄り
			ret = 1;
			
		}else if(Center > 1){//センター右寄り
			ret = 2;
			
		}
	}
	
	return ret;
}

/************************************************************************/
/* wideline検出処理													*/
/* 戻り値 0:なし 1:あり										*/
/************************************************************************/
unsigned char check_wideline( void )
{
	unsigned char ret;
	
	//cam_in();//値の取得
	
	ret = 0;
	if(Wide >= 25){
	
		ret = 1;			/* wideライン発見 */
	}
	return ret;
}

/************************************************************************/
/* サーボモータ制御                                                     */
/* 引数　 なし                                                          */
/* 戻り値 グローバル変数 iServoPwm に代入                               */
/************************************************************************/
void servoControl( void )
{
	long     iRet, iP, iD;
	static int iI = 0;

  
    /* サーボモータ用PWM値計算 */
	if(IR_flag == 0){	
		iP = kp * (Center + Center_offset);      /* 比例                         */
		iI = iI + (iSensorBefore - (Center + Center_offset) );
    	iD = kd * (iSensorBefore - (Center + Center_offset));     /* 微分(目安はPの5〜10倍)       */
		iRet = iP - iD - iI * ki;
	}else{
		iP = kp_ir * (Center_IR);      /* 比例                         */
		iI = iI + (iSensorBeforeIR - (Center_IR) );
    	iD = kd_ir * (iSensorBeforeIR - (Center_IR));     /* 微分(目安はPの5〜10倍)       */
		iRet = iP - iD - iI * ki_ir;
	}
    iRet /= 2;//256 128

    /* PWMの上限の設定 */
    if( iRet >  100 ) iRet =  100;        /* マイコンカーが安定したら     */
    if( iRet < -100) iRet = -100;        /* 上限を90くらいにしてください */
    iServoPwm = iRet;

    iSensorBefore = (Center + Center_offset );                  /* 次回はこの値が1ms前の値となる*/
	iSensorBeforeIR = Center_IR;
}

/************************************************************************/
/* モジュール名 servoControl2                                            */
/* 処理概要     サーボモータ制御                                        */
/* 引数         なし                                                    */
/* 戻り値       グローバル変数 iServoPwm2 に代入                         */
/************************************************************************/
void servoControl2( void )
{
    int      i,j, iRet, iP, iD;


    i = iSetAngle;              	/* 目標角度             */
    j = getServoAngle();              	/* 目標角度センサ値             */

    /* サーボモータ用PWM値計算 */
    iP = - kp2 * (j - i);                        /* 比例                         */
    iD = - kd2 * (iAngleBefore2 - j);     /* 微分(目安はPの5〜10倍)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWMの上限の設定 */
    if( iRet >  100 ) iRet =  100;        /* マイコンカーが安定したら     */
    if( iRet < -100 ) iRet = -100;        /* 上限を90くらいにしてください */

    iServoPwm2 = iRet;

    iAngleBefore2 = j;                  /* 次回はこの値が1ms前の値となる*/
}

/************************************************************************/
/*	wait																*/
/************************************************************************/
void wait(int t)
{
	cnt3 = 0;
	while (cnt3 <= t);
}

/**************************************************************************/
/*	RX62Gからのカメラの値を取得											  */
/**************************************************************************/
void cam_in(){
	int wide = 0;
	
	/*	
	//手配線
	wide += ((p6 >> 5 ) & 0x07);//012
	wide += ((p3 << 2 ) & 0x08);//3
	wide += ((p3 << 1 ) & 0x30);//45
	wide += ( p3 & 0x40);//6
	*/
	
	//NEW
	wide += ((p3 >> 6 ) & 0x01);
	wide += ((p3 >> 3 ) & 0x02);
	wide += ((p3 >> 1 ) & 0x04);
	wide += ((p3 << 2 ) & 0x08);
	wide += ((p6 >> 3 ) & 0x10);
	wide += ((p6 >> 1 ) & 0x20);
	wide += ((p6 << 1 ) & 0x40);
	
	
	Wide = wide;

	//Center = (p5 & 0x7f);
	
	if(1 < Wide  && Wide < 100){//カメラの取り付けで少しずれていすので
		Center = (p5 & 0x7f) - 64 + 3;	
	}else{
		Center = (p5 & 0x7f) - 64;
	}
}

/**************************************************************************/
/*	RX62Gへモードの値を出力											  */
/**************************************************************************/
void mode_out(){
	
	p3_5 = mode & 0x01;//LOW
	p3_7 = (mode & 0x02) >> 1;//HIGH
}

/**************************************************************************/
/*	LED表示用の値を計算													  */
/**************************************************************************/
int camera(int center, int wide){
	
	int start = 17,end = 111,i,led = 0,cnt = 1;
	// (end - start)/7 = 13
	int led_bit[8] = {999,48,55,61,999,67,74,81};//4は不調
	
	center += 64;
	wide /= 2;
	
	for(i = 0; i < 8; i++){
		if(center - wide < led_bit[i] && led_bit[i] < center + wide){
			led += cnt;
		}
		cnt *= 2;
	}

	return led;
}


/**************************************************************************/
/*	ジャイロセンサーのY軸の値を取得										  */
/**************************************************************************/
void get_angle_y(){
	
	pre_angle_y[0] = pre_angle_y[1];
	pre_angle_y[1] = angle_y;
	
	angle_y = (angle_y * 9/10) + (ad0 * 1/10);
	
	if(((pre_angle_y[0] < pre_angle_y[1]) && (pre_angle_y[1] < angle_y))  || ((angle_y < pre_angle_y[1]) && (pre_angle_y[1] < pre_angle_y[0])))
		angle_y = pre_angle_y[1];
		
	else if(((pre_angle_y[1] < pre_angle_y[0]) && (pre_angle_y[0] < angle_y))  || ((angle_y < pre_angle_y[0]) && (pre_angle_y[0] < pre_angle_y[1])))
		angle_y = pre_angle_y[0];
}

/**************************************************************************/
/*	ジャイロセンサーのX軸の値を取得													  */
/**************************************************************************/
void get_angle_x(){
	pre_angle_x[0] = pre_angle_x[1];
	pre_angle_x[1] = angle_x;
	
	angle_x = (angle_x * 85/100) + (ad1 * 15/100);
	
	if(((pre_angle_x[0] < pre_angle_x[1]) && (pre_angle_x[1] < angle_x))  || ((angle_x < pre_angle_x[1]) && (pre_angle_x[1] < pre_angle_x[0])))
		angle_x = pre_angle_x[1];
		
	else if(((pre_angle_x[1] < pre_angle_x[0]) && (pre_angle_x[0] < angle_x))  || ((angle_x < pre_angle_x[0]) && (pre_angle_x[0] < pre_angle_x[1])))
		angle_x = pre_angle_x[0];
}

/**************************************************************************/
/*	ジャイロセンサーのX軸の値をチェック									  */
/**************************************************************************/
int angle_check(){

	if(200 < angle_y && angle_y < 400){
		if(angle_x <= 170)return 2;//上 170
		if(angle_x > 440)return 0;//下
	}
	return 1;//変化無し
}



/**************************************************************************/
/*	IR_L												  */
/**************************************************************************/
int IR_L(){
	
	return 100 * (ad6 - IR_min[0]) / (IR_max[0] - IR_min[0]);
}
/**************************************************************************/
/*	IR_R												  */
/**************************************************************************/
int IR_R(){
	
	return 100 * (ad7 - IR_min[1]) / (IR_max[1] - IR_min[1]);
}
/**************************************************************************/
/*	Get_Center_IR											  */
/**************************************************************************/
void Get_Center_IR(){
	int l = IR_L(),r = IR_R();
	IR_old = Center_IR;
	
	Center_IR = (r-l)/5;
	
	if(l < 15 && r < 15){//赤外線も遠くなったら前回値から
		if(	IR_old < 0)Center_IR = -30;
		else Center_IR = 30;
	}
	
}
/**************************************************************************/
/*	赤外線センサーのキャリブレーション												  */
/**************************************************************************/
void IRcalibration( ){
	int i ,j,l,r,p = 20,a = 60;

	iAngle0 = 0;
	iAngle0 = getServoAngle();  /* 0度の位置記憶                */
	i = getServoAngle();
	
	for(j = 0; j < 2;j++){
		while(-a < i){
			i = getServoAngle();
			servoPwmOut(p);
			r = ad7;
			if(IR_max[1] < r)IR_max[1] = r;
			if(r < IR_min[1])IR_min[1] = r;
			l = ad6;
			if(IR_max[0] < l)IR_max[0] = l;
			if(l < IR_min[0])IR_min[0] = l;
		}
	
		while(i < a){
			i = getServoAngle();
			servoPwmOut(-p);
			r = ad7;
			if(IR_max[1] < r)IR_max[1] = r;
			if(r < IR_min[1])IR_min[1] = r;
			l = ad6;
			if(IR_max[0] < l)IR_max[0] = l;
			if(l < IR_min[0])IR_min[0] = l;
		}
	}
	while(0 < i){
		i = getServoAngle();
		servoPwmOut(p);
		r = ad7;
		if(IR_max[1] < r)IR_max[1] = r;
		if(r < IR_min[1])IR_min[1] = r;
		l = ad6;
		if(IR_max[0] < l)IR_max[0] = l;
		if(l < IR_min[0])IR_min[0] = l;
	}
	servoPwmOut( 0 );
}

int max(int a,int b){
	if(a < b)return b;
	return a;	
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
