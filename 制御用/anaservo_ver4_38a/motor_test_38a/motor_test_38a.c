/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     モータドライブ基板TypeS Ver.4・アナログセンサ基板TypeS Ver.2*/
/*              テストプログラム                                            */
/* バージョン   Ver.2.00                                                    */
/* Date         2013.05.10                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*
本プログラムは、
●モータドライブ基板TypeS Ver.3 および Ver.4
●アナログセンサ基板TypeS Ver.2
の各基板の動作確認を行うプログラムです。
*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "printf_lib.h"                 /* printf使用ライブラリ         */

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

#define 	SERVO_MAX 			130	  	/* ハンドル最大位置 164           */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char startbar_get( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char cn6_get( void );
void led_out( unsigned char led );
void motor_r( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
void beep_out( int flag );
int getServoAngle( void );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
int             pattern;                /* マイコンカー動作パターン     */
unsigned long   cnt1;                   /* タイマ用                     */

/* エンコーダ関連 */
int             iTimer10;               /* 10msカウント用               */
long            lEncoderTotal;          /* 積算値保存用                 */
int             iEncoder;               /* 10ms毎の最新値               */
unsigned int    uEncoderBuff;           /* 計算用　割り込み内で使用     */

/* TRCレジスタのバッファ */
unsigned int    trcgrb_buff;            /* TRCGRBのバッファ             */
unsigned int    trcgrd_buff;            /* TRCGRDのバッファ             */

/* モータドライブ基板TypeS Ver.4上のLED、ディップスイッチ制御 */
unsigned char   types_led;              /* LED値設定                    */
unsigned char   types_dipsw;            /* ディップスイッチ値保存       */

int             iAngle0;                /* 中心時のA/D値保存            */

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    int             i, j;
    unsigned int    u;
    char            c;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
    asm(" fset I ");                    /* 全体の割り込み許可           */

    /* マイコンカーの状態初期化 */
    motor_mode_f( BRAKE, BRAKE );
    motor_mode_r( BRAKE, BRAKE );
    motor_f( 0, 0 );
    motor_r( 0, 0 );
    servoPwmOut( 0 );

	iAngle0 = getServoAngle();  /* 0度の位置記憶                */
	
    while( 1 ) {
        switch( pattern ) {
        case 0:
            /* メニュー */
            printf( "\n\n" );
            printf( "Motor Drive PCB TypeS Ver.4 "
                                "Test Program(R8C/38A Ver.) Ver1.00\n" );
            printf( "\n" );
            printf( "1 : LEDのテスト\n" );
            printf( "2 : スイッチのテスト\n" );
            printf( "3 : CN6の入力テスト\n" );
            printf( "4 : ブザーのテスト\n" );
            printf( "5 : エンコーダのテスト\n" );
            printf( "6 : ボリュームのテスト\n" );
            printf( "7 : アナログセンサ基板のテスト\n" );
            printf( "8 : モータのテスト\n" );
            printf( "\n" );
            printf( "1-8の数字を入力してください " );
            pattern = 1;
            break;

        case 1:
            /* 数字の入力 */
            if( get_uart0(&c) ) {
                if( c >= '1' && c <= '8' ) {
                    printf( "%c\n\n" , c );
                    cnt1 = 0;
                    pattern = (c - 0x30) * 10 + 1;
                }
            }
            break;

        case 11:
            /* LEDのテスト */
            printf( "現在LED3～10を順番に点灯中です。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 12;
            break;

        case 12:
            led_out( 1 << (cnt1 / 500) );
            if( cnt1 >= 4000 ) cnt1 = 0;
            if( get_uart0(&c) == 1 ) {
                led_out( 0 );
                pattern = 0;
            }
            break;

        case 21:
            /* スイッチのテスト */
            printf( "現在スイッチの値を表示中です。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 22;
            break;

        case 22:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "ディップSW(SW2)の値 : %02x ", dipsw_get2() );
                printf( "プッシュSW(SW3)の値 : %1x\r", pushsw_get() );
            }
            break;

        case 31:
            /* CN6の入力テスト */
            printf( "CN6に入力されている値を表示中です。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 32;
            break;

        case 32:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "CN6の入力値 : %1x\r", cn6_get() );
            }
            break;

        case 41:
            /* ブザーのテスト */
            printf( "ブザーをテストします。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 42;
            break;

        case 42:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                beep_out( 0 );
                break;
            }
            if( cnt1 <= 1000 ) {
                beep_out( 1 );
            } else if( cnt1 <= 2000 ) {
                beep_out( 0 );
            } else {
                cnt1 = 0;
            }
            break;

        case 51:
            /* エンコーダのテスト */
            printf( "エンコーダの入力パルス数を表示します。" );
            printf( "終わったらどれかキーを押してください。\n" );
            lEncoderTotal = 0;
            pattern = 52;
            break;

        case 52:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "エンコーダのパルス値 : %8ld\r", lEncoderTotal );
            }
            break;

        case 61:
            /* ボリュームのテスト */
            printf( "ボリューム電圧を表示中です。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 62;
            break;

        case 62:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                i =  ad2;
                j = (long)500 * i / 1023;
                printf( "ボリュームの電圧 : %4d (%01d.%02dV)\r",
                                                i, j/100, j%100 );
            }
            break;

        case 71:
            /* アナログセンサ基板TypeS Ver.2のテスト */
            printf( "アナログセンサ基板の値を表示中です。" );
            printf( "終わったらどれかキーを押してください。\n" );
            pattern = 72;
            break;

        case 72:
            if( get_uart0(&c) == 1 ) {
                pattern = 0;
                break;
            }
            if( cnt1 >= 200 ) {
                cnt1 = 0;
                printf( "Left=%4d , Right=%4d , "
                        "Digital=%1x , Center=%1d , Bar=%1d\r",
                            ad1, ad0, sensor_inp(),
                            center_inp(), startbar_get() );
            }
            break;

        case 81:
            /* モータ(5個)のテスト */
            printf( "モータのテストをします。\n" );
            printf( "1 : 左後ろモータ\n" );
            printf( "2 : 右後ろモータ\n" );
            printf( "3 : サーボモータ\n" );
            printf( "4 : 左前モータ\n" );
            printf( "5 : 右前モータ\n" );
            printf( "0 : 終了\n" );
            printf( "\n" );
            printf( "0-5の数字を入力してください " );
            pattern = 82;
            break;

        case 82:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    pattern = 0;
                    break;
                } else if( c >= '1' && c <= '5' ) {
                    printf( "%c\n\n" , c );
                    cnt1 = 0;
                    pattern = (c - 0x31) * 10 + 101;
                }
            }
            break;

        case 101:
            /* 左後ろモータのテスト */
            printf( "左後ろモータをテストします。\n" );
            printf( "スペースキーを押して動作を変えます。" );
            printf( "0キーで終了です。\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 102;
            break;

        case 102:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "停止中                     \r" );
                    j = i;
                }
                motor2_r( 0, 0 );
                motor_mode_r( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "正転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 50, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "逆転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( -50, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "正転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 50, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "逆転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( -50, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 111:
            /* 右後ろモータのテスト */
            printf( "右後ろモータをテストします。\n" );
            printf( "スペースキーを押して動作を変えます。" );
            printf( "0キーで終了です。\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 112;
            break;

        case 112:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "停止中                     \r" );
                    j = i;
                }
                motor2_r( 0, 0 );
                motor_mode_r( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "正転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, 50 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "逆転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, -50 );
                    motor_mode_r( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "正転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, 50 );
                    motor_mode_r( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "逆転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_r( 0, -50 );
                    motor_mode_r( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_r( 0, 0 );
                    motor_mode_r( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 121:
            /* サーボモータのテスト */
            printf( "サーボモータをテストします。\n" );
            printf( "スペースキーを押して動作を変えます。" );
            printf( "0キーで終了です。\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 122;
            break;

        case 122:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    servoPwmOut( 0 );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 3 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "停止中                     \r" );
                    j = i;
                }
                servoPwmOut( 0 );
                break;
            case 1:
                if( i != j ) {
                    printf( "正転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    servoPwmOut( 50 );
                } else if(  cnt1 <= 2000 ) {
                    servoPwmOut( 0 );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "逆転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    servoPwmOut( -50 );
                } else if(  cnt1 <= 2000 ) {
                    servoPwmOut( 0 );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 131:
            /* 左前モータのテスト */
            printf( "左前モータをテストします。\n" );
            printf( "スペースキーを押して動作を変えます。" );
            printf( "0キーで終了です。\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 132;
            break;

        case 132:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "停止中                     \r" );
                    j = i;
                }
                motor2_f( 0, 0 );
                motor_mode_f( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "正転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 50, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "逆転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( -50, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "正転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 50, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "逆転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( -50, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( FREE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        case 141:
            /* 右前モータのテスト */
            printf( "右前モータをテストします。\n" );
            printf( "スペースキーを押して動作を変えます。" );
            printf( "0キーで終了です。\n" );
            cnt1 = 0;
            i = 0;
            j = 1;
            pattern = 142;
            break;

        case 142:
            if( get_uart0(&c) ) {
                if( c == '0' ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                    pattern = 81;
                    break;
                } else if( c == ' ' ) {
                    cnt1 = 0;
                    i++;
                    if( i >= 5 ) i = 0;
                }
            }
            switch( i ) {
            case 0:
                if( i != j ) {
                    printf( "停止中                     \r" );
                    j = i;
                }
                motor2_f( 0, 0 );
                motor_mode_f( BRAKE , BRAKE );
                break;
            case 1:
                if( i != j ) {
                    printf( "正転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, 50 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 2:
                if( i != j ) {
                    printf( "逆転50%% <-> ブレーキ動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, -50 );
                    motor_mode_f( BRAKE , BRAKE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , BRAKE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 3:
                if( i != j ) {
                    printf( "正転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, 50 );
                    motor_mode_f( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            case 4:
                if( i != j ) {
                    printf( "逆転50%% <-> フリー　動作中\r" );
                    j = i;
                }
                if( cnt1 <= 1000 ) {
                    motor2_f( 0, -50 );
                    motor_mode_f( BRAKE , FREE );
                } else if(  cnt1 <= 2000 ) {
                    motor2_f( 0, 0 );
                    motor_mode_f( BRAKE , FREE );
                } else {
                    cnt1 = 0;
                }
                break;
            }
            break;

        default:
            break;
        }
    }
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

    /*  none            none            none            none
        none            none            none            エンコーダA相   */
    p3  = 0x00;
    pd3 = 0xfe;

	
    /*  XOUT            XIN             ボード上のLED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;

    /*  none            none            none            none
        none            none            none            none            */
    p5  = 0x00;
    pd5 = 0xff;

    /*  none            none            none            none
        none            none            none            none            */
    p6  = 0x00;
    pd6 = 0xff;

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
    trbic  = 0x07;                      /* 割り込み優先レベル設定       */
    trbcr  = 0x01;                      /* カウント開始                 */

    /* A/Dコンバータの設定 */
    admod   = 0x33;                     /* 繰り返し掃引モードに設定     */
    adinsel = 0x90;                     /* 入力端子P7の4端子を選択      */
    adcon1  = 0x30;                     /* A/D動作可能                  */
    asm(" nop ");                       /* φADの1サイクルウエイト入れる*/
    adcon0  = 0x01;                     /* A/D変換スタート              */

    /* タイマRG タイマモード(両エッジでカウント)の設定 */
    timsr = 0x40;                       /* TRGCLKA端子 P3_0に割り当てる */
    trgcr = 0x15;                       /* TRGCLKA端子の両エッジでカウント*/
    trgmr = 0x80;                       /* TRGのカウント開始            */


    /* タイマRC PWMモード設定(左前モータ、右前モータ) */
    trcmr   = 0x0f;                     /* PWMモード選択ビット設定      */
    trccr1  = 0x8e;                     /* ｿｰｽｶｳﾝﾄ:f1,初期出力の設定    */
    trccr2  = 0x00;                     /* 出力レベルの設定             */
    trcpsr0 = 0x40;                     /* TRCIOA,B端子の設定           */
    trcpsr1 = 0x33;                     /* TRCIOC,D端子の設定           */
    trcgra  = TRC_MOTOR_CYCLE - 1;      /* 周期設定                     */
    trcgrb  = trcgrb_buff = trcgra;     /* P0_5端子のON幅(左前モータ)   */
    trcgrc  = trcgra;                   /* P0_7端子のON幅(予備)         */
    trcgrd  = trcgrd_buff = trcgra;     /* P0_6端子のON幅(右前モータ)   */
    trcic   = 0x06;                     /* 割り込み優先レベル設定       */
    trcier  = 0x01;                     /* IMIAを許可                   */
    trcoer  = 0x01;                     /* 出力端子の選択               */
    trcmr  |= 0x80;                     /* TRCカウント開始              */

    /* タイマRD リセット同期PWMモード設定(左後ﾓｰﾀ、右後ﾓｰﾀ、ｽﾃｱﾓｰﾀ) */
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
    unsigned int    i;

    cnt1++;

    /* 10回中1回実行する処理 */
    iTimer10++;
    switch( iTimer10 ) {
    case 1:
        /* エンコーダ制御 */
        i = trg;
        iEncoder       = i - uEncoderBuff;
        lEncoderTotal += iEncoder;
        uEncoderBuff = i;
        break;

    case 2:
        /* スイッチ読み込み準備 */
        p9_4 = 0;                       /* LED出力OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* スイッチ読み込み、LED出力 */
        types_dipsw = ~p8;              /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.4のSW読み込み*/
        p8  = types_led;                /* ﾄﾞﾗｲﾌﾞ基板TypeS Ver.4のLEDへ出力*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED出力ON                    */
        break;

    case 10:
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
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned char sensor_inp( void )
{
    unsigned char sensor;

    sensor = ~p0 & 0x0f;

    return sensor;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2の中心デジタルセンサ読み込み            */
/* 引数　 なし                                                          */
/* 戻り値 中心デジタルセンサ 0:黒 1:白                                  */
/************************************************************************/
unsigned char center_inp( void )
{
    unsigned char sensor;

    sensor = ~p1_7 & 0x01;

    return sensor;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のスタートバー検出センサ読み込み        */
/* 引数　 なし                                                          */
/* 戻り値 0:スタートバーなし 1:スタートバーあり                         */
/************************************************************************/
unsigned char startbar_get( void )
{
    unsigned char sensor;

    sensor = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* マイコンボード上のディップスイッチ値読み込み                         */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3～P1_0読み込み           */

    return sw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.4上のディップスイッチ値読み込み          */
/* 引数　 なし                                                          */
/* 戻り値 スイッチ値 0～255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* 実際の入力はタイマRB割り込み処理で実施 */
    return types_dipsw;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.4上のプッシュスイッチ値読み込み          */
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
/* モータドライブ基板TypeS Ver.4のCN6の状態読み込み                     */
/* 引数　 なし                                                          */
/* 戻り値 0～15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}

/************************************************************************/
/* モータドライブ基板TypeS Ver.4のLED制御                               */
/* 引数　 8個のLED制御 0:OFF 1:ON                                       */
/* 戻り値 なし                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
    /* 実際の出力はタイマRB割り込み処理で実施 */
    types_led = led;
}

/************************************************************************/
/* 後輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 後輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
    /* 左後モータ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* 右後モータ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御                                                       */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
    int sw_data;

    sw_data  = dipsw_get() + 5;         /* ディップスイッチ読み込み     */
    accele_l = accele_l * sw_data / 20;
    accele_r = accele_r * sw_data / 20;

    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* 前輪の速度制御2 ディップスイッチには関係しないmotor関数              */
/* 引数　 左モータ:-100～100 , 右モータ:-100～100                       */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
    /* 左前モータ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= 5 ) {
        trcgrb = trcgrb_buff = trcgra;
    } else {
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* 右前モータ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= 5 ) {
        trcgrd = trcgrd_buff = trcgra;
    } else {
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
/* 引数　 サーボモータPWM：-100～100                                    */
/*        0で停止、100で正転100%、-100で逆転100%                        */
/* 戻り値 なし                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
	int i = getServoAngle();
		
		
/*	if((i < -SERVO_MAX && pwm  > 0) || ( SERVO_MAX < i && pwm  < 0)){//ハンドル曲げすぎ
		pwm = -(pwm / 2);//逆転				
	}
*/	
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
/* ブザー制御(テストプログラム版)                                       */
/* 引数　 0:ブザーOFF 1:ブザーON                                        */
/* 戻り値 なし                                                          */
/* メモ   実際のプログラムでは、types3_beep.cを使用して制御してください */
/************************************************************************/
void beep_out( int flag )
{
    if( flag ) {
        p0_4 = 1;
    } else {
        p0_4 = 0;
    }
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
2013.07.13 Ver.2.01 コメントを一部修正
*/
