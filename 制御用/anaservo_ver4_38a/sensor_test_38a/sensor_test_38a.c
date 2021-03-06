/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     アナログセンサ基板TypeS Ver.2テストプログラム               */
/* バージョン   Ver.1.01                                                    */
/* Date         2011.06.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*
本プログラムは、
●アナログセンサ基板TypeS Ver.2
の動作確認を行うプログラムです。接続はポート0にします。
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

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void init( void );
unsigned char sensor_inp( void );
unsigned char center_inp( void );
unsigned char startbar_get( void );

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
unsigned long   cnt1;                   /* タイマ用                     */

/************************************************************************/
/* メインプログラム                                                     */
/************************************************************************/
void main( void )
{
    int             i;

    /* マイコン機能の初期化 */
    init();                             /* 初期化                       */
    init_uart0_printf( SPEED_9600 );    /* UART0とprintf関連の初期化    */
    asm(" fset I ");                    /* 全体の割り込み許可           */

    printf( "Analog Sensor PCB TypeS Test Program(R8C/38A Ver.) Ver1.00\n" );
    printf( "\n" );

    while( 1 ) {
        if( cnt1 >= 200 ) {
            cnt1 = 0;
            printf( "Left=%4d , Right=%4d , "
                    "Digital=%x , Center=%d , Bar=%d\r",
                        ad4, ad5, ~p0 >> 4,
                        ~p0_1 & 0x01, ~p0_0 & 0x01 );
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
    prc2 = 1;                           /* PD0のプロテクト解除          */
    pd0 = 0x00;                         /* 7-0:ｱﾅﾛｸﾞｾﾝｻ基板TypeS Ver.2  */
    pd1 = 0xd0;                         /* 5:RXD0 4:TXD0 3-0:DIP SW     */
    p2  = 0xc0;
    pd2 = 0xfe;                         /* 7-0:モータドライブ基板Ver.4  */
    pd3 = 0xff;                         /*                              */
    p4  = 0x20;                         /* P4_5のLED:初期は点灯         */
    pd4 = 0xb8;                         /* 7:XOUT 6:XIN 5:LED 2:VREF    */
    pd5 = 0xff;                         /*                              */
    pd6 = 0xff;                         /*                              */
    pd7 = 0xff;                         /*                              */
    pd8 = 0xff;                         /*                              */
    pd9 = 0x3f;                         /*                              */
    pur0 = 0x04;                        /* P1_3〜P1_0のプルアップON     */

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
    adinsel = 0x20;                     /* 入力端子P0の6端子を選択      */
    adcon1  = 0x30;                     /* A/D動作可能                  */
    asm(" nop ");                       /* φADの1サイクルウエイト入れる*/
    adcon0  = 0x01;                     /* A/D変換スタート              */
}

/************************************************************************/
/* タイマRB 割り込み処理                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
    cnt1++;
}

/************************************************************************/
/* アナログセンサ基板TypeS Ver.2のデジタルセンサ値読み込み              */
/* 引数　 なし                                                          */
/* 戻り値 左端、左中、右中、右端のデジタルセンサ 0:黒 1:白              */
/************************************************************************/
unsigned char sensor_inp( void )
{
    unsigned char sensor;

    sensor  = ~p0 & 0x0f;

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

    sensor  = ~p1_7 & 0x01;

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

    sensor  = ~p1_6 & 0x01;

    return sensor;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2011.06.01 Ver.1.00 作成
2012.02.23 Ver.1.01 アナログセンサ基板TypeS Ver.2のコメント変更
*/