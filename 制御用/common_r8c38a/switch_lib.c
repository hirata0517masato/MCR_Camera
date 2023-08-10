/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     液晶・microSD基板 スイッチ制御ライブラリ                    */
/* バージョン   Ver.1.00                                                    */
/* Date         2011.04.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "switch_lib.h"              /* スイッチ制御ライブラリ       */

/*======================================*/
/* 定数設定                             */
/*======================================*/
#define     SWITCH_BIT              5   /* スイッチの数                 */
#define     SWITCH_INTERVAL         10  /* スイッチ処理インターバル 1ms */
#define     SWITCH_ON_COUNT         3   /* ONと判断するまでの回数       */
                                        /* SWITCH_INTERVAL *            */
                                        /*         SWITCH_ON_COUNT [ms] */
#define     SWITCH_REPEAT_INTERVAL  50  /* ONしてからﾘﾋﾟｰﾄするまでの時間*/
                                        /* SWITCH_INTERVAL *            */
                                        /*  SWITCH_REPEAT_INTERVAL [ms] */
#define     SWITCH_REPEAT_COUNT     5   /* リピート間隔                 */
                                        /* SWITCH_INTERVAL *            */
                                        /*     SWITCH_REPEAT_COUNT [ms] */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
static volatile int     iSwitchIntervalCount;
                                        /* 何回ごとに処理するかのｶｳﾝﾄ   */

/* 1bitごとのキー情報 */
struct {
    int     mode;                       /* スイッチ処理モードNo用       */
    int     time;                       /* 信号が続いている時間         */
    int     flag;                       /* "0"→"1"で"1"になる          */
    int     count;                      /* カウント                     */
} static volatile sw_data[ SWITCH_BIT ];

/************************************************************************/
/* モジュール名 initSwitch                                              */
/* 処理概要     スイッチ関連変数初期化                                  */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void initSwitch( void )
{
    volatile int i;

    iSwitchIntervalCount = 0;
    pur1                |= 0x0c;        /* 3:P5_4～P5_7 プルアップON    */
                                        /* 2:P5_0～P5_3 プルアップON    */
    /* スイッチ */
    for( i=0; i<SWITCH_BIT; i++ ) {
        sw_data[i].mode     = 0;
        sw_data[i].time     = 0;
        sw_data[i].flag     = 0;
        sw_data[i].count    = 0;
    }
}

/************************************************************************/
/* モジュール名 getSwNow                                                */
/* 処理概要     スイッチ現在状態取得                                    */
/* 引数         なし                                                    */
/* 戻り値       unsigned char スイッチ値                                */
/************************************************************************/
unsigned char getSwNow( void )
{
/*
スイッチの入力ポートを変える場合は、この関数を変更します。
bit0から詰めて配置されるようにします。
現在の設定 bit  7    6    5    4    3    2    1    0
                --  --  --  P5_7 P5_3 P5_2 P5_1 P5_0
*/
    volatile unsigned char sw;

    pd5 &= 0xf0;                        /* LCDと兼用の端子を入力に設定  */

    sw  = (~p5 & 0x80) >> 3;            /* P5_7読み込み                 */
    sw |= (~p5 & 0x0f);                 /* P5_3～P5_0読み込み           */

    pd5 |= 0x0f;                        /* LCDと兼用の端子を出力に設定  */

    return sw;
}

/************************************************************************/
/* モジュール名 getSwFlag                                               */
/* 処理概要     スイッチフラグ状態取得                                  */
/* 引数         マスク値                                                */
/* 戻り値       unsigned char スイッチ値                                */
/************************************************************************/
unsigned char getSwFlag( unsigned char flag )
{
    volatile unsigned char ret;

    switch( flag ) {
    case SW_4:
        ret = sw_data[4].flag;
        sw_data[4].flag = 0;
        break;
    case SW_3:
        ret = sw_data[3].flag;
        sw_data[3].flag = 0;
        break;
    case SW_2:
        ret = sw_data[2].flag;
        sw_data[2].flag = 0;
        break;
    case SW_1:
        ret = sw_data[1].flag;
        sw_data[1].flag = 0;
        break;
    case SW_0:
        ret = sw_data[0].flag;
        sw_data[0].flag = 0;
        break;
    }
    return ret;
}

/************************************************************************/
/* モジュール名 switchProcess                                           */
/* 処理概要     スイッチ処理                                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void switchProcess( void )
{
    volatile int            i;
    volatile unsigned char  sw;

    /* 処理間隔のチェック */
    iSwitchIntervalCount++;
    if( iSwitchIntervalCount < SWITCH_INTERVAL ) return;

    iSwitchIntervalCount = 0;
    sw = getSwNow();

    /* スイッチのビット毎に処理する */
    for( i=0; i<SWITCH_BIT; i++ ) {

        switch( sw_data[i].mode ) {
        case 0:
            /* ONの判定 */
            if( sw & 0x01 ) {
                sw_data[i].time++;
                if( sw_data[i].time >= SWITCH_ON_COUNT ) {
                    sw_data[i].mode = 1;
                    sw_data[i].count++;
                    sw_data[i].flag = 1;
                    sw_data[i].time = 0;
                }
            } else {
                sw_data[i].time = 0;
            }
            break;

        case 1:
            /* キーリピートの判定 */
            if( sw & 0x01 ) {
                sw_data[i].time++;
                if( sw_data[i].time >= SWITCH_REPEAT_INTERVAL ) {
                    sw_data[i].mode = 2;
                    sw_data[i].count++;
                    sw_data[i].flag = 1;
                    sw_data[i].time = 0;
                }
            } else {
                sw_data[i].mode = 0;
                sw_data[i].time = 0;
            }
            break;

        case 2:
            /* キーリピート中、等間隔でフラグＯＮにする判定 */
            if( sw & 0x01 ) {
                sw_data[i].time++;
                if( sw_data[i].time >= SWITCH_REPEAT_COUNT ) {
                    sw_data[i].count++;
                    sw_data[i].flag = 1;
                    sw_data[i].time = 0;
                }
            } else {
                sw_data[i].mode = 0;
                sw_data[i].time = 0;
            }
            break;
        }

        sw >>= 1;
    }
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2011.04.01 Ver.1.00 作成
*/
