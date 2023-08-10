/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     液晶制御ライブラリ                                          */
/* バージョン   Ver.1.02                                                    */
/* Date         2012.04.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    <stdio.h>                   /* 標準Cﾗｲﾌﾞﾗﾘ 入出力           */
#include    <stdarg.h>                  /* 標準Cﾗｲﾌﾞﾗﾘ 可変個数の実引数 */
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "lcd_lib.h"                 /* 液晶関連処理                 */

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/*
●液晶・microSD基板の液晶接続ポート
P5_7    P5_6    P5_5    P5_4    P5_3    P5_2    P5_1    P5_0
未使用  E       RW      RS      D7      D6      D5      D4
※D7～D4は、必ずポートのbit3～0に接続します。
*/

/* 液晶のポート、ビット配置 */
#define LCD_PORT            p5          /* 液晶の接続されているポート   */
#define LCD_PORT_DIR        pd5         /* 上記ポートの入出力設定ポート */
#define LCD_PORT_PULLUP     pu12        /* D7端子をプルアップするレジスタ*/

#define LCD_NODATA_BIT      0x80        /* 液晶で使っていないビット     */
#define LCD_E_BIT           0x40        /* 液晶 E bit                   */
#define LCD_RW_BIT          0x20        /* 液晶 RW bit                  */
#define LCD_RS_BIT          0x10        /* 液晶 RS bit                  */
#define LCD_D7_BIT          0x08        /* 液晶 D7 bit                  */

/* 液晶関連変数 */
#define LCD_MAX_X           16          /* 表示文字数 横 16 or 20       */
#define LCD_MAX_Y           2           /* 表示文字数 縦  2 or  4       */

/* 液晶制御コード */
#define LCD_INST            0x00        /* インストラクション           */
#define LCD_DATA            LCD_RS_BIT  /* データ                       */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
static volatile char            buffLcdData[ LCD_MAX_X * LCD_MAX_Y ];
                                        /* 表示バッファ                 */
static volatile char            buffLcdData2[ LCD_MAX_X * LCD_MAX_Y + 10 ];
                                        /* 表示バッファ一時作業エリア   */
static volatile int             lcdBuffPosition;
                                        /* バッファに書き込む位置       */
static volatile int             lcdMode = 1;
                                        /* 表示処理No管理               */
static volatile int             lcdNowLocate;
                                        /* 現在の表示している位置       */
static volatile unsigned long   lcdBusyCnt;
                                        /* busyスタートカウンタ         */
static volatile int             lcdRefreshFlag;
                                        /* リフレッシュフラグ           */
static volatile int             lcdConnect = 1;
                                        /* 液晶が接続されているか       */
static volatile unsigned long   lcdCnt; /* 液晶タイミング用カウンタ     */

/************************************************************************/
/* タイマ                                                               */
/* 極短いタイミング用タイマ 1当たり1usくらい                            */
/************************************************************************/
static void lcd_nop( volatile int i )
{
    while( i-- );
}

/************************************************************************/
/* タイマ                                                               */
/* 引数　 タイマ値 1=1ms                                                */
/************************************************************************/
static void lcdTimer( unsigned long timer_set )
{
    volatile unsigned long t;

    t = lcdCnt;

    while( lcdCnt - t < timer_set );
}

/************************************************************************/
/* モジュール名 lcdBusyStart                                            */
/* 処理概要     液晶のbusyスタート                                      */
/* 引数         busy値                                                  */
/* 戻り値       なし                                                    */
/************************************************************************/
static void lcdBusyStart( void )
{
    lcdBusyCnt = lcdCnt;
}

/************************************************************************/
/* モジュール名 lcdOut2                                                 */
/* 処理概要     lcdOutの上位、下位に分けて転送する部分                  */
/* 引数         4bitデータ                                              */
/* 戻り値       なし                                                    */
/************************************************************************/
static void lcdOut2( char data2 )
{
    volatile unsigned char b;

    LCD_PORT = LCD_PORT & LCD_NODATA_BIT | data2;
    lcd_nop(1);
    LCD_PORT |= LCD_E_BIT;
    lcd_nop(5);
    LCD_PORT &= ~LCD_E_BIT;
    lcd_nop(5);
}

/************************************************************************/
/* モジュール名 lcdOut                                                  */
/* 処理概要     液晶コマンド出力                                        */
/* 引数         コマンド種類、データ                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
static void lcdOut( char command, char data )
{
    volatile unsigned char work;

    /* 上位4bit転送 */
    work = (unsigned char)command | ((unsigned char)data >> 4);
    lcdOut2( work );

    /* 下位4bit転送 */
    work = (unsigned char)command | ((unsigned char)data & 0x0f);
    lcdOut2( work );
}

/************************************************************************/
/* モジュール名 lcdLocate                                               */
/* 処理概要     液晶カーソル移動                                        */
/* 引数         x , y                                                   */
/* 戻り値       なし                                                    */
/************************************************************************/
static void lcdLocate( int x, int y )
{
    volatile unsigned char work = 0x80;

    /* xの計算 */
    work += x;

    /* yの計算 */
    if( y == 1 ) {
        work += 0x40;
    } else if( y == 2 ) {
        work += 0x14;
    } else if( y == 3 ) {
        work += 0x54;
    }

    /* カーソル移動 */
    lcdOut( LCD_INST, work );
    lcdBusyStart();
}

/************************************************************************/
/* モジュール名 checkLcdBusy                                            */
/* 処理概要     液晶のbusyチェック                                      */
/* 引数         なし                                                    */
/* 戻り値       2:タイムアウト 1:ok 0:busy中                            */
/************************************************************************/
static int checkLcdBusy( void )
{
    volatile int ret;

    /* D3～D0を入力端子にする */
    LCD_PORT_PULLUP = 1;
    LCD_PORT_DIR   &= 0xf0;

    LCD_PORT &= ( ~LCD_E_BIT & ~LCD_RW_BIT & ~LCD_RS_BIT );

    LCD_PORT |= LCD_RW_BIT;
    lcd_nop(1);

    LCD_PORT |= LCD_E_BIT;
    lcd_nop(5);

    ret = (LCD_PORT & LCD_D7_BIT) ? 0 : 1;
    if( ret == 0 ) {
        /* busyなら、タイムアウトかチェック */
        if( lcdCnt - lcdBusyCnt >= 11 ) ret = 2;
    }

    LCD_PORT &= ~LCD_E_BIT;
    lcd_nop(5);

    LCD_PORT |= LCD_E_BIT;              /* 4bitモードなので             */
    lcd_nop(5);                         /* ダミーでE bitたてる          */

    LCD_PORT &= ~LCD_E_BIT;
    lcd_nop(5);

    LCD_PORT &= ~LCD_RW_BIT;

    /* D3～D0を出力端子にする */
    LCD_PORT_PULLUP = 0;
    LCD_PORT_DIR   |= 0x0f;

    return ret;
}

/************************************************************************/
/* モジュール名 initLcd                                                 */
/* 処理概要     液晶処理　初期化                                        */
/* 引数         なし                                                    */
/* 戻り値       0:異常 1:正常                                           */
/************************************************************************/
int initLcd( void )
{
    volatile int i;

    for( i=0; i<=LCD_MAX_X*LCD_MAX_Y-1; i++ ) { /* 変数初期化           */
        buffLcdData[ i ] = ' ';
    }

    lcdTimer( 15 );                     /* 15ms以上のタイマ             */

    lcdOut2( 0x03 );                    /* ﾌｧﾝｸｼｮﾝｾｯﾄ 8bit長            */
    lcdTimer( 5 );                      /* 4.1ms以上のタイマ            */

    lcdOut2( 0x03 );                    /* ﾌｧﾝｸｼｮﾝｾｯﾄ 8bit長            */
    lcdTimer( 2 );                      /* 100us以上のタイマ            */

    lcdOut2( 0x03 );                    /* ﾌｧﾝｸｼｮﾝｾｯﾄ 8bit長            */
    lcdBusyStart();
    while( !(i = checkLcdBusy()) );
    if( i == 2 ) {
        lcdConnect = 0;
        return 0;                       /* エラー終了                   */
    }

    lcdOut2( 0x02 );                    /* ﾌｧﾝｸｼｮﾝｾｯﾄ 4bit長            */
    lcdBusyStart();
    while( !checkLcdBusy() );

    lcdOut( LCD_INST, 0x28 );           /* 1行、font:5*7設定            */
    lcdBusyStart();
    while( !checkLcdBusy() );

    lcdOut( LCD_INST, 0x08 );           /* 表示off設定                  */
    lcdBusyStart();
    while( !checkLcdBusy() );

    lcdOut( LCD_INST, 0x01 );           /* 表示クリア                   */
    lcdBusyStart();
    while( !checkLcdBusy() );

    lcdOut( LCD_INST, 0x06 );           /* ｲﾝｸﾘﾒﾝﾄする、表示ｼﾌﾄ無し     */
    lcdBusyStart();
    while( !checkLcdBusy() );

    lcdOut( LCD_INST, 0x0c );           /* 表示ON                       */
    lcdBusyStart();
    while( !checkLcdBusy() );

    return 1;
}

/************************************************************************/
/* モジュール名 lcdShowProcess                                          */
/* 処理概要     液晶表示処理                                            */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/* メモ         この関数は割り込みで1msごとに実行してください           */
/************************************************************************/
void lcdShowProcess( void )
{
    if( !lcdConnect ) return;          /* 接続されているかチェック  */

    lcdCnt++;

    switch( lcdMode ) {
    case 1: /* データ更新されたかチェック */
        if( lcdRefreshFlag ) {
            lcdRefreshFlag = 0;
            lcdMode = 2;
        }
        break;

    case 2: /* 位置初期化*/
        if( checkLcdBusy() ) {
            lcdNowLocate = 0;
            lcdLocate( 0, 0 );
            lcdMode = 3;
        }
        break;

    case 3: /* 改行位置の確認 */
        if( checkLcdBusy() ) {
            if( lcdNowLocate % LCD_MAX_X == 0 ) {
                lcdLocate( 0, lcdNowLocate / LCD_MAX_X );
            }
            lcdMode = 4;
        }
        break;

    case 4: /* データ表示処理 */
        if( checkLcdBusy() ) {
            lcdOut( LCD_DATA, buffLcdData[ lcdNowLocate++ ] );
            lcdBusyStart();
            if( lcdNowLocate >= LCD_MAX_X * LCD_MAX_Y ) {
                lcdMode = 1;
            } else {
                lcdMode = 3;
            }
        }
        break;

    default:
        lcdMode = 1;
        break;
    }
    pd5 &= 0xf0;                        /* LCDと兼用の端子を入力に設定  */
}

/************************************************************************/
/* モジュール名 lcdPrintf                                               */
/* 処理概要     液晶へ表示　表示位置は過去に表示した位置の次から        */
/* 引数         printfと同じ                                            */
/* 戻り値       正常時：出力した文字列　異常時：負の数                  */
/************************************************************************/
int lcdPrintf(char *format, ...)
{
    volatile va_list argptr;
    volatile char    *p;
    volatile int     ret = 0;

    va_start(argptr, format);
    ret = vsprintf( buffLcdData2, format, argptr );
    va_end(argptr);

    if( ret > 0 ) {
        /* vsprintfが正常なら液晶バッファへ転送 */
        p = buffLcdData2;
        while( *p ) {
            buffLcdData[lcdBuffPosition++] = *p++;
            if( lcdBuffPosition >= LCD_MAX_X * LCD_MAX_Y ) {
                lcdBuffPosition = 0;
            }
        }
        lcdRefreshFlag = 1;
    }
    return ret;
}

/************************************************************************/
/* モジュール名 lcdPosition                                             */
/* 処理概要     液晶の表示位置指定                                      */
/* 引数         横位置 , 縦位置                                         */
/* 戻り値       なし                                                    */
/************************************************************************/
void lcdPosition(char x ,char y)
{
    if( x >= LCD_MAX_X ) return;
    if( y >= LCD_MAX_Y ) return;

    lcdBuffPosition = x + y * LCD_MAX_X;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2011.04.01 Ver.1.00 作成
2011.09.28 Ver.1.01 checkLcdBusy関数のタイムアウト処理修正
2012.04.01 Ver.1.02 最適化をONにしても動作するよう対応
*/
