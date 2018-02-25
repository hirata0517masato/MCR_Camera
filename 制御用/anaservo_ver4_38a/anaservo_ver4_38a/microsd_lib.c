/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     microSD 制御ライブラリ                                      */
/* バージョン   Ver.3.11                                                    */
/* Date         2014.06.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/
/*

■FAT32を使わない場合
microSDをメモリとしてのみ使用し、FAT32を使わない場合は、FAT32部分のプログラム
をOFFにすることができます。
OFFにする場合は、下記のパラメータをツールチェインのコンパイラオプションに
追加してください。
-DNO_FAT32

*/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include <stdio.h>
#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>                     /* 標準Cﾗｲﾌﾞﾗﾘ 可変個数の実引数 */
#include "sfr_r838a.h"                  /* R8C/38A SFRの定義ファイル    */
#include "microsd_lib.h"                /* microSD制御ライブラリ        */

/*======================================*/
/* 定数設定                             */
/*======================================*/
#define MSD_CS_BIT      p6_1            /* CS端子のビット               */
#define MSD_CS_BIT_DIM  pd6_1           /* CS端子のビットの入出力設定   */
#define LED_BLINK       0x40            /* LED　アクセス時の点滅時間    */
#define LED_ERRORBLINK  0x200           /* LED　エラー時の点滅時間      */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
static unsigned char            msdlibBuff[512 + 128];
                                        /* 一時保管バッファ             */
static unsigned char            msdlibBuff2[512 + 128];
                                        /* 一時保管バッファ２           */
static volatile int             msdlibMode;
                                        /* 状態                         */
static volatile int             msdlibCnt;
                                        /* 書き込み数                   */
static volatile unsigned char*  msdlibRead;
                                        /* 読み込みデータのアドレス     */
static volatile unsigned char*  msdlibWrite;
                                        /* 書き込みデータのアドレス     */
static volatile int             msdlibError;
                                        /* エラー番号                   */
static volatile char*           led_port;
                                        /* モニタLEDのあるポート        */
static volatile char*           led_port_dim;
                                        /* 上記の入出力設定ポート       */
static volatile char            led_port_bit;
                                        /* モニタLEDのあるポートのビット*/
static volatile unsigned int    msdLed;
                                        /* microSD モニタ用LED          */
                                        /* 0:使用せず 0以外:使用        */
static volatile int             msdType;
                                        /* microSDのタイプ              */
                                        /* 1:MMC or SDカードVer.1.x     */
                                        /* 2:SDカードVer.2.0            */
                                        /* 3:SDHCカード                 */
static volatile unsigned long   msdSize;
                                        /* microSDのサイズ(kB単位)      */
static volatile int             modeSector;
                                        /* 1:セクタ指定モード 0:アドレス*/
/////// FAT32関係変数 ここから /////////
#ifndef NO_FAT32
static volatile int             fFileOpen;
                                        /* 書き込み作業中は1            */
static volatile unsigned long   BPB_SctorNo;
                                        /* BPBのセクタNo                */
static volatile unsigned long   BPB_SectorsPerFAT;
                                        /* FATのセクタ数                */
static volatile unsigned int    BPB_SctorsPerCluster;
                                        /* 1クラスタのセクタ数          */
static volatile int             BPB_NumberOfFATs;
                                        /* FATの数 通常は2              */
static volatile int             BPB_ReservedSectors;
                                        /* BPBの領域のセクタ数          */
static volatile unsigned long   BPB_bigTotalSectors;
                                        /* 総セクタ数                   */
static volatile unsigned long   BPB_RootDirCluster;
                                        /* ﾙｰﾄﾃﾞｨﾚｸﾄﾘがあるクラスタ番号 */
static volatile unsigned long   Sec_FAT[4];
                                        /* FATの先頭アドレス            */
static volatile unsigned long   Sec_Data;
                                        /* データ領域の先頭アドレス     */
static volatile unsigned long   Sec_Dir;
                                        /* ディレクトリの先頭アドレス   */
static volatile unsigned long   writeSectorStart;
                                        /* 書き込み開始セクタ           */
static volatile unsigned long   writeSectorEnd;
                                        /* 書き込み終了セクタ           */
static volatile unsigned long   writeSectorWork;
                                        /* 書き込み中のセクタ           */

static const char monthStr[] = { "JanFebMarAprMayJunJulAugSepOctNovDec" };
                                        /* 月変換テーブル               */
static volatile unsigned char   fileYear  = 2012-1980;  /* 書き込み　年 */
static volatile unsigned char   fileMonth = 4;          /* 書き込み　月 */
static volatile unsigned char   fileDay   = 1;          /* 書き込み　日 */
static volatile unsigned char   fileHour  = 12;         /* 書き込み　時 */
static volatile unsigned char   fileMin   = 0;          /* 書き込み　分 */
static volatile unsigned char   fileSec   = 0 / 2;      /* 書き込み　秒 */

static volatile char            msdPrintfBuff_Format[128];
                                        /* msdPrintfの書式              */
static volatile char            msdPrintfBuff_Para[64];
                                        /* msdPrintfの引数              */
static volatile char            *pFormat;
                                        /* msdPrintfの書式のポインタ    */
static volatile char            *pPara;
                                        /* msdPrintfの引数のポインタ    */
static volatile char            *pWriteBuff = msdlibBuff;
                                        /* microSD書き込みﾊﾞｯﾌｧのﾎﾟｲﾝﾀ  */
static volatile int             writeBuffNo;
                                        /* 0:msdlibBuff に書き込み      */
                                        /* 1:msdlibBuff2に書き込み      */
static volatile int             msdPrintfMode;
                                        /*microSDPrintfProcess関数の状態*/
#endif
/////// FAT32関係変数 ここまで /////////

/************************************************************************/
/* モジュール名 msd_write                                               */
/* 処理概要     microSD　１バイト書き込み                               */
/* 引数         char データ                                             */
/* 戻り値       なし                                                    */
/************************************************************************/
static void msd_write( unsigned char data )
{
    volatile unsigned int i;

    u1tbl = data;
    while( ri_u1c1 == 0 );              /* 受信データなしなら繰り返す   */
    i = u1rb;                           /* ダミーリード                 */
}

/************************************************************************/
/* モジュール名 msd_read                                                */
/* 処理概要     microSD　１バイト読み込み                               */
/* 引数         なし                                                    */
/* 戻り値       char データ                                             */
/************************************************************************/
static unsigned char msd_read( void )
{
    volatile unsigned char  ret;
    volatile unsigned int   data;

    u1tbl = 0xff;                       /* ダミーライト                 */
    while( ri_u1c1 == 0 );              /* 受信データなしなら繰り返す   */

    data = u1rb;
    ret = data & 0x00ff;

    return ret;
}

/************************************************************************/
/* モジュール名 sendCMD                                                 */
/* 処理概要     コマンド送信                                            */
/* 引数         コマンド、引数1,引数2,引数3,引数4,CRC                   */
/* 戻り値       microSDからの戻り値                                     */
/************************************************************************/
static unsigned char sendCMD(
    unsigned char cmd,  unsigned char arg1, unsigned char arg2,
    unsigned char arg3, unsigned char arg4, unsigned char crc   )
{
    volatile int            i;
    volatile unsigned char  ret;

    msd_write( 0xff );

    msd_write( cmd );
    msd_write( arg1 );
    msd_write( arg2 );
    msd_write( arg3 );
    msd_write( arg4 );
    msd_write( crc );

    /* レスポンス受信 */
    i = 1500;                           /* リトライ数 Ver3.11 500→1500 */
    while( --i ) {
        MSD_CS_BIT = 1;

        MSD_CS_BIT = 0;

        ret = msd_read();
        if( ret != 0xff ) break;        /* 0xff以外で終了               */
    }
    return ret;
}

/************************************************************************/
/* モジュール名 initMicroSD                                             */
/* 処理概要     microSDカードの初期化                                   */
/* 引数         なし                                                    */
/* 戻り値       0:正常 1以上:エラー                                     */
/************************************************************************/
int initMicroSD( void )
{
    volatile int i, r, ret = 0;
    unsigned char rr[16];

    msdlibMode = 1;

    MSD_CS_BIT_DIM = 1;                 /* CS_BITは出力端子に設定       */

    /* UART1の設定 */
    pd6_4 = 0;                          /* P6_4/RXD1は入力端子に設定    */
    u1sr  = 0x2a;                       /* UART1の端子選択              */
    s1tic = 0x00;                       /* UART1送信割り込み禁止        */
    s1ric = 0x00;                       /* UART1受信割り込み禁止        */
    u1mr  = 0x01;                       /* ｸﾛｯｸ同期式ｼﾘｱﾙI/Oﾓｰﾄﾞに設定  */
    u1c0  = 0x80;                       /* MSBファースト、f1を選択      */
    u1c1  = 0x00;                       /* 送信割り込み要因選択         */
    u1brg = 200-1;                      /* ビットレート 20MHz * 1/f1 *  */
                                        /*       1/200 * 1/2 = 50000bps */
    te_u1c1 = 1;                        /* 送信許可ビット：送信許可     */
    re_u1c1 = 1;                        /* 受信許可ビット：受信許可     */

    /* 各端子を設定 */
    MSD_CS_BIT = 1;

    for( i=0; i<10; i++ );

    /* 10バイトダミーで送りながら、入力端子チェック */
    for( i=0; i<10; i++ ) {
        r = msd_read();
        if( r != 0xff ) {               /* 未接続？                     */
            ret = 1;
            goto initMicroSD_End;
        }
    }

    MSD_CS_BIT = 0;

    /* CMD0送信 */
    i = 0;
    do {
        /* LED点滅処理 */
        if( msdLed == 0x4000 ) msdLed += LED_BLINK;

        r = sendCMD( 0x40, 0x00, 0x00, 0x00, 0x00, 0x95 );
        i++;
        if( i >= 10 ) {
            ret = 2;
            goto initMicroSD_End;
        }
    } while( (r&0xfe) != 0x00 );

    /* CMD8送信 */
    msdType = 0;
    i = 0;
    do {
        /* LED点滅処理 */
        if( msdLed == 0x4000 ) msdLed += LED_BLINK;

        r = sendCMD( 0x48, 0x00, 0x00, 0x01, 0xaa, 0x87 );
        i++;
        if( i >= 20 ) {
            msdType = MSD_TYPE_SD100;
            break;
        }
    } while( r == 0xff );

    if( msdType != MSD_TYPE_SD100 ) {
        /* R7コマンド読み込み */
        for( i=0; i<5; i++ ) {
            rr[i] = msd_read();
        }
        if( (rr[2] != 0x01) || (rr[3] != 0xaa) ) {
            ret = 3;
            goto initMicroSD_End;
        }
    }

    if( msdType == MSD_TYPE_SD100 ) {
        // MMCカード Ver.2以前のカード

        /* CMD1送信 */
        i = 0;
        do {
            /* LED点滅処理 */
            if( msdLed == 0x4000 ) msdLed += LED_BLINK;

            MSD_CS_BIT = 1;
            msd_write( 0xff );
            MSD_CS_BIT = 0;

            r = sendCMD( 0x41, 0x00, 0x00, 0x00, 0x00, 0xf9 );
            i++;
            if( i >= 500 ) {
                ret = 4;
                goto initMicroSD_End;
            }
        } while( r != 0x00 );

        /* CMD16送信 */
        i = 0;
        do {
            r = sendCMD( 0x50, 0x00, 0x00, 0x02, 0x00, 0x00 );
            i++;
            if( i >= 10 ) {
                ret = 5;
                goto initMicroSD_End;
            }
        } while( r != 0x00 );
    } else {
        // Ver.2.00以降のSDカード

        // CMD58
        i = 0;
        do {
            /* LED点滅処理 */
            if( msdLed == 0x4000 ) msdLed += LED_BLINK;

            r = sendCMD( 0x7a, 0x40, 0x00, 0x00, 0x00, 0xaf );
            i++;
            if( i >= 50 ) {
                ret = 6;
                goto initMicroSD_End;
            }
        } while( r != 0x01 );
        // 残り受信
        for( i=0; i<6; i++ ) {
            rr[i] = msd_read();
        }

        // ACMD41
        i = 0;
        do {
            /* LED点滅処理 */
            if( msdLed == 0x4000 ) msdLed += LED_BLINK;

            // CMD55
            r = sendCMD( 0x77, 0x00, 0x00, 0x00, 0x00, 0x65 );
            // ACMD41
            r = sendCMD( 0x69, 0x40, 0x00, 0x00, 0x00, 0x77 );
            i++;
            if( i >= 1000 ) {
                ret = 7;
                goto initMicroSD_End;
            }
        } while( r != 0x00 );

        // CMD58
        i = 0;
        do {
            /* LED点滅処理 */
            if( msdLed == 0x4000 ) msdLed += LED_BLINK;

            r = sendCMD( 0x7a, 0x40, 0x00, 0x00, 0x00, 0xaf );
            i++;
            if( i >= 10 ) {
                ret = 8;
                goto initMicroSD_End;
            }
        } while( r != 0x00 );
        // 残り受信
        for( i=0; i<6; i++ ) {
            rr[i] = msd_read();
        }
        if( rr[0] & 0x40 ) msdType = MSD_TYPE_SDHC;
        else msdType = MSD_TYPE_SD200;
    }

    /* 容量の計算 */
    r = getMicroSD_CSD( rr );
    if( r != 0x00 ) {
        ret = 9;
        goto initMicroSD_End;
    }

    if( msdType != MSD_TYPE_SDHC ) {
        // SDHC以外
        msdSize   = ((unsigned int)rr[ 6]&0x03) << 10;
        msdSize  |=  (unsigned int)rr[ 7]       <<  2;
        msdSize  |= ((unsigned int)rr[ 8]&0xc0) >>  6;
        msdSize  += 1;
        i         = ((unsigned int)rr[ 9]&0x03) <<  1;
        i        |= ((unsigned int)rr[10]&0x80) >>  7;
        i        += 2;
        msdSize <<= i;
        i         = (unsigned int)rr[5] & 0x0f;
        msdSize <<= i;
        msdSize >>= 10;                 /* 単位 kB                      */
    } else {
        // SDHCの場合
        msdSize   = ((unsigned int)rr[7]&0x3f) << 16;
        msdSize  |=  (unsigned int)rr[8] << 8;
        msdSize  |=  (unsigned int)rr[9];
        msdSize  += 1;
        msdSize <<= 9;                  /* 単位 kB                      */
    }

initMicroSD_End:
    MSD_CS_BIT = 1;

    if( ret == 0 ) {
        // 初期化　成功
        te_u1c1 = 0;                    /* 送信許可ビット：送信禁止     */
        re_u1c1 = 0;                    /* 受信許可ビット：受信禁止     */
        u1brg   = 80-1;                 /* ビットレート 20MHz * 1/80 *  */
                                        /*              1/2 = 125000bps */
        te_u1c1 = 1;                    /* 送信許可ビット：送信許可     */
        re_u1c1 = 1;                    /* 受信許可ビット：受信許可     */

        msdlibMode = 0;
    } else {
        // 初期化　エラー
        msdlibMode = 99;
        if( msdLed ) msdLed = 0x8000;
    }
    return ret;
}

/************************************************************************/
/* モジュール名 getMsdType                                              */
/* 処理概要     microSDのタイプを取得                                   */
/* 引数         無し                                                    */
/* 戻り値       MSD_TYPE_SD100、MSD_TYPE_SD200、MSD_TYPE_SDHC           */
/************************************************************************/
int getMsdType( void )
{
    return msdType;
}

/************************************************************************/
/* モジュール名 getMsdSize                                              */
/* 処理概要     microSDのサイズを取得                                   */
/* 引数         無し                                                    */
/* 戻り値       サイズ（kB単位)                                         */
/************************************************************************/
unsigned long getMsdSize( void )
{
    return msdSize;
}

/************************************************************************/
/* モジュール名 readMicroSD                                             */
/* 処理概要     microSDからデータ読み込み(512バイト)                    */
/* 引数         unsigned long   アドレス                                */
/*              signed char     *読み込む配列のアドレス                 */
/* 戻り値       0:正常 1以上:エラー                                     */
/************************************************************************/
int readMicroSD( unsigned long address, signed char *read )
{
    volatile int            i, r, ret = 0;
    volatile unsigned char  a1, a2, a3, a4;

    if( msdlibMode != 0 ) {             /* 別な作業中ならエラー         */
        return 1;
    }
    msdlibMode = 1;

    /* LED点滅処理 */
    if( msdLed == 0x4000 ) msdLed += LED_BLINK;

    if( modeSector == 1 ) {
        // セクタ指定
        if( msdType != MSD_TYPE_SDHC ) {
            // SDHCでないアドレス→セクタ番号に変換
            address <<= 9;
        }
    } else {
        // アドレス指定
        if( msdType == MSD_TYPE_SDHC ) {
            // SDHCならアドレス→セクタ番号に変換
            address >>= 9;
        }
    }

    a1 = (unsigned char)( (address&0xff000000) >> 24);
    a2 = (unsigned char)( (address&0x00ff0000) >> 16);
    a3 = (unsigned char)( (address&0x0000ff00) >>  8);
    a4 = (unsigned char)(  address&0x000000ff       );

    MSD_CS_BIT = 1;

    /* ダミークロック送信 */
    msd_write( 0xff );

    MSD_CS_BIT = 0;

    /* ダミークロック送信 */
    msd_write( 0xff );

    /* CMD17送信 */
    r = sendCMD( 0x51, a1, a2, a3, a4, 0xff );

    /* レスポンス確認 */
    if( r != 0x00 ) {
        ret = 11;
        goto readMicroSD_End;
    }

    /* 0xfeか確認 */
    i = 0;
    do {
        r = msd_read();
        i++;
        if( i >= 1000 ) {
            ret = 12;
            goto readMicroSD_End;
        }
    } while( r != 0xfe );

    /* データ読み込み */
    for( i=0; i<512; i++ ) {
        *read++ = msd_read();
    }

    /* CRC 2バイト読み込み */
    msd_read();
    msd_read();

readMicroSD_End:
    /* ダミークロック送信 */
    msd_write( 0xff );
    MSD_CS_BIT = 1;
    msdlibMode = (ret != 0) ? 99 : 0;   /* 初期化エラーなら99をセット   */

    return ret;
}

/************************************************************************/
/* モジュール名 writeMicroSD                                            */
/* 処理概要     microSD書き込み(512バイト)                              */
/* 引数         unsigned long   アドレス                                */
/*              signed char     *書き込む配列のアドレス                 */
/* 戻り値       0:正常 1以上:エラー                                     */
/************************************************************************/
int writeMicroSD( unsigned long address, signed char *write )
{
    volatile int            i, r, ret = 0;
    volatile unsigned char  a1, a2, a3, a4;

    if( msdlibMode != 0 ) {             /* 別な作業中ならエラー         */
        return 1;
    }
    msdlibMode = 1;

    /* LED点滅処理 */
    if( msdLed == 0x4000 ) msdLed += LED_BLINK;

    if( modeSector == 1 ) {
        // セクタ指定
        if( msdType != MSD_TYPE_SDHC ) {
            // SDHCでないアドレス→セクタ番号に変換
            address <<= 9;
        }
    } else {
        // アドレス指定
        if( msdType == MSD_TYPE_SDHC ) {
            // SDHCならアドレス→セクタ番号に変換
            address >>= 9;
        }
    }

    MSD_CS_BIT = 0;

    /* CMD24送信 */
    a1 = (unsigned char)( (address&0xff000000) >> 24);
    a2 = (unsigned char)( (address&0x00ff0000) >> 16);
    a3 = (unsigned char)( (address&0x0000ff00) >>  8);
    a4 = (unsigned char)(  address&0x000000ff       );
    r = sendCMD( 0x58, a1, a2, a3, a4, 0x00 );

    /* レスポンス確認 */
    if( (r&0x1f) != 0x00 ) {
        ret = 21;
        goto writeMicroSD_End;
    }

    /* 1バイト時間を空ける */
    r = msd_read();

    /* 開始合図 */
    msd_write( 0xfe );

    /* データ送信 */
    for( i=0; i<512; i++ ) {
        msd_write( *write++ );
    }

    /* CRC送信 */
    msd_write( 0xff );
    msd_write( 0xff );

    /* レスポンス読み込み */
    r = msd_read();
    r &= 0x1f;

    if( r == 0x05 ) {
        ret = 0;        /* 書き込み正常終了 */
    } else if( r == 0x0d ) {
        ret = 22;       /* 書き込みエラー   */
    } else {
        ret = 23;       /* それ以外のエラー */
    }

    /* busy状態が終わるまで待つ */
    for( i=0; i<10000; i++ ) {
        if( msd_read() != 0x00 ) break;
    }

writeMicroSD_End:
    /* ダミークロック送信 */
    msd_write( 0xff );
    MSD_CS_BIT = 1;
    msdlibMode = (ret != 0) ? 99 : 0;   /* 初期化エラーなら99をセット   */

    return ret;
}

/************************************************************************/
/* モジュール名 getMicroSD_CSD                                          */
/* 処理概要     microSD Card Specific Data取得                          */
/* 引数         signed char *読み込み配列(16バイト以上)                 */
/* 戻り値       0:正常 1以上:エラー                                     */
/************************************************************************/
int getMicroSD_CSD( unsigned char *p )
{
    volatile int i, r, ret = 0;

    /* LED点滅処理 */
    if( msdLed == 0x4000 ) msdLed += LED_BLINK;

    MSD_CS_BIT = 0;

    /* CMD9送信 */
    r = sendCMD( 0x49, 0x00, 0x00, 0x00, 0x00, 0xaf );

    /* レスポンス確認 */
    i = 0;
    do {
        r = msd_read();
        i++;
        if( i >= 1000 ) {
            ret = 31;
            goto getMicroSD_CSD_End;
        }
    } while( r & 0x80 );

    *p++ = r;

    /* CSD読み込み */
    for( i=1; i<16; i++ ) {
        *p++ = msd_read();
    }

    /* ダミーリード */
    msd_read();
    msd_read();

getMicroSD_CSD_End:
    /* ダミークロック送信 */
    msd_write( 0xff );
    MSD_CS_BIT = 1;

    return ret;
}

/************************************************************************/
/* モジュール名 eraseMicroSD                                            */
/* 処理概要     microSDのデータイレース(0x00書き込み)                   */
/* 引数         unsinged long 開始アドレス , 終了アドレス               */
/* 戻り値       0:正常 1以上:エラー                                     */
/************************************************************************/
int eraseMicroSD( unsigned long st_address, unsigned long ed_address )
{
    volatile int            i, j, r, ret = 0;
    volatile unsigned char  a1, a2, a3, a4;

    if( msdlibMode != 0 ) {             /* 別な作業中ならエラー         */
        return msdlibMode;
    }
    msdlibMode = 1;

    if( msdLed ) msdLed += LED_BLINK;

    if( modeSector == 1 ) {
        // セクタ指定
        if( msdType != MSD_TYPE_SDHC ) {
            // SDHCでないセクタ→アドレス番号に変換
            st_address <<= 9;
            ed_address <<= 9;
            ed_address += 511;
        }
    } else {
        // アドレス指定
        if( msdType == MSD_TYPE_SDHC ) {
            // SDHCならアドレス→セクタ番号に変換
            st_address >>= 9;
            ed_address >>= 9;
        }
    }

    MSD_CS_BIT = 0;

    /* CMD32送信(イレース開始アドレスのセット) */
    a1 = (unsigned char)( (st_address&0xff000000) >> 24);
    a2 = (unsigned char)( (st_address&0x00ff0000) >> 16);
    a3 = (unsigned char)( (st_address&0x0000ff00) >>  8);
    a4 = (unsigned char)(  st_address&0x000000ff       );
    r = sendCMD( 0x60, a1, a2, a3, a4, 0xff );

    /* レスポンス確認 */
    if( r != 0x00 ) {
        ret = 41;
        goto eraseMicroSD_End;
    }

    for( j=0; j<10000; j++ );           // Ver.3.11追加

    /* CMD33送信(イレース終了アドレスのセット) */
    a1 = (unsigned char)( (ed_address&0xff000000) >> 24);
    a2 = (unsigned char)( (ed_address&0x00ff0000) >> 16);
    a3 = (unsigned char)( (ed_address&0x0000ff00) >>  8);
    a4 = (unsigned char)(  ed_address&0x000000ff       );
    r = sendCMD( 0x61, a1, a2, a3, a4, 0xff );

    /* レスポンス確認 */
    if( r != 0x00 ) {
        ret = 42;
        goto eraseMicroSD_End;
    }

    for( j=0; j<10000; j++ );           // Ver.3.11追加

    /* CMD38送信(イレース) */
    r = sendCMD( 0x66, 0, 0, 0, 0, 0xff );

    /* レスポンス確認 */
    if( r != 0x00 ) {
        ret = 43;
        goto eraseMicroSD_End;
    }

    /* busy状態が終わるまで待つ */
    for( i=0; i<10000; i++ ) {
        if( msd_read() != 0x00 ) break;
    }

eraseMicroSD_End:
    /* ダミークロック送信 */
    for( j=0; j<10000; j++ );           // Ver.3.11追加
    msd_write( 0xff );
    MSD_CS_BIT = 1;

    /* イレース後、１回ダミーライト */
    if( ret == 0 ) {
        for( i=0; i<512; i++ ) msdlibBuff[i] = 0;   /* バッファクリア   */

        msdlibMode = 0;
        r = writeMicroSD( st_address, (signed char*)msdlibBuff );
        if( r != 0x00 ) {
            /* 書き込みできず */
            // たまにエラーを返すmicroSDがあるが、正常なようなので
            // エラーは無視しておく
            //ret = 44;
        }
    }

    msdlibMode = (ret != 0) ? 99 : 0;   /* 初期化エラーなら99をセット   */

    return ret;
}

/************************************************************************/
/* モジュール名 setMicroSDdata                                          */
/* 処理概要     microSDにデータセット                                   */
/* 引数         microSDに書き込むデータのある配列                       */
/* 戻り値       12:正常に終了 それ以外:書き込み中で今回のセットは無効   */
/************************************************************************/
int setMicroSDdata( signed char *p )
{
    volatile int ret = 0;

    /* LED点滅処理 */
    if( msdLed == 0x4000 ) msdLed += LED_BLINK;

#ifndef NO_FAT32
    /* 転送中のエラーがないかチェック */
    if( msdlibMode == 0 ) {
        if( fFileOpen ) {
            if( msdLed ) msdLed = 0x8000;
            fFileOpen = 0;
            return 1;
        }
    }

    /* ファイルとして書き込み時の上限のチェック */
    if( fFileOpen ) {
        if( writeSectorWork > writeSectorEnd ) {
            fFileOpen = 0;
            return 1;
        }
        writeSectorWork++;
    }
#endif

    if( msdlibMode != 11) {             /* 現在書き込み処理中か？       */
        goto setMicroSDdata_End;
    }

    /* 512バイト msdlibBuffへ転送 */
    msdlibRead = (unsigned char*)p;     /* 転送元セット                 */
    _asm( " mov.b _msdlibRead >> 16   ,r1h " );
    _asm( " mov.w _msdlibRead & 0ffffh, a0 " );
    _asm( " mov.w #_msdlibBuff        , a1 " );
    _asm( " mov.w #256                , r3 " );
    _asm( " smovf.w" );

    msdlibBuff[512] = 0xff;
    msdlibBuff[513] = 0xff;

    msdlibCnt = 514;                    /* バッファ書き込み数           */
    msdlibWrite = msdlibBuff;
    s1tic = 0x02;
    u1tbl = 0xfc;                       /* 1個だけ送信(残りは割込みで)  */
    msdlibMode = 12;                    /* microSDProcess 処理開始      */

setMicroSDdata_End:
    return msdlibMode;
}

/************************************************************************/
/* モジュール名 microSDProcessStart                                     */
/* 処理概要     microSDProcess開始処理                                  */
/* 引数         microSDの書き込みアドレス                               */
/* 戻り値       0:正常に終了 それ以外:既に書き込み中                    */
/************************************************************************/
int microSDProcessStart( unsigned long address )
{
    volatile int            ret = 0, r;
    volatile unsigned char  a1, a2, a3, a4;

    if( msdlibMode != 0 ) {
        /* 既に処理中 */
        ret = 1;
        goto microSDProcessStart_End;
    }

    if( msdLed ) msdLed += LED_BLINK;

    if( modeSector == 1 ) {
        // セクタ指定
        if( msdType != MSD_TYPE_SDHC ) {
            // SDHCでないアドレス→セクタ番号に変換
            address <<= 9;
        }
    } else {
        // アドレス指定
        if( msdType == MSD_TYPE_SDHC ) {
            // SDHCならアドレス→セクタ番号に変換
            address >>= 9;
        }
    }

    /* microSDに書き込みアドレスセット */
    a1 = (unsigned char)( (address&0xff000000) >> 24);
    a2 = (unsigned char)( (address&0x00ff0000) >> 16);
    a3 = (unsigned char)( (address&0x0000ff00) >>  8);
    a4 = (unsigned char)(  address&0x000000ff       );

    MSD_CS_BIT = 0;

    /* CMD25送信 */
    r = sendCMD( 0x59, a1, a2, a3, a4, 0xff );

    /* レスポンス確認 */
    if( (r&0x80) != 0x00 ) {
        ret = 2;                        /* セットできず                 */
        MSD_CS_BIT = 1;
    } else {
        msdlibMode = 11;                /* セット完了                   */
    }

microSDProcessStart_End:
    return ret;
}

/************************************************************************/
/* モジュール名 microSDProcessEnd                                       */
/* 処理概要     microSDProcess終了処理                                  */
/* 引数         microSDの書き込みアドレス                               */
/* 戻り値       0:正常に終了 それ以外:既に書き込み中                    */
/************************************************************************/
int microSDProcessEnd( void )
{
    volatile int ret = 1;

    if( msdlibMode == 11 ) {
        msdlibMode = 21;                /* 終了処理セット               */
    } else if( msdlibMode == 0 ) {
        /* LED点滅処理 */
        msdLed = 0;
        ret = 0;                        /* 終了                         */
    }

    return ret;
}

/************************************************************************/
/* モジュール名 microSDProcess                                          */
/* 処理概要     microSD　間欠書き込み処理                               */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/* メモ         1msごとに割り込み処理内から実行してください             */
/************************************************************************/
void microSDProcess( void )
{
    volatile int i, r;

    /* microSD msdPrintf関数処理(1msごとに実行) */
    microSDPrintfProcess();

    /* モニタLED処理 */
    if( msdLed ) {
        if( msdLed & 0x8000 ) {
            /* エラーなら */
            msdLed |= LED_ERRORBLINK;
            msdLed--;
            if( msdLed & (LED_ERRORBLINK>>1) ) {
                *led_port |= led_port_bit;
            } else {
                *led_port &= ~led_port_bit;
            }
        } else if( msdLed & 0x4000 ) {
            /* 点滅なら */
            if( msdLed & 0x3fff ) msdLed--;
            msdLed &= ( 0x4000 | ((LED_BLINK<<1)-1) );
            if( msdLed & (LED_BLINK>>1) ) {
                *led_port |= led_port_bit;
            } else {
                *led_port &= ~led_port_bit;
            }
        }
    }

    switch( msdlibMode ) {
    case 0:
        /* 開始前の待機 */
        break;

    case 1:
        /* 別な作業中 */
        break;

    case 11:
        /* 開始後の待機　何もせず */
        break;

    case 12:
        /* 送信割り込みでmicroSDにデータ書き込み中 */
        if( msdlibCnt == 0 ) {
            msdlibMode = 13;
        }
        break;

    case 13:
        /* 最後のデータ送るまで待つ */
        if( ri_u1c1 == 1 ) {
            i = u1rb;                   /* ダミーリード                 */
            msdlibMode = 14;
        }
        break;

    case 14:
        /* 終了処理 */

        /* レスポンス読み込み */
        r = msd_read();
        r &= 0x1f;

        if( r == 0x05 ) {
            msdlibError = 2;            /* 書き込み正常終了             */
            msdlibMode = 15;
        } else if( r == 0xc ) {
            msdlibError = 3;            /* 書き込みエラー               */
            msdlibMode = 31;
        } else {
            msdlibError = 4;            /* それ以外のエラー             */
            msdlibMode = 31;
        }
        break;

    case 15:
        /* busy状態が終わるまで待つ */
        if( msd_read() != 0x00 ) {
            msdlibMode = 11;
        }
        break;

    case 21:
        /* 終了処理 */
        msd_write( 0xfd );
        msdlibMode = 22;
        break;

    case 22:
        /* busy状態が終わるまで待つ */
        if( msd_read() != 0x00 ) {
            msdlibMode = 31;
        }
        break;

    case 31:
        /* CS="1" */
        msd_write( 0xff );              /* ダミークロック送信           */
        MSD_CS_BIT = 0;
        msdlibMode = 32;                /* 処理終了                     */
        msdlibCnt = 10;
        break;

    case 32:
        msdlibCnt--;
        if( msdlibCnt == 0 ) {
            msdlibMode = 0;             /* 処理終了                     */
        }
        break;

    case 99:
        /* 初期化エラー */
        break;
    }
}

/************************************************************************/
/* モジュール名 checkMicroSDProcess                                     */
/* 処理概要     microSD　間欠書き込み処理の終了チェック                 */
/* 引数         なし                                                    */
/* 戻り値       0:処理無し 11:開始後の待機 それ以外:処理中              */
/************************************************************************/
int checkMicroSDProcess( void )
{
  return msdlibMode;
}

/************************************************************************/
/* UART1送信 割り込み処理                                               */
/************************************************************************/
#pragma interrupt intS1T(vect=19)
void intS1T( void )
{
    u1tbl = *msdlibWrite++;             /* データ送信                   */
    msdlibCnt--;

    if( ri_u1c1 == 1 ) u1rb;            /* ダミーリード                 */

    if( msdlibCnt == 0 ) {              /* 送信データが無いなら         */
        s1tic = 0x00;
    }
}

/************************************************************************/
/* モジュール名 setMicroSDLedPort                                       */
/* 処理概要     microSD　モニタLEDの設定                                */
/* 引数         char* モニタLEDのあるポート                             */
/*              char* モニタLEDのあるポートの入出力設定ポート           */
/*              int   モニタLEDのあるポートのビット                     */
/* 戻り値       なし                                                    */
/************************************************************************/
void setMicroSDLedPort( char *p, char *pd, int bit )
{
    led_port     = p;
    led_port_dim = pd;
    led_port_bit = 0x01 << bit;

    *led_port_dim |= led_port_bit;      /* LEDのbitを出力に設定         */

    msdLed = 0x4000;                    /* LED点灯動作 ON               */
}

//////////////////////////////////////////////////////////////////////////
// FAT32関係プログラムここから                                          //
//////////////////////////////////////////////////////////////////////////
#ifndef NO_FAT32

/************************************************************************/
/* モジュール名 convertBinaryToLong                                     */
/* 処理概要     バイナリーデータ4bytesをlongへ変換                      */
/* 引数　       変換元のアドレス                                        */
/* 戻り値       変換後のlong型の値                                      */
/************************************************************************/
static unsigned long convertBinaryToLong( char *p )
{
    unsigned long data;

    data  = *(unsigned long*)(p+3) << 24;
    data |= *(unsigned long*)(p+2) << 16;
    data |= *(unsigned long*)(p+1) <<  8;
    data |= *(unsigned long*)(p+0);

    return  data;
}

/************************************************************************/
/* モジュール名 mountMicroSD_FAT32                                      */
/* 処理概要     FAT32でマウント(FAT16は未対応)                          */
/* 引数　       無し                                                    */
/* 戻り値       0:FAT32でマウント完了 1:マウントできず                  */
/************************************************************************/
int mountMicroSD_FAT32( void )
{
    int ret, i;

    modeSector = 1;                     // アドレス選択＝セクタ

    // 0番地を読み込んでMBRかBPBか確認
    ret = readMicroSD( 0x0000 , (signed char*)msdlibBuff );
    if( ret != 0 ) {
        ret = 1;
        goto mountMicroSD_FAT32_Error;
    }

    // MBR または BPBでないならエラー
    if( msdlibBuff[510] != 0x55 || msdlibBuff[511] != 0xaa ) {
        ret = 2;
        goto mountMicroSD_FAT32_Error;
    }

    // MBRセクタならBPBセクタ番号読み込み
    if( msdlibBuff[0] != 0xeb && msdlibBuff[0] != 0xe9 ) {
        BPB_SctorNo = convertBinaryToLong( msdlibBuff+454 );

        // BPB読み込み
        readMicroSD( BPB_SctorNo , (signed char*)msdlibBuff );
        if( ret != 0 ) {
            ret = 3;
            goto mountMicroSD_FAT32_Error;
        }

        // BPBでないならエラー
        if( msdlibBuff[510] != 0x55 || msdlibBuff[511] != 0xaa ) {
            ret = 4;
            goto mountMicroSD_FAT32_Error;
        }
        // BPBでないならエラー
        if( msdlibBuff[0] != 0xeb && msdlibBuff[0] != 0xe9 ) {
            ret = 5;
            goto mountMicroSD_FAT32_Error;
        }
    }

    // FAT16のBPBセクタ読み込み
    if( msdlibBuff[22] != 0x00 || msdlibBuff[23] != 0x00 ) {
        // FAT12orFAT16は未対応
        ret = 11;
        goto mountMicroSD_FAT32_Error;
    }

    // BPBに関する計算
    BPB_SctorsPerCluster = (unsigned int)msdlibBuff[13];
    BPB_NumberOfFATs     = (unsigned int)msdlibBuff[16];

    BPB_ReservedSectors  = (unsigned int)msdlibBuff[15] << 8;
    BPB_ReservedSectors |= (unsigned int)msdlibBuff[14];

    BPB_bigTotalSectors  = convertBinaryToLong( msdlibBuff+32 );

    BPB_SectorsPerFAT    = convertBinaryToLong( msdlibBuff+36 );

    BPB_RootDirCluster   = convertBinaryToLong( msdlibBuff+44 );

    // FATのセクタ計算
    for( i=0; i<BPB_NumberOfFATs; i++ ) {
        Sec_FAT[i] = BPB_SctorNo+BPB_ReservedSectors + BPB_SectorsPerFAT*i;
    }

    // データのセクタ計算
    Sec_Data = BPB_SctorNo +
            BPB_ReservedSectors + BPB_SectorsPerFAT * BPB_NumberOfFATs;

    // ディレクトリセクタ計算
    Sec_Dir  = (BPB_RootDirCluster-2) * BPB_SctorsPerCluster;
    Sec_Dir += Sec_Data;

    modeSector = 0;                     // アドレス選択＝アドレス

    return 0;

mountMicroSD_FAT32_Error:
    modeSector = 0;                     // アドレス選択＝アドレス
    if( msdLed ) msdLed = 0x8000;

    return ret;
}

/************************************************************************/
/* モジュール名 ClusterToSector                                         */
/* 処理概要     クラスタからセクタを計算                                */
/* 引数　       unsigned long クラスタ                                  */
/* 戻り値       セクタ番号                                              */
/************************************************************************/
static unsigned long ClusterToSector( unsigned long cluster )
{
    unsigned long work;

    work  = (cluster-2) * BPB_SctorsPerCluster;
    work += Sec_Data;

    return work;
}

/************************************************************************/
/* モジュール名 nextCluster                                             */
/* 処理概要     次のクラスタ検索                                        */
/* 引数　       unsigned long 現在のクラスタ                            */
/*              unsigned long 現在のクラスタが最後のとき、この値以上の  */
/*                            クラスタ番号で新しいクラスタを作る        */
/*                            1以下なら作らずに、終了する               */
/* 戻り値       次のクラスタ番号                                        */
/*              0なら容量オーバーで作れず、またはエラー                 */
/************************************************************************/
static unsigned long nextCluster(
                            unsigned long cluster, unsigned long s_cls )
{
    int i, ret;
    unsigned long   i4, s_cls2 = 0xffffffff;

    // クラスタの上限チェック
    if( ClusterToSector(cluster)+7 >= BPB_bigTotalSectors ) {
        return 0;
    }

    ret = readMicroSD(
                Sec_FAT[0] + (cluster>>7), (signed char*)msdlibBuff );
    if( ret != 0 ) {
        return 0;
    }

    i = (cluster & 0x0000007f) << 2;
    i4  = convertBinaryToLong( msdlibBuff+i );;

    if( i4 <= 0x0ffffff6 ) return i4;   // 次のクラスタ発見

    // 最終クラスタだが、クラスタは作らない
    if( s_cls <= 1 ) return i4;

    // 最終クラスタなら、クラスタ作る
    while( 1 ) {
        // クラスタの上限チェック
        if( ClusterToSector(s_cls)+7 >= BPB_bigTotalSectors ) {
            return 0;
        }
        if( (s_cls>>7) != (s_cls2>>7) ) {
            s_cls2 = s_cls;
            readMicroSD(
                Sec_FAT[0] + (s_cls>>7), (signed char*)msdlibBuff );
        }

        i = (s_cls & 0x0000007f) << 2;
        i4  = convertBinaryToLong( msdlibBuff+i );

        if( i4 == 0x00000000 ) {    // 空いているか？
            // 最終クラスタに0x0fffffffを書き込み
            msdlibBuff[i+3] = 0x0f;
            msdlibBuff[i+2] = 0xff;
            msdlibBuff[i+1] = 0xff;
            msdlibBuff[i+0] = 0xff;
            for( i=0; i<BPB_NumberOfFATs; i++ ) {
                writeMicroSD(
                    Sec_FAT[i] + (s_cls>>7), (signed char*)msdlibBuff );
            }
            // 現在のクラスタに最終クラスタをつなぐ
            readMicroSD(
                Sec_FAT[0] + (cluster>>7), (signed char*)msdlibBuff );
            i = (cluster & 0x0000007f) << 2;
            msdlibBuff[i+3] = (s_cls>>24) & 0xff;
            msdlibBuff[i+2] = (s_cls>>16) & 0xff;
            msdlibBuff[i+1] = (s_cls>> 8) & 0xff;
            msdlibBuff[i+0] = (s_cls    ) & 0xff;
            for( i=0; i<BPB_NumberOfFATs; i++ ) {
                writeMicroSD(
                    Sec_FAT[i] + (cluster>>7), (signed char*)msdlibBuff );
            }
            // クラスタの参照先をクリア
            i4 = ClusterToSector( s_cls );
            // イレーズコマンドは、イーレズ後のリードでなぜかエラーになるので、
            // writeで代用する
            // eraseMicroSD( i4, i4 + BPB_SctorsPerCluster - 1 );
            for( i=0; i<512; i++ ) msdlibBuff[i] = 0;   /* バッファクリア   */
            for(i=0; i<BPB_SctorsPerCluster; i++ ) {
                writeMicroSD( i4 + i, (signed char*)msdlibBuff );
            }

            return s_cls;
        }
        s_cls++;
    }
    return 0;
}

/************************************************************************/
/* モジュール名 writeFile                                               */
/* 処理概要     ファイル作成、書き込み準備                              */
/* 引数　       char * ファイル名(8+3形式)                              */
/*              unsigned long ファイルサイズ                            */
/* 戻り値       0:成功 0以外:失敗                                       */
/************************************************************************/
int writeFile( const char *s, unsigned long fileSize )
{
    int  ret = 0, r, i, j;
    char *p ,name[8+3+1+1];
    unsigned long cluster, cluster2, st_cls;
    unsigned long fileSector, sector, i4;

    if( fFileOpen ) {
        // ファイルオープン中
        ret = 1;
        goto writeFile_Error;
    }

    modeSector = 1;                     // アドレス選択＝セクタ

    // ファイルサイズを最小クラスタ数で切り上げ
    fileSector  = fileSize + (BPB_SctorsPerCluster<<9) - 1;
    fileSector /= (BPB_SctorsPerCluster<<9);
    fileSector *= BPB_SctorsPerCluster;     // クラスタ数の倍数にする
    fileSize    = fileSector << 9;

    // ファイルネーム変換
    memset( name, ' ', 8+3 );
    name[11] = '\0';
    for(p = name; *s; s++,p++ ) {
        if( p > name+11 ) {
            ret = 2;
            goto writeFile_Error;
        }
        if( *s == '.' ) p = name + 8, s++;
        if( *s == ',' ) p = name + 8, s++;
        *p = toupper(*s);        /* pの指す中身を大文字に変換 */
    }

    // 同じファイル名があるかチェック
    cluster = BPB_RootDirCluster;
    ret = 0;
    while( ret == 0 ) {
        sector = ClusterToSector( cluster );

        for( j=0;j<BPB_SctorsPerCluster ; j++ ) {
            // 1クラスタのセクタ数を読み込んでチェック
            r = readMicroSD( sector , (signed char*)msdlibBuff );
            if( r != 0 ) {
                ret = 3;
                goto writeFile_Error;
            }
            for( i=0; i<16; i++ ) {
                if( memcmp(msdlibBuff+i*32, name, 11) == 0 ) {
                    ret = 1;    // ファイルあり
                    goto writeFile_FileCechkExit;
                }
            }
            sector++;
        }
        cluster = nextCluster( cluster, 0 );
        if( cluster >= 0x0ffffff8 ) {
            ret = 2;            // ファイルなし
        }
        if( cluster == 0 ) {
            ret = 4;            // 容量オーバー
            goto writeFile_Error;
        }
    }
writeFile_FileCechkExit:

    if( ret == 1 ) {
        // 同じファイル名があるなら削除

        // ファイル削除
        cluster  = (unsigned long)msdlibBuff[i*32+21] << 24;
        cluster |= (unsigned long)msdlibBuff[i*32+20] << 16;
        cluster |= (unsigned long)msdlibBuff[i*32+27] <<  8;
        cluster |= (unsigned long)msdlibBuff[i*32+26];
        msdlibBuff[ i*32+0 ] = 0xe5;  // ファイル削除
        r = writeMicroSD( sector, (signed char*)msdlibBuff );
        if( r != 0 ) {
            ret = 11;
            goto writeFile_Error;
        }

        // FAT領域削除
        cluster2  = 0xffffffff;
        while( 1 ) {
            if( (cluster>>7) != (cluster2>>7) ) {
                r = readMicroSD(
                    Sec_FAT[0] + (cluster>>7), (signed char*)msdlibBuff );
                if( r != 0 ) {
                    ret = 12;
                    goto writeFile_Error;
                }
            }
            cluster2 = cluster;
            i = (cluster2 & 0x0000007f) << 2;
            cluster  = convertBinaryToLong( msdlibBuff+i );
            msdlibBuff[i+3] = 0x00;
            msdlibBuff[i+2] = 0x00;
            msdlibBuff[i+1] = 0x00;
            msdlibBuff[i+0] = 0x00;
            if( (cluster >= 0x0ffffff8) ||
                                    ((cluster>>7) != (cluster2>>7)) ) {
                for( i=0; i<BPB_NumberOfFATs; i++ ) {
                    writeMicroSD(
                        Sec_FAT[i]+(cluster2>>7), (signed char*)msdlibBuff );
                    if( r != 0 ) {
                        ret = 12;
                        goto writeFile_Error;
                    }
                }
            }
            if( cluster >= 0x0ffffff8 ) break;
        }
    }

    // ファイル作成、領域確保

    // ファイルサイズ分、FAT領域確保
    cluster  = 2;
    cluster2 = 0xffffffff;
    ret = 0;
    i = 0;              // スタートクラスタ;
    while( ret == 0 ) {
        // クラスタの上限チェック
        if( ClusterToSector(cluster)+7 >= BPB_bigTotalSectors ) {
            ret = 21;   // 連続したクラスタは見つからない
            goto writeFile_Error;
        }

        if( (cluster>>7) != (cluster2>>7) ) {
            cluster2 = cluster;
            r = readMicroSD(
                Sec_FAT[0]+(cluster>>7) , (signed char*)msdlibBuff );
            if( r != 0 ) {
                ret = 22;
                goto writeFile_Error;
            }
        }

        j = (cluster & 0x0000007f) << 2;
        i4  = convertBinaryToLong( msdlibBuff+j );
        if( i4 != 0 ) {
            cluster += fileSector / BPB_SctorsPerCluster - 1;   // ファイルサイズ分増やす
            i = 0;
        } else {
            i++;
        }
        if( i >= (fileSector/BPB_SctorsPerCluster) ) {
            // 連続したクラスタ発見
            st_cls = cluster - i + 1;   // 連続したクラスタの先頭
            ret = 1;
            break;
        }
        cluster++;
    }

    // RDEの空きをチェック
    cluster = BPB_RootDirCluster;
    ret = 0;
    while( ret == 0 ) {
        sector = ClusterToSector( cluster );

        for( i=0;i<BPB_SctorsPerCluster ; i++ ) {
            // 1クラスタのセクタ数を読み込んでチェック
            r = readMicroSD( sector , (signed char*)msdlibBuff );
            if( r != 0 ) {
                ret = 31;
                goto writeFile_Error;
            }

            for( j=0; j<16; j++ ) {
                if( msdlibBuff[j*32] == 0x00 || msdlibBuff[j*32] == 0xe5 ) {
                    // 空き領域　発見！
                    p = msdlibBuff + j * 32;
                    *p++ = name[ 0];
                    *p++ = name[ 1];
                    *p++ = name[ 2];
                    *p++ = name[ 3];
                    *p++ = name[ 4];
                    *p++ = name[ 5];
                    *p++ = name[ 6];
                    *p++ = name[ 7];
                    *p++ = name[ 8];
                    *p++ = name[ 9];
                    *p++ = name[10];
                    *p++ = 0x20;                                    // 11 属性
                    *p++ = '\0';                                    // 12
                    *p++ = '\0';                                    // 13
                    *p++ = ((fileMin  <<5)&0xe0) | ((fileSec     )&0x1f);//14
                    *p++ = ((fileHour <<3)&0xf8) | ((fileMin  >>3)&0x07);//15
                    *p++ = ((fileMonth<<5)&0xe0) | ((fileDay     )&0x1f);//16
                    *p++ = ((fileYear <<1)&0xfe) | ((fileMonth>>3)&0x01);//17
                    *p++ = msdlibBuff[j * 32 + 16];                 // 18
                    *p++ = msdlibBuff[j * 32 + 17];                 // 19
                    *p++ = (st_cls  >>16)&0xff;                     // 20
                    *p++ = (st_cls  >>24)&0xff;                     // 21
                    *p++ = msdlibBuff[j * 32 + 14];                 // 22
                    *p++ = msdlibBuff[j * 32 + 15];                 // 23
                    *p++ = msdlibBuff[j * 32 + 16];                 // 24
                    *p++ = msdlibBuff[j * 32 + 17];                 // 25
                    *p++ = (st_cls      )&0xff;                     // 26
                    *p++ = (st_cls  >> 8)&0xff;                     // 27
                    *p++ = (fileSize    )&0xff;                     // 28
                    *p++ = (fileSize>> 8)&0xff;                     // 29
                    *p++ = (fileSize>>16)&0xff;                     // 30
                    *p++ = (fileSize>>24)&0xff;                     // 31
                    r = writeMicroSD( sector , (signed char*)msdlibBuff );
                    if( r != 0 ) {
                        ret = 32;
                        goto writeFile_Error;
                    }
                    goto writeFile_FileMakeExit;
                }
            }
            sector++;
        }
        cluster = nextCluster(
                    cluster, st_cls + fileSector/BPB_SctorsPerCluster );
        if( cluster == 0 ) {
            ret = 33;   // 空き無し
            goto writeFile_Error;
        }
    }
writeFile_FileMakeExit:

    // クラスタ領域をつなぐ
    cluster  = st_cls;
    cluster2 = 0xffffffff;
    ret = 0;
    while( ret == 0 ) {
        if( (cluster>>7) != (cluster2>>7) ) {
            cluster2 = cluster;
            r = readMicroSD(
                Sec_FAT[0]+(cluster>>7) , (signed char*)msdlibBuff );
            if( r != 0 ) {
                ret = 41;
                goto writeFile_Error;
            }
        }
        i = (cluster & 0x0000007f) << 2;
        cluster++;
        if( cluster >= (st_cls+fileSector/BPB_SctorsPerCluster) ) {
            cluster = 0x0fffffff;
            ret = 1;
        }
        msdlibBuff[i+3] = (cluster>>24) & 0xff;
        msdlibBuff[i+2] = (cluster>>16) & 0xff;
        msdlibBuff[i+1] = (cluster>> 8) & 0xff;
        msdlibBuff[i+0] = (cluster    ) & 0xff;

        if( (cluster>>7) != (cluster2>>7) ) {
            for( i=0; i<BPB_NumberOfFATs; i++ ) {
                r = writeMicroSD(
                    Sec_FAT[i] + (cluster2>>7), (signed char*)msdlibBuff );
                if( r != 0 ) {
                    ret = 42;
                    goto writeFile_Error;
                }
            }
        }
    }

    // 確保した領域　クリア
    writeSectorStart = ClusterToSector( st_cls );
    writeSectorWork  = writeSectorStart;
    writeSectorEnd   = ClusterToSector(
                        st_cls + fileSector / BPB_SctorsPerCluster ) - 1;

    r = eraseMicroSD( writeSectorStart, writeSectorEnd );
    // エラーがあっても無視

    r = microSDProcessStart( writeSectorStart );
    if( r != 0 ) {
        ret = 43;       // マルチブロックライトの初期化　失敗
        ret = r;
        goto writeFile_Error;
    }

    fFileOpen = 1;      // ファイルオープン

    return 0;           // 成功

writeFile_Error:
    // エラー処理
    if( msdLed ) msdLed = 0x8000;

    return ret;
}

/************************************************************************/
/* モジュール名 convertDecimalToStr                                     */
/* 処理概要     int型を10進数文字列に変換                               */
/* 引数　       int 値                                                  */
/*              int 桁数(マイナスも含んだ数)                            */
/*              char* 変換した値を格納する配列                          */
/* 戻り値       なし                                                    */
/* メモ         実測で46μs ※6桁の場合                                 */
/************************************************************************/
void convertDecimalToStr( int value, int keta, signed char *p )
{
    char temp;

    if( keta > 6 ) keta = 6;

    if( value < 0 ) {
        value = -value;
        *p++ = '-';
        keta--;
    }

    p = p + keta;

    while( keta ) {
        p--;
        keta--;
        temp = value % 10;
        temp += '0';
        *p = temp;
        value /= 10;
    }
}
/************************************************************************/
/* モジュール名 convertHexToStr                                         */
/* 処理概要     int型を16進数文字列に変換                               */
/* 引数　       int 値                                                  */
/*              int 桁数                                                */
/*              char* 変換した値を格納する配列                          */
/* 戻り値       なし                                                    */
/* メモ         実測で24μs ※4桁の場合                                 */
/************************************************************************/
void convertHexToStr( unsigned int value, int keta, signed char *p )
{
    char temp;

    if( keta > 4 ) keta = 4;

    p = p + keta;

    while( keta ) {
        p--;
        keta--;
        temp = value & 0x000f;
        if( temp < 10 ) {
            temp += '0';
        } else {
            temp += ('a' - 10);
        }
        *p = temp;
        value >>= 4;
    }
}

/************************************************************************/
/* モジュール名 convertBinaryToStr                                      */
/* 処理概要     unsigned char型を2進数の文字列に変換                    */
/* 引数　       unsigned char 値                                        */
/*              int 桁数                                                */
/*              char* 変換した値を格納する配列                          */
/* 戻り値       なし                                                    */
/* メモ         実測で38μs ※8桁の場合                                 */
/************************************************************************/
void convertBinaryToStr( unsigned char value, int keta, signed char *p )
{
    char temp;

    if( keta > 8 ) keta = 8;

    p = p + keta;

    while( keta ) {
        p--;
        keta--;
        temp  = value & 0x0001;
        temp += '0';
        *p = temp;
        value >>= 1;
    }
}

/************************************************************************/
/* モジュール名 getCompileYear                                          */
/* 処理概要     コンパイルした時の年を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       年                                                      */
/************************************************************************/
int getCompileYear( const char *p )
{
    int i;

    i = atoi( p + 7 );

    if( i < 1980 || i > 2107 ) i = 2012;

    return i;
}

/************************************************************************/
/* モジュール名 getCompileMonth                                         */
/* 処理概要     コンパイルした時の月を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       月                                                      */
/************************************************************************/
int getCompileMonth( const char *p )
{
    int i, r;

    for( i=0; i<12; i++ ) {
        r = strncmp( monthStr + i * 3, p, 3 );
        if( r == 0 ) return i + 1;
    }
    return 1;
}

/************************************************************************/
/* モジュール名 getCompileDay                                           */
/* 処理概要     コンパイルした時の日を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       日                                                      */
/************************************************************************/
int getCompileDay( const char *p )
{
    int i;

    i = atoi( p + 4 );

    if( i < 1 || i > 31 ) i = 1;

    return i;
}

/************************************************************************/
/* モジュール名 getCompileHour                                          */
/* 処理概要     コンパイルした時の時を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       時                                                      */
/************************************************************************/
int getCompileHour( const char *p )
{
    int i;

    i = atoi( p );

    if( i < 0 || i > 23 ) i = 0;

    return i;
}

/************************************************************************/
/* モジュール名 getCompilerMinute                                       */
/* 処理概要     コンパイルした時の分を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       分                                                      */
/************************************************************************/
int getCompilerMinute( const char *p )
{
    int i;

    i = atoi( p + 3 );

    if( i < 0 || i > 59 ) i = 0;

    return i;
}

/************************************************************************/
/* モジュール名 getCompilerSecond                                       */
/* 処理概要     コンパイルした時の秒を取得                              */
/* 引数　       なし                                                    */
/* 戻り値       秒                                                      */
/************************************************************************/
int getCompilerSecond( const char *p )
{
    int i;

    i = atoi( p + 6 );

    if( i < 0 || i > 59 ) i = 0;

    return i;
}

/************************************************************************/
/* モジュール名 setDateStamp                                            */
/* 処理概要     ファイルを作るときの日付を設定                          */
/* 引数　       年、月、日                                              */
/* 戻り値       無し                                                    */
/************************************************************************/
void setDateStamp( int y, int m, int d )
{
    if( y >= 1980 && y <= 2107 ) {
        fileYear  = y - 1980;
    } else {
        fileYear  = 2012 - 1980;
    }
    if( m >= 1 && m <= 12 ) {
        fileMonth = m;
    } else {
        fileMonth = 1;
    }
    if( d >= 1 && d <= 31 ) {
        fileDay = d;
    } else {
        fileDay = 1;
    }
}

/************************************************************************/
/* モジュール名 setTimeStamp                                            */
/* 処理概要     ファイルを作るときの時間を設定                          */
/* 引数　       時、分、秒                                              */
/* 戻り値       無し                                                    */
/************************************************************************/
void setTimeStamp( int h, int m, int s )
{
    if( h >= 0 && h <= 23 ) {
        fileHour = h;
    } else {
        fileHour = 0;
    }
    if( m >= 0 && m <= 59 ) {
        fileMin = m;
    } else {
        fileMin = 0;
    }
    if( s >= 0 && s <= 59 ) {
        fileSec = s / 2;
    } else {
        fileSec = 0;
    }
}

/************************************************************************/
/* モジュール名 readMicroSDNumber                                       */
/* 処理概要     microSDからファイル番号取得                             */
/* 引数　       無し                                                    */
/* 戻り値       -1:エラー 0以上：値                                     */
/************************************************************************/
int readMicroSDNumber( void )
{
    int ret;

    ret = readMicroSD( BPB_SctorNo, (signed char*)msdlibBuff );
    if( ret != 0 ) return -1;
    ret   = (unsigned char)msdlibBuff[508];
    ret <<= 8;
    ret  |= (unsigned char)msdlibBuff[509];

    return ret;
}

/************************************************************************/
/* モジュール名 writeMicroSDNumber                                      */
/* 処理概要     microSDにファイル番号書き込み                           */
/* 引数　       書き込む番号                                            */
/* 戻り値       -1:エラー 0：書き込み完了                               */
/************************************************************************/
int writeMicroSDNumber( int number )
{
    int ret;

    // BPB読み込み
    ret = readMicroSD( BPB_SctorNo, (signed char*)msdlibBuff );
    if( ret != 0 ) return -1;

    // メモリに設定
    msdlibBuff[508] = (unsigned char)(number >> 8);
    msdlibBuff[509] = (unsigned char)(number & 0xff);

    // 書き込み
    ret = writeMicroSD( BPB_SctorNo, (signed char*)msdlibBuff );
    if( ret != 0 ) return -1;

    return 0;
}

/************************************************************************/
/* モジュール名 msdPrintf                                               */
/* 処理概要     microSD　書式付き書き込み処理                           */
/* 引数　       microSDに書き込む内容　書式はprintfとほぼ同じ           */
/* 戻り値       0:成功  1:書き込み中で書き込みできず                    */
/*              2:書き込み中止(ファイルクローズ)                        */
/************************************************************************/
int msdPrintf(char _far *format, ...)
{
    volatile va_list argptr;
    volatile char _far *p;
    volatile int ret = 0;
    volatile int i;

    // 現在書き込み処理中なら今回は無理
    if( msdPrintfMode != 0 ) return 1;

    // バッファオーバーなら今回は無理
    if( writeBuffNo == 0 )  {
        if( pWriteBuff >= (msdlibBuff+512) ) return 1;
    } else {
        if( pWriteBuff >= (msdlibBuff2+512) ) return 1;
    }

    // ファイルクローズしたので無理
    if( fFileOpen == 0 ) return 2;

    va_start(argptr, format);

    // パラメータをバッファに保存
    pFormat = msdPrintfBuff_Format;
    for( p=format; *p!='\0'; p++ ) {
        *pFormat++ = *p;
    }
    *pFormat = '\0';

    // 引数をバッファに保存
    pPara = msdPrintfBuff_Para;
    for( i=0; i<64; i++ ) {
        *pPara++ = va_arg( argptr, char );
    }

    va_end(argptr);

    // 展開開始
    pFormat     = msdPrintfBuff_Format; // 書式読み込み先
    pPara       = msdPrintfBuff_Para;   // パラメータ読み込み先
    msdPrintfMode = 1;

    return 0;
}

/************************************************************************/
/* モジュール名 microSDPrintfProcess                                    */
/* 処理概要     microSD msdPrintf関数　裏処理                           */
/* 引数　       無し 1msごとに実行してください                          */
/* 戻り値       msdlibModeの値                                          */
/************************************************************************/
int microSDPrintfProcess( void )
{
    int i, j, loop;

    switch( msdPrintfMode ) {
    case 0:
        // 待ち
        break;

    case 1:
        // 展開
        loop = 0;
        while( loop < 3 ) {
            if( *pFormat == '\0' ) {
                msdPrintfMode = 2;
                break;
            }

            switch( *pFormat ) {
            case '%':       // 変換指定文字
                pFormat++;
                loop++;
                j = 0;

                // 文字数
                switch( *pFormat ) {
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                    j = *pFormat - '0';
                    pFormat++;
                    break;
                }

                // 変換指定文字
                switch( *pFormat ) {
                case 'd':
                case 'D':
                    if( j <= 0 ) j = 5;
                    if( j >  6 ) j = 6;
                    convertDecimalToStr(
                        *(int*)pPara, j, (signed char*)pWriteBuff );
                    pFormat++;
                    pPara += 2;
                    pWriteBuff += j;
                    break;
                case 'b':
                case 'B':
                    if( j <= 0 ) j = 8;
                    if( j >  8 ) j = 8;
                    convertBinaryToStr(
                        *pPara, j, (signed char*)pWriteBuff );
                    pFormat++;
                    pPara += 2;
                    pWriteBuff += j;
                    break;
                case 'x':
                case 'X':
                    if( j <= 0 ) j = 4;
                    if( j >  4 ) j = 4;
                    convertHexToStr(
                        *(int*)pPara, j, (signed char*)pWriteBuff );
                    pFormat++;
                    pPara += 2;
                    pWriteBuff += j;
                    break;
                case 'c':
                case 'C':
                    *pWriteBuff = *pPara;
                    pFormat++;
                    pPara += 2;
                    pWriteBuff += 1;
                    break;
                default:
                    break;
                }
                break;
            default:    // それ以外はそのまま代入
                *pWriteBuff++ = *pFormat++;
                break;
            }
        }
        break;

    case 2:
        // バッファ確認処理
        if( writeBuffNo == 0 )  {
            if( pWriteBuff >= (msdlibBuff+512) ) {
                j = (int)(pWriteBuff - msdlibBuff - 512);
                pWriteBuff = msdlibBuff2;
                for( i=0; i<j; i++ ) {
                    *pWriteBuff++ = msdlibBuff[512+i];
                }
                writeBuffNo = 1;
                msdlibBuff[512] = 0xff;
                msdlibBuff[513] = 0xff;

                msdlibWrite = msdlibBuff;
                msdPrintfMode = 3;
            } else {
                msdPrintfMode = 0;
            }
        } else {
            if( pWriteBuff >= (msdlibBuff2+512) ) {
                j = (int)(pWriteBuff - msdlibBuff2 - 512);
                pWriteBuff = msdlibBuff;
                for( i=0; i<j; i++ ) {
                    *pWriteBuff++ = msdlibBuff2[512+i];
                }
                writeBuffNo = 0;
                msdlibBuff2[512] = 0xff;
                msdlibBuff2[513] = 0xff;

                msdlibWrite = msdlibBuff2;
                msdPrintfMode = 3;
            } else {
                msdPrintfMode = 0;
            }
        }
        break;

    case 3:
        // microSD書き込み開始

        /* LED点滅処理 */
        if( msdLed == 0x4000 ) msdLed += LED_BLINK;

        /* 転送中のエラーがないかチェック */
        if( msdlibMode == 0 ) {
            if( fFileOpen ) {
                if( msdLed ) msdLed = 0x8000;
                fFileOpen = 0;
                goto microSDPrintfProcess_End;
            }
        }

        /* ファイルとして書き込み時の上限のチェック */
        if( fFileOpen ) {
            if( writeSectorWork > writeSectorEnd ) {
                fFileOpen = 0;
                goto microSDPrintfProcess_End;
            }
            writeSectorWork++;
        }

        msdlibCnt = 514;                /* バッファ書き込み数           */
        s1tic = 0x02;
        u1tbl = 0xfc;                   /* 1個だけ送信(残りは割込みで)  */
        msdlibMode = 12;                /* microSDProcess 処理開始      */

        msdPrintfMode = 0;

        break;
    }

microSDPrintfProcess_End:
    return msdlibMode;
}

/************************************************************************/
/* モジュール名 checkMsdPrintf                                          */
/* 処理概要     microSD　間欠書き込み処理の終了チェック                 */
/* 引数         なし                                                    */
/* 戻り値       0:処理無し 0以外：処理中                                */
/************************************************************************/
int checkMsdPrintf( void )
{
  return msdPrintfMode;
}

#endif
//////////////////////////////////////////////////////////////////////////
// FAT32関係プログラムここまで                                         //
//////////////////////////////////////////////////////////////////////////

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2011.04.01 Ver.1.00 作成
2012.03.16 Ver.2.00 SDHC対応、その他細かい修正
2012.04.01 Ver.3.00 FAT32へのファイル書き込みに対応(FAT12,FAT16,LFNは未対応)
2013.04.01 Ver.3.10 msdPrintf関数作成
2014.06.01 Ver.3.11 eraseMicroSDでｴﾗｰになるmicroSDがあったためｳｴｲﾄを入れる
*/
