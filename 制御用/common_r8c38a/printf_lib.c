/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     printf,scanf関連処理                                        */
/* バージョン   Ver.1.04                                                     */
/* Date         2016.02.08                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    <stdio.h>
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "printf_lib.h"              /* printf関連処理               */

/*======================================*/
/* シンボル定義                         */
/*======================================*/
#define         SEND_BUFF_SIZE  64      /* 送信バッファサイズ           */
#define         RECV_BUFF_SIZE  32      /* 受信バッファサイズ           */
#define         UART0_INT_LEVEL 1       /* 送信割り込み優先レベル設定   */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/
/* 送信バッファ */
static volatile char    send_buff[SEND_BUFF_SIZE];
static volatile char    *send_w = send_buff;
static volatile char    *send_r = send_buff;
static volatile int     send_count = 0;

/* 受信バッファ */
static volatile char    recv_buff[RECV_BUFF_SIZE];
static volatile char    *recv_w = recv_buff;
static volatile char    *recv_r = recv_buff;

/* printf,scanf関係 */
volatile FILE           _iob[4];

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
int write( int s );
int read( void );
int setSendBuff( char c );
int getSendBuff( char *c );

/************************************************************************/
/* UART0の初期化、及びprintf関係をUART0に割り当て                       */
/* 引数　 通信速度                                                      */
/* 戻り値 なし                                                          */
/************************************************************************/
void init_uart0_printf( int sp )
{
    stdin->_cnt = stdout->_cnt = stdaux->_cnt = stdprn->_cnt = 0;
    stdin->_flag = _IOREAD;
    stdout->_flag = _IOWRT;
    stdaux->_flag = _IORW;
    stdprn->_flag = _IOWRT;

    stdin->_mod = _TEXT;
    stdout->_mod = _TEXT;
    stdaux->_mod = _BIN;
    stdprn->_mod = _TEXT;

    stdin->_func_in = read;
    stdout->_func_in = NULL;
    stdaux->_func_in = read;
    stdprn->_func_in = NULL;

    stdin->_func_out = NULL;
    stdout->_func_out = write;
    stdaux->_func_out = write;
    stdprn->_func_out = NULL;

    /* UART0の設定 */
    u0sr = 0x05;                        /* P14=TXD0,P15=RXD0に設定      */
    if( sp == SPEED_4800 ) {
        /* 4800bps u0brg = 2.5MHz / (4800 * 16) - 1 = 31.552 = 32 */
        u0c0 = 0x01;                    /* カウントソースなどの設定     */
        u0c1 = 0x05;                    /* 送信､受信許可                */
        u0brg = 32;                     /* 通信速度 = 4800pbs           */
    } else if( sp == SPEED_9600 ) {
        /* 9600bps u0brg = 20MHz / (9600 * 16) - 1 =129.208 = 129 */
        u0c0 = 0x00;                    /* カウントソースなどの設定     */
        u0c1 = 0x05;                    /* 送信､受信許可                */
        u0brg = 129;                    /* 通信速度 = 9600pbs           */
    } else if( sp == SPEED_19200 ) {
        /* 19200bps u0brg = 20MHz / (19200 * 16) - 1 = 64.104 = 64 */
        u0c0 = 0x00;                    /* カウントソースなどの設定     */
        u0c1 = 0x05;                    /* 送信､受信許可                */
        u0brg = 64;                     /* 通信速度 = 19200pbs          */
    } else if( sp == SPEED_38400 ) {
        /* 38400bps u0brg = 20MHz / (38400 * 16) - 1 = 31.552 = 32 */
        u0c0 = 0x00;                    /* カウントソースなどの設定     */
        u0c1 = 0x05;                    /* 送信､受信許可                */
        u0brg = 32;                     /* 通信速度 = 38400pbs          */
    }
    u0mr = 0x05;                        /* UART0 ﾃﾞｰﾀ長8bit 1ｽﾄｯﾌﾟﾋﾞｯﾄ  */
}

/************************************************************************/
/* printfで呼び出される関数                                             */
/* ユーザーからは呼び出せません                                         */
/************************************************************************/
int write( int c )
{

// 1で割り込みを使用(main関数の負荷が少ない)
// 0で割り込みを使用せず(文字化けがしづらい)
#if 1
    volatile char   put_data;

    if( c == '\n' )  {
        while( !setSendBuff( '\r' ) );
    } else if( c == '\b' ) {
        while( !setSendBuff( '\b' ) );
        while( !setSendBuff( ' ' ) );
    }
    while( !setSendBuff( c ) );

    if( (s0tic & 0x07) == 0 ) {
        getSendBuff( &put_data );
        s0tic = 0x01;                   /* 送信割り込み優先レベル設定   */
        put_uart0( put_data );
    }

    return 1;   // TRUE
#else
    if( c == '\n' )  {
        while( put_uart0( '\r' ) == 0 );
    } else if( c == '\b' ) {
        while( put_uart0( '\b' ) == 0 );
        while( put_uart0( ' '  ) == 0 );
    }
    while( put_uart0( c ) == 0 );

    return 1;   // TRUE
#endif
}

/************************************************************************/
/* scanfで呼び出される関数                                              */
/* ユーザーからは呼び出せません                                         */
/************************************************************************/
int read( void )
{
    volatile char   c;

    if( recv_r == recv_w ) {
        do {
            /* 受信待ち */
            while( get_uart0( &c ) != 1 );

            switch( c ) {
                case '\b':  /* バックスペース */
                    /* 何もバッファにないならBSは無効 */
                    if( recv_r == recv_w ) continue;
                    /* あるなら一つ戻る */
                    recv_w--;
                    break;
                case '\r':  /* Enterキー */
                    *recv_w++ = c = '\n';
                    *recv_w++ = '\r';
                    break;
                default:
                    if( recv_w >= recv_buff+RECV_BUFF_SIZE-2 ) continue;
                    *recv_w++ = c;
                    break;
            }
            /* エコーバック 入力された文字を返す */
            write( c );
        } while( c != '\n' );
    }
    c = *recv_r++;
    if( recv_r == recv_w ) recv_r = recv_w = recv_buff;

    return c;
}

/************************************************************************/
/* 送信バッファに保存                                                   */
/* 引数　 格納文字                                                      */
/* 戻り値 0:保存できず 1:保存完了                                       */
/************************************************************************/
int setSendBuff( char c )
{
    int ret= 0;

//    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    if( send_count < SEND_BUFF_SIZE ) {
        *send_w++ = c;
        if( send_w >= send_buff+SEND_BUFF_SIZE ) send_w = send_buff;
        send_count++;
        ret = 1;
    }

//    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* 送信バッファから取得                                                 */
/* 引数　 格納する文字のアドレス                                        */
/* 戻り値 0:データなし 1:データあり                                     */
/************************************************************************/
int getSendBuff( char *c )
{
    volatile int    ret = 0;

//    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    if( send_count ) {
        *c = *send_r++;
        if( send_r >= send_buff+SEND_BUFF_SIZE ) send_r = send_buff;
        send_count--;
        ret = 1;
    }

//    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* URAT0 送信割り込み                                                   */
/* 引数　 なし                                                          */
/* 戻り値 なし                                                          */
/************************************************************************/
#pragma interrupt   _uart0_trance(vect=17)
void _uart0_trance( void )
{
    volatile char   c;
    volatile int    ret;

    asm( "FSET I" );                    /* この割り込みレベル以上許可   */

    ir_s0tic = 0;

    ret = getSendBuff( &c );
    if( ret ) {
        while( put_uart0( c ) == 0 );   /* データあるなら送信           */
    } else {
        s0tic = 0x00;                   /* データ無いなら終わり         */
    }
}

/************************************************************************/
/* １文字受信                                                           */
/* 引数　 受信文字格納アドレス                                          */
/* 戻り値 -1:受信エラー 0:受信なし 1:受信あり 文字は*sに格納            */
/************************************************************************/
int get_uart0( char *s )
{
    volatile int ret = 0;
    volatile int data, i;

    if (ri_u0c1 == 1){                  /* 受信データあり？             */
        data = u0rb;
        *s = (char)data;
        ret = 1;
        if( data & 0xf000 ) {           /* エラーあり？                 */
            /* エラー時は再設定 */
            re_u0c1 = 0;
            for( i=0; i<50; i++ );
            re_u0c1 = 1;

            ret = -1;
        }
    }
    return ret;
}

/************************************************************************/
/* １文字出力                                                           */
/* 引数　 送信データ                                                    */
/* 戻り値 0:送信中のため、送信できず 1:送信セット完了                   */
/************************************************************************/
int put_uart0( char r )
{
    if(ti_u0c1 == 1) {                  /* 送信データなし？             */
        u0tbl = r;
        return 1;
    } else {
        /* 別なデータを送信中(今回のデータは送信せずに終了) */
        return 0;
    }
}

/************************************************************************/
/* fgetc(ストリームから１文字入力する)から呼ばれる1文字入力関数         */
/* 引数　 無し                                                          */
/* 戻り値 正常に入力できた場合は、その文字                              */
/*        エラーの場合は、EOF                                           */
/************************************************************************/
int _sget( void ) {

    return 0;

}

/************************************************************************/
/* fgetc(ストリームから１文字入力する)から呼ばれる1文字出力関数         */
/* 引数　 出力文字                                                      */
/* 戻り値 正常:1 以上:EOF                                               */
/************************************************************************/
int _sput( int put_data ) {

    return put_data;

}

/************************************************************************/
/* fgetc(ストリームから１文字入力する)から呼ばれる1文字出力関数         */
/* 引数　 出力文字                                                      */
/* 戻り値 正常:1 以上:EOF                                               */
/************************************************************************/
int _pput( int put_data ) {

    return put_data;

}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2010.04.01 Ver.1.00 作成
2011.03.17 Ver.1.01 送信割り込みが止まる不具合の修正(write関数内の修正)
2011.04.01 Ver.1.02 変数にvolatile追加
2015.04.01 Ver.1.03 write関数に、割り込み未使用のプログラムを追加
2016.02.08 Ver.1.04 write関数内のsetSendBuff関数、送信が終わるまで
                    ループするようにする
*/
