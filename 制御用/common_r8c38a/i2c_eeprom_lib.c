/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     I2CによるEEP-ROM(24C256B)処理                               */
/* バージョン   Ver.1.02                                                    */
/* Date         2011.08.13                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "i2c_eeprom_lib.h"          /* EEP-ROM関連処理              */

/*======================================*/
/* シンボル定義                         */
/*======================================*/

/* setPageWriteI2CEeprom関数１回で保存出来るサイズ */
#define EEP_SAVE_SIZE       16          /* 最大保存数                   */

/* その他定義 */
#define ACK                 0           /* リード時のACK有効(ACK=0)     */
#define NO_ACK              1           /* リード時のACK無効(ACK=1)     */

#define STOP_RD             0           /* ストップの直前は読み込み     */
#define STOP_WR             1           /* ストップの直前は書き込み     */

/*======================================*/
/* グローバル変数の宣言                 */
/*======================================*/

static volatile unsigned char   eep_address;
                                        /* アドレスの選択               */
static volatile int             write_mode;
                                        /* 処理内容                     */
static volatile unsigned char   write_eep_address;
                                        /* 書き込みEEP-ROMのアドレス    */
static volatile unsigned int    write_address;
                                        /* 書き込みアドレス             */
static volatile int             write_count;
                                        /* 書き込み個数                 */
static volatile signed char     write_buff[EEP_SAVE_SIZE];
                                        /* 書き込みデータ保存バッファ   */
static volatile signed char     *write_buff_p;
                                        /* 書き込みデータ読み込み位置   */

/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/

/************************************************************************/
/* モジュール名 i2c_start                                               */
/* 処理概要     スタート関連レジスタ設定、スタート信号送信              */
/************************************************************************/
void i2c_start( void )
{
    stop_icsr = 0;
    while( bbsy_iccr2 == 1 );
    iccr1 |= 0x30;                      /* マスタ送信モード             */
    iccr2 = 0x90;                       /* スタート条件発行             */
}

/************************************************************************/
/* モジュール名 i2c_stop                                                */
/* 処理概要     ストップ信号送信                                        */
/************************************************************************/
void i2c_stop( int mode )
{
    if( mode == STOP_WR ) {
        iccr2 = 0x10;                   /* ストップ条件発行             */
        tend_icsr = 0;
        nackf_icsr = 0;
        while(stop_icsr == 0);
        stop_icsr = 0;
    }
    iccr1 &= 0xcf;                      /* スレーブ受信モード           */
}

/************************************************************************/
/* モジュール名 i2c_write                                               */
/* 処理概要     EEP-ROM　１バイト書き込み                               */
/* 引数         signed char データ                                      */
/* 戻り値       int  acknowledge 0:有効  1:無効                         */
/************************************************************************/
int i2c_write( signed char data )
{
    int ret;

    icdrt = data;
    while( tdre_icsr == 0 );
    while( tend_icsr == 0 );

    ret = ackbr_icier;

    return ret;
}

/************************************************************************/
/* モジュール名 i2c_read                                                */
/* 処理概要     EEP-ROM　１バイト読み込み                               */
/* 引数         int ACK：データが続く NO_ACK：データ終了                */
/* 戻り値       signed char データ                                      */
/************************************************************************/
signed char i2c_read( int ack )
{
    signed char ret;

    tend_icsr = 0;
    iccr1 &= 0xef;
    tdre_icsr = 0;
    ackbt_icier = 1;
    ret = icdrr;                        /* ダミーリード                 */

    if( ack == NO_ACK ) {
        rcvd_iccr1 = 1;                 /* 次の受信動作を禁止           */
    }

    while( rdrf_icsr == 0 );
    iccr2 = 0x10;
    while( stop_icsr == 0 );
    ret = icdrr;
    rcvd_iccr1 = 0;
    ackbt_icier = 0;

    return ret;
}

/************************************************************************/
/* モジュール名 initI2CEeprom                                           */
/* 処理概要     EEP-ROMのポート初期化                                   */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void initI2CEeprom( void )
{
    pd3 &= 0x5f;                        /* P3_7:SDA P3_5:SCL            */

    iicsel = 1;
    stop_icsr = 0;
    ice_iccr1 = 1;
    icmr  = 0x00;
    iccr1 = 0x82;
    icier = 0x00;
    icsr  = 0x00;
    sar   = 0x00;

    selectI2CEepromAddress( 0 );        /* アドレスの選択               */
    write_mode  = 0;
}

/************************************************************************/
/* モジュール名 selectI2CEepromAddress                                  */
/* 処理概要     どの番号のEEP-ROMを使用するか選択                       */
/* 引数         unsigned char EEP-ROMのアドレス 0～3                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void selectI2CEepromAddress( unsigned char address )
{
    address &= 0x03;
    eep_address = address << 1;
}

/************************************************************************/
/* モジュール名 readI2CEeprom                                           */
/* 処理概要     EEP-ROM読み込み                                         */
/* 引数         unsigned long   アドレス 0-32767                        */
/* 戻り値       signed char     データ                                  */
/************************************************************************/
signed char readI2CEeprom( unsigned long address )
{
    signed char ret;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    iccr2 = 0x90;                       /* スタート再送                 */
    i2c_write( 0xa1 | eep_address );    /* device address(read)         */
    ret = i2c_read( NO_ACK );           /* data read                    */
    i2c_stop( STOP_RD );

    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* モジュール名 writeI2CEeprom                                          */
/* 処理概要     EEP-ROM書き込み                                         */
/* 引数         unsigned long アドレス 0-32767 ,signed char データ      */
/* 戻り値       なし                                                    */
/************************************************************************/
void writeI2CEeprom( unsigned long address, signed char write )
{
    int i;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    i2c_start();
    i2c_write( 0xa0 | eep_address );    /* device address(write)        */
    i2c_write( (address >> 8) & 0xff ); /* first address                */
    i2c_write( address & 0xff );        /* second address               */
    i2c_write( write );                 /* data write                   */
    i2c_stop( STOP_WR );

    asm( "FSET I" );                    /* 全体の割り込み許可           */
}

/************************************************************************/
/* モジュール名 setPageWriteI2CEeprom                                   */
/* 処理概要     EEP-ROMページ書き込み                                   */
/*              この関数でページ書き込みを開始する                      */
/* 引数         unsigned long アドレス 0-32767 , int 個数,              */
/*              signed char* データがあるアドレス                       */
/* 戻り値       なし                                                    */
/************************************************************************/
void setPageWriteI2CEeprom(
                unsigned long address, int count, signed char* data )
{
    /* 書き込み中ならこの関数は中止 */
    if( write_mode != 0 ) return;

    write_mode          = 1;
    write_eep_address   = eep_address;
    write_address       = address;
    write_count         = count;
    write_buff_p        = write_buff;

    if( count >= EEP_SAVE_SIZE ) count = EEP_SAVE_SIZE;
    do {
        *write_buff_p++ = *data++;
    } while( --count );

    write_buff_p = write_buff;
}

/************************************************************************/
/* モジュール名 I2CEepromProcess                                        */
/* 処理概要     EEP-ROMページ書き込み                                   */
/* 引数         なし                                                    */
/* 戻り値       なし                                                    */
/************************************************************************/
void I2CEepromProcess( void )
{
    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    switch( write_mode ) {
    case 1:
        i2c_start();
        write_mode = 2;
        break;
    case 2:
        i2c_write( 0xa0 | write_eep_address );  /* device address(write)*/
        write_mode = 3;
        break;
    case 3:
        i2c_write( (write_address >> 8) & 0xff );   /* first address    */
        write_mode = 4;
        break;
    case 4:
        i2c_write( write_address & 0xff );      /* second address       */
        write_mode = 5;
        break;
    case 5:
        i2c_write( *write_buff_p++ );           /* data write           */
        if( !(--write_count) ) write_mode = 6;
        break;
    case 6:
        i2c_stop( STOP_WR );
        write_mode = 0;
        break;
    }

    asm( "FSET I" );                    /* 全体の割り込み許可           */
}

/************************************************************************/
/* モジュール名 clearI2CEeprom                                          */
/* 処理概要     EEP-ROM　オールクリア                                   */
/* 引数         char* クリア中にモニタするLEDのポート                   */
/*              int   クリア中にモニタするLEDのビット                   */
/* 戻り値       なし                                                    */
/************************************************************************/
void clearI2CEeprom( char *led_port , int bit )
{
    unsigned int    address = 0;
    int             i;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    while( address < 32768 ) {
        if( !(address % 0x800) ) *led_port ^= (1<<bit);

        i2c_start();
        i2c_write( 0xa0 | eep_address ); /* device address(write)       */
        i2c_write( address >> 8 );      /* first address                */
        i2c_write( address & 0xff );    /* second address               */
        for( i=0; i<64; i++ ) {
            i2c_write( 0 );             /* data write                   */
        }
        i2c_stop( STOP_WR );

        while( !checkI2CEeprom() );     /* wait                         */

        address += 64;
    }

    asm( "FSET I" );                    /* 全体の割り込み許可           */
}

/************************************************************************/
/* モジュール名 checkI2CEeprom                                          */
/* 処理概要     EEP-ROM書き込み後、次に読み書きできるかチェック         */
/* 引数         なし                                                    */
/* 戻り値       1:次読み書きOK 0:まだ                                   */
/************************************************************************/
int checkI2CEeprom( void )
{
    int ret;

    if( write_mode != 0 ) return 0;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    i2c_start();
    ret = !i2c_write( 0xa0 | eep_address ); /* device address(write)    */
    i2c_stop( STOP_WR );

    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2010.04.01 Ver.1.00 作成
2011.04.01 Ver.1.01 変数にvolatile追加
2011.08.13 Ver.1.02 EEP-ROMに保存するデータ型をchar型からunsigned char型に変更
*/
