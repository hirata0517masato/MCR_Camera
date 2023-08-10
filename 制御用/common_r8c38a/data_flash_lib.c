/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     データフラッシュ処理                                        */
/* バージョン   Ver.1.03                                                    */
/* Date         2011.04.01                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/****************************************************************************/

/*======================================*/
/* インクルード                         */
/*======================================*/
#include    "sfr_r838a.h"               /* R8C/38A SFRの定義ファイル    */
#include    "data_flash_lib.h"          /* データフラッシュライブラリ   */

/*======================================*/
/* シンボル定義                         */
/*======================================*/
/* データフラッシュ関係 */
#define DF_SELECT_A 0                   /* ブロックA選択                */
#define DF_SELECT_B 1                   /* ブロックB選択                */
#define DF_SELECT_C 2                   /* ブロックC選択                */
#define DF_SELECT_D 3                   /* ブロックD選択                */

/*======================================*/
/* プロトタイプ宣言(ローカル)           */
/*======================================*/
int clrStatusRegister( unsigned int c_address );
int checkBlockAddress( unsigned int chk_address );

/************************************************************************/
/* モジュール名 readDataFlash                                           */
/* 処理概要     データフラッシュ読み込み                                */
/* 引数         unsigned int    読み込み元アドレス 0x3000-0x3fff        */
/*              char*           読み込み先アドレス                      */
/*              int             読み込むデータ数                        */
/* 戻り値       なし                                                    */
/************************************************************************/
void readDataFlash( unsigned int r_address, signed char *w_address, int count )
{
    if( count >= 1 && count <= 4096 ) {
        while( count-- ) {
            *w_address = *((signed char*)r_address);
            w_address++;
            r_address++;
        }
    }
}

/************************************************************************/
/* モジュール名 blockEraseDataFlash                                     */
/* 処理概要     ブロックイレーズ                                        */
/* 引数         unsinged int    アドレス                                */
/* 戻り値       1:エラーなし 0:エラーあり                               */
/* メモ         実測で135ms程度                                         */
/************************************************************************/
int blockEraseDataFlash( unsigned int address )
{
    volatile int ret = 1;
    volatile int block;

    block = checkBlockAddress( address );
    if( block == -1 ) return 0;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    fmr01 = 0;                          /* CPU書き換えモードに設定      */
    asm(" ");                           /* 最適化抑制                   */
    fmr01 = 1;
    fmr02 = 0;                          /* EW1モード選択                */
    asm(" ");                           /* 最適化抑制                   */
    fmr02 = 1;
    cmderie = 0;                        /* ｲﾚｰｽﾞ/ﾗｲﾄｴﾗｰ割り込み禁止     */
    bsyaeie = 0;                        /* ﾌﾗｯｼｭｱｸｾｽｴﾗｰ割り込み禁止     */
    rdystie = 0;                        /* ﾌﾗｯｼｭﾚﾃﾞｨｽﾃｰﾀｽ割り込み禁止   */

    /* ﾃﾞｰﾀﾌﾗｯｼｭﾌﾞﾛｯｸ書き換え禁止ビットを書き換え許可にする */
    switch( block ) {
        case DF_SELECT_A:
            fmr14 = 1;
            asm(" ");
            fmr14 = 0;                  /* ﾌﾞﾛｯｸA ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_B:
            fmr15 = 1;
            asm(" ");
            fmr15 = 0;                  /* ﾌﾞﾛｯｸB ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_C:
            fmr16 = 1;
            asm(" ");
            fmr16 = 0;                  /* ﾌﾞﾛｯｸC ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_D:
            fmr17 = 1;
            asm(" ");
            fmr17 = 0;                  /* ﾌﾞﾛｯｸD ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
    }

    fmr20 = 0;                          /* ｲﾚｰｽﾞｻｽﾍﾟﾝﾄﾞ(ES)禁止         */
    fmr22 = 0;                          /* 割り込み要求でESﾘｸｴｽﾄ禁止    */
    fmr27 = 0;                          /* 低消費電流ﾘｰﾄﾞﾓｰﾄﾞ禁止       */

    /* ブロックイレーズ */
    *((char *)address) = 0x20;          /* ブロックイレーズコマンド     */
    *((char *)address) = 0xd0;
    while( fst7 == 0 );                 /* 消去待ち                     */

    ret = clrStatusRegister( address );

    *((char *)address) = 0xff;          /* データフラッシュ読み込み状態 */

    /* ﾃﾞｰﾀﾌﾗｯｼｭﾌﾞﾛｯｸ書き換え禁止ビットを書き換え禁止にする */
    switch( block ) {
        case DF_SELECT_A:
            fmr14 = 1;
            break;
        case DF_SELECT_B:
            fmr15 = 1;
            break;
        case DF_SELECT_C:
            fmr16 = 1;
            break;
        case DF_SELECT_D:
            fmr17 = 1;
            break;
    }
    fmr01 = 0;                          /* CPU書き換えモード無効        */

    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* モジュール名 writeDataFlash                                          */
/* 処理概要     プログラム(書き込み)処理                                */
/* 引数         unsigned int    書き込み元アドレス 0x3000-0x3fff        */
/*              char*           書き込み先アドレス                      */
/*              int             書き込むデータ数                        */
/* 戻り値       1:エラーなし 0:エラーあり                               */
/* メモ         実測で64バイト書き込みに9.5ms                           */
/************************************************************************/
int writeDataFlash( unsigned int w_address, signed char *r_address, int count )
{
    volatile int ret = 1, i;
    volatile int block, block2;

    // 開始アドレスのチェック
    block = checkBlockAddress( w_address );
    if( block == -1 ) return 0;

    // 終了アドレスのチェック
    block2 = checkBlockAddress( w_address + count - 1 );
    if( block2 == -1 ) return 0;

    // 同じブロックかチェック
    if( block != block2 ) return 0;

    asm( "FCLR I" );                    /* 全体の割り込み禁止           */

    fmr01 = 0;                          /* CPU書き換えモードに設定      */
    asm(" ");                           /* 最適化抑制                   */
    fmr01 = 1;
    fmr02 = 0;                          /* EW1モード選択                */
    asm(" ");                           /* 最適化抑制                   */
    fmr02 = 1;
    cmderie = 0;                        /* ｲﾚｰｽﾞ/ﾗｲﾄｴﾗｰ割り込み禁止     */
    bsyaeie = 0;                        /* ﾌﾗｯｼｭｱｸｾｽｴﾗｰ割り込み禁止     */
    rdystie = 0;                        /* ﾌﾗｯｼｭﾚﾃﾞｨｽﾃｰﾀｽ割り込み禁止   */

    /* ﾃﾞｰﾀﾌﾗｯｼｭﾌﾞﾛｯｸ書き換え禁止ビットを書き換え許可にする */
    switch( block ) {
        case DF_SELECT_A:
            fmr14 = 1;
            asm(" ");
            fmr14 = 0;                  /* ﾌﾞﾛｯｸA ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_B:
            fmr15 = 1;
            asm(" ");
            fmr15 = 0;                  /* ﾌﾞﾛｯｸB ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_C:
            fmr16 = 1;
            asm(" ");
            fmr16 = 0;                  /* ﾌﾞﾛｯｸC ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
        case DF_SELECT_D:
            fmr17 = 1;
            asm(" ");
            fmr17 = 0;                  /* ﾌﾞﾛｯｸD ｿﾌﾄｳｪｱｺﾏﾝﾄﾞ受付可能   */
            break;
    }

    fmr20 = 0;                          /* ｲﾚｰｽﾞｻｽﾍﾟﾝﾄﾞ(ES)禁止         */
    fmr22 = 0;                          /* 割り込み要求でESﾘｸｴｽﾄ禁止    */
    fmr27 = 0;                          /* 低消費電流ﾘｰﾄﾞﾓｰﾄﾞ禁止       */

    /* プログラム */
    while( count-- ) {
        *((signed char *)w_address) = 0x40;
        *((signed char *)w_address) = *r_address;
        while( fst7 == 0 );             /* 消去待ち                     */

        ret = clrStatusRegister( w_address );
        if( ret == 0 ) break;
        w_address++;
        r_address++;
    }

    *((unsigned char*)(w_address-1)) = 0xff;  /* ﾃﾞｰﾀﾌﾗｯｼｭ読み込み状態  */

    /* ﾃﾞｰﾀﾌﾗｯｼｭﾌﾞﾛｯｸ書き換え禁止ビットを書き換え禁止にする */
    switch( block ) {
        case DF_SELECT_A:
            fmr14 = 1;
            break;
        case DF_SELECT_B:
            fmr15 = 1;
            break;
        case DF_SELECT_C:
            fmr16 = 1;
            break;
        case DF_SELECT_D:
            fmr17 = 1;
            break;
        default:
            break;
    }
    fmr01 = 0;                          /* CPU書き換えモード無効        */

    asm( "FSET I" );                    /* 全体の割り込み許可           */

    return ret;
}

/************************************************************************/
/* モジュール名 clrStatusRegister                                       */
/* 処理概要     クリアステータスレジスタ                                */
/* 引数         アドレス                                                */
/* 戻り値       1:エラーなし 0:エラーあり                               */
/************************************************************************/
int clrStatusRegister( unsigned int c_address )
{
    volatile int ret = 1;

    if( fst5 == 1 || fst4 == 1 ) {
        *((char*)c_address) = 0x50;
        ret = 0;
    }

    return ret;
}

/************************************************************************/
/* モジュール名 checkBlockAddress                                       */
/* 処理概要     ブロックアドレスのチェック                              */
/* 引数         書き込もうとしているアドレス                            */
/* 戻り値       -1: エラー 0-3:ブロックNo                               */
/************************************************************************/
int checkBlockAddress( unsigned int chk_address )
{
    volatile int ret = -1;

    /* ブロックチェック */
    if( chk_address >= 0x3000 && chk_address <= 0x33ff ) {
        ret = DF_SELECT_A;
    } else if( chk_address >= 0x3400 && chk_address <= 0x37ff ) {
        ret = DF_SELECT_B;
    } else if( chk_address >= 0x3800 && chk_address <= 0x3bff ) {
        ret = DF_SELECT_C;
    } else if( chk_address >= 0x3c00 && chk_address <= 0x3fff ) {
        ret = DF_SELECT_D;
    }

    return ret;
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2010.04.01 Ver.1.00 作成
2011.03.18 Ver.1.02 writeDataFlash関数は、イレーズしないように修正
2011.04.01 Ver.1.03 変数にvolatileの追加
*/
