/****************************************************************************/
/* 対象マイコン R8C/38A                                                     */
/* ﾌｧｲﾙ内容     スタートアッププログラム(C言語版)                           */
/* バージョン   Ver.1.22                                                    */
/* Date         2012.03.08                                                  */
/* Copyright    ジャパンマイコンカーラリー実行委員会                        */
/*              ルネサスマイコンカーラリー事務局                            */
/*              日立インターメディックス株式会社                            */
/****************************************************************************/
/*======================================*/
/* プロトタイプ宣言(ローカル)           */
/*======================================*/
void main( void );

/*======================================*/
/* スタックサイズの設定                 */
/*======================================*/
#pragma STACKSIZE       0x200
#pragma ISTACKSIZE      0x100

/*======================================*/
/* CPUレジスタの宣言                    */
/*======================================*/
#pragma CREG    _flg_   flg
#pragma CREG    _isp_   isp
#pragma CREG    _sp_    sp
#pragma CREG    _sb_    sb
#pragma CREG    _fb_    fb
#pragma CREG    _intbh_ intbh
#pragma CREG    _intbl_ intbl

unsigned int _flg_;
unsigned int _sb_;
unsigned int _fb_;
unsigned int *_sp_;
unsigned int *_isp_;
unsigned int *_intbh_;
unsigned int *_intbl_;

/*======================================*/
/* SBの値をコンパイラに設定             */
/*======================================*/
    _asm("       .glb    __SB__\n"
         "__SB__ .equ    0400H   ");

/*======================================*/
/* オプション機能選択レジスタの設定     */
/*======================================*/
    _asm("  .ofsreg 0BFH ");            /* OFS = 0xbf (ﾊﾟﾜｰｵﾝﾘｾｯﾄ使用)  */

/*======================================*/
/* IDコードの設定                       */
/*======================================*/
    _asm("  .id ""\"#FFFFFFFFFFFFFF\" ");

/*======================================*/
/* RAMを初期化する関数の定義            */
/*======================================*/
#define scopy(X,Y,Z)    _asm("  .initsct "X","Y","Z"\n"\
            "   .initsct "X"I,rom"Y",noalign\n"\
            "   mov.w   #(topof "X"I)&0ffffH,A0\n"\
            "   mov.b   #(topof "X"I)>>16,R1H\n"\
            "   mov.w   #(topof "X")&0ffffH,A1\n"\
            "   mov.w   #sizeof "X",R3\n"\
            "   smovf.b")

#define sclear(X,Y,Z)   _asm("  .initsct "X","Y","Z"\n"\
            "   mov.b   #00H,R0L\n"\
            "   mov.w   #(topof "X") ,A1\n"\
            "   mov.w   #sizeof "X",R3\n"\
            "   sstr.b")

/*======================================*/
/* セクションの先頭アドレスの型定義     */
/*======================================*/
extern unsigned int _stack_top;
extern unsigned int _istack_top;
extern unsigned int _vector_top;

/*======================================*/
/* ヒープ領域の設定                     */
/*======================================*/
#pragma sectaddress heap_NE,DATA
#define __HEAPSIZE__    0x100

unsigned char heap_area[ __HEAPSIZE__ ]; /* ヒープ領域確保              */
extern unsigned char  _far *_mnext;     /* 次に使用できるﾒﾓﾘの先頭ｱﾄﾞﾚｽ */
extern unsigned long        _msize;     /* 残りのバイト数               */

/*======================================*/
/* 固定割り込みベクタアドレスの設定     */
/*======================================*/
#pragma sectaddress fvector,ROMDATA 0xffd8

    _asm("  .addr 0FFFFFFH");           /* 予約                         */
    _asm("  .byte 0FFH");               /* OFS2                         */
#pragma interrupt/v _dummy_int          /* 未定義命令割り込みベクタ     */
#pragma interrupt/v _dummy_int          /* オーバフロー割り込みベクタ   */
#pragma interrupt/v _dummy_int          /* BRK命令割り込みベクタ        */
#pragma interrupt/v _dummy_int          /* アドレス一致割り込みベクタ   */
#pragma interrupt/v _dummy_int          /* ｼﾝｸﾞﾙｽﾃｯﾌﾟ割り込みベクタ     */
#pragma interrupt/v _dummy_int          /* ｳｫｯﾁﾄﾞｯｸﾞﾀｲﾏなどの割り込みﾍﾞｸﾀ*/
#pragma interrupt/v _dummy_int          /* ｱﾄﾞﾚｽﾌﾞﾚｲｸ割り込みベクタ     */
#pragma interrupt/v _dummy_int          /* 予約                         */
#pragma interrupt/v start               /* リセットベクタ               */

/*======================================*/
/* 固定割り込みプログラム               */
/*======================================*/
#pragma sectaddress interrupt,CODE
#pragma interrupt _dummy_int()
void _dummy_int(void)
{
    /* ダミー関数 */
}

/*======================================*/
/* 可変割り込みベクタの設定             */
/*======================================*/
#pragma sectaddress vector,ROMDATA

    /* ここではセクション名の設定のみ行う */

/*======================================*/
/* RAMの初期化                          */
/*======================================*/
#pragma sectaddress program,CODE
void initsct(void)
{
    sclear("bss_SE","data","align");
    sclear("bss_SO","data","noalign");
    sclear("bss_NE","data","align");
    sclear("bss_NO","data","noalign");

    scopy("data_SE","data","align");
    scopy("data_SO","data","noalign");
    scopy("data_NE","data","align");
    scopy("data_NO","data","noalign");
}

/*======================================*/
/* スタートアッププログラム             */
/*======================================*/
#pragma entry start
void start( void )
{
    _isp_   = &_istack_top;             /* ISPに割り込みｽﾀｯｸのｱﾄﾞﾚｽ設定 */
    _flg_   = 0x0080;                   /* FLGのU="1"                   */
    _sp_    = &_stack_top;              /* USPにﾕｰｻﾞｽﾀｯｸのｱﾄﾞﾚｽ設定     */
    _sb_    = 0x0400U;                  /* SB相対アドレッシングの設定   */
    _intbh_ = (unsigned int*)0x00;      /* INTBH = vector(上位)に設定   */
    _intbl_ = &_vector_top;             /* INTBL = vector(下位)に設定   */
    initsct();                          /* RAMの初期化                  */
    _mnext = &heap_area[0];             /* ヒープ領域変数の設定         */
    _msize = (unsigned long)__HEAPSIZE__;
    _fb_    = 0U;                       /* FB = 0                       */

    main();                             /* main関数実行                 */

    while(1);
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/

/*
改訂経歴

2010.04.19 Ver.1.20 各種改訂
2011.04.01 Ver.1.21 main関数のプロトタイプ宣言追加
2012.04.01 Ver.1.22 STACKSIZEのサイズを0x100→0x200に変更
*/
