#include <stdio.h>
#include "iodefine.h"
#include "printf_lib.h"

/*
 * 標準printf()の出力先をSCI1にする
 *
 * CC-RXのprintf() → _write() → SCI1
 */

/*========================================================*/
/* SCI1初期化                                              */
/*========================================================*/

void initSCI1(int sp)
{
    volatile int i;

    /* SCI1モジュールストップ解除 */
    MSTP(SCI1) = 0;

    /* SCI1端子設定
       P30 : RXD1
       P26 : TXD1
    */
    IOPORT.PFFSCI.BIT.SCI1S = 0;

    PORT3.DDR.BIT.B0 = 0;
    PORT3.ICR.BIT.B0 = 1;

    PORT2.DDR.BIT.B6 = 1;

    /* SCI停止 */
    SCI1.SCR.BYTE = 0x00;

    /* ボーレート */
    if (sp == SPEED_4800)
    {
        SCI1.SMR.BYTE = 0x01;
        SCI1.BRR = 40 - 1;
    }
    else if (sp == SPEED_9600)
    {
        SCI1.SMR.BYTE = 0x00;
        SCI1.BRR = 80 - 1;
    }
    else if (sp == SPEED_19200)
    {
        SCI1.SMR.BYTE = 0x00;
        SCI1.BRR = 40 - 1;
    }
    else if (sp == SPEED_38400)
    {
        SCI1.SMR.BYTE = 0x00;
        SCI1.BRR = 20 - 1;
    }

    /* 1ビット程度待つ */
    for (i = 0; i < 4000; i++);

    /* ステータスクリア */
    SCI1.SSR.BYTE = 0x00;

    /*
     * TE = 1
     * RE = 1
     *
     * TIE = 0
     * RIE = 0
     * CKE = 00
     *
     * 0x30
     */
    SCI1.SCR.BYTE = 0x30;
}


/*========================================================*/
/* 標準printf()の出力処理                                 */
/*========================================================*/

/*
 * CC-RXの標準ライブラリから呼ばれるwrite関数
 *
 * printf()
 *   ↓
 * write()
 *   ↓
 * SCI1.TDR
 */
long write(long fileno, const unsigned char *buf, long count)
{
    long i;

    /*
     * stdout / stderr
     */
    if ((fileno != 1) && (fileno != 2))
    {
        return -1;
    }

    for (i = 0; i < count; i++)
    {
        /*
         * 改行をCR+LFにしたい場合
         */
        if (buf[i] == '\n')
        {
            /*
             * TDRE待ち
             */
            while (SCI1.SSR.BIT.TDRE == 0)
            {
                ;
            }

            SCI1.TDR = '\r';
        }

        /*
         * TDRE待ち
         */
        while (SCI1.SSR.BIT.TDRE == 0)
        {
            ;
        }

        /*
         * 1文字送信
         */
        SCI1.TDR = buf[i];
    }

    return count;
}