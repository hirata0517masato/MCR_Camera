#include "iodefine.h"
#include <machine.h>

#include "usb.h"

void _INIT_IOLIB( void ); //プロトタイプ宣言

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：USB CDCの初期化  			                                            */
/* 関 数 詳 細：		                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void USB_init(){
	/*Initialise the USB CDC Class*/
	USBCDC_Init();
   	setpsw_i();							//割込み許可 clrpsw_i()割込み禁止
	_INIT_IOLIB();						// printf(),scanf()の初期化

	while(0 == USBCDC_IsConnected());	//USB接続待ち
}