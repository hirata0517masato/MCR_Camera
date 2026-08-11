#include "iodefine.h"
#include <machine.h>
#include "rspi.h"


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* 関 数 概 要：SPIの初期化                                                                         */
/* 関 数 詳 細：                                                                                    */
/* 引       数：なし										    */
/* 戻  り   値：なし										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void SPI_init(void)
{
	SYSTEM.MSTPCRB.BIT.MSTPB16 = 0; //SPIストップ解除
 
	RSPI1.SPCR.BIT.SPE = 0;  //SPI機能停止
 
	RSPI1.SPCMD0.BIT.BRDV = 0;  //6MHz = PCLK/(2x(SPBR+1)x pow(2,BRDV)) 
	RSPI1.SPBR = 1;  	    //
 
	RSPI1.SPPCR.BYTE = 0x00; // CMOS出力、アイドル時のMOSI信号値は"0"

	RSPI1.SPDCR.BIT.SLSEL = 1; //SSL0 許可
	RSPI1.SPDCR.BIT.SPRDTD = 0; //0:受信 1:送信　バッファ読み出しモード
	RSPI1.SPDCR.BIT.SPLW = 0; //ワードアクセス
 
	RSPI1.SPCMD0.BIT.CPHA = 1; //奇数エッジでデータ変化偶数でサンプル
	RSPI1.SPCMD0.BIT.CPOL = 1; //アイドル時RSPCK=1
	RSPI1.SPCMD0.BIT.SSLA = 0; //SSL0でアサート
	RSPI1.SPCMD0.BIT.SSLKP = 1; //SSL信号を保持
	RSPI1.SPCMD0.BIT.SPB = 7; //データ長8bit
 	RSPI1.SPCMD0.BIT.LSBF = 0; //MSBファースト
 
	PORTE.ICR.BIT.B7=1;  //入力バッファ有効
	IOPORT.PFHSPI.BIT.RSPIS = 1; //SPI端子portEに変更
	IOPORT.PFHSPI.BIT.RSPCKE = 1; //RSPCKB有効
	IOPORT.PFHSPI.BIT.MOSIE = 1; //MOSIB有効
	IOPORT.PFHSPI.BIT.MISOE = 1; //MISOB有効
	IOPORT.PFHSPI.BIT.SSL0E = 1; //SSL0B有効

	RSPI1.SPCR.BYTE |= 0x08;	//マスタモード
 
	RSPI1.SPSR.BYTE = 0xA0;  //ステータスクリア	
}

/*
//センサレジスタから1バイト読み出し
unsigned char rspi_read( unsigned short reg){
	unsigned short	recv;
	RSPI1.SPCR.BIT.SPE = 1;   // enable
	
	RSPI1.SPDR.WORD.H = (reg | 0x80) << 8;  // 送信
	
	while( 0 == RSPI1.SPSR.BIT.SPRF ); // 受信完了待ち
	recv = RSPI1.SPDR.WORD.H;  // 受信バッファ読出し
	RSPI1.SPCR.BIT.SPE = 0;   // disable
	
	recv = recv&0x00ff;   //マスク
	return(recv);
}

void rspi_write( unsigned short reg,unsigned short data){
	
	RSPI1.SPCR.BIT.SPE = 1;   // enable
	
	RSPI1.SPDR.WORD.H = (reg << 8) + data;  // 送信
	
	while( 0 == RSPI1.SPSR.BIT.SPTEF ); // 受信完了待ち
	
	RSPI1.SPCR.BIT.SPE = 0;   // disable
}*/

unsigned char rspi_transfer( unsigned char reg){
	unsigned char	recv;

	RSPI1.SPDR.WORD.H = reg ;  // 送信
	
	while( 0 == RSPI1.SPSR.BIT.SPRF ); // 受信完了待ち
	recv = RSPI1.SPDR.WORD.H;  // 受信バッファ読出し
	
	//recv = recv&0x00ff;   //マスク
	return(recv);
}

void rspi_select(){
	RSPI1.SPCR.BIT.SPE = 1;   // enable
}

void rspi_deselect(){
	RSPI1.SPCR.BIT.SPE = 0;   // disable
}
