/*******************************************************************************
*
* Device       : R8C/38A Group
*
* File Name    : SFR_R838a.h
*
* Abstract     : definition of R8C/38A Group SFR
*
* History      :  0.20  ( 2009-09-29 ) [Hardware Manual Revision : 0.10]
*                                      [Technical Update No : TN-R8C-A001B/J]
*                                      [Technical Update No : TN-R8C-A006A/J]
*                       add : trfisel0, trgioasel, trgiobsel, trgclkasel, trgclkbsel
*                       del : fra7, fra0, fra1, fra2, fra4, fra5, fra6, fra3
*                       correct comment :pm1, cm0, cm1, mstcr, rstfr, cspr, ocvrefcr,
*                                        cmpa, vcac, vca1, vca2, vw1c, vw2c, dtctl,
*                                        dtcen0Å`dtcen6, trfcr0, trfcr1, u2smr4,
*                                        u2smr3, u2smr2, u2smr, admod, adinsel,
*                                        adcon1, p0, p1, pd0, pd1, p2, p3, pd2,
*                                        pd3, p4, p5, pd4, pd5, p6, p7, pd6, pd7,
*                                        p8, p9, pd8, pd9, tracr, lincr2, lincr,
*                                        linst, trcmr, trccr1, trcior0, trcior1,
*                                        trccr2, trcdf, trcadcr, trdstr, trdmr,
*                                        trdpmr, trdfcr, trdocr, trddf0, trddf1,
*                                        trdcr0, trdiora0, trdiorc0, trdcr1, trdiora1,
*                                        trdiorc1, trfout, u2sr0, pinsr, ssbr, sscrh,
*                                        sscrl, ssmr, sssr, ssmr2, fst, fmr1, fmr2,
*                                        aier0, aier1, pur0, pur1, pur2, p1drr,
*                                        p2drr, drr0, drr1, vlt0, vlt1, vlt2
*                       correct define : sstdr, icdrt, ssrdr, sstdr, iccr1, iccr2,
*                                        icmr, icier, icsr, sar, dtcvct0Å`dtcvct63,
*                                        dtsar0Å`dtsar23, dtdar0Å`dtdar23
*                 0.10  ( 2009-02-24 ) [Hardware Manual Revision : 0.10]
*
*  Copyright(c) 2008 Renesas Technology Corp.,All Rights Reserved.
*
*******************************************************************************/

/********************************************************
*  declare SFR addresses                                *
********************************************************/

#pragma	ADDRESS		pm0_addr		0004H	/* Processor Mode Register 0 */

#pragma	ADDRESS		pm1_addr		0005H	/* Processor Mode Register 1 */

#pragma	ADDRESS		cm0_addr		0006H	/* System Clock Control Register 0 */

#pragma	ADDRESS		cm1_addr		0007H	/* System Clock Control Register 1 */

#pragma	ADDRESS		mstcr_addr		0008H	/* Module Standby Control Register */

#pragma	ADDRESS		cm3_addr		0009H	/* System Clock Control Register 3 */

#pragma	ADDRESS		prcr_addr		000AH	/* Protect Register */

#pragma	ADDRESS		rstfr_addr		000BH	/* Reset Source Determination Register */

#pragma	ADDRESS		ocd_addr		000CH	/* Oscillation Stop Detection Register */

#pragma	ADDRESS		wdtr_addr		000DH	/* Watchdog Timer Reset Register */

#pragma	ADDRESS		wdts_addr		000EH	/* Watchdog Timer Start Register */

#pragma	ADDRESS		wdtc_addr		000FH	/* Watchdog Timer Control Register */

#pragma	ADDRESS		cspr_addr		001CH	/* Count Source Protection Mode Register */

#pragma	ADDRESS		ocvrefcr_addr	0026H	/* On-Chip Reference Voltage Control Register */

#pragma	ADDRESS		cpsrf_addr		0028H	/* Clock Prescaler Reset Flag */

#pragma	ADDRESS		cmpa_addr		0030H	/* Voltage Monitor Circuit/Comparator A Control Register */

#pragma	ADDRESS		vcac_addr		0031H	/* Voltage Monitor Circuit Edge Select Register */

#pragma	ADDRESS		vca1_addr		0033H	/* Voltage Detect Register 1 */

#pragma	ADDRESS		vca2_addr		0034H	/* Voltage Detect Register 2 */

#pragma	ADDRESS		vd1ls_addr		0036H	/* Voltage Detection 1 Level Select Register */

#pragma	ADDRESS		vw0c_addr		0038H	/* Voltage Monitor 0 Circuit Control Register */

#pragma	ADDRESS		vw1c_addr		0039H	/* Voltage Monitor 1 Circuit Control Register */

#pragma	ADDRESS		vw2c_addr		003AH	/* Voltage Monitor 2 Circuit Control Register */

#pragma	ADDRESS		fmrdyic_addr	0041H	/* Flash Memory Ready Interrupt Control Register */

#pragma	ADDRESS		int4ic_addr		0046H	/* INT4 Interrupt Control Register */

#pragma	ADDRESS		trcic_addr		0047H	/* Timer RC Interrupt Control Register */

#pragma	ADDRESS		trd0ic_addr		0048H	/* Timer RD0 Interrupt Control Register */

#pragma	ADDRESS		trd1ic_addr		0049H	/* Timer RD1 Interrupt Control Register */

#pragma	ADDRESS		treic_addr		004AH	/* Timer RE Interrupt Control Register */

#pragma	ADDRESS		s2tic_addr		004BH	/* UART2 Transmit Interrupt Control Register */

#pragma	ADDRESS		s2ric_addr		004CH	/* UART2 Receive Interrupt Control Register */

#pragma	ADDRESS		kupic_addr		004DH	/* Key Input Interrupt Control Register */

#pragma	ADDRESS		adic_addr		004EH	/* A/D Conversion Interrupt Control Register */

#pragma	ADDRESS		ssuic_addr		004FH	/* SSU Interrupt Control Register */

#pragma	ADDRESS		iicic_addr		004FH	/* IIC bus Interrupt Control Register */

#pragma	ADDRESS		cmp1ic_addr		0050H	/* Timer RF Compare 1 Interrupt Control Register */

#pragma	ADDRESS		s0tic_addr		0051H	/* UART0 Transmit Interrupt Control Register */

#pragma	ADDRESS		s0ric_addr		0052H	/* UART0 Receive Interrupt Control Register */

#pragma	ADDRESS		s1tic_addr		0053H	/* UART1 Transmit Interrupt Control Register */

#pragma	ADDRESS		s1ric_addr		0054H	/* UART1 Receive Interrupt Control Register */

#pragma	ADDRESS		int2ic_addr		0055H	/* INT2 Interrupt Control Register */

#pragma	ADDRESS		traic_addr		0056H	/* Timer RA Interrupt Control Register */

#pragma	ADDRESS		trbic_addr		0058H	/* Timer RB Interrupt Control Register */

#pragma	ADDRESS		int1ic_addr		0059H	/* INT1 Interrupt Control Register */

#pragma	ADDRESS		int3ic_addr		005AH	/* INT3 Interrupt Control Register */

#pragma	ADDRESS		trfic_addr		005BH	/* Timer RF Interrupt Control Register */

#pragma	ADDRESS		cmp0ic_addr		005CH	/* Timer RF Compare 0 Interrupt Control Register */

#pragma	ADDRESS		int0ic_addr		005DH	/* INT0 Interrupt Control Register */

#pragma	ADDRESS		u2bcnic_addr	005EH	/* UART2 Bus Collision Detection Interrupt Control Register */

#pragma	ADDRESS		capic_addr		005FH	/* Timer RF Capture Interrupt Control Register */

#pragma	ADDRESS		trgic_addr		006BH	/* Timer RG Interrupt Control Register */

#pragma	ADDRESS		vcmp1ic_addr	0072H	/* Voltage Monitor 1/Compare A1 Interrupt Control Register */

#pragma	ADDRESS		vcmp2ic_addr	0073H	/* Voltage Monitor 2/Compare A2 Interrupt Control Register */

#pragma	ADDRESS		dtctl_addr		0080H	/* DTC Activation Control Register */

#pragma	ADDRESS		dtcen0_addr		0088H	/* DTC Activation Enable Register 0 */

#pragma	ADDRESS		dtcen1_addr		0089H	/* DTC Activation Enable Register 1 */

#pragma	ADDRESS		dtcen2_addr		008AH	/* DTC Activation Enable Register 2 */

#pragma	ADDRESS		dtcen3_addr		008BH	/* DTC Activation Enable Register 3 */

#pragma	ADDRESS		dtcen4_addr		008CH	/* DTC Activation Enable Register 4 */

#pragma	ADDRESS		dtcen5_addr		008DH	/* DTC Activation Enable Register 5 */

#pragma	ADDRESS		dtcen6_addr		008EH	/* DTC Activation Enable Register 6 */

#pragma	ADDRESS		trf_addr		0090H	/* Timer RF Register */

#pragma	ADDRESS		trfcr0_addr		009AH	/* Timer RF Control Register 0 */

#pragma	ADDRESS		trfcr1_addr		009BH	/* Timer RF Control Register 1 */

#pragma	ADDRESS		trfm0_addr		009CH	/* Capture and Compare 0 Register */

#pragma	ADDRESS		trfm1_addr		009EH	/* Compare 1 Register */

#pragma	ADDRESS		u0mr_addr		00A0H	/* UART0 Transmit/Receive Mode Register */

#pragma	ADDRESS		u0brg_addr		00A1H	/* UART0 Bit Rate Register */

#pragma	ADDRESS		u0tb_addr		00A2H	/* UART0 Transmit Buffer Register */

#pragma	ADDRESS		u0c0_addr		00A4H	/* UART0 Transmit/Receive Control Register 0 */

#pragma	ADDRESS		u0c1_addr		00A5H	/* UART0 Transmit/Receive Control Register 1 */

#pragma	ADDRESS		u0rb_addr		00A6H	/* UART0 Receive Buffer Register */

#pragma	ADDRESS		u2mr_addr		00A8H	/* UART2 Transmit/Receive Mode Register */

#pragma	ADDRESS		u2brg_addr		00A9H	/* UART2 Bit Rate Register */

#pragma	ADDRESS		u2tb_addr		00AAH	/* UART2 Transmit Buffer Register */

#pragma	ADDRESS		u2c0_addr		00ACH	/* UART2 Transmit/Receive Control Register 0 */

#pragma	ADDRESS		u2c1_addr		00ADH	/* UART2 Transmit/Receive Control Register 1 */

#pragma	ADDRESS		u2rb_addr		00AEH	/* UART2 Receive Buffer Register */

#pragma	ADDRESS		urxdf_addr		00B0H	/* UART2 Digital Filter Function Select Register */

#pragma	ADDRESS		u2smr5_addr		00BBH	/* UART2 Special Mode Register 5 */

#pragma	ADDRESS		u2smr4_addr		00BCH	/* UART2 Special Mode Register 4 */

#pragma	ADDRESS		u2smr3_addr		00BDH	/* UART2 Special Mode Register 3 */

#pragma	ADDRESS		u2smr2_addr		00BEH	/* UART2 Special Mode Register 2 */

#pragma	ADDRESS		u2smr_addr		00BFH	/* UART2 Special Mode Register */

#pragma	ADDRESS		ad0_addr		00C0H	/* A/D Register 0 */

#pragma	ADDRESS		ad1_addr		00C2H	/* A/D Register 1 */

#pragma	ADDRESS		ad2_addr		00C4H	/* A/D Register 2 */

#pragma	ADDRESS		ad3_addr		00C6H	/* A/D Register 3 */

#pragma	ADDRESS		ad4_addr		00C8H	/* A/D Register 4 */

#pragma	ADDRESS		ad5_addr		00CAH	/* A/D Register 5 */

#pragma	ADDRESS		ad6_addr		00CCH	/* A/D Register 6 */

#pragma	ADDRESS		ad7_addr		00CEH	/* A/D Register 7 */

#pragma	ADDRESS		admod_addr		00D4H	/* A/D Mode Register */

#pragma	ADDRESS		adinsel_addr	00D5H	/* A/D Input Select Register */

#pragma	ADDRESS		adcon0_addr		00D6H	/* A/D Control Register 0 */

#pragma	ADDRESS		adcon1_addr		00D7H	/* A/D Control Register 1 */

#pragma	ADDRESS		da0_addr		00D8H	/* D/A0 Register */

#pragma	ADDRESS		da1_addr		00D9H	/* D/A1 Register */

#pragma	ADDRESS		dacon_addr		00DCH	/* D/A Control Register */

#pragma	ADDRESS		p0_addr			00E0H	/* Port P0 Register */

#pragma	ADDRESS		p1_addr			00E1H	/* Port P1 Register */

#pragma	ADDRESS		pd0_addr		00E2H	/* Port P0 Direction Register */

#pragma	ADDRESS		pd1_addr		00E3H	/* Port P1 Direction Register */

#pragma	ADDRESS		p2_addr			00E4H	/* Port P2 Register */

#pragma	ADDRESS		p3_addr			00E5H	/* Port P3 Register */

#pragma	ADDRESS		pd2_addr		00E6H	/* Port P2 Direction Register */

#pragma	ADDRESS		pd3_addr		00E7H	/* Port P3 Direction Register */

#pragma	ADDRESS		p4_addr			00E8H	/* Port P4 Register */

#pragma	ADDRESS		p5_addr			00E9H	/* Port P5 Register */

#pragma	ADDRESS		pd4_addr		00EAH	/* Port P4 Direction Register */

#pragma	ADDRESS		pd5_addr		00EBH	/* Port P5 Direction Register */

#pragma	ADDRESS		p6_addr			00ECH	/* Port P6 Register */

#pragma	ADDRESS		p7_addr			00EDH	/* Port P7 Register */

#pragma	ADDRESS		pd6_addr		00EEH	/* Port P6 Direction Register */

#pragma	ADDRESS		pd7_addr		00EFH	/* Port P7 Direction Register */

#pragma	ADDRESS		p8_addr			00F0H	/* Port P8 Register */

#pragma	ADDRESS		p9_addr			00F1H	/* Port P9 Register */

#pragma	ADDRESS		pd8_addr		00F2H	/* Port P8 Direction Register */

#pragma	ADDRESS		pd9_addr		00F3H	/* Port P9 Direction Register */

#pragma	ADDRESS		tracr_addr		0100H	/* Timer RA Control Register */

#pragma	ADDRESS		traioc_addr		0101H	/* Timer RA I/O Control Register */

#pragma	ADDRESS		tramr_addr		0102H	/* Timer RA Mode Register */

#pragma	ADDRESS		trapre_addr		0103H	/* Timer RA Prescaler Register */

#pragma	ADDRESS		tra_addr		0104H	/* Timer RA Register */

#pragma	ADDRESS		lincr2_addr		0105H	/* LIN Control Register 2 */

#pragma	ADDRESS		lincr_addr		0106H	/* LIN Control Register */

#pragma	ADDRESS		linst_addr		0107H	/* LIN Status Register */

#pragma	ADDRESS		trbcr_addr		0108H	/* Timer RB Control Register */

#pragma	ADDRESS		trbocr_addr		0109H	/* Timer RB One-Shot Control Register */

#pragma	ADDRESS		trbioc_addr		010AH	/* Timer RB I/O Control Register */

#pragma	ADDRESS		trbmr_addr		010BH	/* Timer RB Mode Register */

#pragma	ADDRESS		trbpre_addr		010CH	/* Timer RB Prescaler Register */

#pragma	ADDRESS		trbsc_addr		010DH	/* Timer RB Secondary Register */

#pragma	ADDRESS		trbpr_addr		010EH	/* Timer RB Primary Register */

#pragma	ADDRESS		tresec_addr		0118H	/* Timer RE Second Data Register / Counter Data Register */

#pragma	ADDRESS		tremin_addr		0119H	/* Timer RE Minute Data Register / Compare Data Register */

#pragma	ADDRESS		trehr_addr		011AH	/* Timer RE Hour Data Register */

#pragma	ADDRESS		trewk_addr		011BH	/* Timer RE Day of Week Data Register */

#pragma	ADDRESS		trecr1_addr		011CH	/* Timer RE Control Register 1 */

#pragma	ADDRESS		trecr2_addr		011DH	/* Timer RE Control Register 2 */

#pragma	ADDRESS		trecsr_addr		011EH	/* Timer RE Count Source Select Register */

#pragma	ADDRESS		trcmr_addr		0120H	/* Timer RC Mode Register */

#pragma	ADDRESS		trccr1_addr		0121H	/* Timer RC Control Register 1 */

#pragma	ADDRESS		trcier_addr		0122H	/* Timer RC Interrupt Enable Register */

#pragma	ADDRESS		trcsr_addr		0123H	/* Timer RC Status Register */

#pragma	ADDRESS		trcior0_addr	0124H	/* Timer RC I/O Control Register 0 */

#pragma	ADDRESS		trcior1_addr	0125H	/* Timer RC I/O Control Register 1 */

#pragma	ADDRESS		trc_addr		0126H	/* Timer RC Counter */

#pragma	ADDRESS		trcgra_addr		0128H	/* Timer RC General Register A */

#pragma	ADDRESS		trcgrb_addr		012AH	/* Timer RC General Register B */

#pragma	ADDRESS		trcgrc_addr		012CH	/* Timer RC General Register C */

#pragma	ADDRESS		trcgrd_addr		012EH	/* Timer RC General Register D */

#pragma	ADDRESS		trccr2_addr		0130H	/* Timer RC Control Register 2 */

#pragma	ADDRESS		trcdf_addr		0131H	/* Timer RC Digital Filter Function Select Register */

#pragma	ADDRESS		trcoer_addr		0132H	/* Timer RC Output Master Enable Register */

#pragma	ADDRESS		trcadcr_addr	0133H	/* Timer RC Trigger Control Register */

#pragma	ADDRESS		trdecr_addr		0135H	/* Timer RD Control Expansion Register */

#pragma	ADDRESS		trdadcr_addr	0136H	/* Timer RD Trigger Control Register */

#pragma	ADDRESS		trdstr_addr		0137H	/* Timer RD Start Register */

#pragma	ADDRESS		trdmr_addr		0138H	/* Timer RD Mode Register */

#pragma	ADDRESS		trdpmr_addr		0139H	/* Timer RD PWM Mode Register */

#pragma	ADDRESS		trdfcr_addr		013AH	/* Timer RD Function Control Register */

#pragma	ADDRESS		trdoer1_addr	013BH	/* Timer RD Output Master Enable Register 1 */

#pragma	ADDRESS		trdoer2_addr	013CH	/* Timer RD Output Master Enable Register 2 */

#pragma	ADDRESS		trdocr_addr		013DH	/* Timer RD Output Control Register */

#pragma	ADDRESS		trddf0_addr		013EH	/* Timer RD Digital Filter Function Select Register 0 */

#pragma	ADDRESS		trddf1_addr		013FH	/* Timer RD Digital Filter Function Select Register 1 */

#pragma	ADDRESS		trdcr0_addr		0140H	/* Timer RD Control Register 0 */

#pragma	ADDRESS		trdiora0_addr	0141H	/* Timer RD I/O Control Register A0 */

#pragma	ADDRESS		trdiorc0_addr	0142H	/* Timer RD I/O Control Register C0 */

#pragma	ADDRESS		trdsr0_addr		0143H	/* Timer RD Status Register 0 */

#pragma	ADDRESS		trdier0_addr	0144H	/* Timer RD Interrupt Enable Register 0 */

#pragma	ADDRESS		trdpocr0_addr	0145H	/* Timer RD PWM Mode Output Level Control Register 0 */

#pragma	ADDRESS		trd0_addr		0146H	/* Timer RD Counter 0 */

#pragma	ADDRESS		trdgra0_addr	0148H	/* Timer RD General Register A0 */

#pragma	ADDRESS		trdgrb0_addr	014AH	/* Timer RD General Register B0 */

#pragma	ADDRESS		trdgrc0_addr	014CH	/* Timer RD General Register C0 */

#pragma	ADDRESS		trdgrd0_addr	014EH	/* Timer RD General Register D0 */

#pragma	ADDRESS		trdcr1_addr		0150H	/* Timer RD Control Register 1 */

#pragma	ADDRESS		trdiora1_addr	0151H	/* Timer RD I/O Control Register A1 */

#pragma	ADDRESS		trdiorc1_addr	0152H	/* Timer RD I/O Control Register C1 */

#pragma	ADDRESS		trdsr1_addr		0153H	/* Timer RD Status Register 1 */

#pragma	ADDRESS		trdier1_addr	0154H	/* Timer RD Interrupt Enable Register 1 */

#pragma	ADDRESS		trdpocr1_addr	0155H	/* Timer RD PWM Mode Output Level Control Register 1 */

#pragma	ADDRESS		trd1_addr		0156H	/* Timer RD Counter 1 */

#pragma	ADDRESS		trdgra1_addr	0158H	/* Timer RD General Register A1 */

#pragma	ADDRESS		trdgrb1_addr	015AH	/* Timer RD General Register B1 */

#pragma	ADDRESS		trdgrc1_addr	015CH	/* Timer RD General Register C1 */

#pragma	ADDRESS		trdgrd1_addr	015EH	/* Timer RD General Register D1 */

#pragma	ADDRESS		u1mr_addr		0160H	/* UART1 Transmit/Receive Mode Register */

#pragma	ADDRESS		u1brg_addr		0161H	/* UART1 Bit Rate Register */

#pragma	ADDRESS		u1tb_addr		0162H	/* UART1 Transmit Buffer Register */

#pragma	ADDRESS		u1c0_addr		0164H	/* UART1 Transmit/Receive Control Register 0 */

#pragma	ADDRESS		u1c1_addr		0165H	/* UART1 Transmit/Receive Control Register 1 */

#pragma	ADDRESS		u1rb_addr		0166H	/* UART1 Receive Buffer Register */

#pragma	ADDRESS		trgmr_addr		0170H	/* Timer RG Mode Register */

#pragma	ADDRESS		trgcntc_addr	0171H	/* Timer RG Count Control Register */

#pragma	ADDRESS		trgcr_addr		0172H	/* Timer RG Control Register */

#pragma	ADDRESS		trgier_addr		0173H	/* Timer RG Interrupt Enable Register */

#pragma	ADDRESS		trgsr_addr		0174H	/* Timer RG Status Register */

#pragma	ADDRESS		trgior_addr		0175H	/* Timer RG I/O Control Register */

#pragma	ADDRESS		trg_addr		0176H	/* Timer RG Counter */

#pragma	ADDRESS		trggra_addr		0178H	/* Timer RG General Register A */

#pragma	ADDRESS		trggrb_addr		017AH	/* Timer RG General Register B */

#pragma	ADDRESS		trggrc_addr		017CH	/* Timer RG General Register C */

#pragma	ADDRESS		trggrd_addr		017EH	/* Timer RG General Register D */

#pragma	ADDRESS		trasr_addr		0180H	/* Timer RA Pin Select Register */

#pragma	ADDRESS		trbrcsr_addr	0181H	/* Timer RB/RC Pin Select Register */

#pragma	ADDRESS		trcpsr0_addr	0182H	/* Timer RC Pin Select Register 0 */

#pragma	ADDRESS		trcpsr1_addr	0183H	/* Timer RC Pin Select Register 1 */

#pragma	ADDRESS		trdpsr0_addr	0184H	/* Timer RD Pin Select Register 0 */

#pragma	ADDRESS		trdpsr1_addr	0185H	/* Timer RD Pin Select Register 1 */

#pragma	ADDRESS		timsr_addr		0186H	/* Timer Pin Select Register */

#pragma	ADDRESS		trfout_addr		0187H	/* Timer RF Output Control Register */

#pragma	ADDRESS		u0sr_addr		0188H	/* UART0 Pin Select Register */

#pragma	ADDRESS		u1sr_addr		0189H	/* UART1 Pin Select Register */

#pragma	ADDRESS		u2sr0_addr		018AH	/* UART2 Pin Select Register 0 */

#pragma	ADDRESS		u2sr1_addr		018BH	/* UART2 Pin Select Register 1 */

#pragma	ADDRESS		ssuiicsr_addr	018CH	/* SSU/IIC Pin Select Register */

#pragma	ADDRESS		intsr_addr		018EH	/* INT Interrupt Input Pin Select Register */

#pragma	ADDRESS		pinsr_addr		018FH	/* I/O Function Pin Select Register */

#pragma	ADDRESS		ssbr_addr		0193H	/* SS Bit Counter Register */

#pragma	ADDRESS		sstdr_addr		0194H	/* SS Transmit Data Register */

#pragma	ADDRESS		icdrt_addr		0194H	/* IIC bus Transmit Data Register */

#pragma	ADDRESS		ssrdr_addr		0196H	/* SS Receive Data Register */

#pragma	ADDRESS		icdrr_addr		0196H	/* IIC bus Receive Data Register */

#pragma	ADDRESS		sscrh_addr		0198H	/* SS Control Register H */

#pragma	ADDRESS		iccr1_addr		0198H	/* IIC bus Control Register 1 */

#pragma	ADDRESS		sscrl_addr		0199H	/* SS Control Register L */

#pragma	ADDRESS		iccr2_addr		0199H	/* IIC bus Control Register 2 */

#pragma	ADDRESS		ssmr_addr		019AH	/* SS Mode Register */

#pragma	ADDRESS		icmr_addr		019AH	/* IIC bus Mode Register */

#pragma	ADDRESS		sser_addr		019BH	/* SS Enable Register */

#pragma	ADDRESS		icier_addr		019BH	/* IIC bus Interrupt Enable Register */

#pragma	ADDRESS		sssr_addr		019CH	/* SS Status Register */

#pragma	ADDRESS		icsr_addr		019CH	/* IIC bus Status Register */

#pragma	ADDRESS		ssmr2_addr		019DH	/* SS Mode Register 2 */

#pragma	ADDRESS		sar_addr		019DH	/* Slave Address Register */

#pragma	ADDRESS		fst_addr		01B2H	/* Flash Memory Status Register */

#pragma	ADDRESS		fmr0_addr		01B4H	/* Flash Memory Control Register 0 */

#pragma	ADDRESS		fmr1_addr		01B5H	/* Flash Memory Control Register 1 */

#pragma	ADDRESS		fmr2_addr		01B6H	/* Flash Memory Control Register 2 */

#pragma	ADDRESS		rmad0_addr		01C0H	/* Address Match Interrupt Register 0 */

#pragma	ADDRESS		aier0_addr		01C3H	/* Address Match Interrupt Enable Register 0 */

#pragma	ADDRESS		rmad1_addr		01C4H	/* Address Match Interrupt Register 1 */

#pragma	ADDRESS		aier1_addr		01C7H	/* Address Match Interrupt Enable Register 1 */

#pragma	ADDRESS		pur0_addr		01E0H	/* Pull-Up Control Register 0 */

#pragma	ADDRESS		pur1_addr		01E1H	/* Pull-Up Control Register 1 */

#pragma	ADDRESS		pur2_addr		01E2H	/* Pull-Up Control Register 2 */

#pragma	ADDRESS		p1drr_addr		01F0H	/* Port P1 Drive Capacity Control Register */

#pragma	ADDRESS		p2drr_addr		01F1H	/* Port P2 Drive Capacity Control Register */

#pragma	ADDRESS		drr0_addr		01F2H	/* Drive Capacity Control Register 0 */

#pragma	ADDRESS		drr1_addr		01F3H	/* Drive Capacity Control Register 1 */

#pragma	ADDRESS		drr2_addr		01F4H	/* Drive Capacity Control Register 2 */

#pragma	ADDRESS		vlt0_addr		01F5H	/* Input Threshold Control Register 0 */

#pragma	ADDRESS		vlt1_addr		01F6H	/* Input Threshold Control Register 1 */

#pragma	ADDRESS		vlt2_addr		01F7H	/* Input Threshold Control Register 2 */

#pragma	ADDRESS		intcmp_addr		01F8H	/* Comparator B Control Register 0 */

#pragma	ADDRESS		inten_addr		01FAH	/* External Input Enable Register 0 */

#pragma	ADDRESS		inten1_addr		01FBH	/* External Input Enable Register 1 */

#pragma	ADDRESS		intf_addr		01FCH	/* INT Input Filter Select Register 0 */

#pragma	ADDRESS		intf1_addr		01FDH	/* INT Input Filter Select Register 1 */

#pragma	ADDRESS		kien_addr		01FEH	/* Key Input Enable Register 0 */

#pragma	ADDRESS		dtcvct0_addr	2C00H	/* DTC Transfer Vector Area0 (INT0)  */

#pragma	ADDRESS		dtcvct1_addr	2C01H	/* DTC Transfer Vector Area1 (INT1)  */

#pragma	ADDRESS		dtcvct2_addr	2C02H	/* DTC Transfer Vector Area2 (INT2)  */

#pragma	ADDRESS		dtcvct3_addr	2C03H	/* DTC Transfer Vector Area3 (INT3)  */

#pragma	ADDRESS		dtcvct4_addr	2C04H	/* DTC Transfer Vector Area4 (INT4)  */

#pragma	ADDRESS		dtcvct5_addr	2C05H	/* DTC Transfer Vector Area5 */

#pragma	ADDRESS		dtcvct6_addr	2C06H	/* DTC Transfer Vector Area6 */

#pragma	ADDRESS		dtcvct7_addr	2C07H	/* DTC Transfer Vector Area7 */

#pragma	ADDRESS		dtcvct8_addr	2C08H	/* DTC Transfer Vector Area8 (Key Input)  */

#pragma	ADDRESS		dtcvct9_addr	2C09H	/* DTC Transfer Vector Area9 (A-D)  */

#pragma	ADDRESS		dtcvct10_addr	2C0AH	/* DTC Transfer Vector Area10 (Uart0 Receive)  */

#pragma	ADDRESS		dtcvct11_addr	2C0BH	/* DTC Transfer Vector Area11 (Uart0 Transfer)  */

#pragma	ADDRESS		dtcvct12_addr	2C0CH	/* DTC Transfer Vector Area12 (Uart1 Receive)  */

#pragma	ADDRESS		dtcvct13_addr	2C0DH	/* DTC Transfer Vector Area13 (Uart1 Transfer)  */

#pragma	ADDRESS		dtcvct14_addr	2C0EH	/* DTC Transfer Vector Area14 (Uart2 Receive)  */

#pragma	ADDRESS		dtcvct15_addr	2C0FH	/* DTC Transfer Vector Area15 (Uart2 Transfer)  */

#pragma	ADDRESS		dtcvct16_addr	2C10H	/* DTC Transfer Vector Area16 (SSU/I2C Receive data register full)  */

#pragma	ADDRESS		dtcvct17_addr	2C11H	/* DTC Transfer Vector Area17 (SSU/I2C Transmit data empty)  */

#pragma	ADDRESS		dtcvct18_addr	2C12H	/* DTC Transfer Vector Area18 (Voltage monitor circuit 2 / comparator A2)  */

#pragma	ADDRESS		dtcvct19_addr	2C13H	/* DTC Transfer Vector Area19 (Voltage monitor circuit 1 / comparator A1)  */

#pragma	ADDRESS		dtcvct20_addr	2C14H	/* DTC Transfer Vector Area20 */

#pragma	ADDRESS		dtcvct21_addr	2C15H	/* DTC Transfer Vector Area21 */

#pragma	ADDRESS		dtcvct22_addr	2C16H	/* DTC Transfer Vector Area22 (TRC Input capture / compare match A)  */

#pragma	ADDRESS		dtcvct23_addr	2C17H	/* DTC Transfer Vector Area23 (TRC Input capture / compare match B)  */

#pragma	ADDRESS		dtcvct24_addr	2C18H	/* DTC Transfer Vector Area24 (TRC Input capture / compare match C)  */

#pragma	ADDRESS		dtcvct25_addr	2C19H	/* DTC Transfer Vector Area25 (TRC Input capture / compare match D)  */

#pragma	ADDRESS		dtcvct26_addr	2C1AH	/* DTC Transfer Vector Area26 (TRD0 Input capture / compare match A)  */

#pragma	ADDRESS		dtcvct27_addr	2C1BH	/* DTC Transfer Vector Area27 (TRD0 Input capture / compare match B)  */

#pragma	ADDRESS		dtcvct28_addr	2C1CH	/* DTC Transfer Vector Area28 (TRD0 Input capture / compare match C)  */

#pragma	ADDRESS		dtcvct29_addr	2C1DH	/* DTC Transfer Vector Area29 (TRD0 Input capture / compare match D)  */

#pragma	ADDRESS		dtcvct30_addr	2C1EH	/* DTC Transfer Vector Area30 (TRD1 Input capture / compare match A)  */

#pragma	ADDRESS		dtcvct31_addr	2C1FH	/* DTC Transfer Vector Area31 (TRD1 Input capture / compare match B)  */

#pragma	ADDRESS		dtcvct32_addr	2C20H	/* DTC Transfer Vector Area32 (TRD1 Input capture / compare match C)  */

#pragma	ADDRESS		dtcvct33_addr	2C21H	/* DTC Transfer Vector Area33 (TRD1 Input capture / compare match D)  */

#pragma	ADDRESS		dtcvct34_addr	2C22H	/* DTC Transfer Vector Area34 */

#pragma	ADDRESS		dtcvct35_addr	2C23H	/* DTC Transfer Vector Area35 */

#pragma	ADDRESS		dtcvct36_addr	2C24H	/* DTC Transfer Vector Area36 */

#pragma	ADDRESS		dtcvct37_addr	2C25H	/* DTC Transfer Vector Area37 */

#pragma	ADDRESS		dtcvct38_addr	2C26H	/* DTC Transfer Vector Area38 */

#pragma	ADDRESS		dtcvct39_addr	2C27H	/* DTC Transfer Vector Area39 */

#pragma	ADDRESS		dtcvct40_addr	2C28H	/* DTC Transfer Vector Area40 */

#pragma	ADDRESS		dtcvct41_addr	2C29H	/* DTC Transfer Vector Area41 */

#pragma	ADDRESS		dtcvct42_addr	2C2AH	/* DTC Transfer Vector Area42 (TRE)  */

#pragma	ADDRESS		dtcvct43_addr	2C2BH	/* DTC Transfer Vector Area43 (TRF)  */

#pragma	ADDRESS		dtcvct44_addr	2C2CH	/* DTC Transfer Vector Area44 (TRF Compare match 0)  */

#pragma	ADDRESS		dtcvct45_addr	2C2DH	/* DTC Transfer Vector Area45 (TRF Compare match 1)  */

#pragma	ADDRESS		dtcvct46_addr	2C2EH	/* DTC Transfer Vector Area46 (TRF Input capture)  */

#pragma	ADDRESS		dtcvct47_addr	2C2FH	/* DTC Transfer Vector Area47 (TRG Input capture / compare match A)  */

#pragma	ADDRESS		dtcvct48_addr	2C30H	/* DTC Transfer Vector Area48 (TRG Input capture / compare match B)  */

#pragma	ADDRESS		dtcvct49_addr	2C31H	/* DTC Transfer Vector Area49 (TRA)  */

#pragma	ADDRESS		dtcvct50_addr	2C32H	/* DTC Transfer Vector Area50 */

#pragma	ADDRESS		dtcvct51_addr	2C33H	/* DTC Transfer Vector Area51 (TRB)  */

#pragma	ADDRESS		dtcvct52_addr	2C34H	/* DTC Transfer Vector Area52 (Flash memory ready)  */

#pragma	ADDRESS		dtcvct53_addr	2C35H	/* DTC Transfer Vector Area53 */

#pragma	ADDRESS		dtcvct54_addr	2C36H	/* DTC Transfer Vector Area54 */

#pragma	ADDRESS		dtcvct55_addr	2C37H	/* DTC Transfer Vector Area55 */

#pragma	ADDRESS		dtcvct56_addr	2C38H	/* DTC Transfer Vector Area56 */

#pragma	ADDRESS		dtcvct57_addr	2C39H	/* DTC Transfer Vector Area57 */

#pragma	ADDRESS		dtcvct58_addr	2C3AH	/* DTC Transfer Vector Area58 */

#pragma	ADDRESS		dtcvct59_addr	2C3BH	/* DTC Transfer Vector Area59 */

#pragma	ADDRESS		dtcvct60_addr	2C3CH	/* DTC Transfer Vector Area60 */

#pragma	ADDRESS		dtcvct61_addr	2C3DH	/* DTC Transfer Vector Area61 */

#pragma	ADDRESS		dtcvct62_addr	2C3EH	/* DTC Transfer Vector Area62 */

#pragma	ADDRESS		dtcvct63_addr	2C3FH	/* DTC Transfer Vector Area63 */

#pragma	ADDRESS		dtccr0_addr		2C40H	/* DTC Control Register 0 */

#pragma	ADDRESS		dtbls0_addr		2C41H	/* DTC Block Size Register 0 */

#pragma	ADDRESS		dtcct0_addr		2C42H	/* DTC Transfer Count Register 0 */

#pragma	ADDRESS		dtrld0_addr		2C43H	/* DTC Transfer Count Reload Register 0 */

#pragma	ADDRESS		dtsar0_addr		2C44H	/* DTC Source Address Register 0 */

#pragma	ADDRESS		dtdar0_addr		2C46H	/* DTC Destination Register 0 */

#pragma	ADDRESS		dtccr1_addr		2C48H	/* DTC Control Register 1 */

#pragma	ADDRESS		dtbls1_addr		2C49H	/* DTC Block Size Register 1 */

#pragma	ADDRESS		dtcct1_addr		2C4AH	/* DTC Transfer Count Register 1 */

#pragma	ADDRESS		dtrld1_addr		2C4BH	/* DTC Transfer Count Reload Register 1 */

#pragma	ADDRESS		dtsar1_addr		2C4CH	/* DTC Source Address Register 1 */

#pragma	ADDRESS		dtdar1_addr		2C4EH	/* DTC Destination Register 1 */

#pragma	ADDRESS		dtccr2_addr		2C50H	/* DTC Control Register 2 */

#pragma	ADDRESS		dtbls2_addr		2C51H	/* DTC Block Size Register 2 */

#pragma	ADDRESS		dtcct2_addr		2C52H	/* DTC Transfer Count Register 2 */

#pragma	ADDRESS		dtrld2_addr		2C53H	/* DTC Transfer Count Reload Register 2 */

#pragma	ADDRESS		dtsar2_addr		2C54H	/* DTC Source Address Register 2 */

#pragma	ADDRESS		dtdar2_addr		2C56H	/* DTC Destination Register 2 */

#pragma	ADDRESS		dtccr3_addr		2C58H	/* DTC Control Register 3 */

#pragma	ADDRESS		dtbls3_addr		2C59H	/* DTC Block Size Register 3 */

#pragma	ADDRESS		dtcct3_addr		2C5AH	/* DTC Transfer Count Register 3 */

#pragma	ADDRESS		dtrld3_addr		2C5BH	/* DTC Transfer Count Reload Register 3 */

#pragma	ADDRESS		dtsar3_addr		2C5CH	/* DTC Source Address Register 3 */

#pragma	ADDRESS		dtdar3_addr		2C5EH	/* DTC Destination Register 3 */

#pragma	ADDRESS		dtccr4_addr		2C60H	/* DTC Control Register 4 */

#pragma	ADDRESS		dtbls4_addr		2C61H	/* DTC Block Size Register 4 */

#pragma	ADDRESS		dtcct4_addr		2C62H	/* DTC Transfer Count Register 4 */

#pragma	ADDRESS		dtrld4_addr		2C63H	/* DTC Transfer Count Reload Register 4 */

#pragma	ADDRESS		dtsar4_addr		2C64H	/* DTC Source Address Register 4 */

#pragma	ADDRESS		dtdar4_addr		2C66H	/* DTC Destination Register 4 */

#pragma	ADDRESS		dtccr5_addr		2C68H	/* DTC Control Register 5 */

#pragma	ADDRESS		dtbls5_addr		2C69H	/* DTC Block Size Register 5 */

#pragma	ADDRESS		dtcct5_addr		2C6AH	/* DTC Transfer Count Register 5 */

#pragma	ADDRESS		dtrld5_addr		2C6BH	/* DTC Transfer Count Reload Register 5 */

#pragma	ADDRESS		dtsar5_addr		2C6CH	/* DTC Source Address Register 5 */

#pragma	ADDRESS		dtdar5_addr		2C6EH	/* DTC Destination Register 5 */

#pragma	ADDRESS		dtccr6_addr		2C70H	/* DTC Control Register 6 */

#pragma	ADDRESS		dtbls6_addr		2C71H	/* DTC Block Size Register 6 */

#pragma	ADDRESS		dtcct6_addr		2C72H	/* DTC Transfer Count Register 6 */

#pragma	ADDRESS		dtrld6_addr		2C73H	/* DTC Transfer Count Reload Register 6 */

#pragma	ADDRESS		dtsar6_addr		2C74H	/* DTC Source Address Register 6 */

#pragma	ADDRESS		dtdar6_addr		2C76H	/* DTC Destination Register 6 */

#pragma	ADDRESS		dtccr7_addr		2C78H	/* DTC Control Register 7 */

#pragma	ADDRESS		dtbls7_addr		2C79H	/* DTC Block Size Register 7 */

#pragma	ADDRESS		dtcct7_addr		2C7AH	/* DTC Transfer Count Register 7 */

#pragma	ADDRESS		dtrld7_addr		2C7BH	/* DTC Transfer Count Reload Register 7 */

#pragma	ADDRESS		dtsar7_addr		2C7CH	/* DTC Source Address Register 7 */

#pragma	ADDRESS		dtdar7_addr		2C7EH	/* DTC Destination Register 7 */

#pragma	ADDRESS		dtccr8_addr		2C80H	/* DTC Control Register 8 */

#pragma	ADDRESS		dtbls8_addr		2C81H	/* DTC Block Size Register 8 */

#pragma	ADDRESS		dtcct8_addr		2C82H	/* DTC Transfer Count Register 8 */

#pragma	ADDRESS		dtrld8_addr		2C83H	/* DTC Transfer Count Reload Register 8 */

#pragma	ADDRESS		dtsar8_addr		2C84H	/* DTC Source Address Register 8 */

#pragma	ADDRESS		dtdar8_addr		2C86H	/* DTC Destination Register 8 */

#pragma	ADDRESS		dtccr9_addr		2C88H	/* DTC Control Register 9 */

#pragma	ADDRESS		dtbls9_addr		2C89H	/* DTC Block Size Register 9 */

#pragma	ADDRESS		dtcct9_addr		2C8AH	/* DTC Transfer Count Register 9 */

#pragma	ADDRESS		dtrld9_addr		2C8BH	/* DTC Transfer Count Reload Register 9 */

#pragma	ADDRESS		dtsar9_addr		2C8CH	/* DTC Source Address Register 9 */

#pragma	ADDRESS		dtdar9_addr		2C8EH	/* DTC Destination Register 9 */

#pragma	ADDRESS		dtccr10_addr	2C90H	/* DTC Control Register 10 */

#pragma	ADDRESS		dtbls10_addr	2C91H	/* DTC Block Size Register 10 */

#pragma	ADDRESS		dtcct10_addr	2C92H	/* DTC Transfer Count Register 10 */

#pragma	ADDRESS		dtrld10_addr	2C93H	/* DTC Transfer Count Reload Register 10 */

#pragma	ADDRESS		dtsar10_addr	2C94H	/* DTC Source Address Register 10 */

#pragma	ADDRESS		dtdar10_addr	2C96H	/* DTC Destination Register 10 */

#pragma	ADDRESS		dtccr11_addr	2C98H	/* DTC Control Register 11 */

#pragma	ADDRESS		dtbls11_addr	2C99H	/* DTC Block Size Register 11 */

#pragma	ADDRESS		dtcct11_addr	2C9AH	/* DTC Transfer Count Register 11 */

#pragma	ADDRESS		dtrld11_addr	2C9BH	/* DTC Transfer Count Reload Register 11 */

#pragma	ADDRESS		dtsar11_addr	2C9CH	/* DTC Source Address Register 11 */

#pragma	ADDRESS		dtdar11_addr	2C9EH	/* DTC Destination Register 11 */

#pragma	ADDRESS		dtccr12_addr	2CA0H	/* DTC Control Register 12 */

#pragma	ADDRESS		dtbls12_addr	2CA1H	/* DTC Block Size Register 12 */

#pragma	ADDRESS		dtcct12_addr	2CA2H	/* DTC Transfer Count Register 12 */

#pragma	ADDRESS		dtrld12_addr	2CA3H	/* DTC Transfer Count Reload Register 12 */

#pragma	ADDRESS		dtsar12_addr	2CA4H	/* DTC Source Address Register 12 */

#pragma	ADDRESS		dtdar12_addr	2CA6H	/* DTC Destination Register 12 */

#pragma	ADDRESS		dtccr13_addr	2CA8H	/* DTC Control Register 13 */

#pragma	ADDRESS		dtbls13_addr	2CA9H	/* DTC Block Size Register 13 */

#pragma	ADDRESS		dtcct13_addr	2CAAH	/* DTC Transfer Count Register 13 */

#pragma	ADDRESS		dtrld13_addr	2CABH	/* DTC Transfer Count Reload Register 13 */

#pragma	ADDRESS		dtsar13_addr	2CACH	/* DTC Source Address Register 13 */

#pragma	ADDRESS		dtdar13_addr	2CAEH	/* DTC Destination Register 13 */

#pragma	ADDRESS		dtccr14_addr	2CB0H	/* DTC Control Register 14 */

#pragma	ADDRESS		dtbls14_addr	2CB1H	/* DTC Block Size Register 14 */

#pragma	ADDRESS		dtcct14_addr	2CB2H	/* DTC Transfer Count Register 14 */

#pragma	ADDRESS		dtrld14_addr	2CB3H	/* DTC Transfer Count Reload Register 14 */

#pragma	ADDRESS		dtsar14_addr	2CB4H	/* DTC Source Address Register 14 */

#pragma	ADDRESS		dtdar14_addr	2CB6H	/* DTC Destination Register 14 */

#pragma	ADDRESS		dtccr15_addr	2CB8H	/* DTC Control Register 15 */

#pragma	ADDRESS		dtbls15_addr	2CB9H	/* DTC Block Size Register 15 */

#pragma	ADDRESS		dtcct15_addr	2CBAH	/* DTC Transfer Count Register 15 */

#pragma	ADDRESS		dtrld15_addr	2CBBH	/* DTC Transfer Count Reload Register 15 */

#pragma	ADDRESS		dtsar15_addr	2CBCH	/* DTC Source Address Register 15 */

#pragma	ADDRESS		dtdar15_addr	2CBEH	/* DTC Destination Register 15 */

#pragma	ADDRESS		dtccr16_addr	2CC0H	/* DTC Control Register 16 */

#pragma	ADDRESS		dtbls16_addr	2CC1H	/* DTC Block Size Register 16 */

#pragma	ADDRESS		dtcct16_addr	2CC2H	/* DTC Transfer Count Register 16 */

#pragma	ADDRESS		dtrld16_addr	2CC3H	/* DTC Transfer Count Reload Register 16 */

#pragma	ADDRESS		dtsar16_addr	2CC4H	/* DTC Source Address Register 16 */

#pragma	ADDRESS		dtdar16_addr	2CC6H	/* DTC Destination Register 16 */

#pragma	ADDRESS		dtccr17_addr	2CC8H	/* DTC Control Register 17 */

#pragma	ADDRESS		dtbls17_addr	2CC9H	/* DTC Block Size Register 17 */

#pragma	ADDRESS		dtcct17_addr	2CCAH	/* DTC Transfer Count Register 17 */

#pragma	ADDRESS		dtrld17_addr	2CCBH	/* DTC Transfer Count Reload Register 17 */

#pragma	ADDRESS		dtsar17_addr	2CCCH	/* DTC Source Address Register 17 */

#pragma	ADDRESS		dtdar17_addr	2CCEH	/* DTC Destination Register 17 */

#pragma	ADDRESS		dtccr18_addr	2CD0H	/* DTC Control Register 18 */

#pragma	ADDRESS		dtbls18_addr	2CD1H	/* DTC Block Size Register 18 */

#pragma	ADDRESS		dtcct18_addr	2CD2H	/* DTC Transfer Count Register 18 */

#pragma	ADDRESS		dtrld18_addr	2CD3H	/* DTC Transfer Count Reload Register 18 */

#pragma	ADDRESS		dtsar18_addr	2CD4H	/* DTC Source Address Register 18 */

#pragma	ADDRESS		dtdar18_addr	2CD6H	/* DTC Destination Register 18 */

#pragma	ADDRESS		dtccr19_addr	2CD8H	/* DTC Control Register 19 */

#pragma	ADDRESS		dtbls19_addr	2CD9H	/* DTC Block Size Register 19 */

#pragma	ADDRESS		dtcct19_addr	2CDAH	/* DTC Transfer Count Register 19 */

#pragma	ADDRESS		dtrld19_addr	2CDBH	/* DTC Transfer Count Reload Register 19 */

#pragma	ADDRESS		dtsar19_addr	2CDCH	/* DTC Source Address Register 19 */

#pragma	ADDRESS		dtdar19_addr	2CDEH	/* DTC Destination Register 19 */

#pragma	ADDRESS		dtccr20_addr	2CE0H	/* DTC Control Register 20 */

#pragma	ADDRESS		dtbls20_addr	2CE1H	/* DTC Block Size Register 20 */

#pragma	ADDRESS		dtcct20_addr	2CE2H	/* DTC Transfer Count Register 20 */

#pragma	ADDRESS		dtrld20_addr	2CE3H	/* DTC Transfer Count Reload Register 20 */

#pragma	ADDRESS		dtsar20_addr	2CE4H	/* DTC Source Address Register 20 */

#pragma	ADDRESS		dtdar20_addr	2CE6H	/* DTC Destination Register 20 */

#pragma	ADDRESS		dtccr21_addr	2CE8H	/* DTC Control Register 21 */

#pragma	ADDRESS		dtbls21_addr	2CE9H	/* DTC Block Size Register 21 */

#pragma	ADDRESS		dtcct21_addr	2CEAH	/* DTC Transfer Count Register 21 */

#pragma	ADDRESS		dtrld21_addr	2CEBH	/* DTC Transfer Count Reload Register 21 */

#pragma	ADDRESS		dtsar21_addr	2CECH	/* DTC Source Address Register 21 */

#pragma	ADDRESS		dtdar21_addr	2CEEH	/* DTC Destination Register 21 */

#pragma	ADDRESS		dtccr22_addr	2CF0H	/* DTC Control Register 22 */

#pragma	ADDRESS		dtbls22_addr	2CF1H	/* DTC Block Size Register 22 */

#pragma	ADDRESS		dtcct22_addr	2CF2H	/* DTC Transfer Count Register 22 */

#pragma	ADDRESS		dtrld22_addr	2CF3H	/* DTC Transfer Count Reload Register 22 */

#pragma	ADDRESS		dtsar22_addr	2CF4H	/* DTC Source Address Register 22 */

#pragma	ADDRESS		dtdar22_addr	2CF6H	/* DTC Destination Register 22 */

#pragma	ADDRESS		dtccr23_addr	2CF8H	/* DTC Control Register 23 */

#pragma	ADDRESS		dtbls23_addr	2CF9H	/* DTC Block Size Register 23 */

#pragma	ADDRESS		dtcct23_addr	2CFAH	/* DTC Transfer Count Register 23 */

#pragma	ADDRESS		dtrld23_addr	2CFBH	/* DTC Transfer Count Reload Register 23 */

#pragma	ADDRESS		dtsar23_addr	2CFCH	/* DTC Source Address Register 23 */

#pragma	ADDRESS		dtdar23_addr	2CFEH	/* DTC Destination Register 23 */


/********************************************************
*  declare SFR bit                                      *
********************************************************/
struct	bit_def {
	char	b0:1;
	char	b1:1;
	char	b2:1;
	char	b3:1;
	char	b4:1;
	char	b5:1;
	char	b6:1;
	char	b7:1;
};
union	byte_def{
	struct	bit_def bit;
	char	byte;
};

/*------------------------------------------------------
  Processor Mode Register 0
------------------------------------------------------*/
union	byte_def	pm0_addr;
#define		pm0			pm0_addr.byte

#define		pm03		pm0_addr.bit.b3			/* Software reset bit */

/*------------------------------------------------------
  Processor Mode Register 1
------------------------------------------------------*/
union	byte_def	pm1_addr;
#define		pm1			pm1_addr.byte

#define		pm12		pm1_addr.bit.b2			/* WDT interrupt/reset switch bit */

/*------------------------------------------------------
  System Clock Control Register 0
------------------------------------------------------*/
union	byte_def	cm0_addr;
#define		cm0			cm0_addr.byte

#define		cm02		cm0_addr.bit.b2			/* Wait mode peripheral function clock stop bit */
#define		cm03		cm0_addr.bit.b3			/* XCIN clock stop bit */
#define		cm04		cm0_addr.bit.b4			/* Port/XCIN-XCOUT switch bit */
#define		cm05		cm0_addr.bit.b5			/* XIN clock (XIN-XOUT) stop bit */
#define		cm06		cm0_addr.bit.b6			/* CPU clock division select bit */
#define		cm07		cm0_addr.bit.b7			/* XIN, XCIN clock select bit */

/*------------------------------------------------------
  System Clock Control Register 1
------------------------------------------------------*/
union	byte_def	cm1_addr;
#define		cm1			cm1_addr.byte

#define		cm10		cm1_addr.bit.b0			/* All clock stop control bit */
#define		cm11		cm1_addr.bit.b1			/* XIN-XOUT on-chip feedback resistor select bit */
#define		cm12		cm1_addr.bit.b2			/* XCIN-XCOUT on-chip feedback resistor select bit */
#define		cm13		cm1_addr.bit.b3			/* Port/XCIN-XCOUT switch bit */
#define		cm14		cm1_addr.bit.b4			/* Low-speed on-chip oscillator stop bit */
#define		cm16		cm1_addr.bit.b6			/* CPU clock division select bit 1 */
#define		cm17		cm1_addr.bit.b7			/* CPU clock division select bit 1 */

/*------------------------------------------------------
  Module Standby Control Register
------------------------------------------------------*/
union	byte_def	mstcr_addr;
#define		mstcr		mstcr_addr.byte

#define		mstiic		mstcr_addr.bit.b3		/* SSU, I2C bus standby bit */
#define		msttrd		mstcr_addr.bit.b4		/* Timer RD standby bit */
#define		msttrc		mstcr_addr.bit.b5		/* Timer RC standby bit */
#define		msttrg		mstcr_addr.bit.b6		/* Timer RG standby bit */

/*------------------------------------------------------
  System Clock Control Register 3
------------------------------------------------------*/
union	byte_def	cm3_addr;
#define		cm3			cm3_addr.byte

#define		cm30		cm3_addr.bit.b0			/* Wait control bit */
#define		cm35		cm3_addr.bit.b5			/* CPU clock division when exiting wait mode select bit */
#define		cm36		cm3_addr.bit.b6			/* System clock when exiting wait mode or stop mode select bit */
#define		cm37		cm3_addr.bit.b7			/* System clock when exiting wait mode or stop mode select bit */

/*------------------------------------------------------
  Protect Register
------------------------------------------------------*/
union	byte_def	prcr_addr;
#define		prcr		prcr_addr.byte

#define		prc0		prcr_addr.bit.b0		/* Protect bit 0 */
#define		prc1		prcr_addr.bit.b1		/* Protect bit 1 */
#define		prc2		prcr_addr.bit.b2		/* Protect bit 2 */
#define		prc3		prcr_addr.bit.b3		/* Protect bit 3 */

/*------------------------------------------------------
  Reset Source Determination Register
------------------------------------------------------*/
union	byte_def	rstfr_addr;
#define		rstfr		rstfr_addr.byte

#define		cwr			rstfr_addr.bit.b0		/* Cold start-up/warm start-up determine flag */
#define		hwr			rstfr_addr.bit.b1		/* Hardware reset detect flag */
#define		swr			rstfr_addr.bit.b2		/* Software reset detect flag */
#define		wdr			rstfr_addr.bit.b3		/* Watchdog timer reset detect flag */

/*------------------------------------------------------
  Oscillation Stop Detection Register
------------------------------------------------------*/
union	byte_def	ocd_addr;
#define		ocd			ocd_addr.byte

#define		ocd0		ocd_addr.bit.b0			/* Oscillation stop detection enable bit */
#define		ocd1		ocd_addr.bit.b1			/* Oscillation stop detection interrupt enable bit */
#define		ocd2		ocd_addr.bit.b2			/* System clock select bit */
#define		ocd3		ocd_addr.bit.b3			/* Clock monitor bit */

/*------------------------------------------------------
  Watchdog Timer Reset Register
------------------------------------------------------*/
union	byte_def	wdtr_addr;
#define		wdtr		wdtr_addr.byte

/*------------------------------------------------------
  Watchdog Timer Start Register
------------------------------------------------------*/
union	byte_def	wdts_addr;
#define		wdts		wdts_addr.byte

/*------------------------------------------------------
  Watchdog Timer Control Register
------------------------------------------------------*/
union	byte_def	wdtc_addr;
#define		wdtc		wdtc_addr.byte

#define		wdtc7		wdtc_addr.bit.b7		/* Prescaler select bit */

/*------------------------------------------------------
  Count Source Protection Mode Register
------------------------------------------------------*/
union	byte_def	cspr_addr;
#define		cspr		cspr_addr.byte

#define		cspro		cspr_addr.bit.b7		/* Count source protection mode select bit */

/*------------------------------------------------------
  On-Chip Reference Voltage Control Register
------------------------------------------------------*/
union	byte_def	ocvrefcr_addr;
#define		ocvrefcr	ocvrefcr_addr.byte

#define		ocvrefan	ocvrefcr_addr.bit.b0	/* On-chip reference voltage to analog input connect bit */
/*------------------------------------------------------
  Clock Prescaler Reset Flag
------------------------------------------------------*/
union	byte_def	cpsrf_addr;
#define		cpsrf		cpsrf_addr.byte

#define		cpsr		cpsrf_addr.bit.b7		/* Clock prescaler reset flag */

/*------------------------------------------------------
  Voltage Monitor Circuit/Comparator A Control Register
------------------------------------------------------*/
union	byte_def	cmpa_addr;
#define		cmpa		cmpa_addr.byte

#define		cm1por		cmpa_addr.bit.b0		/* LVCOUT1 output polarity select bit */
#define		cm2por		cmpa_addr.bit.b1		/* LVCOUT2 output polarity select bit */
#define		cm1oe		cmpa_addr.bit.b2		/* LVCOUT1 output enable bit */
#define		cm2oe		cmpa_addr.bit.b3		/* LVCOUT2 output enable bit */
#define		irq1sel		cmpa_addr.bit.b4		/* Voltage monitor 1/comparator A1 interrupt type select bit */
#define		irq2sel		cmpa_addr.bit.b5		/* Voltage monitor 2/comparator A2 interrupt type select bit */
#define		compsel		cmpa_addr.bit.b7		/* Voltage monitor/comparator A interrupt type selection enable bit */

/*------------------------------------------------------
  Voltage Monitor Circuit Edge Select Register
------------------------------------------------------*/
union	byte_def	vcac_addr;
#define		vcac		vcac_addr.byte

#define		vcac1		vcac_addr.bit.b1		/* Voltage monitor 1 circuit edge select bit / Comparator A1 circuit edge select bit */
#define		vcac2		vcac_addr.bit.b2		/* Voltage monitor 2 circuit edge select bit / Comparator A2 circuit edge select bit */
/*------------------------------------------------------
  Voltage Detect Register 1
------------------------------------------------------*/
union	byte_def	vca1_addr;
#define		vca1		vca1_addr.byte

#define		vca13		vca1_addr.bit.b3		/* Voltage detection 2 signal monitor flag / Comparator A2 signal monitor flag */

/*------------------------------------------------------
  Voltage Detect Register 2
------------------------------------------------------*/
union	byte_def	vca2_addr;
#define		vca2		vca2_addr.byte

#define		vca20		vca2_addr.bit.b0		/* Internal power low consumption enable bit */
#define		vca21		vca2_addr.bit.b1		/* Comparator A1 reference voltage input select bit */
#define		vca22		vca2_addr.bit.b2		/* LVCMP1 comparison voltage external input select bit */
#define		vca23		vca2_addr.bit.b3		/* Comparator A2 reference voltage input select bit */
#define		vca24		vca2_addr.bit.b4		/* LVCMP2 comparison voltage external input select bit */
#define		vca25		vca2_addr.bit.b5		/* Voltage detection 0 enable bit */
#define		vca26		vca2_addr.bit.b6		/* Voltage detection 1/comparator A1 enable bit */
#define		vca27		vca2_addr.bit.b7		/* Voltage detection 2/comparator A2 enable bit */

/*------------------------------------------------------
  Voltage Detection 1 Level Select Register
------------------------------------------------------*/
union	byte_def	vd1ls_addr;
#define		vd1ls		vd1ls_addr.byte

#define		vd1s0		vd1ls_addr.bit.b0		/* Voltage detection 1 level select bit */
#define		vd1s1		vd1ls_addr.bit.b1		/* Voltage detection 1 level select bit */
#define		vd1s2		vd1ls_addr.bit.b2		/* Voltage detection 1 level select bit */
#define		vd1s3		vd1ls_addr.bit.b3		/* Voltage detection 1 level select bit */
/*------------------------------------------------------
  Voltage Monitor 0 Circuit Control Register
------------------------------------------------------*/
union	byte_def	vw0c_addr;
#define		vw0c		vw0c_addr.byte

#define		vw0c0		vw0c_addr.bit.b0		/* Voltage monitor 0 reset enable bit */

/*------------------------------------------------------
  Voltage Monitor 1 Circuit Control Register
------------------------------------------------------*/
union	byte_def	vw1c_addr;
#define		vw1c		vw1c_addr.byte

#define		vw1c0		vw1c_addr.bit.b0		/* Voltage monitor 1 reset enable bit / Comparator A1 interrupt enable bit */
#define		vw1c1		vw1c_addr.bit.b1		/* Voltage monitor 0 digital filter disable mode select bit / Comparator A1 digital filter disable mode select bit */
#define		vw1c2		vw1c_addr.bit.b2		/* Voltage change detection flag / Comparator A1 interrupt flag */
#define		vw1c3		vw1c_addr.bit.b3		/* Voltage detection 1 signal monitor flag / Comparator A1 signal monitor flag */
#define		vw1f0		vw1c_addr.bit.b4		/* Sampling clock select bit */
#define		vw1f1		vw1c_addr.bit.b5		/* Sampling clock select bit */
#define		vw1c7		vw1c_addr.bit.b7		/* Voltage monitor 1 reset generation condition select bit / Comparator A1 interrupt generation condition select bit */

/*------------------------------------------------------
  Voltage Monitor 2 Circuit Control Register
------------------------------------------------------*/
union	byte_def	vw2c_addr;
#define		vw2c		vw2c_addr.byte

#define		vw2c0		vw2c_addr.bit.b0		/* Voltage monitor 2 interrupt enable bit / Comparator A2 interrupt enable bit */
#define		vw2c1		vw2c_addr.bit.b1		/* Voltage monitor 2 digital filter disable mode select bit / Comparator A2 digital filter disable mode select bit */
#define		vw2c2		vw2c_addr.bit.b2		/* Voltage change detection flag / Comparator A2 interrupt flag */
#define		vw2c3		vw2c_addr.bit.b3		/* WDT detection monitor flag */
#define		vw2f0		vw2c_addr.bit.b4		/* Sampling clock select bit */
#define		vw2f1		vw2c_addr.bit.b5		/* Sampling clock select bit */
#define		vw2c7		vw2c_addr.bit.b7		/* Voltage monitor 2 interrupt generation condition select bit / Comparator A2 interrupt generation condition select bit */

/*------------------------------------------------------
  DTC Activation Control Register
------------------------------------------------------*/
union	byte_def	dtctl_addr;
#define		dtctl		dtctl_addr.byte

#define		nmif			dtctl_addr.bit.b1	/* Non-maskable interrupt generation bit */
/*------------------------------------------------------
  DTC Activation Enable Register 0
------------------------------------------------------*/
union	byte_def	dtcen0_addr;
#define		dtcen0		dtcen0_addr.byte

#define		dtcen00			dtcen0_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen01			dtcen0_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen02			dtcen0_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen03			dtcen0_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen04			dtcen0_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen05			dtcen0_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen06			dtcen0_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen07			dtcen0_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 1
------------------------------------------------------*/
union	byte_def	dtcen1_addr;
#define		dtcen1		dtcen1_addr.byte

#define		dtcen10			dtcen1_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen11			dtcen1_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen12			dtcen1_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen13			dtcen1_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen14			dtcen1_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen15			dtcen1_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen16			dtcen1_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen17			dtcen1_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 2
------------------------------------------------------*/
union	byte_def	dtcen2_addr;
#define		dtcen2		dtcen2_addr.byte

#define		dtcen20			dtcen2_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen21			dtcen2_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen22			dtcen2_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen23			dtcen2_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen24			dtcen2_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen25			dtcen2_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen26			dtcen2_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen27			dtcen2_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 3
------------------------------------------------------*/
union	byte_def	dtcen3_addr;
#define		dtcen3		dtcen3_addr.byte

#define		dtcen30			dtcen3_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen31			dtcen3_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen32			dtcen3_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen33			dtcen3_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen34			dtcen3_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen35			dtcen3_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen36			dtcen3_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen37			dtcen3_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 4
------------------------------------------------------*/
union	byte_def	dtcen4_addr;
#define		dtcen4		dtcen4_addr.byte

#define		dtcen40			dtcen4_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen41			dtcen4_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen42			dtcen4_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen43			dtcen4_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen44			dtcen4_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen45			dtcen4_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen46			dtcen4_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen47			dtcen4_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 5
------------------------------------------------------*/
union	byte_def	dtcen5_addr;
#define		dtcen5		dtcen5_addr.byte

#define		dtcen50			dtcen5_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen51			dtcen5_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen52			dtcen5_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen53			dtcen5_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen54			dtcen5_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen55			dtcen5_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen56			dtcen5_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen57			dtcen5_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  DTC Activation Enable Register 6
------------------------------------------------------*/
union	byte_def	dtcen6_addr;
#define		dtcen6		dtcen6_addr.byte

#define		dtcen60			dtcen6_addr.bit.b0	/* DTC activation enable bit */
#define		dtcen61			dtcen6_addr.bit.b1	/* DTC activation enable bit */
#define		dtcen62			dtcen6_addr.bit.b2	/* DTC activation enable bit */
#define		dtcen63			dtcen6_addr.bit.b3	/* DTC activation enable bit */
#define		dtcen64			dtcen6_addr.bit.b4	/* DTC activation enable bit */
#define		dtcen65			dtcen6_addr.bit.b5	/* DTC activation enable bit */
#define		dtcen66			dtcen6_addr.bit.b6	/* DTC activation enable bit */
#define		dtcen67			dtcen6_addr.bit.b7	/* DTC activation enable bit */

/*------------------------------------------------------
  Timer RF Register
------------------------------------------------------*/
unsigned	short	trf_addr;
#define		trf			trf_addr				/* Timer RF Register */

/*------------------------------------------------------
  Timer RF Control Register 0
------------------------------------------------------*/
union	byte_def	trfcr0_addr;
#define		trfcr0		trfcr0_addr.byte

#define		tstart_trfcr0	trfcr0_addr.bit.b0	/* Timer RF count start bit */
#define		tck0_trfcr0		trfcr0_addr.bit.b1	/* Timer RF count source select bits */
#define		tck1_trfcr0		trfcr0_addr.bit.b2	/* Timer RF count source select bits */
#define		trfc03_trfcr0	trfcr0_addr.bit.b3	/* Capture polarity select bits */
#define		trfc04_trfcr0	trfcr0_addr.bit.b4	/* Capture polarity select bits */
#define		trfc05_trfcr0	trfcr0_addr.bit.b5	/* CMP output select bit 0 when count stops */
#define		trfc06_trfcr0	trfcr0_addr.bit.b6	/* CMP output select bit 1 when count stops */

#define		trfc00_trfcr0	tstart_trfcr0		/* Timer RF count start bit */
#define		trfc01_trfcr0	tck0_trfcr0			/* Timer RF count source selection bit */
#define		trfc02_trfcr0	tck1_trfcr0			/* Timer RF count source selection bit */

/*------------------------------------------------------
  Timer RF Control Register 1
------------------------------------------------------*/
union	byte_def	trfcr1_addr;
#define		trfcr1		trfcr1_addr.byte

#define		tipf0_trfcr1	trfcr1_addr.bit.b0	/* TRFI filter select bits */
#define		tipf1_trfcr1	trfcr1_addr.bit.b1	/* TRFI filter select bits */
#define		cclr_trfcr1		trfcr1_addr.bit.b2	/* TRF register count operation select bit */
#define		tmod_trfcr1		trfcr1_addr.bit.b3	/* Timer RF operation mode select bit */
#define		trfc14_trfcr1	trfcr1_addr.bit.b4	/* Compare 0 output select bits */
#define		trfc15_trfcr1	trfcr1_addr.bit.b5	/* Compare 0 output select bits */
#define		trfc16_trfcr1	trfcr1_addr.bit.b6	/* Compare 1 output select bits */
#define		trfc17_trfcr1	trfcr1_addr.bit.b7	/* Compare 1 output select bits */

#define		trfc10_trfcr1	tipf0_trfcr1		/* TRFI filter select bit */
#define		trfc11_trfcr1	tipf1_trfcr1		/* TRFI filter select bit */
#define		trfc12_trfcr1	cclr_trfcr1			/* TRF register count operation select bit */
#define		trfc13_trfcr1	tmod_trfcr1			/* Timer RF operation mode select bit */

/*------------------------------------------------------
  Capture and Compare 0 Register
------------------------------------------------------*/
unsigned	short	trfm0_addr;
#define		trfm0		trfm0_addr				/* Capture and Compare 0 Register */

/*------------------------------------------------------
  Compare 1 Register
------------------------------------------------------*/
unsigned	short	trfm1_addr;
#define		trfm1		trfm1_addr				/* Compare 1 Register */

/*------------------------------------------------------
  UART0 Bit Rate Register
------------------------------------------------------*/
union	byte_def	u0brg_addr;
#define		u0brg		u0brg_addr.byte

/*------------------------------------------------------
  UART2 Bit Rate Register
------------------------------------------------------*/
union	byte_def	u2brg_addr;
#define		u2brg		u2brg_addr.byte

/*------------------------------------------------------
  UART2 Digital Filter Function Select Register
------------------------------------------------------*/
union	byte_def	urxdf_addr;
#define		urxdf		urxdf_addr.byte

#define		df2en			urxdf_addr.bit.b2	/* RXD2 digital filter enable bit */

/*------------------------------------------------------
  UART2 Special Mode Register 5
------------------------------------------------------*/
union	byte_def	u2smr5_addr;
#define		u2smr5		u2smr5_addr.byte

#define		mp				u2smr5_addr.bit.b0	/* Multiprocessor communication enable bit */
#define		mpie			u2smr5_addr.bit.b4	/* Multiprocessor communication control bit */
/*------------------------------------------------------
  UART2 Special Mode Register 4
------------------------------------------------------*/
union	byte_def	u2smr4_addr;
#define		u2smr4		u2smr4_addr.byte

#define		stareq			u2smr4_addr.bit.b0	/* Start condition generate bit */
#define		rstareq			u2smr4_addr.bit.b1	/* Restart condition generate bit */
#define		stpreq			u2smr4_addr.bit.b2	/* Stop condition generate bit */
#define		stspsel			u2smr4_addr.bit.b3	/* SCL, SDA output select bit */
#define		ackd			u2smr4_addr.bit.b4	/* ACK data bit */
#define		ackc			u2smr4_addr.bit.b5	/* ACK data output enable bit */
#define		sclhi			u2smr4_addr.bit.b6	/* SCL output stop enable bit */
#define		swc9			u2smr4_addr.bit.b7	/* SCL wait bit 3 */

/*------------------------------------------------------
  UART2 Special Mode Register 3
------------------------------------------------------*/
union	byte_def	u2smr3_addr;
#define		u2smr3		u2smr3_addr.byte

#define		ckph			u2smr3_addr.bit.b1	/* Clock phase set bit */
#define		nodc			u2smr3_addr.bit.b3	/* Clock output select bit */
#define		dl0				u2smr3_addr.bit.b5	/* SDA2 digital delay setup bit */
#define		dl1				u2smr3_addr.bit.b6	/* SDA2 digital delay setup bit */
#define		dl2				u2smr3_addr.bit.b7	/* SDA2 digital delay setup bit */

/*------------------------------------------------------
  UART2 Special Mode Register 2
------------------------------------------------------*/
union	byte_def	u2smr2_addr;
#define		u2smr2		u2smr2_addr.byte

#define		iicm2			u2smr2_addr.bit.b0	/* I2C mode select bit 2 */
#define		csc				u2smr2_addr.bit.b1	/* Clock synchronization bit */
#define		swc				u2smr2_addr.bit.b2	/* SCL wait output bit */
#define		stac			u2smr2_addr.bit.b4	/* UART2 initialization bit */
#define		swc2			u2smr2_addr.bit.b5	/* SCL wait output bit 2 */
#define		sdhi			u2smr2_addr.bit.b6	/* SDA output disable bit */

/*------------------------------------------------------
  UART2 Special Mode Register
------------------------------------------------------*/
union	byte_def	u2smr_addr;
#define		u2smr		u2smr_addr.byte

#define		iicm			u2smr_addr.bit.b0	/* I2C mode select bit */
#define		bbs				u2smr_addr.bit.b2	/* Bus busy flag */

/*------------------------------------------------------
  A/D Mode Register
------------------------------------------------------*/
union	byte_def	admod_addr;
#define		admod		admod_addr.byte

#define		cks0			admod_addr.bit.b0	/* Division select bit */
#define		cks1			admod_addr.bit.b1	/* Division select bit */
#define		cks2			admod_addr.bit.b2	/* Clock source select bit */
#define		md0				admod_addr.bit.b3	/* A/D operating mode select bit */
#define		md1				admod_addr.bit.b4	/* A/D operating mode select bit */
#define		md2				admod_addr.bit.b5	/* A/D operating mode select bit */
#define		adcap0			admod_addr.bit.b6	/* A/D conversion trigger select bit */
#define		adcap1			admod_addr.bit.b7	/* A/D conversion trigger select bit */

/*------------------------------------------------------
  A/D Input Select Register
------------------------------------------------------*/
union	byte_def	adinsel_addr;
#define		adinsel		adinsel_addr.byte

#define		ch0				adinsel_addr.bit.b0	/* Analog input pin select bit */
#define		ch1				adinsel_addr.bit.b1	/* Analog input pin select bit */
#define		ch2				adinsel_addr.bit.b2	/* Analog input pin select bit */
#define		scan0			adinsel_addr.bit.b4	/* A/D sweep pin count select bit */
#define		scan1			adinsel_addr.bit.b5	/* A/D sweep pin count select bit */
#define		adgsel0			adinsel_addr.bit.b6	/* A/D input group select bit */
#define		adgsel1			adinsel_addr.bit.b7	/* A/D input group select bit */

/*------------------------------------------------------
  A/D Control Register 0
------------------------------------------------------*/
union	byte_def	adcon0_addr;
#define		adcon0		adcon0_addr.byte

#define		adst			adcon0_addr.bit.b0	/* A/D conversion start flag */

/*------------------------------------------------------
  A/D Control Register 1
------------------------------------------------------*/
union	byte_def	adcon1_addr;
#define		adcon1		adcon1_addr.byte

#define		adex0			adcon1_addr.bit.b0	/* Extended analog input pin select bit */
#define		bits			adcon1_addr.bit.b4	/* 8/10-bit mode select bit */
#define		adstby			adcon1_addr.bit.b5	/* A/D standby bit */
#define		adddaen			adcon1_addr.bit.b6	/* A/D open-circuit detection assist function enable bit */
#define		adddael			adcon1_addr.bit.b7	/* A/D open-circuit detection assist method select bit */

/*------------------------------------------------------
  D/A0 Register
------------------------------------------------------*/
union	byte_def	da0_addr;
#define		da0			da0_addr.byte

/*------------------------------------------------------
  D/A1 Register
------------------------------------------------------*/
union	byte_def	da1_addr;
#define		da1			da1_addr.byte

/*------------------------------------------------------
  D/A Control Register
------------------------------------------------------*/
union	byte_def	dacon_addr;
#define		dacon		dacon_addr.byte

#define		da0e_dacon		dacon_addr.bit.b0	/* D/A0 output enable bit */
#define		da1e_dacon		dacon_addr.bit.b1	/* D/A1 output enable bit */

/*------------------------------------------------------
  Port P0 Register
------------------------------------------------------*/
union	byte_def	p0_addr;
#define		p0			p0_addr.byte

#define		p0_0			p0_addr.bit.b0		/* Port P0_0 bit */
#define		p0_1			p0_addr.bit.b1		/* Port P0_1 bit */
#define		p0_2			p0_addr.bit.b2		/* Port P0_2 bit */
#define		p0_3			p0_addr.bit.b3		/* Port P0_3 bit */
#define		p0_4			p0_addr.bit.b4		/* Port P0_4 bit */
#define		p0_5			p0_addr.bit.b5		/* Port P0_5 bit */
#define		p0_6			p0_addr.bit.b6		/* Port P0_6 bit */
#define		p0_7			p0_addr.bit.b7		/* Port P0_7 bit */

/*------------------------------------------------------
  Port P1 Register
------------------------------------------------------*/
union	byte_def	p1_addr;
#define		p1			p1_addr.byte

#define		p1_0			p1_addr.bit.b0		/* Port P1_0 bit */
#define		p1_1			p1_addr.bit.b1		/* Port P1_1 bit */
#define		p1_2			p1_addr.bit.b2		/* Port P1_2 bit */
#define		p1_3			p1_addr.bit.b3		/* Port P1_3 bit */
#define		p1_4			p1_addr.bit.b4		/* Port P1_4 bit */
#define		p1_5			p1_addr.bit.b5		/* Port P1_5 bit */
#define		p1_6			p1_addr.bit.b6		/* Port P1_6 bit */
#define		p1_7			p1_addr.bit.b7		/* Port P1_7 bit */

/*------------------------------------------------------
  Port P0 Direction Register
------------------------------------------------------*/
union	byte_def	pd0_addr;
#define		pd0			pd0_addr.byte

#define		pd0_0			pd0_addr.bit.b0		/* Port P0_0 direction bit */
#define		pd0_1			pd0_addr.bit.b1		/* Port P0_1 direction bit */
#define		pd0_2			pd0_addr.bit.b2		/* Port P0_2 direction bit */
#define		pd0_3			pd0_addr.bit.b3		/* Port P0_3 direction bit */
#define		pd0_4			pd0_addr.bit.b4		/* Port P0_4 direction bit */
#define		pd0_5			pd0_addr.bit.b5		/* Port P0_5 direction bit */
#define		pd0_6			pd0_addr.bit.b6		/* Port P0_6 direction bit */
#define		pd0_7			pd0_addr.bit.b7		/* Port P0_7 direction bit */

/*------------------------------------------------------
  Port P1 Direction Register
------------------------------------------------------*/
union	byte_def	pd1_addr;
#define		pd1			pd1_addr.byte

#define		pd1_0			pd1_addr.bit.b0		/* Port P1_0 direction bit */
#define		pd1_1			pd1_addr.bit.b1		/* Port P1_1 direction bit */
#define		pd1_2			pd1_addr.bit.b2		/* Port P1_2 direction bit */
#define		pd1_3			pd1_addr.bit.b3		/* Port P1_3 direction bit */
#define		pd1_4			pd1_addr.bit.b4		/* Port P1_4 direction bit */
#define		pd1_5			pd1_addr.bit.b5		/* Port P1_5 direction bit */
#define		pd1_6			pd1_addr.bit.b6		/* Port P1_6 direction bit */
#define		pd1_7			pd1_addr.bit.b7		/* Port P1_7 direction bit */

/*------------------------------------------------------
  Port P2 Register
------------------------------------------------------*/
union	byte_def	p2_addr;
#define		p2			p2_addr.byte

#define		p2_0			p2_addr.bit.b0		/* Port P2_0 bit */
#define		p2_1			p2_addr.bit.b1		/* Port P2_1 bit */
#define		p2_2			p2_addr.bit.b2		/* Port P2_2 bit */
#define		p2_3			p2_addr.bit.b3		/* Port P2_3 bit */
#define		p2_4			p2_addr.bit.b4		/* Port P2_4 bit */
#define		p2_5			p2_addr.bit.b5		/* Port P2_5 bit */
#define		p2_6			p2_addr.bit.b6		/* Port P2_6 bit */
#define		p2_7			p2_addr.bit.b7		/* Port P2_7 bit */

/*------------------------------------------------------
  Port P3 Register
------------------------------------------------------*/
union	byte_def	p3_addr;
#define		p3			p3_addr.byte

#define		p3_0			p3_addr.bit.b0		/* Port P3_0 bit */
#define		p3_1			p3_addr.bit.b1		/* Port P3_1 bit */
#define		p3_2			p3_addr.bit.b2		/* Port P3_2 bit */
#define		p3_3			p3_addr.bit.b3		/* Port P3_3 bit */
#define		p3_4			p3_addr.bit.b4		/* Port P3_4 bit */
#define		p3_5			p3_addr.bit.b5		/* Port P3_5 bit */
#define		p3_6			p3_addr.bit.b6		/* Port P3_6 bit */
#define		p3_7			p3_addr.bit.b7		/* Port P3_7 bit */

/*------------------------------------------------------
  Port P2 Direction Register
------------------------------------------------------*/
union	byte_def	pd2_addr;
#define		pd2			pd2_addr.byte

#define		pd2_0			pd2_addr.bit.b0		/* Port P2_0 direction bit */
#define		pd2_1			pd2_addr.bit.b1		/* Port P2_1 direction bit */
#define		pd2_2			pd2_addr.bit.b2		/* Port P2_2 direction bit */
#define		pd2_3			pd2_addr.bit.b3		/* Port P2_3 direction bit */
#define		pd2_4			pd2_addr.bit.b4		/* Port P2_4 direction bit */
#define		pd2_5			pd2_addr.bit.b5		/* Port P2_5 direction bit */
#define		pd2_6			pd2_addr.bit.b6		/* Port P2_6 direction bit */
#define		pd2_7			pd2_addr.bit.b7		/* Port P2_7 direction bit */

/*------------------------------------------------------
  Port P3 Direction Register
------------------------------------------------------*/
union	byte_def	pd3_addr;
#define		pd3			pd3_addr.byte

#define		pd3_0			pd3_addr.bit.b0		/* Port P3_0 direction bit */
#define		pd3_1			pd3_addr.bit.b1		/* Port P3_1 direction bit */
#define		pd3_2			pd3_addr.bit.b2		/* Port P3_2 direction bit */
#define		pd3_3			pd3_addr.bit.b3		/* Port P3_3 direction bit */
#define		pd3_4			pd3_addr.bit.b4		/* Port P3_4 direction bit */
#define		pd3_5			pd3_addr.bit.b5		/* Port P3_5 direction bit */
#define		pd3_6			pd3_addr.bit.b6		/* Port P3_6 direction bit */
#define		pd3_7			pd3_addr.bit.b7		/* Port P3_7 direction bit */

/*------------------------------------------------------
  Port P4 Register
------------------------------------------------------*/
union	byte_def	p4_addr;
#define		p4			p4_addr.byte

#define		p4_2			p4_addr.bit.b2		/* Port P4_2 bit */
#define		p4_3			p4_addr.bit.b3		/* Port P4_3 bit */
#define		p4_4			p4_addr.bit.b4		/* Port P4_4 bit */
#define		p4_5			p4_addr.bit.b5		/* Port P4_5 bit */
#define		p4_6			p4_addr.bit.b6		/* Port P4_6 bit */
#define		p4_7			p4_addr.bit.b7		/* Port P4_7 bit */

/*------------------------------------------------------
  Port P5 Register
------------------------------------------------------*/
union	byte_def	p5_addr;
#define		p5			p5_addr.byte

#define		p5_0			p5_addr.bit.b0		/* Port P5_0 bit */
#define		p5_1			p5_addr.bit.b1		/* Port P5_1 bit */
#define		p5_2			p5_addr.bit.b2		/* Port P5_2 bit */
#define		p5_3			p5_addr.bit.b3		/* Port P5_3 bit */
#define		p5_4			p5_addr.bit.b4		/* Port P5_4 bit */
#define		p5_5			p5_addr.bit.b5		/* Port P5_5 bit */
#define		p5_6			p5_addr.bit.b6		/* Port P5_6 bit */
#define		p5_7			p5_addr.bit.b7		/* Port P5_7 bit */

/*------------------------------------------------------
  Port P4 Direction Register
------------------------------------------------------*/
union	byte_def	pd4_addr;
#define		pd4			pd4_addr.byte

#define		pd4_3			pd4_addr.bit.b3		/* Port P4_3 direction bit */
#define		pd4_4			pd4_addr.bit.b4		/* Port P4_4 direction bit */
#define		pd4_5			pd4_addr.bit.b5		/* Port P4_5 direction bit */
#define		pd4_6			pd4_addr.bit.b6		/* Port P4_6 direction bit */
#define		pd4_7			pd4_addr.bit.b7		/* Port P4_7 direction bit */

/*------------------------------------------------------
  Port P5 Direction Register
------------------------------------------------------*/
union	byte_def	pd5_addr;
#define		pd5			pd5_addr.byte

#define		pd5_0			pd5_addr.bit.b0		/* Port P5_0 direction bit */
#define		pd5_1			pd5_addr.bit.b1		/* Port P5_1 direction bit */
#define		pd5_2			pd5_addr.bit.b2		/* Port P5_2 direction bit */
#define		pd5_3			pd5_addr.bit.b3		/* Port P5_3 direction bit */
#define		pd5_4			pd5_addr.bit.b4		/* Port P5_4 direction bit */
#define		pd5_5			pd5_addr.bit.b5		/* Port P5_5 direction bit */
#define		pd5_6			pd5_addr.bit.b6		/* Port P5_6 direction bit */
#define		pd5_7			pd5_addr.bit.b7		/* Port P5_7 direction bit */

/*------------------------------------------------------
  Port P6 Register
------------------------------------------------------*/
union	byte_def	p6_addr;
#define		p6			p6_addr.byte

#define		p6_0			p6_addr.bit.b0		/* Port P6_0 bit */
#define		p6_1			p6_addr.bit.b1		/* Port P6_1 bit */
#define		p6_2			p6_addr.bit.b2		/* Port P6_2 bit */
#define		p6_3			p6_addr.bit.b3		/* Port P6_3 bit */
#define		p6_4			p6_addr.bit.b4		/* Port P6_4 bit */
#define		p6_5			p6_addr.bit.b5		/* Port P6_5 bit */
#define		p6_6			p6_addr.bit.b6		/* Port P6_6 bit */
#define		p6_7			p6_addr.bit.b7		/* Port P6_7 bit */

/*------------------------------------------------------
  Port P7 Register
------------------------------------------------------*/
union	byte_def	p7_addr;
#define		p7			p7_addr.byte

#define		p7_0			p7_addr.bit.b0		/* Port P7_0 bit */
#define		p7_1			p7_addr.bit.b1		/* Port P7_1 bit */
#define		p7_2			p7_addr.bit.b2		/* Port P7_2 bit */
#define		p7_3			p7_addr.bit.b3		/* Port P7_3 bit */
#define		p7_4			p7_addr.bit.b4		/* Port P7_4 bit */
#define		p7_5			p7_addr.bit.b5		/* Port P7_5 bit */
#define		p7_6			p7_addr.bit.b6		/* Port P7_6 bit */
#define		p7_7			p7_addr.bit.b7		/* Port P7_7 bit */

/*------------------------------------------------------
  Port P6 Direction Register
------------------------------------------------------*/
union	byte_def	pd6_addr;
#define		pd6			pd6_addr.byte

#define		pd6_0			pd6_addr.bit.b0		/* Port P6_0 direction bit */
#define		pd6_1			pd6_addr.bit.b1		/* Port P6_1 direction bit */
#define		pd6_2			pd6_addr.bit.b2		/* Port P6_2 direction bit */
#define		pd6_3			pd6_addr.bit.b3		/* Port P6_3 direction bit */
#define		pd6_4			pd6_addr.bit.b4		/* Port P6_4 direction bit */
#define		pd6_5			pd6_addr.bit.b5		/* Port P6_5 direction bit */
#define		pd6_6			pd6_addr.bit.b6		/* Port P6_6 direction bit */
#define		pd6_7			pd6_addr.bit.b7		/* Port P6_7 direction bit */

/*------------------------------------------------------
  Port P7 Direction Register
------------------------------------------------------*/
union	byte_def	pd7_addr;
#define		pd7			pd7_addr.byte

#define		pd7_0			pd7_addr.bit.b0		/* Port P7_0 direction bit */
#define		pd7_1			pd7_addr.bit.b1		/* Port P7_1 direction bit */
#define		pd7_2			pd7_addr.bit.b2		/* Port P7_2 direction bit */
#define		pd7_3			pd7_addr.bit.b3		/* Port P7_3 direction bit */
#define		pd7_4			pd7_addr.bit.b4		/* Port P7_4 direction bit */
#define		pd7_5			pd7_addr.bit.b5		/* Port P7_5 direction bit */
#define		pd7_6			pd7_addr.bit.b6		/* Port P7_6 direction bit */
#define		pd7_7			pd7_addr.bit.b7		/* Port P7_7 direction bit */

/*------------------------------------------------------
  Port P8 Register
------------------------------------------------------*/
union	byte_def	p8_addr;
#define		p8			p8_addr.byte

#define		p8_0			p8_addr.bit.b0		/* Port P8_0 bit */
#define		p8_1			p8_addr.bit.b1		/* Port P8_1 bit */
#define		p8_2			p8_addr.bit.b2		/* Port P8_2 bit */
#define		p8_3			p8_addr.bit.b3		/* Port P8_3 bit */
#define		p8_4			p8_addr.bit.b4		/* Port P8_4 bit */
#define		p8_5			p8_addr.bit.b5		/* Port P8_5 bit */
#define		p8_6			p8_addr.bit.b6		/* Port P8_6 bit */
#define		p8_7			p8_addr.bit.b7		/* Port P8_7 bit */

/*------------------------------------------------------
  Port P9 Register
------------------------------------------------------*/
union	byte_def	p9_addr;
#define		p9			p9_addr.byte

#define		p9_0			p9_addr.bit.b0		/* Port P9_0 bit */
#define		p9_1			p9_addr.bit.b1		/* Port P9_1 bit */
#define		p9_2			p9_addr.bit.b2		/* Port P9_2 bit */
#define		p9_3			p9_addr.bit.b3		/* Port P9_3 bit */
#define		p9_4			p9_addr.bit.b4		/* Port P9_4 bit */
#define		p9_5			p9_addr.bit.b5		/* Port P9_5 bit */

/*------------------------------------------------------
  Port P8 Direction Register
------------------------------------------------------*/
union	byte_def	pd8_addr;
#define		pd8			pd8_addr.byte

#define		pd8_0			pd8_addr.bit.b0		/* Port P8_0 direction bit */
#define		pd8_1			pd8_addr.bit.b1		/* Port P8_1 direction bit */
#define		pd8_2			pd8_addr.bit.b2		/* Port P8_2 direction bit */
#define		pd8_3			pd8_addr.bit.b3		/* Port P8_3 direction bit */
#define		pd8_4			pd8_addr.bit.b4		/* Port P8_4 direction bit */
#define		pd8_5			pd8_addr.bit.b5		/* Port P8_5 direction bit */
#define		pd8_6			pd8_addr.bit.b6		/* Port P8_6 direction bit */
#define		pd8_7			pd8_addr.bit.b7		/* Port P8_7 direction bit */

/*------------------------------------------------------
  Port P9 Direction Register
------------------------------------------------------*/
union	byte_def	pd9_addr;
#define		pd9			pd9_addr.byte

#define		pd9_0			pd9_addr.bit.b0		/* Port P9_0 direction bit */
#define		pd9_1			pd9_addr.bit.b1		/* Port P9_1 direction bit */
#define		pd9_2			pd9_addr.bit.b2		/* Port P9_2 direction bit */
#define		pd9_3			pd9_addr.bit.b3		/* Port P9_3 direction bit */
#define		pd9_4			pd9_addr.bit.b4		/* Port P9_4 direction bit */
#define		pd9_5			pd9_addr.bit.b5		/* Port P9_5 direction bit */

/*------------------------------------------------------
  Timer RA Control Register
------------------------------------------------------*/
union	byte_def	tracr_addr;
#define		tracr		tracr_addr.byte

#define		tstart_tracr	tracr_addr.bit.b0	/* Timer RA count start bit */
#define		tcstf_tracr		tracr_addr.bit.b1	/* Timer RA count status flag */
#define		tstop_tracr		tracr_addr.bit.b2	/* Timer RA count forcible stop bit */
#define		tedgf_tracr		tracr_addr.bit.b4	/* Active edge judgment flag */
#define		tundf_tracr		tracr_addr.bit.b5	/* Timer RA underflow flag */

/*------------------------------------------------------
  Timer RA I/O Control Register
------------------------------------------------------*/
union	byte_def	traioc_addr;
#define		traioc		traioc_addr.byte

#define		tedgsel_traioc	traioc_addr.bit.b0	/* TRAIO polarity switch bit */
#define		topcr_traioc	traioc_addr.bit.b1	/* TRAIO output control bit */
#define		toena_traioc	traioc_addr.bit.b2	/* TRAO output enable bit */
#define		tiosel_traioc	traioc_addr.bit.b3	/* Hardware LIN function select bit */
#define		tipf0_traioc	traioc_addr.bit.b4	/* TRAIO input filter select bit */
#define		tipf1_traioc	traioc_addr.bit.b5	/* TRAIO input filter select bit */
#define		tiogt0_traioc	traioc_addr.bit.b6	/* TRAIO event input control bit */
#define		tiogt1_traioc	traioc_addr.bit.b7	/* TRAIO event input control bit */

/*------------------------------------------------------
  Timer RA Mode Register
------------------------------------------------------*/
union	byte_def	tramr_addr;
#define		tramr		tramr_addr.byte

#define		tmod0_tramr		tramr_addr.bit.b0	/* Timer RA operating mode select bit */
#define		tmod1_tramr		tramr_addr.bit.b1	/* Timer RA operating mode select bit */
#define		tmod2_tramr		tramr_addr.bit.b2	/* Timer RA operating mode select bit */
#define		tck0_tramr		tramr_addr.bit.b4	/* Timer RA count source select bit */
#define		tck1_tramr		tramr_addr.bit.b5	/* Timer RA count source select bit */
#define		tck2_tramr		tramr_addr.bit.b6	/* Timer RA count source select bit */
#define		tckcut_tramr	tramr_addr.bit.b7	/* Timer RA count source cutoff bit */

/*------------------------------------------------------
  Timer RA Prescaler Register
------------------------------------------------------*/
union	byte_def	trapre_addr;
#define		trapre		trapre_addr.byte

/*------------------------------------------------------
  Timer RA Register
------------------------------------------------------*/
union	byte_def	tra_addr;
#define		tra			tra_addr.byte

/*------------------------------------------------------
  LIN Control Register 2
------------------------------------------------------*/
union	byte_def	lincr2_addr;
#define		lincr2		lincr2_addr.byte

#define		bce_lincr2		lincr2_addr.bit.b0	/* Bus collision detection during Sync Break transmission enable bit */

/*------------------------------------------------------
  LIN Control Register
------------------------------------------------------*/
union	byte_def	lincr_addr;
#define		lincr		lincr_addr.byte

#define		sfie_lincr		lincr_addr.bit.b0	/* Synch Field measurement-completed interrupt enable bit */
#define		sbie_lincr		lincr_addr.bit.b1	/* Synch Break detection interrupt enable bit */
#define		bcie_lincr		lincr_addr.bit.b2	/* Bus collision detection interrupt enable bit */
#define		rxdsf_lincr		lincr_addr.bit.b3	/* RXD0 input status flag */
#define		lstart_lincr	lincr_addr.bit.b4	/* Synch Break detection start bit */
#define		sbe_lincr		lincr_addr.bit.b5	/* RXD0 input unmasking timing select bit (effective only in slave mode) */
#define		mst_lincr		lincr_addr.bit.b6	/* LIN operation mode setting bit */
#define		line_lincr		lincr_addr.bit.b7	/* LIN operation start bit */

/*------------------------------------------------------
  LIN Status Register
------------------------------------------------------*/
union	byte_def	linst_addr;
#define		linst		linst_addr.byte

#define		sfdct_linst		linst_addr.bit.b0	/* Synch Field measurement-completed flag */
#define		sbdct_linst		linst_addr.bit.b1	/* Synch Break detection flag */
#define		bcdct_linst		linst_addr.bit.b2	/* Bus collision detection flag */
#define		b0clr_linst		linst_addr.bit.b3	/* SFDCT bit clear bit */
#define		b1clr_linst		linst_addr.bit.b4	/* SBDCT bit clear bit */
#define		b2clr_linst		linst_addr.bit.b5	/* BCDCT bit clear bit */

/*------------------------------------------------------
  Timer RB Control Register
------------------------------------------------------*/
union	byte_def	trbcr_addr;
#define		trbcr		trbcr_addr.byte

#define		tstart_trbcr	trbcr_addr.bit.b0	/* Timer RB count start bit */
#define		tcstf_trbcr		trbcr_addr.bit.b1	/* Timer RB count status flag */
#define		tstop_trbcr		trbcr_addr.bit.b2	/* Timer RB count forcible stop bit */

/*------------------------------------------------------
  Timer RB One-Shot Control Register
------------------------------------------------------*/
union	byte_def	trbocr_addr;
#define		trbocr		trbocr_addr.byte

#define		tosst_trbocr	trbocr_addr.bit.b0	/* Timer RB one-shot start bit */
#define		tossp_trbocr	trbocr_addr.bit.b1	/* Timer RB one-shot stop bit */
#define		tosstf_trbocr	trbocr_addr.bit.b2	/* Timer RB one-shot status flag */

/*------------------------------------------------------
  Timer RB I/O Control Register
------------------------------------------------------*/
union	byte_def	trbioc_addr;
#define		trbioc		trbioc_addr.byte

#define		topl_trbioc		trbioc_addr.bit.b0	/* Timer RB output level select bit */
#define		tocnt_trbioc	trbioc_addr.bit.b1	/* Timer RB output switch bit */
#define		inostg_trbioc	trbioc_addr.bit.b2	/* One-shot trigger control bit */
#define		inoseg_trbioc	trbioc_addr.bit.b3	/* One-shot trigger polarity select bit */

/*------------------------------------------------------
  Timer RB Mode Register
------------------------------------------------------*/
union	byte_def	trbmr_addr;
#define		trbmr		trbmr_addr.byte

#define		tmod0_trbmr		trbmr_addr.bit.b0	/* Timer RB operating mode select bit */
#define		tmod1_trbmr		trbmr_addr.bit.b1	/* Timer RB operating mode select bit */
#define		twrc_trbmr		trbmr_addr.bit.b3	/* Timer RB write control bit */
#define		tck0_trbmr		trbmr_addr.bit.b4	/* Timer RB count source select bit */
#define		tck1_trbmr		trbmr_addr.bit.b5	/* Timer RB count source select bit */
#define		tckcut_trbmr	trbmr_addr.bit.b7	/* Timer RB count source cutoff bit */

/*------------------------------------------------------
  Timer RB Prescaler Register
------------------------------------------------------*/
union	byte_def	trbpre_addr;
#define		trbpre		trbpre_addr.byte

/*------------------------------------------------------
  Timer RB Secondary Register
------------------------------------------------------*/
union	byte_def	trbsc_addr;
#define		trbsc		trbsc_addr.byte

/*------------------------------------------------------
  Timer RB Primary Register
------------------------------------------------------*/
union	byte_def	trbpr_addr;
#define		trbpr		trbpr_addr.byte

/*------------------------------------------------------
  Timer RE Second Data Register / Counter Data Register
------------------------------------------------------*/
union	byte_def	tresec_addr;
#define		tresec		tresec_addr.byte

#define		sc00_tresec		tresec_addr.bit.b0	/* 1st digit of second count bit */
#define		sc01_tresec		tresec_addr.bit.b1	/* 1st digit of second count bit */
#define		sc02_tresec		tresec_addr.bit.b2	/* 1st digit of second count bit */
#define		sc03_tresec		tresec_addr.bit.b3	/* 1st digit of second count bit */
#define		sc10_tresec		tresec_addr.bit.b4	/* 2nd digit of second count bit */
#define		sc11_tresec		tresec_addr.bit.b5	/* 2nd digit of second count bit */
#define		sc12_tresec		tresec_addr.bit.b6	/* 2nd digit of second count bit */
#define		bsy_tresec		tresec_addr.bit.b7	/* Timer RE busy flag */

/*------------------------------------------------------
  Timer RE Minute Data Register / Compare Data Register
------------------------------------------------------*/
union	byte_def	tremin_addr;
#define		tremin		tremin_addr.byte

#define		mn00_tremin		tremin_addr.bit.b0	/* 1st digit of minute count bit */
#define		mn01_tremin		tremin_addr.bit.b1	/* 1st digit of minute count bit */
#define		mn02_tremin		tremin_addr.bit.b2	/* 1st digit of minute count bit */
#define		mn03_tremin		tremin_addr.bit.b3	/* 1st digit of minute count bit */
#define		mn10_tremin		tremin_addr.bit.b4	/* 2nd digit of minute count bit */
#define		mn11_tremin		tremin_addr.bit.b5	/* 2nd digit of minute count bit */
#define		mn12_tremin		tremin_addr.bit.b6	/* 2nd digit of minute count bit */
#define		bsy_tremin		tremin_addr.bit.b7	/* Timer RE busy flag */

/*------------------------------------------------------
  Timer RE Hour Data Register
------------------------------------------------------*/
union	byte_def	trehr_addr;
#define		trehr		trehr_addr.byte

#define		hr00_trehr		trehr_addr.bit.b0	/* 1st digit of hour count bit */
#define		hr01_trehr		trehr_addr.bit.b1	/* 1st digit of hour count bit */
#define		hr02_trehr		trehr_addr.bit.b2	/* 1st digit of hour count bit */
#define		hr03_trehr		trehr_addr.bit.b3	/* 1st digit of hour count bit */
#define		hr10_trehr		trehr_addr.bit.b4	/* 2nd digit of hour count bit */
#define		hr11_trehr		trehr_addr.bit.b5	/* 2nd digit of hour count bit */
#define		bsy_trehr		trehr_addr.bit.b7	/* Timer RE busy flag */

/*------------------------------------------------------
  Timer RE Day of Week Data Register
------------------------------------------------------*/
union	byte_def	trewk_addr;
#define		trewk		trewk_addr.byte

#define		wk0_trewk		trewk_addr.bit.b0	/* Day of week count bit */
#define		wk1_trewk		trewk_addr.bit.b1	/* Day of week count bit */
#define		wk2_trewk		trewk_addr.bit.b2	/* Day of week count bit */
#define		bsy_trewk		trewk_addr.bit.b7	/* Timer RE busy flag */

/*------------------------------------------------------
  Timer RE Control Register 1
------------------------------------------------------*/
union	byte_def	trecr1_addr;
#define		trecr1		trecr1_addr.byte

#define		tcstf_trecr1	trecr1_addr.bit.b1	/* Timer RE count status flag */
#define		toena_trecr1	trecr1_addr.bit.b2	/* TREO pin output enable bit */
#define		int_trecr1		trecr1_addr.bit.b3	/* Interrupt request timing bit */
#define		trerst_trecr1	trecr1_addr.bit.b4	/* Timer RE reset bit */
#define		pm_trecr1		trecr1_addr.bit.b5	/* A.m./p.m. bit */
#define		h12_h24_trecr1	trecr1_addr.bit.b6	/* Operating mode select bit */
#define		tstart_trecr1	trecr1_addr.bit.b7	/* Timer RE count start bit */

/*------------------------------------------------------
  Timer RE Control Register 2
------------------------------------------------------*/
union	byte_def	trecr2_addr;
#define		trecr2		trecr2_addr.byte

#define		seie_trecr2		trecr2_addr.bit.b0	/* Periodic interrupt triggered every second enable bit */
#define		mnie_trecr2		trecr2_addr.bit.b1	/* Periodic interrupt triggered every minute enable bit */
#define		hrie_trecr2		trecr2_addr.bit.b2	/* Periodic interrupt triggered every hour enable bit */
#define		dyie_trecr2		trecr2_addr.bit.b3	/* Periodic interrupt triggered every day enable bit */
#define		wkie_trecr2		trecr2_addr.bit.b4	/* Periodic interrupt triggered every week enable bit */
#define		comie_trecr2	trecr2_addr.bit.b5	/* Compare match interrupt enable bit */

/*------------------------------------------------------
  Timer RE Count Source Select Register
------------------------------------------------------*/
union	byte_def	trecsr_addr;
#define		trecsr		trecsr_addr.byte

#define		rcs0_trecsr		trecsr_addr.bit.b0	/* Count source select bit */
#define		rcs1_trecsr		trecsr_addr.bit.b1	/* Count source select bit */
#define		rcs2_trecsr		trecsr_addr.bit.b2	/* 4-bit counter select bit */
#define		rcs3_trecsr		trecsr_addr.bit.b3	/* Real-time clock mode select bit */
#define		rcs4_trecsr		trecsr_addr.bit.b4	/* Clock output select bit */
#define		rcs5_trecsr		trecsr_addr.bit.b5	/* Clock output select bit */
#define		rcs6_trecsr		trecsr_addr.bit.b6	/* Clock output select bit */

/*------------------------------------------------------
  Timer RC Mode Register
------------------------------------------------------*/
union	byte_def	trcmr_addr;
#define		trcmr		trcmr_addr.byte

#define		pwmb_trcmr		trcmr_addr.bit.b0	/* PWM mode of TRCIOB select bit */
#define		pwmc_trcmr		trcmr_addr.bit.b1	/* PWM mode of TRCIOC select bit */
#define		pwmd_trcmr		trcmr_addr.bit.b2	/* PWM mode of TRCIOD select bit */
#define		pwm2_trcmr		trcmr_addr.bit.b3	/* PWM2 mode select bit */
#define		bfc_trcmr		trcmr_addr.bit.b4	/* TRCGRC register function select bit */
#define		bfd_trcmr		trcmr_addr.bit.b5	/* TRCGRD register function select bit */
#define		tstart_trcmr	trcmr_addr.bit.b7	/* TRC count start bit */

/*------------------------------------------------------
  Timer RC Control Register 1
------------------------------------------------------*/
union	byte_def	trccr1_addr;
#define		trccr1		trccr1_addr.byte

#define		toa_trccr1		trccr1_addr.bit.b0	/* TRCIOA output level select bit */
#define		tob_trccr1		trccr1_addr.bit.b1	/* TRCIOB output level select bit */
#define		toc_trccr1		trccr1_addr.bit.b2	/* TRCIOC output level select bit */
#define		tod_trccr1		trccr1_addr.bit.b3	/* TRCIOD output level select bit */
#define		tck0_trccr1		trccr1_addr.bit.b4	/* Count source select bit */
#define		tck1_trccr1		trccr1_addr.bit.b5	/* Count source select bit */
#define		tck2_trccr1		trccr1_addr.bit.b6	/* Count source select bit */
#define		cclr_trccr1		trccr1_addr.bit.b7	/* TRC counter clear select bit */

/*------------------------------------------------------
  Timer RC Interrupt Enable Register
------------------------------------------------------*/
union	byte_def	trcier_addr;
#define		trcier		trcier_addr.byte

#define		imiea_trcier	trcier_addr.bit.b0	/* Input capture/compare match interrupt enable bit A */
#define		imieb_trcier	trcier_addr.bit.b1	/* Input capture/compare match interrupt enable bit B */
#define		imiec_trcier	trcier_addr.bit.b2	/* Input capture/compare match interrupt enable bit C */
#define		imied_trcier	trcier_addr.bit.b3	/* Input capture/compare match interrupt enable bit D */
#define		ovie_trcier		trcier_addr.bit.b7	/* Overflow interrupt enable bit */

/*------------------------------------------------------
  Timer RC Status Register
------------------------------------------------------*/
union	byte_def	trcsr_addr;
#define		trcsr		trcsr_addr.byte

#define		imfa_trcsr		trcsr_addr.bit.b0	/* Input capture/compare match flag A */
#define		imfb_trcsr		trcsr_addr.bit.b1	/* Input capture/compare match flag B */
#define		imfc_trcsr		trcsr_addr.bit.b2	/* Input capture/compare match flag C */
#define		imfd_trcsr		trcsr_addr.bit.b3	/* Input capture/compare match flag D */
#define		ovf_trcsr		trcsr_addr.bit.b7	/* Overflow flag */

/*------------------------------------------------------
  Timer RC I/O Control Register 0
------------------------------------------------------*/
union	byte_def	trcior0_addr;
#define		trcior0		trcior0_addr.byte

#define		ioa0_trcior0	trcior0_addr.bit.b0	/* TRCGRA control bit */
#define		ioa1_trcior0	trcior0_addr.bit.b1	/* TRCGRA control bit */
#define		ioa2_trcior0	trcior0_addr.bit.b2	/* TRCGRA mode select bit */
#define		ioa3_trcior0	trcior0_addr.bit.b3	/* TRCGRA input capture input switch bit */
#define		iob0_trcior0	trcior0_addr.bit.b4	/* TRCGRB control bit */
#define		iob1_trcior0	trcior0_addr.bit.b5	/* TRCGRB control bit */
#define		iob2_trcior0	trcior0_addr.bit.b6	/* TRCGRB mode select bit */

/*------------------------------------------------------
  Timer RC I/O Control Register 1
------------------------------------------------------*/
union	byte_def	trcior1_addr;
#define		trcior1		trcior1_addr.byte

#define		ioc0_trcior1	trcior1_addr.bit.b0	/* TRCGRC control bit */
#define		ioc1_trcior1	trcior1_addr.bit.b1	/* TRCGRC control bit */
#define		ioc2_trcior1	trcior1_addr.bit.b2	/* TRCGRC mode select bit */
#define		ioc3_trcior1	trcior1_addr.bit.b3	/* TRCGRC register function select bit */
#define		iod0_trcior1	trcior1_addr.bit.b4	/* TRCGRD control bit */
#define		iod1_trcior1	trcior1_addr.bit.b5	/* TRCGRD control bit */
#define		iod2_trcior1	trcior1_addr.bit.b6	/* TRCGRD mode select bit */
#define		iod3_trcior1	trcior1_addr.bit.b7	/* TRCGRD register function select bit */

/*------------------------------------------------------
  Timer RC Counter
------------------------------------------------------*/
unsigned	short	trc_addr;
#define		trc			trc_addr				/* Timer RC Counter */

/*------------------------------------------------------
  Timer RC General Register A
------------------------------------------------------*/
unsigned	short	trcgra_addr;
#define		trcgra		trcgra_addr				/* Timer RC General Register A */

/*------------------------------------------------------
  Timer RC General Register B
------------------------------------------------------*/
unsigned	short	trcgrb_addr;
#define		trcgrb		trcgrb_addr				/* Timer RC General Register B */

/*------------------------------------------------------
  Timer RC General Register C
------------------------------------------------------*/
unsigned	short	trcgrc_addr;
#define		trcgrc		trcgrc_addr				/* Timer RC General Register C */

/*------------------------------------------------------
  Timer RC General Register D
------------------------------------------------------*/
unsigned	short	trcgrd_addr;
#define		trcgrd		trcgrd_addr				/* Timer RC General Register D */

/*------------------------------------------------------
  Timer RC Control Register 2
------------------------------------------------------*/
union	byte_def	trccr2_addr;
#define		trccr2		trccr2_addr.byte

#define		polb_trccr2		trccr2_addr.bit.b0	/* PWM mode output level control bit B */
#define		polc_trccr2		trccr2_addr.bit.b1	/* PWM mode output level control bit C */
#define		pold_trccr2		trccr2_addr.bit.b2	/* PWM mode output level control bit D */
#define		csel_trccr2		trccr2_addr.bit.b5	/* TRC count operation select bit */
#define		tceg0_trccr2	trccr2_addr.bit.b6	/* TRCTRG input edge select bit */
#define		tceg1_trccr2	trccr2_addr.bit.b7	/* TRCTRG input edge select bit */

/*------------------------------------------------------
  Timer RC Digital Filter Function Select Register
------------------------------------------------------*/
union	byte_def	trcdf_addr;
#define		trcdf		trcdf_addr.byte

#define		dfa_trcdf		trcdf_addr.bit.b0	/* TRCIOA pin digital filter function select bit */
#define		dfb_trcdf		trcdf_addr.bit.b1	/* TRCIOB pin digital filter function select bit */
#define		dfc_trcdf		trcdf_addr.bit.b2	/* TRCIOC pin digital filter function select bit */
#define		dfd_trcdf		trcdf_addr.bit.b3	/* TRCIOD pin digital filter function select bit */
#define		dftrg_trcdf		trcdf_addr.bit.b4	/* TRCTRG pin digital filter function select bit */
#define		dfck0_trcdf		trcdf_addr.bit.b6	/* Clock select bits for digital filter function */
#define		dfck1_trcdf		trcdf_addr.bit.b7	/* Clock select bits for digital filter function */

/*------------------------------------------------------
  Timer RC Output Master Enable Register
------------------------------------------------------*/
union	byte_def	trcoer_addr;
#define		trcoer		trcoer_addr.byte

#define		ea_trcoer		trcoer_addr.bit.b0	/* TRCIOA output disable bit */
#define		eb_trcoer		trcoer_addr.bit.b1	/* TRCIOB output disable bit */
#define		ec_trcoer		trcoer_addr.bit.b2	/* TRCIOC output disable bit */
#define		ed_trcoer		trcoer_addr.bit.b3	/* TRCIOD output disable bit */
#define		pto_trcoer		trcoer_addr.bit.b7	/* INT0 of pulse output forced cutoff signal input enabled bit */

/*------------------------------------------------------
  Timer RC Trigger Control Register
------------------------------------------------------*/
union	byte_def	trcadcr_addr;
#define		trcadcr		trcadcr_addr.byte

#define		adtrgae_trcadcr	trcadcr_addr.bit.b0	/* A/D trigger A enable bit */
#define		adtrgbe_trcadcr	trcadcr_addr.bit.b1	/* A/D trigger B enable bit */
#define		adtrgce_trcadcr	trcadcr_addr.bit.b2	/* A/D trigger C enable bit */
#define		adtrgde_trcadcr	trcadcr_addr.bit.b3	/* A/D trigger D enable bit */

/*------------------------------------------------------
  Timer RD Control Expansion Register
------------------------------------------------------*/
union	byte_def	trdecr_addr;
#define		trdecr		trdecr_addr.byte

#define		itclk0_trdecr	trdecr_addr.bit.b3	/* Timer RD0 fC2 select bit */
#define		itclk1_trdecr	trdecr_addr.bit.b7	/* Timer RD1 fC2 select bit */

/*------------------------------------------------------
  Timer RD Trigger Control Register
------------------------------------------------------*/
union	byte_def	trdadcr_addr;
#define		trdadcr		trdadcr_addr.byte

#define		adtrga0e_trdadcr	trdadcr_addr.bit.b0	/* A/D trigger A0 enable bit */
#define		adtrgb0e_trdadcr	trdadcr_addr.bit.b1	/* A/D trigger B0 enable bit */
#define		adtrgc0e_trdadcr	trdadcr_addr.bit.b2	/* A/D trigger C0 enable bit */
#define		adtrgd0e_trdadcr	trdadcr_addr.bit.b3	/* A/D trigger D0 enable bit */
#define		adtrga1e_trdadcr	trdadcr_addr.bit.b4	/* A/D trigger A1 enable bit */
#define		adtrgb1e_trdadcr	trdadcr_addr.bit.b5	/* A/D trigger B1 enable bit */
#define		adtrgc1e_trdadcr	trdadcr_addr.bit.b6	/* A/D trigger C1 enable bit */
#define		adtrgd1e_trdadcr	trdadcr_addr.bit.b7	/* A/D trigger D1 enable bit */

/*------------------------------------------------------
  Timer RD Start Register
------------------------------------------------------*/
union	byte_def	trdstr_addr;
#define		trdstr		trdstr_addr.byte

#define		tstart0_trdstr	trdstr_addr.bit.b0	/* TRD0 count start flag */
#define		tstart1_trdstr	trdstr_addr.bit.b1	/* TRD1 count start flag */
#define		csel0_trdstr	trdstr_addr.bit.b2	/* TRD0 count operation select bit */
#define		csel1_trdstr	trdstr_addr.bit.b3	/* TRD1 count operation select bit */

/*------------------------------------------------------
  Timer RD Mode Register
------------------------------------------------------*/
union	byte_def	trdmr_addr;
#define		trdmr		trdmr_addr.byte

#define		sync_trdmr		trdmr_addr.bit.b0	/* Timer RD synchronous bit */
#define		bfc0_trdmr		trdmr_addr.bit.b4	/* TRDGRC0 register function select bit */
#define		bfd0_trdmr		trdmr_addr.bit.b5	/* TRDGRD0 register function select bit */
#define		bfc1_trdmr		trdmr_addr.bit.b6	/* TRDGRC1 register function select bit */
#define		bfd1_trdmr		trdmr_addr.bit.b7	/* TRDGRD1 register function select bit */

/*------------------------------------------------------
  Timer RD PWM Mode Register
------------------------------------------------------*/
union	byte_def	trdpmr_addr;
#define		trdpmr		trdpmr_addr.byte

#define		pwmb0_trdpmr	trdpmr_addr.bit.b0	/* PWM mode of TRDIOB0 select bit */
#define		pwmc0_trdpmr	trdpmr_addr.bit.b1	/* PWM mode of TRDIOC0 select bit */
#define		pwmd0_trdpmr	trdpmr_addr.bit.b2	/* PWM mode of TRDIOD0 select bit */
#define		pwmb1_trdpmr	trdpmr_addr.bit.b4	/* PWM mode of TRDIOB1 select bit */
#define		pwmc1_trdpmr	trdpmr_addr.bit.b5	/* PWM mode of TRDIOC1 select bit */
#define		pwmd1_trdpmr	trdpmr_addr.bit.b6	/* PWM mode of TRDIOD1 select bit */

/*------------------------------------------------------
  Timer RD Function Control Register
------------------------------------------------------*/
union	byte_def	trdfcr_addr;
#define		trdfcr		trdfcr_addr.byte

#define		cmd0_trdfcr		trdfcr_addr.bit.b0	/* Combination mode select bit */
#define		cmd1_trdfcr		trdfcr_addr.bit.b1	/* Combination mode select bit */
#define		ols0_trdfcr		trdfcr_addr.bit.b2	/* Normal-phase output level select bit */
#define		ols1_trdfcr		trdfcr_addr.bit.b3	/* Counter-phase output level select bit */
#define		adtrg_trdfcr	trdfcr_addr.bit.b4	/* A/D trigger enable bit */
#define		adeg_trdfcr		trdfcr_addr.bit.b5	/* A/D trigger edge select bit */
#define		stclk_trdfcr	trdfcr_addr.bit.b6	/* External clock input select bit */
#define		pwm3_trdfcr		trdfcr_addr.bit.b7	/* PWM3 mode select bit */

/*------------------------------------------------------
  Timer RD Output Master Enable Register 1
------------------------------------------------------*/
union	byte_def	trdoer1_addr;
#define		trdoer1		trdoer1_addr.byte

#define		ea0_trdoer1		trdoer1_addr.bit.b0	/* TRDIOA0 output disable bit */
#define		eb0_trdoer1		trdoer1_addr.bit.b1	/* TRDIOB0 output disable bit */
#define		ec0_trdoer1		trdoer1_addr.bit.b2	/* TRDIOC0 output disable bit */
#define		ed0_trdoer1		trdoer1_addr.bit.b3	/* TRDIOD0 output disable bit */
#define		ea1_trdoer1		trdoer1_addr.bit.b4	/* TRDIOA1 output disable bit */
#define		eb1_trdoer1		trdoer1_addr.bit.b5	/* TRDIOB1 output disable bit */
#define		ec1_trdoer1		trdoer1_addr.bit.b6	/* TRDIOC1 output disable bit */
#define		ed1_trdoer1		trdoer1_addr.bit.b7	/* TRDIOD1 output disable bit */

/*------------------------------------------------------
  Timer RD Output Master Enable Register 2
------------------------------------------------------*/
union	byte_def	trdoer2_addr;
#define		trdoer2		trdoer2_addr.byte

#define		pto_trdoer2		trdoer2_addr.bit.b7	/* INT0 of pulse output forced cutoff signal input enabled bit */

/*------------------------------------------------------
  Timer RD Output Control Register
------------------------------------------------------*/
union	byte_def	trdocr_addr;
#define		trdocr		trdocr_addr.byte

#define		toa0_trdocr		trdocr_addr.bit.b0	/* TRDIOA0 output level select bit */
#define		tob0_trdocr		trdocr_addr.bit.b1	/* TRDIOB0 output level select bit */
#define		toc0_trdocr		trdocr_addr.bit.b2	/* TRDIOC0 initial output level select bit */
#define		tod0_trdocr		trdocr_addr.bit.b3	/* TRDIOD0 initial output level select bit */
#define		toa1_trdocr		trdocr_addr.bit.b4	/* TRDIOA1 initial output level select bit */
#define		tob1_trdocr		trdocr_addr.bit.b5	/* TRDIOB1 initial output level select bit */
#define		toc1_trdocr		trdocr_addr.bit.b6	/* TRDIOC1 initial output level select bit */
#define		tod1_trdocr		trdocr_addr.bit.b7	/* TRDIOD1 initial output level select bit */

/*------------------------------------------------------
  Timer RD Digital Filter Function Select Register 0
------------------------------------------------------*/
union	byte_def	trddf0_addr;
#define		trddf0		trddf0_addr.byte

#define		dfa_trddf0		trddf0_addr.bit.b0	/* TRDIOA pin digital filter function select bit */
#define		dfb_trddf0		trddf0_addr.bit.b1	/* TRDIOB pin digital filter function select bit */
#define		dfc_trddf0		trddf0_addr.bit.b2	/* TRDIOC pin digital filter function select bit */
#define		dfd_trddf0		trddf0_addr.bit.b3	/* TRDIOD pin digital filter function select */
#define		dfck0_trddf0	trddf0_addr.bit.b6	/* Clock select bits for digital filter function */
#define		dfck1_trddf0	trddf0_addr.bit.b7	/* Clock select bits for digital filter function */

/*------------------------------------------------------
  Timer RD Digital Filter Function Select Register 1
------------------------------------------------------*/
union	byte_def	trddf1_addr;
#define		trddf1		trddf1_addr.byte

#define		dfa_trddf1		trddf1_addr.bit.b0	/* TRDIOA pin digital filter function select bit */
#define		dfb_trddf1		trddf1_addr.bit.b1	/* TRDIOB pin digital filter function select bit */
#define		dfc_trddf1		trddf1_addr.bit.b2	/* TRDIOC pin digital filter function select bit */
#define		dfd_trddf1		trddf1_addr.bit.b3	/* TRDIOD pin digital filter function select */
#define		dfck0_trddf1	trddf1_addr.bit.b6	/* Clock select bits for digital filter function */
#define		dfck1_trddf1	trddf1_addr.bit.b7	/* Clock select bits for digital filter function */

/*------------------------------------------------------
  Timer RD Control Register 0
------------------------------------------------------*/
union	byte_def	trdcr0_addr;
#define		trdcr0		trdcr0_addr.byte

#define		tck0_trdcr0		trdcr0_addr.bit.b0	/* Count source select bit */
#define		tck1_trdcr0		trdcr0_addr.bit.b1	/* Count source select bit */
#define		tck2_trdcr0		trdcr0_addr.bit.b2	/* Count source select bit */
#define		ckeg0_trdcr0	trdcr0_addr.bit.b3	/* External clock edge select bit */
#define		ckeg1_trdcr0	trdcr0_addr.bit.b4	/* External clock edge select bit */
#define		cclr0_trdcr0	trdcr0_addr.bit.b5	/* TRD0 counter clear select bit */
#define		cclr1_trdcr0	trdcr0_addr.bit.b6	/* TRD0 counter clear select bit */
#define		cclr2_trdcr0	trdcr0_addr.bit.b7	/* TRD0 counter clear select bit */

/*------------------------------------------------------
  Timer RD I/O Control Register A0
------------------------------------------------------*/
union	byte_def	trdiora0_addr;
#define		trdiora0	trdiora0_addr.byte

#define		ioa0_trdiora0	trdiora0_addr.bit.b0	/* TRDGRA control bit */
#define		ioa1_trdiora0	trdiora0_addr.bit.b1	/* TRDGRA control bit */
#define		ioa2_trdiora0	trdiora0_addr.bit.b2	/* TRDGRA mode select bit */
#define		ioa3_trdiora0	trdiora0_addr.bit.b3	/* Input capture input switch bit */
#define		iob0_trdiora0	trdiora0_addr.bit.b4	/* TRDGRB control bit */
#define		iob1_trdiora0	trdiora0_addr.bit.b5	/* TRDGRB control bit */
#define		iob2_trdiora0	trdiora0_addr.bit.b6	/* TRDGRB mode select bit */

/*------------------------------------------------------
  Timer RD I/O Control Register C0
------------------------------------------------------*/
union	byte_def	trdiorc0_addr;
#define		trdiorc0	trdiorc0_addr.byte

#define		ioc0_trdiorc0	trdiorc0_addr.bit.b0	/* TRDGRC control bit */
#define		ioc1_trdiorc0	trdiorc0_addr.bit.b1	/* TRDGRC control bit */
#define		ioc2_trdiorc0	trdiorc0_addr.bit.b2	/* TRDGRC mode select bit */
#define		ioc3_trdiorc0	trdiorc0_addr.bit.b3	/* TRDGRC register function select bit */
#define		iod0_trdiorc0	trdiorc0_addr.bit.b4	/* TRDGRD control bit */
#define		iod1_trdiorc0	trdiorc0_addr.bit.b5	/* TRDGRD control bit */
#define		iod2_trdiorc0	trdiorc0_addr.bit.b6	/* TRDGRD mode select bit */
#define		iod3_trdiorc0	trdiorc0_addr.bit.b7	/* TRDGRD register function select bit */

/*------------------------------------------------------
  Timer RD Status Register 0
------------------------------------------------------*/
union	byte_def	trdsr0_addr;
#define		trdsr0		trdsr0_addr.byte

#define		imfa_trdsr0		trdsr0_addr.bit.b0	/* Input capture/compare match flag A */
#define		imfb_trdsr0		trdsr0_addr.bit.b1	/* Input capture/compare match flag B */
#define		imfc_trdsr0		trdsr0_addr.bit.b2	/* Input capture/compare match flag C */
#define		imfd_trdsr0		trdsr0_addr.bit.b3	/* Input capture/compare match flag D */
#define		ovf_trdsr0		trdsr0_addr.bit.b4	/* Overflow flag */
#define		udf_trdsr0		trdsr0_addr.bit.b5	/* Underflow flag */

/*------------------------------------------------------
  Timer RD Interrupt Enable Register 0
------------------------------------------------------*/
union	byte_def	trdier0_addr;
#define		trdier0		trdier0_addr.byte

#define		imiea_trdier0	trdier0_addr.bit.b0	/* Input capture/compare match interrupt enable bit A */
#define		imieb_trdier0	trdier0_addr.bit.b1	/* Input capture/compare match interrupt enable bit B */
#define		imiec_trdier0	trdier0_addr.bit.b2	/* Input capture/compare match interrupt enable bit C */
#define		imied_trdier0	trdier0_addr.bit.b3	/* Input capture/compare match interrupt enable bit D */
#define		ovie_trdier0	trdier0_addr.bit.b4	/* Overflow/underflow interrupt enable bit */

/*------------------------------------------------------
  Timer RD PWM Mode Output Level Control Register 0
------------------------------------------------------*/
union	byte_def	trdpocr0_addr;
#define		trdpocr0	trdpocr0_addr.byte

#define		polb_trdpocr0	trdpocr0_addr.bit.b0	/* PWM mode output level control bit B */
#define		polc_trdpocr0	trdpocr0_addr.bit.b1	/* PWM mode output level control bit C */
#define		pold_trdpocr0	trdpocr0_addr.bit.b2	/* PWM mode output level control bit D */

/*------------------------------------------------------
  Timer RD Counter 0
------------------------------------------------------*/
unsigned	short	trd0_addr;
#define		trd0		trd0_addr					/* Timer RD Counter 0 */

/*------------------------------------------------------
  Timer RD General Register A0
------------------------------------------------------*/
unsigned	short	trdgra0_addr;
#define		trdgra0		trdgra0_addr				/* Timer RD General Register A0 */

/*------------------------------------------------------
  Timer RD General Register B0
------------------------------------------------------*/
unsigned	short	trdgrb0_addr;
#define		trdgrb0		trdgrb0_addr				/* Timer RD General Register B0 */

/*------------------------------------------------------
  Timer RD General Register C0
------------------------------------------------------*/
unsigned	short	trdgrc0_addr;
#define		trdgrc0		trdgrc0_addr				/* Timer RD General Register C0 */

/*------------------------------------------------------
  Timer RD General Register D0
------------------------------------------------------*/
unsigned	short	trdgrd0_addr;
#define		trdgrd0		trdgrd0_addr				/* Timer RD General Register D0 */

/*------------------------------------------------------
  Timer RD Control Register 1
------------------------------------------------------*/
union	byte_def	trdcr1_addr;
#define		trdcr1		trdcr1_addr.byte

#define		tck0_trdcr1		trdcr1_addr.bit.b0	/* Count source select bit */
#define		tck1_trdcr1		trdcr1_addr.bit.b1	/* Count source select bit */
#define		tck2_trdcr1		trdcr1_addr.bit.b2	/* Count source select bit */
#define		ckeg0_trdcr1	trdcr1_addr.bit.b3	/* External clock edge select bit */
#define		ckeg1_trdcr1	trdcr1_addr.bit.b4	/* External clock edge select bit */
#define		cclr0_trdcr1	trdcr1_addr.bit.b5	/* TRDi counter clear select bit */
#define		cclr1_trdcr1	trdcr1_addr.bit.b6	/* TRDi counter clear select bit */
#define		cclr2_trdcr1	trdcr1_addr.bit.b7	/* TRDi counter clear select bit */

/*------------------------------------------------------
  Timer RD I/O Control Register A1
------------------------------------------------------*/
union	byte_def	trdiora1_addr;
#define		trdiora1	trdiora1_addr.byte

#define		ioa0_trdiora1	trdiora1_addr.bit.b0	/* TRDGRA control bit */
#define		ioa1_trdiora1	trdiora1_addr.bit.b1	/* TRDGRA control bit */
#define		ioa2_trdiora1	trdiora1_addr.bit.b2	/* TRDGRA mode select bit */
#define		ioa3_trdiora1	trdiora1_addr.bit.b3	/* Input capture input switch bit */
#define		iob0_trdiora1	trdiora1_addr.bit.b4	/* TRDGRB control bit */
#define		iob1_trdiora1	trdiora1_addr.bit.b5	/* TRDGRB control bit */
#define		iob2_trdiora1	trdiora1_addr.bit.b6	/* TRDGRB mode select bit */

/*------------------------------------------------------
  Timer RD I/O Control Register C1
------------------------------------------------------*/
union	byte_def	trdiorc1_addr;
#define		trdiorc1	trdiorc1_addr.byte

#define		ioc0_trdiorc1	trdiorc1_addr.bit.b0	/* TRDGRC control bit */
#define		ioc1_trdiorc1	trdiorc1_addr.bit.b1	/* TRDGRC control bit */
#define		ioc2_trdiorc1	trdiorc1_addr.bit.b2	/* TRDGRC mode select bit */
#define		ioc3_trdiorc1	trdiorc1_addr.bit.b3	/* TRDGRC register function select bit */
#define		iod0_trdiorc1	trdiorc1_addr.bit.b4	/* TRDGRD control bit */
#define		iod1_trdiorc1	trdiorc1_addr.bit.b5	/* TRDGRD control bit */
#define		iod2_trdiorc1	trdiorc1_addr.bit.b6	/* TRDGRD mode select bit */
#define		iod3_trdiorc1	trdiorc1_addr.bit.b7	/* TRDGRD register function select bit */

/*------------------------------------------------------
  Timer RD Status Register 1
------------------------------------------------------*/
union	byte_def	trdsr1_addr;
#define		trdsr1		trdsr1_addr.byte

#define		imfa_trdsr1		trdsr1_addr.bit.b0	/* Input capture/compare match flag A */
#define		imfb_trdsr1		trdsr1_addr.bit.b1	/* Input capture/compare match flag B */
#define		imfc_trdsr1		trdsr1_addr.bit.b2	/* Input capture/compare match flag C */
#define		imfd_trdsr1		trdsr1_addr.bit.b3	/* Input capture/compare match flag D */
#define		ovf_trdsr1		trdsr1_addr.bit.b4	/* Overflow flag */
#define		udf_trdsr1		trdsr1_addr.bit.b5	/* Underflow flag */

/*------------------------------------------------------
  Timer RD Interrupt Enable Register 1
------------------------------------------------------*/
union	byte_def	trdier1_addr;
#define		trdier1		trdier1_addr.byte

#define		imiea_trdier1	trdier1_addr.bit.b0	/* Input capture/compare match interrupt enable bit A */
#define		imieb_trdier1	trdier1_addr.bit.b1	/* Input capture/compare match interrupt enable bit B */
#define		imiec_trdier1	trdier1_addr.bit.b2	/* Input capture/compare match interrupt enable bit C */
#define		imied_trdier1	trdier1_addr.bit.b3	/* Input capture/compare match interrupt enable bit D */
#define		ovie_trdier1	trdier1_addr.bit.b4	/* Overflow/underflow interrupt enable bit */

/*------------------------------------------------------
  Timer RD PWM Mode Output Level Control Register 1
------------------------------------------------------*/
union	byte_def	trdpocr1_addr;
#define		trdpocr1	trdpocr1_addr.byte

#define		polb_trdpocr1	trdpocr1_addr.bit.b0	/* PWM mode output level control bit B */
#define		polc_trdpocr1	trdpocr1_addr.bit.b1	/* PWM mode output level control bit C */
#define		pold_trdpocr1	trdpocr1_addr.bit.b2	/* PWM mode output level control bit D */

/*------------------------------------------------------
  Timer RD Counter 1
------------------------------------------------------*/
unsigned	short	trd1_addr;
#define		trd1		trd1_addr					/* Timer RD Counter 1 */

/*------------------------------------------------------
  Timer RD General Register A1
------------------------------------------------------*/
unsigned	short	trdgra1_addr;
#define		trdgra1		trdgra1_addr				/* Timer RD General Register A1 */

/*------------------------------------------------------
  Timer RD General Register B1
------------------------------------------------------*/
unsigned	short	trdgrb1_addr;
#define		trdgrb1		trdgrb1_addr				/* Timer RD General Register B1 */

/*------------------------------------------------------
  Timer RD General Register C1
------------------------------------------------------*/
unsigned	short	trdgrc1_addr;
#define		trdgrc1		trdgrc1_addr				/* Timer RD General Register C1 */

/*------------------------------------------------------
  Timer RD General Register D1
------------------------------------------------------*/
unsigned	short	trdgrd1_addr;
#define		trdgrd1		trdgrd1_addr				/* Timer RD General Register D1 */

/*------------------------------------------------------
  UART1 Bit Rate Register
------------------------------------------------------*/
union	byte_def	u1brg_addr;
#define		u1brg		u1brg_addr.byte

/*------------------------------------------------------
  Timer RG Mode Register
------------------------------------------------------*/
union	byte_def	trgmr_addr;
#define		trgmr		trgmr_addr.byte

#define		pwm_trgmr		trgmr_addr.bit.b0	/* PWM mode select bit */
#define		mdf_trgmr		trgmr_addr.bit.b1	/* Phase counting mode select bit */
#define		dfa_trgmr		trgmr_addr.bit.b2	/* Digital filer function select bit for TRGIOA pin */
#define		dfb_trgmr		trgmr_addr.bit.b3	/* Digital filer function select bit for TRGIOB pin */
#define		dfck0_trgmr		trgmr_addr.bit.b4	/* Digital filter function clock select bit */
#define		dfck1_trgmr		trgmr_addr.bit.b5	/* Digital filter function clock select bit */
#define		tstart_trgmr	trgmr_addr.bit.b7	/* TRG count start bit */

/*------------------------------------------------------
  Timer RG Count Control Register
------------------------------------------------------*/
union	byte_def	trgcntc_addr;
#define		trgcntc		trgcntc_addr.byte

#define		cnten0_trgcntc	trgcntc_addr.bit.b0	/* Counter enable bit 0 */
#define		cnten1_trgcntc	trgcntc_addr.bit.b1	/* Counter enable bit 1 */
#define		cnten2_trgcntc	trgcntc_addr.bit.b2	/* Counter enable bit 2 */
#define		cnten3_trgcntc	trgcntc_addr.bit.b3	/* Counter enable bit 3 */
#define		cnten4_trgcntc	trgcntc_addr.bit.b4	/* Counter enable bit 4 */
#define		cnten5_trgcntc	trgcntc_addr.bit.b5	/* Counter enable bit 5 */
#define		cnten6_trgcntc	trgcntc_addr.bit.b6	/* Counter enable bit 6 */
#define		cnten7_trgcntc	trgcntc_addr.bit.b7	/* Counter enable bit 7 */

/*------------------------------------------------------
  Timer RG Control Register
------------------------------------------------------*/
union	byte_def	trgcr_addr;
#define		trgcr		trgcr_addr.byte

#define		tck0_trgcr		trgcr_addr.bit.b0	/* Count source select bit */
#define		tck1_trgcr		trgcr_addr.bit.b1	/* Count source select bit */
#define		tck2_trgcr		trgcr_addr.bit.b2	/* Count source select bit */
#define		ckeg0_trgcr		trgcr_addr.bit.b3	/* External clock active edge select bit */
#define		ckeg1_trgcr		trgcr_addr.bit.b4	/* External clock active edge select bit */
#define		cclr0_trgcr		trgcr_addr.bit.b5	/* TRG register clear select bit */
#define		cclr1_trgcr		trgcr_addr.bit.b6	/* TRG register clear select bit */

/*------------------------------------------------------
  Timer RG Interrupt Enable Register
------------------------------------------------------*/
union	byte_def	trgier_addr;
#define		trgier		trgier_addr.byte

#define		imiea_trgier	trgier_addr.bit.b0	/* Input-capture/compare-match interrupt enable bit A */
#define		imieb_trgier	trgier_addr.bit.b1	/* Input-capture/compare-match interrupt enable bit B */
#define		udie_trgier		trgier_addr.bit.b2	/* Underflow interrupt enable bit */
#define		ovie_trgier		trgier_addr.bit.b3	/* Overflow interrupt enable bit */

/*------------------------------------------------------
  Timer RG Status Register
------------------------------------------------------*/
union	byte_def	trgsr_addr;
#define		trgsr		trgsr_addr.byte

#define		imfa_trgsr		trgsr_addr.bit.b0	/* Input-capture/compare-match flag A */
#define		imfb_trgsr		trgsr_addr.bit.b1	/* Input-capture/compare-match flag B */
#define		udf_trgsr		trgsr_addr.bit.b2	/* Underflow flag */
#define		ovf_trgsr		trgsr_addr.bit.b3	/* Overflow flag */
#define		dirf_trgsr		trgsr_addr.bit.b4	/* Count direction flag */

/*------------------------------------------------------
  Timer RG I/O Control Register
------------------------------------------------------*/
union	byte_def	trgior_addr;
#define		trgior		trgior_addr.byte

#define		ioa0_trgior		trgior_addr.bit.b0	/* TRGGRA control bit */
#define		ioa1_trgior		trgior_addr.bit.b1	/* TRGGRA control bit */
#define		ioa2_trgior		trgior_addr.bit.b2	/* TRGGRA mode select bit */
#define		bufa_trgior		trgior_addr.bit.b3	/* TRGGRC register function select bit */
#define		iob0_trgior		trgior_addr.bit.b4	/* TRGGRB control bit */
#define		iob1_trgior		trgior_addr.bit.b5	/* TRGGRB control bit */
#define		iob2_trgior		trgior_addr.bit.b6	/* TRGGRB mode select bit */
#define		bufb_trgior		trgior_addr.bit.b7	/* TRGGRD register function select bit */

/*------------------------------------------------------
  Timer RG Counter
------------------------------------------------------*/
unsigned	short	trg_addr;
#define		trg			trg_addr				/* Timer RG Counter */

/*------------------------------------------------------
  Timer RG General Register A
------------------------------------------------------*/
unsigned	short	trggra_addr;
#define		trggra		trggra_addr				/* Timer RG General Register A */

/*------------------------------------------------------
  Timer RG General Register B
------------------------------------------------------*/
unsigned	short	trggrb_addr;
#define		trggrb		trggrb_addr				/* Timer RG General Register B */

/*------------------------------------------------------
  Timer RG General Register C
------------------------------------------------------*/
unsigned	short	trggrc_addr;
#define		trggrc		trggrc_addr				/* Timer RG General Register C */

/*------------------------------------------------------
  Timer RG General Register D
------------------------------------------------------*/
unsigned	short	trggrd_addr;
#define		trggrd		trggrd_addr				/* Timer RG General Register D */

/*------------------------------------------------------
  Timer RA Pin Select Register
------------------------------------------------------*/
union	byte_def	trasr_addr;
#define		trasr		trasr_addr.byte

#define		traiosel0		trasr_addr.bit.b0	/* TRAIO pin select bit */
#define		traiosel1		trasr_addr.bit.b1	/* TRAIO pin select bit */
#define		traiosel2		trasr_addr.bit.b2	/* TRAIO pin select bit */
#define		traosel0		trasr_addr.bit.b3	/* TRAO pin select bit */
#define		traosel1		trasr_addr.bit.b4	/* TRAO pin select bit */

/*------------------------------------------------------
  Timer RB/RC Pin Select Register
------------------------------------------------------*/
union	byte_def	trbrcsr_addr;
#define		trbrcsr		trbrcsr_addr.byte

#define		trbosel0		trbrcsr_addr.bit.b0	/* TRBO pin select bit */
#define		trcclksel0		trbrcsr_addr.bit.b4	/* TRCCLK pin select bit */
#define		trcclksel1		trbrcsr_addr.bit.b5	/* TRCCLK pin select bit */
#define		trcclksel2		trbrcsr_addr.bit.b6	/* TRCCLK pin select bit */

/*------------------------------------------------------
  Timer RC Pin Select Register 0
------------------------------------------------------*/
union	byte_def	trcpsr0_addr;
#define		trcpsr0		trcpsr0_addr.byte

#define		trcioasel0		trcpsr0_addr.bit.b0	/* TRCIOA/TRCTRG pin select bit */
#define		trcioasel1		trcpsr0_addr.bit.b1	/* TRCIOA/TRCTRG pin select bit */
#define		trcioasel2		trcpsr0_addr.bit.b2	/* TRCIOA/TRCTRG pin select bit */
#define		trciobsel0		trcpsr0_addr.bit.b4	/* TRCIOB pin select bit */
#define		trciobsel1		trcpsr0_addr.bit.b5	/* TRCIOB pin select bit */
#define		trciobsel2		trcpsr0_addr.bit.b6	/* TRCIOB pin select bit */

/*------------------------------------------------------
  Timer RC Pin Select Register 1
------------------------------------------------------*/
union	byte_def	trcpsr1_addr;
#define		trcpsr1		trcpsr1_addr.byte

#define		trciocsel0		trcpsr1_addr.bit.b0	/* TRCIOC pin select bit */
#define		trciocsel1		trcpsr1_addr.bit.b1	/* TRCIOC pin select bit */
#define		trciocsel2		trcpsr1_addr.bit.b2	/* TRCIOC pin select bit */
#define		trciodsel0		trcpsr1_addr.bit.b4	/* TRCIOD pin select bit */
#define		trciodsel1		trcpsr1_addr.bit.b5	/* TRCIOD pin select bit */
#define		trciodsel2		trcpsr1_addr.bit.b6	/* TRCIOD pin select bit */

/*------------------------------------------------------
  Timer RD Pin Select Register 0
------------------------------------------------------*/
union	byte_def	trdpsr0_addr;
#define		trdpsr0		trdpsr0_addr.byte

#define		trdioa0sel0		trdpsr0_addr.bit.b0	/* TRDIOA0/TRDCLK pin select bit */
#define		trdiob0sel0		trdpsr0_addr.bit.b2	/* TRDIOB0 pin select bit */
#define		trdiob0sel1		trdpsr0_addr.bit.b3	/* TRDIOB0 pin select bit */
#define		trdioc0sel0		trdpsr0_addr.bit.b4	/* TRDIOC0 pin select bit */
#define		trdioc0sel1		trdpsr0_addr.bit.b5	/* TRDIOC0 pin select bit */
#define		trdiod0sel0		trdpsr0_addr.bit.b6	/* TRDIOD0 pin select bit */

/*------------------------------------------------------
  Timer RD Pin Select Register 1
------------------------------------------------------*/
union	byte_def	trdpsr1_addr;
#define		trdpsr1		trdpsr1_addr.byte

#define		trdioa1sel0		trdpsr1_addr.bit.b0	/* TRDIOA1 pin select bit */
#define		trdiob1sel0		trdpsr1_addr.bit.b2	/* TRDIOB1 pin select bit */
#define		trdioc1sel0		trdpsr1_addr.bit.b4	/* TRDIOC1 pin select bit */
#define		trdiod1sel0		trdpsr1_addr.bit.b6	/* TRDIOD1 pin select bit */

/*------------------------------------------------------
  Timer Pin Select Register
------------------------------------------------------*/
union	byte_def	timsr_addr;
#define		timsr		timsr_addr.byte

#define		treosel0		timsr_addr.bit.b0	/* TREO pin select bit */
#define		trfisel0		timsr_addr.bit.b2	/* TRFI pin select bit */
#define		trgioasel		timsr_addr.bit.b4	/* TRGIOA pin select bit */
#define		trgiobsel		timsr_addr.bit.b5	/* TRGIOB pin select bit */
#define		trgclkasel		timsr_addr.bit.b6	/* TRGCLKA pin select bit */
#define		trgclkbsel		timsr_addr.bit.b7	/* TRGCLKB pin select bit */

/*------------------------------------------------------
  Timer RF Output Control Register
------------------------------------------------------*/
union	byte_def	trfout_addr;
#define		trfout		trfout_addr.byte

#define		trfout0			trfout_addr.bit.b0	/* TRFO00 output enable bit */
#define		trfout1			trfout_addr.bit.b1	/* TRFO01 output enable bit */
#define		trfout2			trfout_addr.bit.b2	/* TRFO02 output enable bit */
#define		trfout3			trfout_addr.bit.b3	/* TRFO10 output enable bit */
#define		trfout4			trfout_addr.bit.b4	/* TRFO11 output enable bit */
#define		trfout5			trfout_addr.bit.b5	/* TRFO12 output enable bit */
#define		trfout6			trfout_addr.bit.b6	/* TRFO00 to TRFO02 output invert bit */
#define		trfout7			trfout_addr.bit.b7	/* TRFO10 to TRFO12 output invert bit */

/*------------------------------------------------------
  UART0 Pin Select Register
------------------------------------------------------*/
union	byte_def	u0sr_addr;
#define		u0sr		u0sr_addr.byte

#define		txd0sel0		u0sr_addr.bit.b0	/* TXD0 pin select bit */
#define		rxd0sel0		u0sr_addr.bit.b2	/* RXD0 pin select bit */
#define		clk0sel0		u0sr_addr.bit.b4	/* CLK0 pin select bit */

/*------------------------------------------------------
  UART1 Pin Select Register
------------------------------------------------------*/
union	byte_def	u1sr_addr;
#define		u1sr		u1sr_addr.byte

#define		txd1sel0		u1sr_addr.bit.b0	/* TXD1 pin select bit */
#define		txd1sel1		u1sr_addr.bit.b1	/* TXD1 pin select bit */
#define		rxd1sel0		u1sr_addr.bit.b2	/* RXD1 pin select bit */
#define		rxd1sel1		u1sr_addr.bit.b3	/* RXD1 pin select bit */
#define		clk1sel0		u1sr_addr.bit.b4	/* CLK1 pin select bit */
#define		clk1sel1		u1sr_addr.bit.b5	/* CLK1 pin select bit */

/*------------------------------------------------------
  UART2 Pin Select Register 0
------------------------------------------------------*/
union	byte_def	u2sr0_addr;
#define		u2sr0		u2sr0_addr.byte

#define		txd2sel0		u2sr0_addr.bit.b0	/* TXD2/SDA2 pin select bit */
#define		txd2sel1		u2sr0_addr.bit.b1	/* TXD2/SDA2 pin select bit */
#define		txd2sel2		u2sr0_addr.bit.b2	/* TXD2/SDA2 pin select bit */
#define		rxd2sel0		u2sr0_addr.bit.b4	/* RXD2/SCL2 pin select bit */
#define		rxd2sel1		u2sr0_addr.bit.b5	/* RXD2/SCL2 pin select bit */

/*------------------------------------------------------
  UART2 Pin Select Register 1
------------------------------------------------------*/
union	byte_def	u2sr1_addr;
#define		u2sr1		u2sr1_addr.byte

#define		clk2sel0		u2sr1_addr.bit.b0	/* CLK2 pin select bit */
#define		clk2sel1		u2sr1_addr.bit.b1	/* CLK2 pin select bit */
#define		cts2sel0		u2sr1_addr.bit.b4	/* CTS2/RTS2 pin select bit */

/*------------------------------------------------------
  SSU/IIC Pin Select Register
------------------------------------------------------*/
union	byte_def	ssuiicsr_addr;
#define		ssuiicsr	ssuiicsr_addr.byte

#define		iicsel			ssuiicsr_addr.bit.b0	/* SSU/I2C bus switch bit */
/*------------------------------------------------------
  INT Interrupt Input Pin Select Register
------------------------------------------------------*/
union	byte_def	intsr_addr;
#define		intsr		intsr_addr.byte

#define		int1sel0		intsr_addr.bit.b1	/* INT1 pin select bit */
#define		int1sel1		intsr_addr.bit.b2	/* INT1 pin select bit */
#define		int1sel2		intsr_addr.bit.b3	/* INT1 pin select bit */
#define		int2sel0		intsr_addr.bit.b4	/* INT2 pin select bit */
#define		int3sel0		intsr_addr.bit.b6	/* INT3 pin select bit */
#define		int3sel1		intsr_addr.bit.b7	/* INT3 pin select bit */

/*------------------------------------------------------
  I/O Function Pin Select Register
------------------------------------------------------*/
union	byte_def	pinsr_addr;
#define		pinsr		pinsr_addr.byte

#define		xcsel			pinsr_addr.bit.b0	/* XCIN/XCOUT pin connect bit */
#define		ioinsel			pinsr_addr.bit.b3	/* I/O port input function select bit */
#define		iictctwi		pinsr_addr.bit.b4	/* I2C double transfer rate select bit */
#define		iictchalf		pinsr_addr.bit.b5	/* I2C half transfer rate select bit */
#define		sdadly0			pinsr_addr.bit.b6	/* SDA digital delay select bit */
#define		sdadly1			pinsr_addr.bit.b7	/* SDA digital delay select bit */

/*------------------------------------------------------
  SS Bit Counter Register
------------------------------------------------------*/
union	byte_def	ssbr_addr;
#define		ssbr		ssbr_addr.byte

#define		bs0				ssbr_addr.bit.b0	/* SSU data transfer length set bit */
#define		bs1				ssbr_addr.bit.b1	/* SSU data transfer length set bit */
#define		bs2				ssbr_addr.bit.b2	/* SSU data transfer length set bit */
#define		bs3				ssbr_addr.bit.b3	/* SSU data transfer length set bit */

/*------------------------------------------------------
  IIC bus Transmit Data Register
------------------------------------------------------*/
union	byte_def	icdrt_addr;
#define		icdrt		icdrt_addr.byte

/*------------------------------------------------------
  IIC bus Receive Data Register
------------------------------------------------------*/
union	byte_def	icdrr_addr;
#define		icdrr		icdrr_addr.byte

/*------------------------------------------------------
  SS Control Register H
------------------------------------------------------*/
union	byte_def	sscrh_addr;
#define		sscrh		sscrh_addr.byte

#define		cks0_sscrh		sscrh_addr.bit.b0	/* Transfer clock select bit */
#define		cks1_sscrh		sscrh_addr.bit.b1	/* Transfer clock select bit */
#define		cks2_sscrh		sscrh_addr.bit.b2	/* Transfer clock select bit */
#define		mss_sscrh		sscrh_addr.bit.b5	/* Master/slave device select bit */
#define		rsstp_sscrh		sscrh_addr.bit.b6	/* Receive single stop bit */

/*------------------------------------------------------
  IIC bus Control Register 1
------------------------------------------------------*/
union	byte_def	iccr1_addr;
#define		iccr1		iccr1_addr.byte

#define		cks0_iccr1		iccr1_addr.bit.b0	/* Transmit clock select bits */
#define		cks1_iccr1		iccr1_addr.bit.b1	/* Transmit clock select bits */
#define		cks2_iccr1		iccr1_addr.bit.b2	/* Transmit clock select bits */
#define		cks3_iccr1		iccr1_addr.bit.b3	/* Transmit clock select bits */
#define		trs_iccr1		iccr1_addr.bit.b4	/* Transfer/receive select bit */
#define		mst_iccr1		iccr1_addr.bit.b5	/* Master/slave select bit */
#define		rcvd_iccr1		iccr1_addr.bit.b6	/* Receive disable bit */
#define		ice_iccr1		iccr1_addr.bit.b7	/* I2C bus interface enable bit */

/*------------------------------------------------------
  SS Control Register L
------------------------------------------------------*/
union	byte_def	sscrl_addr;
#define		sscrl		sscrl_addr.byte

#define		sres_sscrl		sscrl_addr.bit.b1	/* SSU control unit reset bit */
#define		solp_sscrl		sscrl_addr.bit.b4	/* SOL write protect bit */
#define		sol_sscrl		sscrl_addr.bit.b5	/* Serial data output value setting bit */

/*------------------------------------------------------
  IIC bus Control Register 2
------------------------------------------------------*/
union	byte_def	iccr2_addr;
#define		iccr2		iccr2_addr.byte

#define		iicrst_iccr2	iccr2_addr.bit.b1	/* I2C bus control block reset bit */
#define		sclo_iccr2		iccr2_addr.bit.b3	/* SCL monitor flag */
#define		sdaop_iccr2		iccr2_addr.bit.b4	/* SDAO write protect bit */
#define		sdao_iccr2		iccr2_addr.bit.b5	/* SDA output value control bit */
#define		scp_iccr2		iccr2_addr.bit.b6	/* Start/stop condition generation disable bit */
#define		bbsy_iccr2		iccr2_addr.bit.b7	/* Bus busy bit */

/*------------------------------------------------------
  SS Mode Register
------------------------------------------------------*/
union	byte_def	ssmr_addr;
#define		ssmr		ssmr_addr.byte

#define		bc0_ssmr		ssmr_addr.bit.b0	/* Bits counter 3 to 0 */
#define		bc1_ssmr		ssmr_addr.bit.b1	/* Bits counter 3 to 0 */
#define		bc2_ssmr		ssmr_addr.bit.b2	/* Bits counter 3 to 0 */
#define		bc3_ssmr		ssmr_addr.bit.b3	/* Bits counter 3 to 0 */
#define		cphs_ssmr		ssmr_addr.bit.b5	/* SSCK clock phase select bit */
#define		cpos_ssmr		ssmr_addr.bit.b6	/* SSCK clock polarity select bit */
#define		mls_ssmr		ssmr_addr.bit.b7	/* MSB first/LSB first select bit */

/*------------------------------------------------------
  IIC bus Mode Register
------------------------------------------------------*/
union	byte_def	icmr_addr;
#define		icmr		icmr_addr.byte

#define		bc0_icmr		icmr_addr.bit.b0	/* Bit counters 2 to 0 */
#define		bc1_icmr		icmr_addr.bit.b1	/* Bit counters 2 to 0 */
#define		bc2_icmr		icmr_addr.bit.b2	/* Bit counters 2 to 0 */
#define		bcwp_icmr		icmr_addr.bit.b3	/* BC write protect bit */
#define		wait_icmr		icmr_addr.bit.b6	/* Wait insertion bit */
#define		mls_icmr		icmr_addr.bit.b7	/* MSB-first/LSB-first select bit */

/*------------------------------------------------------
  SS Enable Register
------------------------------------------------------*/
union	byte_def	sser_addr;
#define		sser		sser_addr.byte

#define		ceie_sser		sser_addr.bit.b0	/* Conflict error interrupt enable bit */
#define		re_sser			sser_addr.bit.b3	/* Receive enable bit */
#define		te_sser			sser_addr.bit.b4	/* Transmit enable bit */
#define		rie_sser		sser_addr.bit.b5	/* Receive interrupt enable bit */
#define		teie_sser		sser_addr.bit.b6	/* Transmit end interrupt enable bit */
#define		tie_sser		sser_addr.bit.b7	/* Transmit interrupt enable bit */

/*------------------------------------------------------
  IIC bus Interrupt Enable Register
------------------------------------------------------*/
union	byte_def	icier_addr;
#define		icier		icier_addr.byte

#define		ackbt_icier		icier_addr.bit.b0	/* Transmit acknowledge select bit */
#define		ackbr_icier		icier_addr.bit.b1	/* Receive acknowledge bit */
#define		acke_icier		icier_addr.bit.b2	/* Acknowledge bit detection select bit */
#define		stie_icier		icier_addr.bit.b3	/* Stop condition detection interrupt enable bit */
#define		nakie_icier		icier_addr.bit.b4	/* NACK receive interrupt enable bit */
#define		rie_icier		icier_addr.bit.b5	/* Receive interrupt enable bit */
#define		teie_icier		icier_addr.bit.b6	/* Transmit end interrupt enable bit */
#define		tie_icier		icier_addr.bit.b7	/* Transmit interrupt enable bit */

/*------------------------------------------------------
  SS Status Register
------------------------------------------------------*/
union	byte_def	sssr_addr;
#define		sssr		sssr_addr.byte

#define		ce_sssr			sssr_addr.bit.b0	/* Conflict error flag */
#define		orer_sssr		sssr_addr.bit.b2	/* Overrun error flag */
#define		rdrf_sssr		sssr_addr.bit.b5	/* Receive data register full flag */
#define		tend_sssr		sssr_addr.bit.b6	/* Transmit end flag */
#define		tdre_sssr		sssr_addr.bit.b7	/* Transmit data empty flag */

/*------------------------------------------------------
  IIC bus Status Register
------------------------------------------------------*/
union	byte_def	icsr_addr;
#define		icsr		icsr_addr.byte

#define		adz_icsr		icsr_addr.bit.b0	/* General call address recognition flag */
#define		aas_icsr		icsr_addr.bit.b1	/* Slave address recognition flag */
#define		al_icsr			icsr_addr.bit.b2	/* Arbitration lost flag/overrun error flag */
#define		stop_icsr		icsr_addr.bit.b3	/* Stop condition detection flag */
#define		nackf_icsr		icsr_addr.bit.b4	/* No acknowledge detection flag */
#define		rdrf_icsr		icsr_addr.bit.b5	/* Receive data register full flag */
#define		tend_icsr		icsr_addr.bit.b6	/* Transmit end flag */
#define		tdre_icsr		icsr_addr.bit.b7	/* Transmit data empty flag */

/*------------------------------------------------------
  SS Mode Register 2
------------------------------------------------------*/
union	byte_def	ssmr2_addr;
#define		ssmr2		ssmr2_addr.byte

#define		ssums_ssmr2		ssmr2_addr.bit.b0	/* SSU mode select bit */
#define		csos_ssmr2		ssmr2_addr.bit.b1	/* SCS pin open drain output select bit */
#define		soos_ssmr2		ssmr2_addr.bit.b2	/* Serial data pin open output drain select bit */
#define		sckos_ssmr2		ssmr2_addr.bit.b3	/* SSCK pin open drain output select bit */
#define		css0_ssmr2		ssmr2_addr.bit.b4	/* SCS pin select bit */
#define		css1_ssmr2		ssmr2_addr.bit.b5	/* SCS pin select bit */
#define		scks_ssmr2		ssmr2_addr.bit.b6	/* SSCK pin select bit */
#define		bide_ssmr2		ssmr2_addr.bit.b7	/* Bidirectional mode enable bit */

/*------------------------------------------------------
  Slave Address Register
------------------------------------------------------*/
union	byte_def	sar_addr;
#define		sar			sar_addr.byte

#define		fs_sar			sar_addr.bit.b0		/* Format select bit */
#define		sva0_sar		sar_addr.bit.b1		/* Slave addresses 6 to 0 */
#define		sva1_sar		sar_addr.bit.b2		/* Slave addresses 6 to 0 */
#define		sva2_sar		sar_addr.bit.b3		/* Slave addresses 6 to 0 */
#define		sva3_sar		sar_addr.bit.b4		/* Slave addresses 6 to 0 */
#define		sva4_sar		sar_addr.bit.b5		/* Slave addresses 6 to 0 */
#define		sva5_sar		sar_addr.bit.b6		/* Slave addresses 6 to 0 */
#define		sva6_sar		sar_addr.bit.b7		/* Slave addresses 6 to 0 */

/*------------------------------------------------------
  Flash Memory Status Register
------------------------------------------------------*/
union	byte_def	fst_addr;
#define		fst			fst_addr.byte

#define		rdysti			fst_addr.bit.b0		/* Flash ready status interrupt request flag */
#define		bsyaei			fst_addr.bit.b1		/* Flash access error interrupt request flag */
#define		lbdata			fst_addr.bit.b2		/* LBDATA monitor flag */
#define		fst4			fst_addr.bit.b4		/* Program error flag */
#define		fst5			fst_addr.bit.b5		/* Erase error/blank check error flag */
#define		fst6			fst_addr.bit.b6		/* Erase-suspend status flag */
#define		fst7			fst_addr.bit.b7		/* Ready/busy status flag */
/*------------------------------------------------------
  Flash Memory Control Register 0
------------------------------------------------------*/
union	byte_def	fmr0_addr;
#define		fmr0		fmr0_addr.byte

#define		fmr01			fmr0_addr.bit.b1	/* CPU rewrite mode select bit */
#define		fmr02			fmr0_addr.bit.b2	/* EW1 mode select bit */
#define		fmstp			fmr0_addr.bit.b3	/* Flash memory stop bit */
#define		cmdrst			fmr0_addr.bit.b4	/* Erase/write sequence reset bit */
#define		cmderie			fmr0_addr.bit.b5	/* Erase/write error interrupt enable bit */
#define		bsyaeie			fmr0_addr.bit.b6	/* Flash access error interrupt enable bit */
#define		rdystie			fmr0_addr.bit.b7	/* Flash ready status interrupt enable bit */

/*------------------------------------------------------
  Flash Memory Control Register 1
------------------------------------------------------*/
union	byte_def	fmr1_addr;
#define		fmr1		fmr1_addr.byte

#define		fmr13			fmr1_addr.bit.b3	/* Lock bit disable select bit */
#define		fmr14			fmr1_addr.bit.b4	/* Data flash block A rewrite disable bit */
#define		fmr15			fmr1_addr.bit.b5	/* Data flash block B rewrite disable bit */
#define		fmr16			fmr1_addr.bit.b6	/* Data flash block C rewrite disable bit */
#define		fmr17			fmr1_addr.bit.b7	/* Data flash block D rewrite disable bit */

/*------------------------------------------------------
  Flash Memory Control Register 2
------------------------------------------------------*/
union	byte_def	fmr2_addr;
#define		fmr2		fmr2_addr.byte

#define		fmr20			fmr2_addr.bit.b0	/* Erase-suspend enable bit */
#define		fmr21			fmr2_addr.bit.b1	/* Erase-suspend request bit */
#define		fmr22			fmr2_addr.bit.b2	/* Interrupt request suspend request enable bit */
#define		fmr27			fmr2_addr.bit.b7	/* Low-current-consumption read mode enable bit */

/*------------------------------------------------------
  Address Match Interrupt Enable Register 0
------------------------------------------------------*/
union	byte_def	aier0_addr;
#define		aier0		aier0_addr.byte

#define		aier00			aier0_addr.bit.b0	/* Address match interrupt 0 enable bit */

/*------------------------------------------------------
  Address Match Interrupt Enable Register 1
------------------------------------------------------*/
union	byte_def	aier1_addr;
#define		aier1		aier1_addr.byte

#define		aier10			aier1_addr.bit.b0	/* Address match interrupt 1 enable bit */

/*------------------------------------------------------
  Pull-Up Control Register 0
------------------------------------------------------*/
union	byte_def	pur0_addr;
#define		pur0		pur0_addr.byte

#define		pu00			pur0_addr.bit.b0	/* P0_0 to P0_3 pull-up */
#define		pu01			pur0_addr.bit.b1	/* P0_4 to P0_7 pull-up */
#define		pu02			pur0_addr.bit.b2	/* P1_0 to P1_3 pull-up */
#define		pu03			pur0_addr.bit.b3	/* P1_4 to P1_7 pull-up */
#define		pu04			pur0_addr.bit.b4	/* P2_0 to P2_3 pull-up */
#define		pu05			pur0_addr.bit.b5	/* P2_4 to P2_7 pull-up */
#define		pu06			pur0_addr.bit.b6	/* P3_0 to P3_3 pull-up */
#define		pu07			pur0_addr.bit.b7	/* P3_4 to P3_7 pull-up */

/*------------------------------------------------------
  Pull-Up Control Register 1
------------------------------------------------------*/
union	byte_def	pur1_addr;
#define		pur1		pur1_addr.byte

#define		pu10			pur1_addr.bit.b0	/* P4_3 pull-up */
#define		pu11			pur1_addr.bit.b1	/* P4_4 to P4_7 pull-up */
#define		pu12			pur1_addr.bit.b2	/* P5_0 to P5_3 pull-up */
#define		pu13			pur1_addr.bit.b3	/* P5_4 to P5_7 pull-up */
#define		pu14			pur1_addr.bit.b4	/* P6_0 to P6_3 pull-up */
#define		pu15			pur1_addr.bit.b5	/* P6_4 to P6_7 pull-up */
#define		pu16			pur1_addr.bit.b6	/* P7_0 to P7_3 pull-up */
#define		pu17			pur1_addr.bit.b7	/* P7_4 to P7_7 pull-up */

/*------------------------------------------------------
  Pull-Up Control Register 2
------------------------------------------------------*/
union	byte_def	pur2_addr;
#define		pur2		pur2_addr.byte

#define		pu20			pur2_addr.bit.b0	/* P8_0 to P8_3 pull-up */
#define		pu21			pur2_addr.bit.b1	/* P8_4 to P8_7 pull-up */
#define		pu22			pur2_addr.bit.b2	/* P9_0 to P9_3 pull-up */
#define		pu23			pur2_addr.bit.b3	/* P9_4 and P9_5 pull-up */
/*------------------------------------------------------
  Port P1 Drive Capacity Control Register
------------------------------------------------------*/
union	byte_def	p1drr_addr;
#define		p1drr		p1drr_addr.byte

#define		p1drr0			p1drr_addr.bit.b0	/* P1_0 drive capacity */
#define		p1drr1			p1drr_addr.bit.b1	/* P1_1 drive capacity */
#define		p1drr2			p1drr_addr.bit.b2	/* P1_2 drive capacity */
#define		p1drr3			p1drr_addr.bit.b3	/* P1_3 drive capacity */
#define		p1drr4			p1drr_addr.bit.b4	/* P1_4 drive capacity */
#define		p1drr5			p1drr_addr.bit.b5	/* P1_5 drive capacity */
#define		p1drr6			p1drr_addr.bit.b6	/* P1_6 drive capacity */
#define		p1drr7			p1drr_addr.bit.b7	/* P1_7 drive capacity */

/*------------------------------------------------------
  Port P2 Drive Capacity Control Register
------------------------------------------------------*/
union	byte_def	p2drr_addr;
#define		p2drr		p2drr_addr.byte

#define		p2drr0			p2drr_addr.bit.b0	/* P2_0 drive capacity */
#define		p2drr1			p2drr_addr.bit.b1	/* P2_1 drive capacity */
#define		p2drr2			p2drr_addr.bit.b2	/* P2_2 drive capacity */
#define		p2drr3			p2drr_addr.bit.b3	/* P2_3 drive capacity */
#define		p2drr4			p2drr_addr.bit.b4	/* P2_4 drive capacity */
#define		p2drr5			p2drr_addr.bit.b5	/* P2_5 drive capacity */
#define		p2drr6			p2drr_addr.bit.b6	/* P2_6 drive capacity */
#define		p2drr7			p2drr_addr.bit.b7	/* P2_7 drive capacity */

/*------------------------------------------------------
  Drive Capacity Control Register 0
------------------------------------------------------*/
union	byte_def	drr0_addr;
#define		drr0		drr0_addr.byte

#define		drr00			drr0_addr.bit.b0	/* P0_0 to P0_3 drive capacity */
#define		drr01			drr0_addr.bit.b1	/* P0_4 to P0_7 drive capacity */
#define		drr06			drr0_addr.bit.b6	/* P3_0 to P3_3 drive capacity */
#define		drr07			drr0_addr.bit.b7	/* P3_4 to P3_7 drive capacity */

/*------------------------------------------------------
  Drive Capacity Control Register 1
------------------------------------------------------*/
union	byte_def	drr1_addr;
#define		drr1		drr1_addr.byte

#define		drr10			drr1_addr.bit.b0	/* P4_3 drive capacity */
#define		drr11			drr1_addr.bit.b1	/* P4_4 to P4_7 drive capacity */
#define		drr12			drr1_addr.bit.b2	/* P5_0 to P5_3 drive capacity */
#define		drr13			drr1_addr.bit.b3	/* P5_4 to P5_7 drive capacity */
#define		drr14			drr1_addr.bit.b4	/* P6_0 to P6_3 drive capacity */
#define		drr15			drr1_addr.bit.b5	/* P6_4 to P6_7 drive capacity */
#define		drr16			drr1_addr.bit.b6	/* P7_0 to P7_3 drive capacity */
#define		drr17			drr1_addr.bit.b7	/* P7_4 to P7_7 drive capacity */

/*------------------------------------------------------
  Drive Capacity Control Register 2
------------------------------------------------------*/
union	byte_def	drr2_addr;
#define		drr2		drr2_addr.byte

#define		drr20			drr2_addr.bit.b0	/* P8_0 to P8_3 drive capacity */
#define		drr21			drr2_addr.bit.b1	/* P8_4 to P8_7 drive capacity */
#define		drr22			drr2_addr.bit.b2	/* P9_0 to P9_3 drive capacity */
#define		drr23			drr2_addr.bit.b3	/* P9_4 and P9_5 drive capacity */

/*------------------------------------------------------
  Input Threshold Control Register 0
------------------------------------------------------*/
union	byte_def	vlt0_addr;
#define		vlt0		vlt0_addr.byte

#define		vlt00			vlt0_addr.bit.b0	/* P0 input level select bit */
#define		vlt01			vlt0_addr.bit.b1	/* P0 input level select bit */
#define		vlt02			vlt0_addr.bit.b2	/* P1 input level select bit */
#define		vlt03			vlt0_addr.bit.b3	/* P1 input level select bit */
#define		vlt04			vlt0_addr.bit.b4	/* P2 input level select bit */
#define		vlt05			vlt0_addr.bit.b5	/* P2 input level select bit */
#define		vlt06			vlt0_addr.bit.b6	/* P3 input level select bit */
#define		vlt07			vlt0_addr.bit.b7	/* P3 input level select bit */

/*------------------------------------------------------
  Input Threshold Control Register 1
------------------------------------------------------*/
union	byte_def	vlt1_addr;
#define		vlt1		vlt1_addr.byte

#define		vlt10			vlt1_addr.bit.b0	/* P4_2 to P4_7 input level select bit */
#define		vlt11			vlt1_addr.bit.b1	/* P4_2 to P4_7 input level select bit */
#define		vlt12			vlt1_addr.bit.b2	/* P5 input level select bit */
#define		vlt13			vlt1_addr.bit.b3	/* P5 input level select bit */
#define		vlt14			vlt1_addr.bit.b4	/* P6 input level select bit */
#define		vlt15			vlt1_addr.bit.b5	/* P6 input level select bit */
#define		vlt16			vlt1_addr.bit.b6	/* P7 input level select bit */
#define		vlt17			vlt1_addr.bit.b7	/* P7 input level select bit */

/*------------------------------------------------------
  Input Threshold Control Register 2
------------------------------------------------------*/
union	byte_def	vlt2_addr;
#define		vlt2		vlt2_addr.byte

#define		vlt20			vlt2_addr.bit.b0	/* P8 input level select bit */
#define		vlt21			vlt2_addr.bit.b1	/* P8 input level select bit */
#define		vlt22			vlt2_addr.bit.b2	/* P9_0 to P9_5 input level select bit */
#define		vlt23			vlt2_addr.bit.b3	/* P9_0 to P9_5 input level select bit */

/*------------------------------------------------------
  Comparator B Control Register 0
------------------------------------------------------*/
union	byte_def	intcmp_addr;
#define		intcmp		intcmp_addr.byte

#define		int1cp0			intcmp_addr.bit.b0	/* Comparator B1 operation enable bit */
#define		int1cout		intcmp_addr.bit.b3	/* Comparator B1 monitor flag */
#define		int3cp0			intcmp_addr.bit.b4	/* Comparator B3 operation enable bit */
#define		int3cout		intcmp_addr.bit.b7	/* Comparator B3 monitor flag */

/*------------------------------------------------------
  External Input Enable Register 0
------------------------------------------------------*/
union	byte_def	inten_addr;
#define		inten		inten_addr.byte

#define		int0en			inten_addr.bit.b0	/* INT0 input enable bit */
#define		int0pl			inten_addr.bit.b1	/* INT0 input polarity select bit */
#define		int1en			inten_addr.bit.b2	/* INT1 input enable bit */
#define		int1pl			inten_addr.bit.b3	/* INT1 input polarity select bit */
#define		int2en			inten_addr.bit.b4	/* INT2 input enable bit */
#define		int2pl			inten_addr.bit.b5	/* INT2 input polarity select bit */
#define		int3en			inten_addr.bit.b6	/* INT3 input enable bit */
#define		int3pl			inten_addr.bit.b7	/* INT3 input polarity select bit */

/*------------------------------------------------------
  External Input Enable Register 1
------------------------------------------------------*/
union	byte_def	inten1_addr;
#define		inten1		inten1_addr.byte

#define		int4en			inten1_addr.bit.b0	/* INT4 input enable bit */
#define		int4pl			inten1_addr.bit.b1	/* INT4 input polarity select bit */

/*------------------------------------------------------
  INT Input Filter Select Register 0
------------------------------------------------------*/
union	byte_def	intf_addr;
#define		intf		intf_addr.byte

#define		int0f0			intf_addr.bit.b0	/* INT0 input filter select bit */
#define		int0f1			intf_addr.bit.b1	/* INT0 input filter select bit */
#define		int1f0			intf_addr.bit.b2	/* INT1 input filter select bit */
#define		int1f1			intf_addr.bit.b3	/* INT1 input filter select bit */
#define		int2f0			intf_addr.bit.b4	/* INT2 input filter select bit */
#define		int2f1			intf_addr.bit.b5	/* INT2 input filter select bit */
#define		int3f0			intf_addr.bit.b6	/* INT3 input filter select bit */
#define		int3f1			intf_addr.bit.b7	/* INT3 input filter select bit */

/*------------------------------------------------------
  INT Input Filter Select Register 1
------------------------------------------------------*/
union	byte_def	intf1_addr;
#define		intf1		intf1_addr.byte

#define		int4f0			intf1_addr.bit.b0	/* INT4 input filter select bit */
#define		int4f1			intf1_addr.bit.b1	/* INT4 input filter select bit */

/*------------------------------------------------------
  Key Input Enable Register 0
------------------------------------------------------*/
union	byte_def	kien_addr;
#define		kien		kien_addr.byte

#define		ki0en			kien_addr.bit.b0	/* KI0 input enable bit */
#define		ki0pl			kien_addr.bit.b1	/* KI0 input polarity select bit */
#define		ki1en			kien_addr.bit.b2	/* KI1 input enable bit */
#define		ki1pl			kien_addr.bit.b3	/* KI1 input polarity select bit */
#define		ki2en			kien_addr.bit.b4	/* KI2 input enable bit */
#define		ki2pl			kien_addr.bit.b5	/* KI2 input polarity select bit */
#define		ki3en			kien_addr.bit.b6	/* KI3 input enable bit */
#define		ki3pl			kien_addr.bit.b7	/* KI3 input polarity select bit */


/*------------------------------------------------------
  DTC Transfer Vector Area0 (INT0) 
------------------------------------------------------*/
union	byte_def	dtcvct0_addr;
#define		dtcvct0		dtcvct0_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area1 (INT1) 
------------------------------------------------------*/
union	byte_def	dtcvct1_addr;
#define		dtcvct1		dtcvct1_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area2 (INT2) 
------------------------------------------------------*/
union	byte_def	dtcvct2_addr;
#define		dtcvct2		dtcvct2_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area3 (INT3) 
------------------------------------------------------*/
union	byte_def	dtcvct3_addr;
#define		dtcvct3		dtcvct3_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area4 (INT4) 
------------------------------------------------------*/
union	byte_def	dtcvct4_addr;
#define		dtcvct4		dtcvct4_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area5
------------------------------------------------------*/
union	byte_def	dtcvct5_addr;
#define		dtcvct5		dtcvct5_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area6
------------------------------------------------------*/
union	byte_def	dtcvct6_addr;
#define		dtcvct6		dtcvct6_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area7
------------------------------------------------------*/
union	byte_def	dtcvct7_addr;
#define		dtcvct7		dtcvct7_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area8 (Key Input) 
------------------------------------------------------*/
union	byte_def	dtcvct8_addr;
#define		dtcvct8		dtcvct8_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area9 (A-D) 
------------------------------------------------------*/
union	byte_def	dtcvct9_addr;
#define		dtcvct9		dtcvct9_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area10 (Uart0 Receive) 
------------------------------------------------------*/
union	byte_def	dtcvct10_addr;
#define		dtcvct10	dtcvct10_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area11 (Uart0 Transfer) 
------------------------------------------------------*/
union	byte_def	dtcvct11_addr;
#define		dtcvct11	dtcvct11_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area12 (Uart1 Receive) 
------------------------------------------------------*/
union	byte_def	dtcvct12_addr;
#define		dtcvct12	dtcvct12_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area13 (Uart1 Transfer) 
------------------------------------------------------*/
union	byte_def	dtcvct13_addr;
#define		dtcvct13	dtcvct13_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area14 (Uart2 Receive) 
------------------------------------------------------*/
union	byte_def	dtcvct14_addr;
#define		dtcvct14	dtcvct14_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area15 (Uart2 Transfer) 
------------------------------------------------------*/
union	byte_def	dtcvct15_addr;
#define		dtcvct15	dtcvct15_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area16 (SSU/I2C Receive data register full) 
------------------------------------------------------*/
union	byte_def	dtcvct16_addr;
#define		dtcvct16	dtcvct16_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area17 (SSU/I2C Transmit data empty) 
------------------------------------------------------*/
union	byte_def	dtcvct17_addr;
#define		dtcvct17	dtcvct17_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area18 (Voltage monitor circuit 2 / comparator A2) 
------------------------------------------------------*/
union	byte_def	dtcvct18_addr;
#define		dtcvct18	dtcvct18_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area19 (Voltage monitor circuit 1 / comparator A1) 
------------------------------------------------------*/
union	byte_def	dtcvct19_addr;
#define		dtcvct19	dtcvct19_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area20
------------------------------------------------------*/
union	byte_def	dtcvct20_addr;
#define		dtcvct20	dtcvct20_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area21
------------------------------------------------------*/
union	byte_def	dtcvct21_addr;
#define		dtcvct21	dtcvct21_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area22 (TRC Input capture / compare match A) 
------------------------------------------------------*/
union	byte_def	dtcvct22_addr;
#define		dtcvct22	dtcvct22_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area23 (TRC Input capture / compare match B) 
------------------------------------------------------*/
union	byte_def	dtcvct23_addr;
#define		dtcvct23	dtcvct23_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area24 (TRC Input capture / compare match C) 
------------------------------------------------------*/
union	byte_def	dtcvct24_addr;
#define		dtcvct24	dtcvct24_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area25 (TRC Input capture / compare match D) 
------------------------------------------------------*/
union	byte_def	dtcvct25_addr;
#define		dtcvct25	dtcvct25_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area26 (TRD0 Input capture / compare match A) 
------------------------------------------------------*/
union	byte_def	dtcvct26_addr;
#define		dtcvct26	dtcvct26_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area27 (TRD0 Input capture / compare match B) 
------------------------------------------------------*/
union	byte_def	dtcvct27_addr;
#define		dtcvct27	dtcvct27_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area28 (TRD0 Input capture / compare match C) 
------------------------------------------------------*/
union	byte_def	dtcvct28_addr;
#define		dtcvct28	dtcvct28_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area29 (TRD0 Input capture / compare match D) 
------------------------------------------------------*/
union	byte_def	dtcvct29_addr;
#define		dtcvct29	dtcvct29_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area30 (TRD1 Input capture / compare match A) 
------------------------------------------------------*/
union	byte_def	dtcvct30_addr;
#define		dtcvct30	dtcvct30_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area31 (TRD1 Input capture / compare match B) 
------------------------------------------------------*/
union	byte_def	dtcvct31_addr;
#define		dtcvct31	dtcvct31_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area32 (TRD1 Input capture / compare match C) 
------------------------------------------------------*/
union	byte_def	dtcvct32_addr;
#define		dtcvct32	dtcvct32_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area33 (TRD1 Input capture / compare match D) 
------------------------------------------------------*/
union	byte_def	dtcvct33_addr;
#define		dtcvct33	dtcvct33_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area34
------------------------------------------------------*/
union	byte_def	dtcvct34_addr;
#define		dtcvct34	dtcvct34_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area35
------------------------------------------------------*/
union	byte_def	dtcvct35_addr;
#define		dtcvct35	dtcvct35_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area36
------------------------------------------------------*/
union	byte_def	dtcvct36_addr;
#define		dtcvct36	dtcvct36_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area37
------------------------------------------------------*/
union	byte_def	dtcvct37_addr;
#define		dtcvct37	dtcvct37_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area38
------------------------------------------------------*/
union	byte_def	dtcvct38_addr;
#define		dtcvct38	dtcvct38_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area39
------------------------------------------------------*/
union	byte_def	dtcvct39_addr;
#define		dtcvct39	dtcvct39_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area40
------------------------------------------------------*/
union	byte_def	dtcvct40_addr;
#define		dtcvct40	dtcvct40_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area41
------------------------------------------------------*/
union	byte_def	dtcvct41_addr;
#define		dtcvct41	dtcvct41_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area42 (TRE) 
------------------------------------------------------*/
union	byte_def	dtcvct42_addr;
#define		dtcvct42	dtcvct42_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area43 (TRF) 
------------------------------------------------------*/
union	byte_def	dtcvct43_addr;
#define		dtcvct43	dtcvct43_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area44 (TRF Compare match 0) 
------------------------------------------------------*/
union	byte_def	dtcvct44_addr;
#define		dtcvct44	dtcvct44_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area45 (TRF Compare match 1) 
------------------------------------------------------*/
union	byte_def	dtcvct45_addr;
#define		dtcvct45	dtcvct45_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area46 (TRF Input capture) 
------------------------------------------------------*/
union	byte_def	dtcvct46_addr;
#define		dtcvct46	dtcvct46_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area47 (TRG Input capture / compare match A) 
------------------------------------------------------*/
union	byte_def	dtcvct47_addr;
#define		dtcvct47	dtcvct47_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area48 (TRG Input capture / compare match B) 
------------------------------------------------------*/
union	byte_def	dtcvct48_addr;
#define		dtcvct48	dtcvct48_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area49 (TRA) 
------------------------------------------------------*/
union	byte_def	dtcvct49_addr;
#define		dtcvct49	dtcvct49_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area50
------------------------------------------------------*/
union	byte_def	dtcvct50_addr;
#define		dtcvct50	dtcvct50_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area51 (TRB) 
------------------------------------------------------*/
union	byte_def	dtcvct51_addr;
#define		dtcvct51	dtcvct51_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area52 (Flash memory ready) 
------------------------------------------------------*/
union	byte_def	dtcvct52_addr;
#define		dtcvct52	dtcvct52_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area53
------------------------------------------------------*/
union	byte_def	dtcvct53_addr;
#define		dtcvct53	dtcvct53_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area54
------------------------------------------------------*/
union	byte_def	dtcvct54_addr;
#define		dtcvct54	dtcvct54_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area55
------------------------------------------------------*/
union	byte_def	dtcvct55_addr;
#define		dtcvct55	dtcvct55_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area56
------------------------------------------------------*/
union	byte_def	dtcvct56_addr;
#define		dtcvct56	dtcvct56_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area57
------------------------------------------------------*/
union	byte_def	dtcvct57_addr;
#define		dtcvct57	dtcvct57_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area58
------------------------------------------------------*/
union	byte_def	dtcvct58_addr;
#define		dtcvct58	dtcvct58_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area59
------------------------------------------------------*/
union	byte_def	dtcvct59_addr;
#define		dtcvct59	dtcvct59_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area60
------------------------------------------------------*/
union	byte_def	dtcvct60_addr;
#define		dtcvct60	dtcvct60_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area61
------------------------------------------------------*/
union	byte_def	dtcvct61_addr;
#define		dtcvct61	dtcvct61_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area62
------------------------------------------------------*/
union	byte_def	dtcvct62_addr;
#define		dtcvct62	dtcvct62_addr.byte

/*------------------------------------------------------
  DTC Transfer Vector Area63
------------------------------------------------------*/
union	byte_def	dtcvct63_addr;
#define		dtcvct63	dtcvct63_addr.byte

/*------------------------------------------------------
  DTC Control Register 0
------------------------------------------------------*/
union	byte_def	dtccr0_addr;
#define		dtccr0		dtccr0_addr.byte

#define		mode_dtccr0		dtccr0_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr0	dtccr0_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr0	dtccr0_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr0	dtccr0_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr0		dtccr0_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr0	dtccr0_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 0
------------------------------------------------------*/
union	byte_def	dtbls0_addr;
#define		dtbls0		dtbls0_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 0
------------------------------------------------------*/
union	byte_def	dtcct0_addr;
#define		dtcct0		dtcct0_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 0
------------------------------------------------------*/
union	byte_def	dtrld0_addr;
#define		dtrld0		dtrld0_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 0
------------------------------------------------------*/
unsigned	short	dtsar0_addr;
#define		dtsar0		dtsar0_addr

/*------------------------------------------------------
  DTC Destination Register 0
------------------------------------------------------*/
unsigned	short	dtdar0_addr;
#define		dtdar0		dtdar0_addr

/*------------------------------------------------------
  DTC Control Register 1
------------------------------------------------------*/
union	byte_def	dtccr1_addr;
#define		dtccr1		dtccr1_addr.byte

#define		mode_dtccr1		dtccr1_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr1	dtccr1_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr1	dtccr1_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr1	dtccr1_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr1		dtccr1_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr1	dtccr1_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 1
------------------------------------------------------*/
union	byte_def	dtbls1_addr;
#define		dtbls1		dtbls1_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 1
------------------------------------------------------*/
union	byte_def	dtcct1_addr;
#define		dtcct1		dtcct1_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 1
------------------------------------------------------*/
union	byte_def	dtrld1_addr;
#define		dtrld1		dtrld1_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 1
------------------------------------------------------*/
unsigned	short	dtsar1_addr;
#define		dtsar1		dtsar1_addr

/*------------------------------------------------------
  DTC Destination Register 1
------------------------------------------------------*/
unsigned	short	dtdar1_addr;
#define		dtdar1		dtdar1_addr

/*------------------------------------------------------
  DTC Control Register 2
------------------------------------------------------*/
union	byte_def	dtccr2_addr;
#define		dtccr2		dtccr2_addr.byte

#define		mode_dtccr2		dtccr2_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr2	dtccr2_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr2	dtccr2_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr2	dtccr2_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr2		dtccr2_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr2	dtccr2_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 2
------------------------------------------------------*/
union	byte_def	dtbls2_addr;
#define		dtbls2		dtbls2_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 2
------------------------------------------------------*/
union	byte_def	dtcct2_addr;
#define		dtcct2		dtcct2_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 2
------------------------------------------------------*/
union	byte_def	dtrld2_addr;
#define		dtrld2		dtrld2_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 2
------------------------------------------------------*/
unsigned	short	dtsar2_addr;
#define		dtsar2		dtsar2_addr

/*------------------------------------------------------
  DTC Destination Register 2
------------------------------------------------------*/
unsigned	short	dtdar2_addr;
#define		dtdar2		dtdar2_addr

/*------------------------------------------------------
  DTC Control Register 3
------------------------------------------------------*/
union	byte_def	dtccr3_addr;
#define		dtccr3		dtccr3_addr.byte

#define		mode_dtccr3		dtccr3_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr3	dtccr3_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr3	dtccr3_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr3	dtccr3_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr3		dtccr3_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr3	dtccr3_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 3
------------------------------------------------------*/
union	byte_def	dtbls3_addr;
#define		dtbls3		dtbls3_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 3
------------------------------------------------------*/
union	byte_def	dtcct3_addr;
#define		dtcct3		dtcct3_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 3
------------------------------------------------------*/
union	byte_def	dtrld3_addr;
#define		dtrld3		dtrld3_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 3
------------------------------------------------------*/
unsigned	short	dtsar3_addr;
#define		dtsar3		dtsar3_addr

/*------------------------------------------------------
  DTC Destination Register 3
------------------------------------------------------*/
unsigned	short	dtdar3_addr;
#define		dtdar3		dtdar3_addr

/*------------------------------------------------------
  DTC Control Register 4
------------------------------------------------------*/
union	byte_def	dtccr4_addr;
#define		dtccr4		dtccr4_addr.byte

#define		mode_dtccr4		dtccr4_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr4	dtccr4_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr4	dtccr4_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr4	dtccr4_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr4		dtccr4_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr4	dtccr4_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 4
------------------------------------------------------*/
union	byte_def	dtbls4_addr;
#define		dtbls4		dtbls4_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 4
------------------------------------------------------*/
union	byte_def	dtcct4_addr;
#define		dtcct4		dtcct4_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 4
------------------------------------------------------*/
union	byte_def	dtrld4_addr;
#define		dtrld4		dtrld4_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 4
------------------------------------------------------*/
unsigned	short	dtsar4_addr;
#define		dtsar4		dtsar4_addr

/*------------------------------------------------------
  DTC Destination Register 4
------------------------------------------------------*/
unsigned	short	dtdar4_addr;
#define		dtdar4		dtdar4_addr

/*------------------------------------------------------
  DTC Control Register 5
------------------------------------------------------*/
union	byte_def	dtccr5_addr;
#define		dtccr5		dtccr5_addr.byte

#define		mode_dtccr5		dtccr5_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr5	dtccr5_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr5	dtccr5_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr5	dtccr5_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr5		dtccr5_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr5	dtccr5_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 5
------------------------------------------------------*/
union	byte_def	dtbls5_addr;
#define		dtbls5		dtbls5_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 5
------------------------------------------------------*/
union	byte_def	dtcct5_addr;
#define		dtcct5		dtcct5_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 5
------------------------------------------------------*/
union	byte_def	dtrld5_addr;
#define		dtrld5		dtrld5_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 5
------------------------------------------------------*/
unsigned	short	dtsar5_addr;
#define		dtsar5		dtsar5_addr

/*------------------------------------------------------
  DTC Destination Register 5
------------------------------------------------------*/
unsigned	short	dtdar5_addr;
#define		dtdar5		dtdar5_addr

/*------------------------------------------------------
  DTC Control Register 6
------------------------------------------------------*/
union	byte_def	dtccr6_addr;
#define		dtccr6		dtccr6_addr.byte

#define		mode_dtccr6		dtccr6_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr6	dtccr6_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr6	dtccr6_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr6	dtccr6_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr6		dtccr6_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr6	dtccr6_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 6
------------------------------------------------------*/
union	byte_def	dtbls6_addr;
#define		dtbls6		dtbls6_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 6
------------------------------------------------------*/
union	byte_def	dtcct6_addr;
#define		dtcct6		dtcct6_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 6
------------------------------------------------------*/
union	byte_def	dtrld6_addr;
#define		dtrld6		dtrld6_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 6
------------------------------------------------------*/
unsigned	short	dtsar6_addr;
#define		dtsar6		dtsar6_addr

/*------------------------------------------------------
  DTC Destination Register 6
------------------------------------------------------*/
unsigned	short	dtdar6_addr;
#define		dtdar6		dtdar6_addr

/*------------------------------------------------------
  DTC Control Register 7
------------------------------------------------------*/
union	byte_def	dtccr7_addr;
#define		dtccr7		dtccr7_addr.byte

#define		mode_dtccr7		dtccr7_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr7	dtccr7_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr7	dtccr7_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr7	dtccr7_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr7		dtccr7_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr7	dtccr7_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 7
------------------------------------------------------*/
union	byte_def	dtbls7_addr;
#define		dtbls7		dtbls7_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 7
------------------------------------------------------*/
union	byte_def	dtcct7_addr;
#define		dtcct7		dtcct7_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 7
------------------------------------------------------*/
union	byte_def	dtrld7_addr;
#define		dtrld7		dtrld7_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 7
------------------------------------------------------*/
unsigned	short	dtsar7_addr;
#define		dtsar7		dtsar7_addr

/*------------------------------------------------------
  DTC Destination Register 7
------------------------------------------------------*/
unsigned	short	dtdar7_addr;
#define		dtdar7		dtdar7_addr

/*------------------------------------------------------
  DTC Control Register 8
------------------------------------------------------*/
union	byte_def	dtccr8_addr;
#define		dtccr8		dtccr8_addr.byte

#define		mode_dtccr8		dtccr8_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr8	dtccr8_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr8	dtccr8_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr8	dtccr8_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr8		dtccr8_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr8	dtccr8_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 8
------------------------------------------------------*/
union	byte_def	dtbls8_addr;
#define		dtbls8		dtbls8_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 8
------------------------------------------------------*/
union	byte_def	dtcct8_addr;
#define		dtcct8		dtcct8_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 8
------------------------------------------------------*/
union	byte_def	dtrld8_addr;
#define		dtrld8		dtrld8_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 8
------------------------------------------------------*/
unsigned	short	dtsar8_addr;
#define		dtsar8		dtsar8_addr

/*------------------------------------------------------
  DTC Destination Register 8
------------------------------------------------------*/
unsigned	short	dtdar8_addr;
#define		dtdar8		dtdar8_addr

/*------------------------------------------------------
  DTC Control Register 9
------------------------------------------------------*/
union	byte_def	dtccr9_addr;
#define		dtccr9		dtccr9_addr.byte

#define		mode_dtccr9		dtccr9_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr9	dtccr9_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr9	dtccr9_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr9	dtccr9_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr9		dtccr9_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr9	dtccr9_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 9
------------------------------------------------------*/
union	byte_def	dtbls9_addr;
#define		dtbls9		dtbls9_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 9
------------------------------------------------------*/
union	byte_def	dtcct9_addr;
#define		dtcct9		dtcct9_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 9
------------------------------------------------------*/
union	byte_def	dtrld9_addr;
#define		dtrld9		dtrld9_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 9
------------------------------------------------------*/
unsigned	short	dtsar9_addr;
#define		dtsar9		dtsar9_addr

/*------------------------------------------------------
  DTC Destination Register 9
------------------------------------------------------*/
unsigned	short	dtdar9_addr;
#define		dtdar9		dtdar9_addr

/*------------------------------------------------------
  DTC Control Register 10
------------------------------------------------------*/
union	byte_def	dtccr10_addr;
#define		dtccr10		dtccr10_addr.byte

#define		mode_dtccr10	dtccr10_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr10	dtccr10_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr10	dtccr10_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr10	dtccr10_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr10	dtccr10_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr10	dtccr10_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 10
------------------------------------------------------*/
union	byte_def	dtbls10_addr;
#define		dtbls10		dtbls10_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 10
------------------------------------------------------*/
union	byte_def	dtcct10_addr;
#define		dtcct10		dtcct10_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 10
------------------------------------------------------*/
union	byte_def	dtrld10_addr;
#define		dtrld10		dtrld10_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 10
------------------------------------------------------*/
unsigned	short	dtsar10_addr;
#define		dtsar10		dtsar10_addr

/*------------------------------------------------------
  DTC Destination Register 10
------------------------------------------------------*/
unsigned	short	dtdar10_addr;
#define		dtdar10		dtdar10_addr

/*------------------------------------------------------
  DTC Control Register 11
------------------------------------------------------*/
union	byte_def	dtccr11_addr;
#define		dtccr11		dtccr11_addr.byte

#define		mode_dtccr11	dtccr11_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr11	dtccr11_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr11	dtccr11_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr11	dtccr11_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr11	dtccr11_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr11	dtccr11_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 11
------------------------------------------------------*/
union	byte_def	dtbls11_addr;
#define		dtbls11		dtbls11_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 11
------------------------------------------------------*/
union	byte_def	dtcct11_addr;
#define		dtcct11		dtcct11_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 11
------------------------------------------------------*/
union	byte_def	dtrld11_addr;
#define		dtrld11		dtrld11_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 11
------------------------------------------------------*/
unsigned	short	dtsar11_addr;
#define		dtsar11		dtsar11_addr

/*------------------------------------------------------
  DTC Destination Register 11
------------------------------------------------------*/
unsigned	short	dtdar11_addr;
#define		dtdar11		dtdar11_addr

/*------------------------------------------------------
  DTC Control Register 12
------------------------------------------------------*/
union	byte_def	dtccr12_addr;
#define		dtccr12		dtccr12_addr.byte

#define		mode_dtccr12	dtccr12_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr12	dtccr12_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr12	dtccr12_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr12	dtccr12_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr12	dtccr12_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr12	dtccr12_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 12
------------------------------------------------------*/
union	byte_def	dtbls12_addr;
#define		dtbls12		dtbls12_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 12
------------------------------------------------------*/
union	byte_def	dtcct12_addr;
#define		dtcct12		dtcct12_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 12
------------------------------------------------------*/
union	byte_def	dtrld12_addr;
#define		dtrld12		dtrld12_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 12
------------------------------------------------------*/
unsigned	short	dtsar12_addr;
#define		dtsar12		dtsar12_addr

/*------------------------------------------------------
  DTC Destination Register 12
------------------------------------------------------*/
unsigned	short	dtdar12_addr;
#define		dtdar12		dtdar12_addr

/*------------------------------------------------------
  DTC Control Register 13
------------------------------------------------------*/
union	byte_def	dtccr13_addr;
#define		dtccr13		dtccr13_addr.byte

#define		mode_dtccr13	dtccr13_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr13	dtccr13_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr13	dtccr13_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr13	dtccr13_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr13	dtccr13_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr13	dtccr13_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 13
------------------------------------------------------*/
union	byte_def	dtbls13_addr;
#define		dtbls13		dtbls13_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 13
------------------------------------------------------*/
union	byte_def	dtcct13_addr;
#define		dtcct13		dtcct13_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 13
------------------------------------------------------*/
union	byte_def	dtrld13_addr;
#define		dtrld13		dtrld13_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 13
------------------------------------------------------*/
unsigned	short	dtsar13_addr;
#define		dtsar13		dtsar13_addr

/*------------------------------------------------------
  DTC Destination Register 13
------------------------------------------------------*/
unsigned	short	dtdar13_addr;
#define		dtdar13		dtdar13_addr

/*------------------------------------------------------
  DTC Control Register 14
------------------------------------------------------*/
union	byte_def	dtccr14_addr;
#define		dtccr14		dtccr14_addr.byte

#define		mode_dtccr14	dtccr14_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr14	dtccr14_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr14	dtccr14_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr14	dtccr14_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr14	dtccr14_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr14	dtccr14_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 14
------------------------------------------------------*/
union	byte_def	dtbls14_addr;
#define		dtbls14		dtbls14_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 14
------------------------------------------------------*/
union	byte_def	dtcct14_addr;
#define		dtcct14		dtcct14_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 14
------------------------------------------------------*/
union	byte_def	dtrld14_addr;
#define		dtrld14		dtrld14_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 14
------------------------------------------------------*/
unsigned	short	dtsar14_addr;
#define		dtsar14		dtsar14_addr

/*------------------------------------------------------
  DTC Destination Register 14
------------------------------------------------------*/
unsigned	short	dtdar14_addr;
#define		dtdar14		dtdar14_addr

/*------------------------------------------------------
  DTC Control Register 15
------------------------------------------------------*/
union	byte_def	dtccr15_addr;
#define		dtccr15		dtccr15_addr.byte

#define		mode_dtccr15	dtccr15_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr15	dtccr15_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr15	dtccr15_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr15	dtccr15_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr15	dtccr15_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr15	dtccr15_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 15
------------------------------------------------------*/
union	byte_def	dtbls15_addr;
#define		dtbls15		dtbls15_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 15
------------------------------------------------------*/
union	byte_def	dtcct15_addr;
#define		dtcct15		dtcct15_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 15
------------------------------------------------------*/
union	byte_def	dtrld15_addr;
#define		dtrld15		dtrld15_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 15
------------------------------------------------------*/
unsigned	short	dtsar15_addr;
#define		dtsar15		dtsar15_addr

/*------------------------------------------------------
  DTC Destination Register 15
------------------------------------------------------*/
unsigned	short	dtdar15_addr;
#define		dtdar15		dtdar15_addr

/*------------------------------------------------------
  DTC Control Register 16
------------------------------------------------------*/
union	byte_def	dtccr16_addr;
#define		dtccr16		dtccr16_addr.byte

#define		mode_dtccr16	dtccr16_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr16	dtccr16_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr16	dtccr16_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr16	dtccr16_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr16	dtccr16_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr16	dtccr16_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 16
------------------------------------------------------*/
union	byte_def	dtbls16_addr;
#define		dtbls16		dtbls16_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 16
------------------------------------------------------*/
union	byte_def	dtcct16_addr;
#define		dtcct16		dtcct16_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 16
------------------------------------------------------*/
union	byte_def	dtrld16_addr;
#define		dtrld16		dtrld16_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 16
------------------------------------------------------*/
unsigned	short	dtsar16_addr;
#define		dtsar16		dtsar16_addr

/*------------------------------------------------------
  DTC Destination Register 16
------------------------------------------------------*/
unsigned	short	dtdar16_addr;
#define		dtdar16		dtdar16_addr

/*------------------------------------------------------
  DTC Control Register 17
------------------------------------------------------*/
union	byte_def	dtccr17_addr;
#define		dtccr17		dtccr17_addr.byte

#define		mode_dtccr17	dtccr17_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr17	dtccr17_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr17	dtccr17_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr17	dtccr17_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr17	dtccr17_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr17	dtccr17_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 17
------------------------------------------------------*/
union	byte_def	dtbls17_addr;
#define		dtbls17		dtbls17_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 17
------------------------------------------------------*/
union	byte_def	dtcct17_addr;
#define		dtcct17		dtcct17_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 17
------------------------------------------------------*/
union	byte_def	dtrld17_addr;
#define		dtrld17		dtrld17_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 17
------------------------------------------------------*/
unsigned	short	dtsar17_addr;
#define		dtsar17		dtsar17_addr

/*------------------------------------------------------
  DTC Destination Register 17
------------------------------------------------------*/
unsigned	short	dtdar17_addr;
#define		dtdar17		dtdar17_addr

/*------------------------------------------------------
  DTC Control Register 18
------------------------------------------------------*/
union	byte_def	dtccr18_addr;
#define		dtccr18		dtccr18_addr.byte

#define		mode_dtccr18	dtccr18_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr18	dtccr18_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr18	dtccr18_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr18	dtccr18_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr18	dtccr18_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr18	dtccr18_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 18
------------------------------------------------------*/
union	byte_def	dtbls18_addr;
#define		dtbls18		dtbls18_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 18
------------------------------------------------------*/
union	byte_def	dtcct18_addr;
#define		dtcct18		dtcct18_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 18
------------------------------------------------------*/
union	byte_def	dtrld18_addr;
#define		dtrld18		dtrld18_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 18
------------------------------------------------------*/
unsigned	short	dtsar18_addr;
#define		dtsar18		dtsar18_addr

/*------------------------------------------------------
  DTC Destination Register 18
------------------------------------------------------*/
unsigned	short	dtdar18_addr;
#define		dtdar18		dtdar18_addr

/*------------------------------------------------------
  DTC Control Register 19
------------------------------------------------------*/
union	byte_def	dtccr19_addr;
#define		dtccr19		dtccr19_addr.byte

#define		mode_dtccr19	dtccr19_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr19	dtccr19_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr19	dtccr19_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr19	dtccr19_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr19	dtccr19_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr19	dtccr19_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 19
------------------------------------------------------*/
union	byte_def	dtbls19_addr;
#define		dtbls19		dtbls19_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 19
------------------------------------------------------*/
union	byte_def	dtcct19_addr;
#define		dtcct19		dtcct19_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 19
------------------------------------------------------*/
union	byte_def	dtrld19_addr;
#define		dtrld19		dtrld19_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 19
------------------------------------------------------*/
unsigned	short	dtsar19_addr;
#define		dtsar19		dtsar19_addr

/*------------------------------------------------------
  DTC Destination Register 19
------------------------------------------------------*/
unsigned	short	dtdar19_addr;
#define		dtdar19		dtdar19_addr

/*------------------------------------------------------
  DTC Control Register 20
------------------------------------------------------*/
union	byte_def	dtccr20_addr;
#define		dtccr20		dtccr20_addr.byte

#define		mode_dtccr20	dtccr20_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr20	dtccr20_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr20	dtccr20_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr20	dtccr20_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr20	dtccr20_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr20	dtccr20_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 20
------------------------------------------------------*/
union	byte_def	dtbls20_addr;
#define		dtbls20		dtbls20_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 20
------------------------------------------------------*/
union	byte_def	dtcct20_addr;
#define		dtcct20		dtcct20_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 20
------------------------------------------------------*/
union	byte_def	dtrld20_addr;
#define		dtrld20		dtrld20_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 20
------------------------------------------------------*/
unsigned	short	dtsar20_addr;
#define		dtsar20		dtsar20_addr

/*------------------------------------------------------
  DTC Destination Register 20
------------------------------------------------------*/
unsigned	short	dtdar20_addr;
#define		dtdar20		dtdar20_addr

/*------------------------------------------------------
  DTC Control Register 21
------------------------------------------------------*/
union	byte_def	dtccr21_addr;
#define		dtccr21		dtccr21_addr.byte

#define		mode_dtccr21	dtccr21_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr21	dtccr21_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr21	dtccr21_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr21	dtccr21_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr21	dtccr21_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr21	dtccr21_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 21
------------------------------------------------------*/
union	byte_def	dtbls21_addr;
#define		dtbls21		dtbls21_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 21
------------------------------------------------------*/
union	byte_def	dtcct21_addr;
#define		dtcct21		dtcct21_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 21
------------------------------------------------------*/
union	byte_def	dtrld21_addr;
#define		dtrld21		dtrld21_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 21
------------------------------------------------------*/
unsigned	short	dtsar21_addr;
#define		dtsar21		dtsar21_addr

/*------------------------------------------------------
  DTC Destination Register 21
------------------------------------------------------*/
unsigned	short	dtdar21_addr;
#define		dtdar21		dtdar21_addr

/*------------------------------------------------------
  DTC Control Register 22
------------------------------------------------------*/
union	byte_def	dtccr22_addr;
#define		dtccr22		dtccr22_addr.byte

#define		mode_dtccr22	dtccr22_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr22	dtccr22_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr22	dtccr22_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr22	dtccr22_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr22	dtccr22_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr22	dtccr22_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 22
------------------------------------------------------*/
union	byte_def	dtbls22_addr;
#define		dtbls22		dtbls22_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 22
------------------------------------------------------*/
union	byte_def	dtcct22_addr;
#define		dtcct22		dtcct22_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 22
------------------------------------------------------*/
union	byte_def	dtrld22_addr;
#define		dtrld22		dtrld22_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 22
------------------------------------------------------*/
unsigned	short	dtsar22_addr;
#define		dtsar22		dtsar22_addr

/*------------------------------------------------------
  DTC Destination Register 22
------------------------------------------------------*/
unsigned	short	dtdar22_addr;
#define		dtdar22		dtdar22_addr

/*------------------------------------------------------
  DTC Control Register 23
------------------------------------------------------*/
union	byte_def	dtccr23_addr;
#define		dtccr23		dtccr23_addr.byte

#define		mode_dtccr23	dtccr23_addr.bit.b0	/* Transfer mode select bit */
#define		rptsel_dtccr23	dtccr23_addr.bit.b1	/* Repeat area select bit */
#define		samod_dtccr23	dtccr23_addr.bit.b2	/* Source address control bit */
#define		damod_dtccr23	dtccr23_addr.bit.b3	/* Destination address control bit */
#define		chne_dtccr23	dtccr23_addr.bit.b4	/* Chain transfer enable bit */
#define		rptint_dtccr23	dtccr23_addr.bit.b5	/* Repeat mode interrupt enable bit */

/*------------------------------------------------------
  DTC Block Size Register 23
------------------------------------------------------*/
union	byte_def	dtbls23_addr;
#define		dtbls23		dtbls23_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Register 23
------------------------------------------------------*/
union	byte_def	dtcct23_addr;
#define		dtcct23		dtcct23_addr.byte

/*------------------------------------------------------
  DTC Transfer Count Reload Register 23
------------------------------------------------------*/
union	byte_def	dtrld23_addr;
#define		dtrld23		dtrld23_addr.byte

/*------------------------------------------------------
  DTC Source Address Register 23
------------------------------------------------------*/
unsigned	short	dtsar23_addr;
#define		dtsar23		dtsar23_addr

/*------------------------------------------------------
  DTC Destination Register 23
------------------------------------------------------*/
unsigned	short	dtdar23_addr;
#define		dtdar23		dtdar23_addr

/*------------------------------------------------------
  Interrupt control register
------------------------------------------------------*/
union{
	struct{
		char	ilvl0:1;						/* Interrupt priority level select bit */
		char	ilvl1:1;						/* Interrupt priority level select bit */
		char	ilvl2:1;						/* Interrupt priority level select bit */
		char	ir:1;							/* Interrupt request bit */
		char	pol:1;							/* Polarity select bit */
		char	b5:1;
		char	b6:1;
		char	b7:1;
	} bit;
	char	byte;
} fmrdyic_addr, int4ic_addr, trcic_addr, trd0ic_addr, trd1ic_addr, treic_addr,
  s2tic_addr, s2ric_addr, kupic_addr, adic_addr, ssuic_addr, iicic_addr, cmp1ic_addr,
  s0tic_addr, s0ric_addr, s1tic_addr, s1ric_addr, int2ic_addr, traic_addr,
  trbic_addr, int1ic_addr, int3ic_addr, trfic_addr, cmp0ic_addr, int0ic_addr, u2bcnic_addr,
  capic_addr, trgic_addr, vcmp1ic_addr, vcmp2ic_addr;

/*------------------------------------------------------
  Flash Memory Ready Interrupt Control Register
------------------------------------------------------*/
#define		fmrdyic		fmrdyic_addr.byte

#define		ilvl0_fmrdyic	fmrdyic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_fmrdyic	fmrdyic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_fmrdyic	fmrdyic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_fmrdyic		fmrdyic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  INT4 Interrupt Control Register
------------------------------------------------------*/
#define		int4ic		int4ic_addr.byte

#define		ilvl0_int4ic	int4ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_int4ic	int4ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_int4ic	int4ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_int4ic		int4ic_addr.bit.ir		/* Interrupt request bit */
#define		pol_int4ic		int4ic_addr.bit.pol		/* Polarity switch bit */

/*------------------------------------------------------
  Timer RC Interrupt Control Register
------------------------------------------------------*/
#define		trcic		trcic_addr.byte

#define		ilvl0_trcic		trcic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trcic		trcic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trcic		trcic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trcic		trcic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RD0 Interrupt Control Register
------------------------------------------------------*/
#define		trd0ic		trd0ic_addr.byte

#define		ilvl0_trd0ic	trd0ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trd0ic	trd0ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trd0ic	trd0ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trd0ic		trd0ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RD1 Interrupt Control Register
------------------------------------------------------*/
#define		trd1ic		trd1ic_addr.byte

#define		ilvl0_trd1ic	trd1ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trd1ic	trd1ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trd1ic	trd1ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trd1ic		trd1ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RE Interrupt Control Register
------------------------------------------------------*/
#define		treic		treic_addr.byte

#define		ilvl0_treic		treic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_treic		treic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_treic		treic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_treic		treic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART2 Transmit Interrupt Control Register
------------------------------------------------------*/
#define		s2tic		s2tic_addr.byte

#define		ilvl0_s2tic		s2tic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s2tic		s2tic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s2tic		s2tic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s2tic		s2tic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART2 Receive Interrupt Control Register
------------------------------------------------------*/
#define		s2ric		s2ric_addr.byte

#define		ilvl0_s2ric		s2ric_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s2ric		s2ric_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s2ric		s2ric_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s2ric		s2ric_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Key Input Interrupt Control Register
------------------------------------------------------*/
#define		kupic		kupic_addr.byte

#define		ilvl0_kupic		kupic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_kupic		kupic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_kupic		kupic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_kupic		kupic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  A/D Conversion Interrupt Control Register
------------------------------------------------------*/
#define		adic		adic_addr.byte

#define		ilvl0_adic		adic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_adic		adic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_adic		adic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_adic			adic_addr.bit.ir	/* Interrupt request bit */

/*------------------------------------------------------
  SSU Interrupt Control Register
------------------------------------------------------*/
#define		ssuic		ssuic_addr.byte

#define		ilvl0_ssuic		ssuic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_ssuic		ssuic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_ssuic		ssuic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_ssuic		ssuic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  IIC bus Interrupt Control Register
------------------------------------------------------*/
#define		iicic		iicic_addr.byte

#define		ilvl0_iicic		iicic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_iicic		iicic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_iicic		iicic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_iicic		iicic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RF Compare 1 Interrupt Control Register
------------------------------------------------------*/
#define		cmp1ic		cmp1ic_addr.byte

#define		ilvl0_cmp1ic	cmp1ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_cmp1ic	cmp1ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_cmp1ic	cmp1ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_cmp1ic		cmp1ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART0 Transmit Interrupt Control Register
------------------------------------------------------*/
#define		s0tic		s0tic_addr.byte

#define		ilvl0_s0tic		s0tic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s0tic		s0tic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s0tic		s0tic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s0tic		s0tic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART0 Receive Interrupt Control Register
------------------------------------------------------*/
#define		s0ric		s0ric_addr.byte

#define		ilvl0_s0ric		s0ric_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s0ric		s0ric_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s0ric		s0ric_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s0ric		s0ric_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART1 Transmit Interrupt Control Register
------------------------------------------------------*/
#define		s1tic		s1tic_addr.byte

#define		ilvl0_s1tic		s1tic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s1tic		s1tic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s1tic		s1tic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s1tic		s1tic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UART1 Receive Interrupt Control Register
------------------------------------------------------*/
#define		s1ric		s1ric_addr.byte

#define		ilvl0_s1ric		s1ric_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_s1ric		s1ric_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_s1ric		s1ric_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_s1ric		s1ric_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  INT2 Interrupt Control Register
------------------------------------------------------*/
#define		int2ic		int2ic_addr.byte

#define		ilvl0_int2ic	int2ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_int2ic	int2ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_int2ic	int2ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_int2ic		int2ic_addr.bit.ir		/* Interrupt request bit */
#define		pol_int2ic		int2ic_addr.bit.pol		/* Polarity switch bit */

/*------------------------------------------------------
  Timer RA Interrupt Control Register
------------------------------------------------------*/
#define		traic		traic_addr.byte

#define		ilvl0_traic		traic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_traic		traic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_traic		traic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_traic		traic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RB Interrupt Control Register
------------------------------------------------------*/
#define		trbic		trbic_addr.byte

#define		ilvl0_trbic		trbic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trbic		trbic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trbic		trbic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trbic		trbic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  INT1 Interrupt Control Register
------------------------------------------------------*/
#define		int1ic		int1ic_addr.byte

#define		ilvl0_int1ic	int1ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_int1ic	int1ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_int1ic	int1ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_int1ic		int1ic_addr.bit.ir		/* Interrupt request bit */
#define		pol_int1ic		int1ic_addr.bit.pol		/* Polarity switch bit */

/*------------------------------------------------------
  INT3 Interrupt Control Register
------------------------------------------------------*/
#define		int3ic		int3ic_addr.byte

#define		ilvl0_int3ic	int3ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_int3ic	int3ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_int3ic	int3ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_int3ic		int3ic_addr.bit.ir		/* Interrupt request bit */
#define		pol_int3ic		int3ic_addr.bit.pol		/* Polarity switch bit */

/*------------------------------------------------------
  Timer RF Interrupt Control Register
------------------------------------------------------*/
#define		trfic		trfic_addr.byte

#define		ilvl0_trfic		trfic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trfic		trfic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trfic		trfic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trfic		trfic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RF Compare 0 Interrupt Control Register
------------------------------------------------------*/
#define		cmp0ic		cmp0ic_addr.byte

#define		ilvl0_cmp0ic	cmp0ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_cmp0ic	cmp0ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_cmp0ic	cmp0ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_cmp0ic		cmp0ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  INT0 Interrupt Control Register
------------------------------------------------------*/
#define		int0ic		int0ic_addr.byte

#define		ilvl0_int0ic	int0ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_int0ic	int0ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_int0ic	int0ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_int0ic		int0ic_addr.bit.ir		/* Interrupt request bit */
#define		pol_int0ic		int0ic_addr.bit.pol		/* Polarity switch bit */

/*------------------------------------------------------
  UART2 Bus Collision Detection Interrupt Control Register
------------------------------------------------------*/
#define		u2bcnic		u2bcnic_addr.byte

#define		ilvl0_u2bcnic	u2bcnic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_u2bcnic	u2bcnic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_u2bcnic	u2bcnic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_u2bcnic		u2bcnic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RF Capture Interrupt Control Register
------------------------------------------------------*/
#define		capic		capic_addr.byte

#define		ilvl0_capic		capic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_capic		capic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_capic		capic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_capic		capic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Timer RG Interrupt Control Register
------------------------------------------------------*/
#define		trgic		trgic_addr.byte

#define		ilvl0_trgic		trgic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_trgic		trgic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_trgic		trgic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_trgic		trgic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Voltage Monitor 1/Compare A1 Interrupt Control Register
------------------------------------------------------*/
#define		vcmp1ic		vcmp1ic_addr.byte

#define		ilvl0_vcmp1ic	vcmp1ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_vcmp1ic	vcmp1ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_vcmp1ic	vcmp1ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_vcmp1ic		vcmp1ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  Voltage Monitor 2/Compare A2 Interrupt Control Register
------------------------------------------------------*/
#define		vcmp2ic		vcmp2ic_addr.byte

#define		ilvl0_vcmp2ic	vcmp2ic_addr.bit.ilvl0	/* Interrupt priority level select bit */
#define		ilvl1_vcmp2ic	vcmp2ic_addr.bit.ilvl1	/* Interrupt priority level select bit */
#define		ilvl2_vcmp2ic	vcmp2ic_addr.bit.ilvl2	/* Interrupt priority level select bit */
#define		ir_vcmp2ic		vcmp2ic_addr.bit.ir		/* Interrupt request bit */

/*------------------------------------------------------
  UARTi transmit/receive mode register
------------------------------------------------------*/
union{
	struct{
		char	smd0:1;							/* Serial I/O mode select bit */
		char	smd1:1;							/* Serial I/O mode select bit */
		char	smd2:1;							/* Serial I/O mode select bit */
		char	ckdir:1;						/* Internal/external clock select bit */
		char	stps:1;							/* Stop bit length select bit */
		char	pry:1;							/* Odd/even parity select bit */
		char	prye:1;							/* Parity enable bit */
		char	b7:1;
	} bit;
	char	byte;
} u0mr_addr, u1mr_addr, u2mr_addr;

/*------------------------------------------------------
  UART0 Transmit/Receive Mode Register
------------------------------------------------------*/
#define		u0mr		u0mr_addr.byte

#define		smd0_u0mr		u0mr_addr.bit.smd0	/* Serial I/O mode select bit */
#define		smd1_u0mr		u0mr_addr.bit.smd1	/* Serial I/O mode select bit */
#define		smd2_u0mr		u0mr_addr.bit.smd2	/* Serial I/O mode select bit */
#define		ckdir_u0mr		u0mr_addr.bit.ckdir	/* Internal/external clock select bit */
#define		stps_u0mr		u0mr_addr.bit.stps	/* Stop bit length select bit */
#define		pry_u0mr		u0mr_addr.bit.pry	/* Odd/even parity select bit */
#define		prye_u0mr		u0mr_addr.bit.prye	/* Parity enable bit */

/*------------------------------------------------------
  UART1 Transmit/Receive Mode Register
------------------------------------------------------*/
#define		u1mr		u1mr_addr.byte

#define		smd0_u1mr		u1mr_addr.bit.smd0	/* Serial I/O mode select bit */
#define		smd1_u1mr		u1mr_addr.bit.smd1	/* Serial I/O mode select bit */
#define		smd2_u1mr		u1mr_addr.bit.smd2	/* Serial I/O mode select bit */
#define		ckdir_u1mr		u1mr_addr.bit.ckdir	/* Internal/external clock select bit */
#define		stps_u1mr		u1mr_addr.bit.stps	/* Stop bit length select bit */
#define		pry_u1mr		u1mr_addr.bit.pry	/* Odd/even parity select bit */
#define		prye_u1mr		u1mr_addr.bit.prye	/* Parity enable bit */

/*------------------------------------------------------
  UART2 Transmit/Receive Mode Register
------------------------------------------------------*/
#define		u2mr		u2mr_addr.byte

#define		smd0_u2mr		u2mr_addr.bit.smd0	/* Serial I/O mode select bit */
#define		smd1_u2mr		u2mr_addr.bit.smd1	/* Serial I/O mode select bit */
#define		smd2_u2mr		u2mr_addr.bit.smd2	/* Serial I/O mode select bit */
#define		ckdir_u2mr		u2mr_addr.bit.ckdir	/* Internal/external clock select bit */
#define		stps_u2mr		u2mr_addr.bit.stps	/* Stop bit length select bit */
#define		pry_u2mr		u2mr_addr.bit.pry	/* Odd/even parity select bit */
#define		prye_u2mr		u2mr_addr.bit.prye	/* Parity enable bit */
#define		iopol_u2mr		u2mr_addr.bit.b7	/* TXD, RXD I/O polarity switch bit */

/*------------------------------------------------------
  UARTi transmit/receive control register0
------------------------------------------------------*/
union{
	struct{
		char	clk0:1;							/* BRG count source select bit */
		char	clk1:1;							/* BRG count source select bit */
		char	crs:1;							/* CTS/RTS function select bit */
		char	txept:1;						/* Transmit register empty flag */
		char	crd:1;							/* CTS/RTS disable bit */
		char	nch:1;							/* Data output select bit */
		char	ckpol:1;						/* CLK polarity select bit */
		char	uform:1;						/* Transfer format select bit */
	} bit;
	char	byte;
} u0c0_addr, u1c0_addr, u2c0_addr;

/*------------------------------------------------------
  UART0 Transmit/Receive Control Register 0
------------------------------------------------------*/
#define		u0c0		u0c0_addr.byte

#define		clk0_u0c0		u0c0_addr.bit.clk0	/* BRG count source select bit */
#define		clk1_u0c0		u0c0_addr.bit.clk1	/* BRG count source select bit */
#define		txept_u0c0		u0c0_addr.bit.txept	/* Transmit register empty flag */
#define		nch_u0c0		u0c0_addr.bit.nch	/* Data output select bit */
#define		ckpol_u0c0		u0c0_addr.bit.ckpol	/* CLK polarity select bit */
#define		uform_u0c0		u0c0_addr.bit.uform	/* Transfer format select bit */

/*------------------------------------------------------
  UART1 Transmit/Receive Control Register 0
------------------------------------------------------*/
#define		u1c0		u1c0_addr.byte

#define		clk0_u1c0		u1c0_addr.bit.clk0	/* BRG count source select bit */
#define		clk1_u1c0		u1c0_addr.bit.clk1	/* BRG count source select bit */
#define		txept_u1c0		u1c0_addr.bit.txept	/* Transmit register empty flag */
#define		nch_u1c0		u1c0_addr.bit.nch	/* Data output select bit */
#define		ckpol_u1c0		u1c0_addr.bit.ckpol	/* CLK polarity select bit */
#define		uform_u1c0		u1c0_addr.bit.uform	/* Transfer format select bit */

/*------------------------------------------------------
  UART2 Transmit/Receive Control Register 0
------------------------------------------------------*/
#define		u2c0		u2c0_addr.byte

#define		clk0_u2c0		u2c0_addr.bit.clk0	/* U2BRG count source select bit */
#define		clk1_u2c0		u2c0_addr.bit.clk1	/* U2BRG count source select bit */
#define		crs_u2c0		u2c0_addr.bit.crs	/* CTS/RTS function select bit */
#define		txept_u2c0		u2c0_addr.bit.txept	/* Transmit register empty flag */
#define		crd_u2c0		u2c0_addr.bit.crd	/* CTS/RTS disable bit */
#define		nch_u2c0		u2c0_addr.bit.nch	/* Data output select bit */
#define		ckpol_u2c0		u2c0_addr.bit.ckpol	/* CLK polarity select bit */
#define		uform_u2c0		u2c0_addr.bit.uform	/* Transfer format select bit */

/*------------------------------------------------------
  UARTi transmit/receive control register1
------------------------------------------------------*/
union{
	struct{
		char	te:1;							/* Transmit enable bit */
		char	ti:1;							/* Transmit buffer empty flag */
		char	re:1;							/* Receive enable bit */
		char	ri:1;							/* Receive complete flag */
		char	irs:1;
		char	rrm:1;
		char	lch:1;
		char	ere:1;
	} bit;
	char	byte;
} u0c1_addr, u1c1_addr, u2c1_addr;

/*------------------------------------------------------
  UART0 Transmit/Receive Control Register 1
------------------------------------------------------*/
#define		u0c1		u0c1_addr.byte

#define		te_u0c1			u0c1_addr.bit.te	/* Transmit enable bit */
#define		ti_u0c1			u0c1_addr.bit.ti	/* Transmit buffer empty flag */
#define		re_u0c1			u0c1_addr.bit.re	/* Receive enable bit */
#define		ri_u0c1			u0c1_addr.bit.ri	/* Receive complete flag */
#define		u0irs_u0c1		u0c1_addr.bit.irs	/* UART0 transmit interrupt source select bit */
#define		u0rrm_u0c1		u0c1_addr.bit.rrm	/* UART0 continuous receive mode enable bit */
/*------------------------------------------------------
  UART1 Transmit/Receive Control Register 1
------------------------------------------------------*/
#define		u1c1		u1c1_addr.byte

#define		te_u1c1			u1c1_addr.bit.te	/* Transmit enable bit */
#define		ti_u1c1			u1c1_addr.bit.ti	/* Transmit buffer empty flag */
#define		re_u1c1			u1c1_addr.bit.re	/* Receive enable bit */
#define		ri_u1c1			u1c1_addr.bit.ri	/* Receive complete flag */
#define		u1irs_u1c1		u1c1_addr.bit.irs	/* UART1 transmit interrupt source select bit */
#define		u1rrm_u1c1		u1c1_addr.bit.rrm	/* UART1 continuous receive mode enable bit */
/*------------------------------------------------------
  UART2 Transmit/Receive Control Register 1
------------------------------------------------------*/
#define		u2c1		u2c1_addr.byte

#define		te_u2c1			u2c1_addr.bit.te	/* Transmit enable bit */
#define		ti_u2c1			u2c1_addr.bit.ti	/* Transmit buffer empty flag */
#define		re_u2c1			u2c1_addr.bit.re	/* Receive enable bit */
#define		ri_u2c1			u2c1_addr.bit.ri	/* Receive complete flag */
#define		u2irs_u2c1		u2c1_addr.bit.irs	/* UART2 transmit interrupt source select bit */
#define		u2rrm_u2c1		u2c1_addr.bit.rrm	/* UART2 continuous receive mode enable bit */
#define		u2lch_u2c1		u2c1_addr.bit.lch	/* Data logic select bit */
#define		u2ere_u2c1		u2c1_addr.bit.ere	/* Error signal output enable bit */

/*------------------------------------------------------
  UARTi receive buffer register
------------------------------------------------------*/
union{
	struct{
		char	b0:1;
		char	b1:1;
		char	b2:1;
		char	b3:1;
		char	b4:1;
		char	b5:1;
		char	b6:1;
		char	b7:1;
		char	mprb:1;							/* Multiprocessor select bit */
		char	b9:1;
		char	b10:1;
		char	b11:1;
		char	oer:1;							/* Overrun error flag */
		char	fer:1;							/* Framing error flag */
		char	per:1;							/* Parity error flag */
		char	sum:1;							/* Error sum flag */
	} bit;
	struct{
		char	low;							/* Low  8 bit */
		char	high;							/* High 8 bit */
	} byte;
	unsigned short	word;
} u0rb_addr, u1rb_addr, u2rb_addr;

/*------------------------------------------------------
  UARTi receive buffer register
------------------------------------------------------*/
#define		u0rb			u0rb_addr.word
#define		u0rbl			u0rb_addr.byte.low
#define		u0rbh			u0rb_addr.byte.high

#define		oer_u0rb		u0rb_addr.bit.oer	/* Overrun error flag */
#define		fer_u0rb		u0rb_addr.bit.fer	/* Framing error flag */
#define		per_u0rb		u0rb_addr.bit.per	/* Parity error flag */
#define		sum_u0rb		u0rb_addr.bit.sum	/* Error sum flag */

#define		u1rb			u1rb_addr.word
#define		u1rbl			u1rb_addr.byte.low
#define		u1rbh			u1rb_addr.byte.high

#define		oer_u1rb		u1rb_addr.bit.oer	/* Overrun error flag */
#define		fer_u1rb		u1rb_addr.bit.fer	/* Framing error flag */
#define		per_u1rb		u1rb_addr.bit.per	/* Parity error flag */
#define		sum_u1rb		u1rb_addr.bit.sum	/* Error sum flag */

#define		u2rb			u2rb_addr.word
#define		u2rbl			u2rb_addr.byte.low
#define		u2rbh			u2rb_addr.byte.high

#define		mprb_u2rb		u2rb_addr.bit.mprb	/* Multiprocessor select bit */
#define		oer_u2rb		u2rb_addr.bit.oer	/* Overrun error flag */
#define		fer_u2rb		u2rb_addr.bit.fer	/* Framing error flag */
#define		per_u2rb		u2rb_addr.bit.per	/* Parity error flag */
#define		sum_u2rb		u2rb_addr.bit.sum	/* Error sum flag */

/********************************************************
*  declare SFR union                                    *
********************************************************/

union{
	struct{
		char	b0:1;
		char	b1:1;
		char	b2:1;
		char	b3:1;
		char	b4:1;
		char	b5:1;
		char	b6:1;
		char	b7:1;
		char	b8:1;
		char	b9:1;
		char	b10:1;
		char	b11:1;
		char	b12:1;
		char	b13:1;
		char	b14:1;
		char	b15:1;
		char	b16:1;
		char	b17:1;
		char	b18:1;
		char	b19:1;
	} bit;
	struct{
		char	low;							/* low  8 bit */
		char	mid;							/* mid  8 bit */
		char	high;							/* high 8 bit */
		char	nc;								/* non use */
	} byte;

	unsigned long	dword;
} rmad0_addr,rmad1_addr;

#define		rmad0		rmad0_addr.dword		/* Address match interrupt register0 */
#define		rmad0l		rmad0_addr.byte.low		/* Address match interrupt register0 Low */
#define		rmad0m		rmad0_addr.byte.mid		/* Address match interrupt register0 Middle */
#define		rmad0h		rmad0_addr.byte.high	/* Address match interrupt register0 High */
#define		rmad1		rmad1_addr.dword		/* Address match interrupt register1 */
#define		rmad1l		rmad1_addr.byte.low		/* Address match interrupt register1 Low */
#define		rmad1m		rmad1_addr.byte.mid		/* Address match interrupt register1 Middle */
#define		rmad1h		rmad1_addr.byte.high	/* Address match interrupt register1 High */

union{
	struct{
		char	b0:1;
		char	b1:1;
		char	b2:1;
		char	b3:1;
		char	b4:1;
		char	b5:1;
		char	b6:1;
		char	b7:1;
		char	b8:1;
		char	b9:1;
		char	b10:1;
		char	b11:1;
		char	b12:1;
		char	b13:1;
		char	b14:1;
		char	b15:1;
	} bit;
	struct{
		char	low;							/* low  8 bit */
		char	high;							/* high 8 bit */
	} byte;
	unsigned int	word;
} u0tb_addr, u1tb_addr, u2tb_addr, ad0_addr, ad1_addr, ad2_addr, ad3_addr, ad4_addr, ad5_addr, ad6_addr, ad7_addr, sstdr_addr, ssrdr_addr;

#define		u0tb		u0tb_addr.word			/* UART0 transmit buffer register */
#define		u0tbl		u0tb_addr.byte.low		/* UART0 transmit buffer register Low */
#define		u0tbh		u0tb_addr.byte.high		/* UART0 transmit buffer register High */
#define		u1tb		u1tb_addr.word			/* UART1 transmit buffer register */
#define		u1tbl		u1tb_addr.byte.low		/* UART1 transmit buffer register Low */
#define		u1tbh		u1tb_addr.byte.high		/* UART1 transmit buffer register High */
#define		u2tb		u2tb_addr.word			/* UART2 transmit buffer register */
#define		u2tbl		u2tb_addr.byte.low		/* UART2 transmit buffer register Low */
#define		u2tbh		u2tb_addr.byte.high		/* UART2 transmit buffer register High */
#define		mptb_u2tb	u2tb_addr.bit.b8		/* UART2 MPTB  */
#define		ad0			ad0_addr.word			/* A-D register 0 */
#define		ad0l		ad0_addr.byte.low		/* A-D register 0 Low */
#define		ad0h		ad0_addr.byte.high		/* A-D register 0 High */
#define		ad1			ad1_addr.word			/* A-D register 1 */
#define		ad1l		ad1_addr.byte.low		/* A-D register 1 Low */
#define		ad1h		ad1_addr.byte.high		/* A-D register 1 High */
#define		ad2			ad2_addr.word			/* A-D register 2 */
#define		ad2l		ad2_addr.byte.low		/* A-D register 2 Low */
#define		ad2h		ad2_addr.byte.high		/* A-D register 2 High */
#define		ad3			ad3_addr.word			/* A-D register 3 */
#define		ad3l		ad3_addr.byte.low		/* A-D register 3 Low */
#define		ad3h		ad3_addr.byte.high		/* A-D register 3 High */
#define		ad4			ad4_addr.word			/* A-D register 4 */
#define		ad4l		ad4_addr.byte.low		/* A-D register 4 Low */
#define		ad4h		ad4_addr.byte.high		/* A-D register 4 High */
#define		ad5			ad5_addr.word			/* A-D register 5 */
#define		ad5l		ad5_addr.byte.low		/* A-D register 5 Low */
#define		ad5h		ad5_addr.byte.high		/* A-D register 5 High */
#define		ad6			ad6_addr.word			/* A-D register 6 */
#define		ad6l		ad6_addr.byte.low		/* A-D register 6 Low */
#define		ad6h		ad6_addr.byte.high		/* A-D register 6 High */
#define		ad7			ad7_addr.word			/* A-D register 7 */
#define		ad7l		ad7_addr.byte.low		/* A-D register 7 Low */
#define		ad7h		ad7_addr.byte.high		/* A-D register 7 High */

/*------------------------------------------------------
  SS Transmit Data Register
------------------------------------------------------*/
#define		sstdr		sstdr_addr.word
#define		sstdrl		sstdr_addr.byte.low
#define		sstdrh		sstdr_addr.byte.high

/*------------------------------------------------------
  SS Receive Data Register
------------------------------------------------------*/
#define		ssrdr		ssrdr_addr.word
#define		ssrdrl		ssrdr_addr.byte.low
#define		ssrdrh		ssrdr_addr.byte.high
