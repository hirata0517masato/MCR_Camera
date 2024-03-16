/****************************************************************************************************/
/* �v���W�F�N�g���FRX621_SAMPLE     	                �@�@�@�@�@          			    */
/* ���W���[�����F  			                     					    */
/* ��    ��    �F										    */
/* �g�p�}�C�R���FRX621 R5F56218BDFP (Flash ROM:512KB, SRAM:64KB, DATA Flash 32KB)                   */
/* �쐬��      �Fhirata0517masato               				                                            */
/* �o�[�W����  �F0.00                                                                               */
/* �쐬��      �F2017/02/10 									    */
/****************************************************************************************************/                  
#include "iodefine.h"
#include <machine.h>
//#include "lowsrc.h"

#include "usb.h"

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init(void);

void CLK_init(void);
void IO_init(void);
void AD_init(void);
void CMT_init(void);

void cam_out(void);
void ImageCapture(int,int);
int  get_ad( void );
void expose( void );
void expose2( void );
void binarization(int,int);
void WhiteLineWide(int,int);


/* �萔�ݒ� */

//#define PRINT//�f�[�^�o�͎��̂ݗL���ɂ���

/* TAOS TSL1401CL */
#define	TAOS_SI_HIGH	PORTD.DR.BIT.B0 = 1	/* CN3-17 */
#define	TAOS_SI_LOW	PORTD.DR.BIT.B0 = 0	/* CN3-17 */
#define	TAOS_CLK_HIGH	PORTD.DR.BIT.B1 = 1	/* CN3-18 */
#define	TAOS_CLK_LOW	PORTD.DR.BIT.B1 = 0	/* CN3-18 */

#define MODE_HIGH_BIT	PORT3.PORT.BIT.B0 /* CN2-8 */
#define MODE_LOW_BIT	PORT3.PORT.BIT.B1 /* CN2-7 */

#define WIDE_OUT 	PORT2.DR.BYTE /* CN2-16-9 */
#define CENTER_OUT 	PORTC.DR.BYTE /* CN4-19-20 CN2-30-25 */

//AD0 /* CN3-9 P40 */

#define		Line_Max	760		/* ���C�����FMAX�l�̐ݒ� */ 

#define 	LineStart 	35		/* �J�����Ō���͈�(�ʏ탂�[�h) */
#define 	LineStop  	92

#define 	LineStartSaka 	50		/* �J�����Ō���͈�(�⃂�[�h) */
#define 	LineStopSaka  	77

#define 	LineStartNonR 	35		/* �J�����Ō���͈�(�E����) */
#define 	LineStopNonR  	64

#define 	LineStartNonL 	64		/* �J�����Ō���͈�(������) */
#define 	LineStopNonL  	92



/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
unsigned long   cnt1000 =  0;

/* �J�����֘A */
long	  	EXPOSURE_timer = 15000;	/* �I������	20000				*/
int		ImageData[130];			/* �J�����̒l				*/
int 		BinarizationData[130];	/* �Q�l��					*/

int		Max = 0,Min,Ave;	/*�J�����ǂݎ��ő�l�A�ŏ��l�A���ϒl*/
int 	Ave_old = 0;

unsigned int 	Rsensor;				/* ���C���̉E�[ */
unsigned int 	Lsensor;				/* ���C���̍��[ */
unsigned int 	Wide = 0;				/* ���C���̕� */
int		Center = 0;				/* ���C���̏d�S */

unsigned int 	Wide_old;				/* �ߋ��̑������C���̕� */
int		Center_old;				/* �ߋ��̑������C���̏d�S */
int		old_flag = 0;			/*0 = �ʏ�o��, 1 = old���o��*/
int		old_cnt = 0;			/* old�o�͗p�J�E���g	*/

int		Center_lasttime;		/* �O��̃��C���̏d�S */

int             White;					/* ���̐�	*/

int		mode;				/* 0 = �ʏ� 1 = �� 2 = �E���� 3 = ������	*/	

int		EXPOSURE_cnt = 0;		/* */		

/***********************************************************************/
/* ���C���v���O����                                                    */
/***********************************************************************/
void main(void)
{
	int	i = 0;
 	
    	/* �}�C�R���@�\�̏����� */	
	CLK_init();  // �N���b�N�̏�����
 	IO_init();   // IO�̏�����
 //	CMT_init();  // CMT�̏�����
 	AD_init();   // A/D�̏�����
		
#ifdef PRINT
	USB_init();  //USB CDC�̏�����
#endif //PRINT
	
	/* Data Initialization */
	for(i=0;i<130;i++){
		ImageData[i] = 0;			/* �J�����̒l				*/
		BinarizationData[i] = 0;	/* �Q�l��					*/
	}
	
	cam_out();//����p�֏o��
	
	
	//���邳�̍ő�l���ڕW�l�ɋ߂Â��܂Ń��[�v && ���C���𔭌�����܂Ń��[�v
	do{
		
		#ifndef PRINT
		mode = (MODE_HIGH_BIT & 0x01) + ((MODE_LOW_BIT & 0x01) << 1);//���[�h����Ɏg�p
		#endif
		
		expose2();				//�I�����ԁi�S���A�S���ł����ԕύX)
		
		ImageCapture(LineStart,LineStop);			//�C���[�W�L���v�`���[
		
		binarization(LineStart,LineStop); 		//�Q�l��
		
		WhiteLineWide(LineStart,LineStop);		//�����C���̑���	
		
		//cam_out();//����p�֏o��
		Center_lasttime = Center;//�ߋ��̒l��ۑ�
		
	}while(!((-50 < (Line_Max - Max)) && ((Line_Max - Max) < 50)) && (!((Wide != 0) && (Wide < 30))));
	
	
	while( 1 ) {
	
		#ifndef PRINT
		mode = (MODE_HIGH_BIT & 0x01) + ((MODE_LOW_BIT & 0x01) << 1);//���[�h����Ɏg�p
		#endif
		
		switch(mode){
			case 0://�ʏ탂�[�h
				expose();				//�I������
				
				ImageCapture(LineStart,LineStop);			//�C���[�W�L���v�`���[
		
				binarization(LineStart,LineStop); 		//�Q�l��
		
				WhiteLineWide(LineStart,LineStop);		//�����C���̑���
				
				break;
				
			case 1://�⃂�[�h
				expose();				//�I������
				
				ImageCapture(LineStartSaka,LineStopSaka);			//�C���[�W�L���v�`���[
		
				binarization(LineStartSaka,LineStopSaka); 		//�Q�l��
		
				WhiteLineWide(LineStartSaka,LineStopSaka);		//�����C���̑���
				
				break;
				
			case 2://�E����
				expose();				//�I������
				
				ImageCapture(LineStartNonR,LineStopNonR);			//�C���[�W�L���v�`���[
		
				binarization(LineStartNonR,LineStopNonR); 		//�Q�l��
		
				WhiteLineWide(LineStartNonR,LineStopNonR);		//�����C���̑���
				
				break;
				
			case 3://������
				expose();				//�I������
				
				ImageCapture(LineStartNonL,LineStopNonL);			//�C���[�W�L���v�`���[
		
				binarization(LineStartNonL,LineStopNonL); 		//�Q�l��
		
				WhiteLineWide(LineStartNonL,LineStopNonL);		//�����C���̑���
			
				break;
		}
	
		
		cam_out();//����p�֏o��
		Center_lasttime = Center;//�ߋ��̒l��ۑ�
		
				
		#ifdef PRINT
			cnt1000++;
			
			if(cnt1000 > 500){
				//for(i = LineStart; i <= LineStop; i++)printf("%d",BinarizationData[i]);
				for(i = LineStart; i <= LineStop; i+=2)printf("%d",BinarizationData[i]);
				printf("Max = %d Min = %d Center = %d Wide = %d Lsensor = %d Rsensor = %d time = %d mode = %d",Max,Min,Center,Wide,Lsensor,Rsensor,EXPOSURE_timer,mode);
				printf("\n");
				cnt1000=0;
			}
		#endif
    }
}
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�F�N���b�N�̏�����                                                                    */
/* �� �� �� �ׁF�V�X�e���N���b�N96MHz,���Ӄ��W���[���N���b�N24MHz                                   */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CLK_init(void)
{
   SYSTEM.SCKCR.BIT.ICK = 0;		// �V�X�e���N���b�N(ICLK)       EXTAL�~8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// ���Ӄ��W���[���N���b�N(PCLK)	EXTAL�~2 (24MHz)
     
   //SYSTEM.SUBOSCCR = 1;              // 0�F�T�u�N���b�N���U�퓮��i0�F�f�t�H���g�œ��� 1:��~�j
   //RTC.RCR2.BIT.RTCOE = 1;           // 1�FRTCOUT��[�q(P32)����o�͂���
   
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FI/O�̏�����                                                                    �@�@ */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void IO_init(void)
{
	//unsigned int uc;
	
 //  PORTC.PCR.BYTE   = 0x03;        // PC0,1���v���A�b�v�w��
    
    PORT2.DDR.BYTE = 0xff;           // P2���o�͂ɐݒ�
    PORTC.DDR.BYTE = 0xff;           // PC���o�͂ɐݒ�
    PORTD.DDR.BYTE = 0xff;           // PD���o�͂ɐݒ�

    PORT3.DDR.BYTE = 0x00;           // P3����͂ɐݒ�    
    PORT4.DDR.BYTE = 0x00;           // P4����͂ɐݒ�   AN0  
 /*   
    // �V���A���ʐM 
	MSTP(SCI1) = 0;			//�X�g�b�v����
	SCI1.SCR.BYTE = 0x00;	// �����N���b�N�A����M�֎~ 
	SCI1.SMR.BYTE = 0x00;	// PCLK�N���b�N�ASTOP1bit�A�p���e�B�����A8Bit�f�[�^�A���������� 
	SCI1.BRR = 80;			// 77 = 19200bps 
	for(uc=0;uc<1000;uc++);
	SCI1.SSR.BYTE = 0x00;
	// ���M���� 
	SCI1.SCR.BYTE = 0x20;
	IEN(SCI1,RXI1) = 1;	// SCI1��RXI1���荞�ݗv������ 
	IPR(SCI1,RXI1) = 2;	// SCI1��RXI1���荞�݃��x���ݒ� 
	IR(SCI1,RXI1) = 0;
*/	
}

/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FA/D�̏�����                                                                         */
/* �� �� �� �ׁF                                                                                    */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void AD_init(void)
{
   //A/D�̏�����
  // SYSTEM.MSTPCRA.BIT.MSTPA22 = 0; // AD1���W���[���X�g�b�v�̉���
   SYSTEM.MSTPCRA.BIT.MSTPA23 = 0; // AD0���W���[���X�g�b�v�̉���
   SYSTEM.MSTPCRA.BIT.MSTPA17 = 0; //12bit��A/D�ϊ�����
   
   MSTP(S12AD) = 0;//���W���[���X�g�b�v��Ԃ�����

   S12AD.ADCSR.BYTE = 0x0c;//���샂�[�h�A�ϊ��J�n�v���A�N���b�N�̐ݒ�
   S12AD.ADANS.WORD = 0x0001;//�ϊ��[�q�̐ݒ�
   S12AD.ADSTRGR.BYTE = 0x0000;//A/D �ϊ��J�n�v���̐ݒ�
	
}


/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */
/* �� �� �T �v�FCMT0,CMT1 (�R���y�A�}�b�`�^�C�}�[)�̏�����                                          */
/* �� �� �� �ׁF1ms���荞�ݎ���                                                                     */
/* ��       ���F�Ȃ�										    */
/* ��  ��   �l�F�Ȃ�										    */
/* ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ */ 
void CMT_init(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT0.CMCR.WORD = 0x0040;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 3000-1;           // 1ms Count�F PCLK = 24MHz/8=3MHz 3M/1mS=3000 (�������J�E���g��-1)
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
    	
    MSTP(CMT1) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT1 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT1.CMCR.WORD = 0x0040;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT1.CMCOR = 3000-1;           // 1ms Count�F PCLK = 24MHz/8=3MHz 3M/1mS=3000 (�������J�E���g��-1)
    IPR(CMT1,CMI1) = 3;
    IEN(CMT1,CMI1) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0�^�C�}�[�X�^�[�g
    CMT.CMSTR0.BIT.STR1 = 1;       // CMT1�^�C�}�[�X�^�[�g

}
	


/************************************************************************/
/* �I�����Ԓ���(�S���@�S���̎��͕ύX���Ȃ�)                             */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ��@�@�@�@�@                                                */
/************************************************************************/
void expose( void )
{
	unsigned long i;
	int sa = Line_Max - Max;
	
	//if( Wide != 0 && White <= 60){//���łȂ����ł��Ȃ�
	if( Wide == 0 || White >= 50){//��or��
		EXPOSURE_cnt++;
	}else{
		EXPOSURE_cnt = 0;
	}
	
	if(EXPOSURE_cnt < 1){
		if(-20 < sa && sa < 20){
			//�덷�Ȃ̂ŕύX���Ȃ�
		}else{ 
			EXPOSURE_timer += (long)(sa*4);
		}
	}	
		
	
	if( EXPOSURE_timer > 100000) EXPOSURE_timer = 100000;
	else if( EXPOSURE_timer <= 1000 ) EXPOSURE_timer = 1000;

	for(i=0;i<EXPOSURE_timer;i++);

}

/************************************************************************/
/* �I�����Ԓ���                                                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �Ȃ��@�@�@�@�@                                                */
/************************************************************************/
void expose2( void )
{
	unsigned long i;
	
	//EXPOSURE_timer += (long)((Line_Max - Max)*10);
	
	if(Line_Max - Max < 0){
		EXPOSURE_timer -= 100;
	}else{
		EXPOSURE_timer += 100;
	}
	if( EXPOSURE_timer > 100000) EXPOSURE_timer = 100000;
	else if( EXPOSURE_timer <= 0 ) EXPOSURE_timer = 0;
	
	for(i=0;i<EXPOSURE_timer;i++);

}
 
 /************************************************************************/
/* �C���[�W�L���v�`��                                                   */
/************************************************************************/
void ImageCapture(int linestart, int linestop){	 
	
	unsigned char i;

	Max = 0,Min = 4096;

	TAOS_SI_HIGH;
	TAOS_CLK_HIGH;
	TAOS_SI_LOW;
	ImageData[0] = 0;
	TAOS_CLK_LOW;
	for(i = 1; i < LineStart; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}
	for(i = LineStart; i < linestart; i++) {		
		TAOS_CLK_HIGH;	
		ImageData[i] = get_ad();
		TAOS_CLK_LOW;
	}
	
	for(i = linestart; i <= linestop; i++) {				
		 
		TAOS_CLK_HIGH;
		ImageData[i] = get_ad();	// inputs data from camera (one pixel each time through loop) 
		TAOS_CLK_LOW;
		
		if(Max < ImageData[i]){
			Max = ImageData[i];
		}			
		if(Min > ImageData[i]){
			Min = ImageData[i];
		}
		
	}
	
	for(i = linestop+1; i <= LineStop; i++) {		
		TAOS_CLK_HIGH;	
		ImageData[i] = get_ad();
		TAOS_CLK_LOW;
	}
	for(i = LineStop+1; i < 128; i++) {		
		TAOS_CLK_HIGH;		
		TAOS_CLK_LOW;
	}
	
	TAOS_CLK_HIGH;
	TAOS_CLK_LOW;
}
/************************************************************************/
/* A/D�l�ǂݍ���(AN0)                                                 */
/* �����@ �Ȃ�                                                          */
/* �߂�l A/D�l 0�`4069                                                 */
/************************************************************************/
int get_ad(void)
{
/*	AD0.ADCSR.BIT.ADST = 1;  	// A/D 0�̕ϊ��J�n
   	while(AD0.ADCSR.BIT.ADST == 0); // ���肪�I������܂ő҂�
	return AD0.ADDRA;
*/	
	
	S12AD.ADCSR.BIT.ADST = 1;  	// A/D 0�̕ϊ��J�n
   	while(S12AD.ADCSR.BIT.ADST == 0); // ���肪�I������܂ő҂�
	return S12AD.ADDR0;	
}
/************************************************************************/
/* �Q�l��                                                               */
/************************************************************************/
void binarization(int linestart, int linestop)
{
	int i,a;
	
	/* �ō��l�ƍŒ�l����Ԃ̒l�����߂� */
		
	//if(mode == 1) Ave = Min + 130;
	//else Ave = ((Max + Min) >> 1) - 50;
	//else{
		a  = ((Max + Min) >> 1);
		Ave = ((a+Min) >> 1);
		Ave = ((a+Ave) >> 1);
		//Ave  = ((Max + Min) >> 1);
	//}
	
	/* ���͂O�@���͂P�ɂ��� */
	White = 0;					/* ���̐����O�ɂ��� */
	
	if( Max > Line_Max - 400 ){//320 -150  250
		/* �����꒼���̂Ƃ� */
		if(Min > 290 ){//260
		//if(Max - Min < 130){//130
			White = 127;
			for(i = linestart ; i <= linestop; i++) {
				BinarizationData[i] = 1;
			}
		}else{		
			for(i = linestart ; i <= linestop; i++) {
				if(  ImageData[i] > Ave ){	
					White++;			
					BinarizationData[i] = 1;
				}else{
					BinarizationData[i] = 0;
				}	
			}
		}
	/* ������ʂ̂Ƃ� */
	}else{
		for(i = linestart ; i <= linestop; i++) {
			BinarizationData[i] = 0;
		}
	}

	//�͈͊O�͍���
	for(i = 0; i < linestart; i++){
		BinarizationData[i] = 0;
	}
	for(i = linestop+1; i < 128; i++){
		BinarizationData[i] = 0;
	}
}


/************************************************************************/
/* �����̕��𑪒�                                                       */
/************************************************************************/
void WhiteLineWide(int linestart, int linestop)
{
	int t,i;
		
	Lsensor = linestart;Rsensor = linestop;Wide = 0;t = 0;		
	
	if(Center_lasttime < 60){//���C�������
		for(i = linestart ; i <= linestop; i++) {
			if(t==0){
				if( BinarizationData[i] ){					/* ������ŏ��̔� */
					Lsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* ������ŏ��̍� */			
					Rsensor = i;
					t = 2;
				}
			}
		}
	}else{//���C���E���
		for(i = linestop; i >= linestart; i--) {
			if(t==0){
				if( BinarizationData[i] ){					/* �E����ŏ��̔� */
					Rsensor = i;
					t = 1;
				}
			}else if(t==1){
				if( !BinarizationData[i] ){					/* �E����ŏ��̍� */			
					Lsensor = i;
					t = 2;
				}
			}
		}	
	}
		
	
	if(White > 70){//�S���ɂ���
		Wide = 127;Center = 64;						/* ����� */
		
	}else if((White > 6) && ((linestop - linestart) > 5)){//�������Ȃ����Ȃ� && ���C����T���͈͂��������Ȃ�
	
		Wide = Rsensor - Lsensor;					/* �������߂� */	
		Center = (Lsensor + Rsensor) >> 1;		/* �d�S�����߂� */	
			
			
		//���C���ׂ��� || ( �O��A�����͔���F�ł͂Ȃ� && �n�[�t���C���Ȃǂł͂Ȃ� &&  (�}�Ƀ��C�����ړ�����))
		if((((mode == 1) && (Wide < 4)) || ((mode != 1) && (Wide < 6))) || ((Center_lasttime != 64) && (White < 20) && (((Center - Center_lasttime) > 15) || ((Center - Center_lasttime) < -15)))){
					
			if(Center_lasttime < 60){
						
				WhiteLineWide(Rsensor,linestop);//������x���C����T��			
			}else{
					
				WhiteLineWide(linestart,Lsensor);//������x���C����T��
			}
		}	
	}else{//�S���ɂ���
		Wide = 0;Center = 64;						/* ����� */
	}	
}

/**********************************************************************/
/*	R8C��Center��Wide�̑��M											  */
/*																	  */
/**********************************************************************/
void cam_out(){
	
	/*
	int wide,center;
	
	if(old_flag == 0){//�ʏ�o��
		wide = Wide;
		center = Center;
			
		if(Wide > 40){//��������
			Center_old = Center;
			Wide_old = Wide;
			
			old_flag = 1;//old�o�͂ɂ���
			old_cnt = 0;
		}
	}else{//old�o��
			
		if(Wide_old < Wide ){//����ɑ���������������
			Center_old = Center;
			Wide_old = Wide;
					
			old_cnt--;
		}
		
		wide = Wide_old;
		center = Center_old;		
			
		old_cnt++;
		if((old_cnt > 3) || (Center_old - Center < -5 || Center_old - Center > 5)){
			
			wide = Wide;
			center = Center;
		
			old_flag = 0;//�ʏ�o�͂ɖ߂�
				
		}
	}
	
	WIDE_OUT  = wide;
	CENTER_OUT = center;
	*/
	
	WIDE_OUT  = (Wide << 1)&0xfe;//NEW
	//WIDE_OUT  = Wide; //��z��
	CENTER_OUT = Center;
	
	//WIDE_OUT  = Max/10;
	//CENTER_OUT = Min/10;
}

