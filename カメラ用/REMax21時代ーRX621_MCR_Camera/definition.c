/***********************************************************************/
/*                                                                     */
/*  FILE        :Main.c or Main.cpp                                    */
/*  DATE        :Tue, Oct 31, 2006                                     */
/*  DESCRIPTION :Main Program                                          */
/*  CPU TYPE    :                                                      */
/*                                                                     */
/*  NOTE:THIS IS A TYPICAL EXAMPLE.                                    */
/*                                                                     */
/***********************************************************************/
/********************************************************************************
*																				*
*	�t�@�C����	:Trace2.c												�@�@	*
*	�T�v		:RX621���C���g���[�X�v���O����									*
*																				*
********************************************************************************/
                  
#include "iodefine.h"
#include <machine.h>


void main(void);
void init_CMT(void);
void init(void);

 // �O���[�o���ϐ��̒�`
 //**********************************************************//
 //   �ϐ��̒�`                                             //
 //                                                          //
 //   �^                    �l�͈̔�       �f�[�^�T�C�Y      //
 //  char:                -128 �` 127          1Byte         //
 //  unsigned char:          0 �` 255          1Byte         //
 //  short:             -32768 �` 32767        2Byte         //
 //  unsigned short          0 �` 65535        2Byte         //
 //  int                -32768 �` 32767        2Byte         //
 //  unsigned int            0 �` 65535        2Byte         //
 //  long          -2147483648 �` 2147483647   4byte         //
 //  unsigned long           0 �` 4294967295   4Byte         //
 //                                                          //
 //**********************************************************//
#define	vshort	volatile short

vshort mode;             // ���C�����[�h�I��
short  beep_pulse = 0;
short  beep_enable;	
short  beep_long;	
unsigned long n;
unsigned long goal_cnt = 0;    

//short i;
//short x;
//short y;

unsigned short GOAL;
unsigned short GOAL_LIM;
unsigned char  GOAL_FLG1 = 0;
unsigned short GOAL_FLG2 = 0;
unsigned short GOAL_REF; 

unsigned short SIDE_R;
unsigned short SIDE_R_LIM;
unsigned char  SIDE_R_FLG = 0;
unsigned short SIDE_R_REF;

unsigned short SIDE_L;
unsigned short SIDE_L_LIM;
unsigned char  SIDE_L_FLG = 0;
unsigned short SIDE_L_REF;

unsigned short CENTER_R;
unsigned short CENTER_R_LIM;
unsigned char  CENTER_R_FLG = 0;
unsigned short CENTER_R_REF;

unsigned short CENTER_L;
unsigned short CENTER_L_LIM;
unsigned char  CENTER_L_FLG = 0;
unsigned short CENTER_L_REF;

unsigned short MARKER;
unsigned short MARKER_LIM;
unsigned char  MARKER_FLG1 = 0;
unsigned short MARKER_FLG2 = 0;
unsigned short MARKER_FLG3 = 0;
unsigned short MARKER_REF;

unsigned short SEN_LIM;

short beep_cnt = 300;
short task;
short RUN_MODE;
unsigned short LOOP = 0;
short LED_speed;        // 7�Z�OLED�T�C�N���̉�]�X�s�[�h
short dig;              // 7�Z�OLED�\��
short LED_cycle;        // 7�Z�OLED�T�[�L�b�g���t���O

char START_FLG = 0;
char SLOW_SPD  = 0;
 
unsigned short ct;
unsigned short cnt;
unsigned short cnt1;
unsigned long wait_timer;

short R_control_err;
short L_control_err;

char control_gain;

float STR_SPD;
float TURN_SPD;
float STR_SPD_NORMAL;
float STR_SPD_LOW;   
float TURN_SPD_HIGH;  
float TURN_SPD_LOW;  
  
float SPEED1;
float SPEED2;  


//**************************************************************************************
// �ϐ��̒�`
//**************************************************************************************
#define		START_SW	  PORTC.PORT.BIT.B0		// PC0: �X�^�[�g�X�C�b�`
#define		MODE_SW	  	  PORTC.PORT.BIT.B1		// PC1: ���[�h�X�C�b�`
#define		SW_ON	      0				        // �^�N�g�X�C�b�`"�n�m"   �̘_��			
#define		SW_OFF	      1				        // �^�N�g�X�C�b�`"�n�e�e" �̘_��
#define	    SEG_LED       PORTD.DR.BYTE         // PD0-7: 7�Z�OLED �̓_��
#define     SEN_DRV       PORT3.DR.BIT.B0       // P30: �Z���T�쓮�p���X
#define     BEEP          PORTE.DR.BIT.B0       // PE0: BEEP

#define     L_PWM		  PORT2.DR.BIT.B0       // P20: L���[�^�[PWM�o��  
#define     L_IN2		  PORT2.DR.BIT.B1       // P21: L���[�^�[IN2
#define     L_IN1		  PORT2.DR.BIT.B2       // P22: L���[�^�[IN1
#define     STBY		  PORT2.DR.BIT.B3       // P23: ���[�^�[STBY
#define     R_IN1		  PORT2.DR.BIT.B4       // P24: R���[�^�[IN1  
#define     R_IN2		  PORT2.DR.BIT.B5       // P25: R���[�^�[IN2
#define     R_PWM		  PORT2.DR.BIT.B6       // P26: R���[�^�[PWM�o��

#define		PD0	          PORTD.DR.BIT.B0 		// PD0: 7�Z�OLED: A          
#define		PD1 	      PORTD.DR.BIT.B1 		// PD1: 7�Z�OLED: B          
#define		PD2	          PORTD.DR.BIT.B2 		// PD2: 7�Z�OLED: C          
#define		PD3 	      PORTD.DR.BIT.B3 		// PD3: 7�Z�OLED: D          
#define		PD4	          PORTD.DR.BIT.B4 		// PD4: 7�Z�OLED: E          
#define		PD5 	      PORTD.DR.BIT.B5 		// PD5: 7�Z�OLED: F          
#define		PD6	          PORTD.DR.BIT.B6 		// PD6: 7�Z�OLED: G
#define		PD7	          PORTD.DR.BIT.B7 		// PD7: 7�Z�OLED: DP


//**************************************************************************************
// I/O�̏�����
//**************************************************************************************
void init(void)
{

// �N���b�N�̐ݒ� 
   SYSTEM.SCKCR.BIT.ICK = 0;		// �V�X�e���N���b�N(ICLK)       EXTAL�~8 (96MHz)
   SYSTEM.SCKCR.BIT.PCK = 2;		// ���Ӄ��W���[���N���b�N(PCLK)	EXTAL�~2 (24MHz)
	
// IO�|�[�g�̐ݒ� 
    PORTC.ICR.BIT.B0 = 0;               // PC0����͂ɐݒ�(START SW)
    PORTC.ICR.BIT.B1 = 0;	        // PC1����͂ɐݒ�(MODE SW)
    PORTC.PCR.BYTE   = 0x03;            // PC0,1���v���A�b�v�w��

    PORTD.DDR.BYTE = 0xFF;	        // PD0�`PE7���o�͂ɐݒ�(7�Z�O)
    PORT2.DDR.BYTE = 0xBE;	        // P20�`P27���o�͂ɐݒ�(���[�^�[)
    PORTE.DDR.BYTE = 0xFF;	        // PE0���o�͂ɐݒ�(BEEP)
    PORT3.DDR.BYTE = 0x01;	        // P30���o�͂ɐݒ�(�Z���T�[�h���C�u)
    PORT0.DDR.BIT.B5 =  1;		// P05

//A/D�̏�����
   SYSTEM.MSTPCRA.BIT.MSTPA22 = 0; // AD1���W���[���X�g�b�v�̉���
   SYSTEM.MSTPCRA.BIT.MSTPA23 = 0; // AD0���W���[���X�g�b�v�̉���

}

//***************************************************************************
// beep data  "�h���~�t�@�\���V�h  �h�V���\�t�@�~���h�h
//****************************************************************************
 
 unsigned char beep_data[17] = 
 
 {0,220, 196, 175, 165, 147, 131, 117, 110,          
  110, 117, 131, 147, 165, 175, 196, 220};
  
//**************************************************************************************
// CMT0,CMT1 (�R���y�A�}�b�`�^�C�}�[)�̏����� 
//**************************************************************************************
void init_CMT(void)
{
    MSTP(CMT0) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT0.CMCR.WORD = 0x0041;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT0.CMCOR = 50;               // �ݒ� 50;
    IPR(CMT0,CMI0) = 3;
    IEN(CMT0,CMI0) = 1;
    	
    MSTP(CMT1) = 0;                // SYSTEM.MSTPCRA.BIT.MSTPA15 = 0; // CMT0 �^�C�}�[�X�^���o�C���� �i0�ŉ����j
    CMT1.CMCR.WORD = 0x0041;       // 4:���荞�݋��@0:PCLK/8 1:PCLK/32 2:PCLK/128 3:PCLK/512
    CMT1.CMCOR = 400;              // �����̐ݒ� 800;
    IPR(CMT1,CMI1) = 6;
    IEN(CMT1,CMI1) = 1;
	
    set_psw(0x00010000);
	
    CMT.CMSTR0.BIT.STR0 = 1;   	   // CMT0�^�C�}�[�X�^�[�g
    CMT.CMSTR0.BIT.STR1 = 1;       // CMT1�^�C�}�[�X�^�[�g
}
	
//------------------------------------------------------------------------
// 	�r�[�v�������W���[��     j:�����I��            		                            
//------------------------------------------------------------------------
void beep()
{
    beep_long = 0;
	 
} 
 
//**************************************************************************************
// MTU1(�����[�^�[�j,MTU2(�E���[�^�[�j �^�C�}�[�̏����� 
//**************************************************************************************
void mtu_init(void)
{
	volatile int C_cycle,duty;
	
	C_cycle = 24e6 / 1000;
	
	SYSTEM.MSTPCRA.BIT.MSTPA9 = 0;  // MUT���j�b�g�O�@���W���[���X�g�b�v����
	MTUA.TSTR.BYTE = 0x00;  // �J�E���^�̒�~
	
	MTU1.TCR.BIT.CCLR = 0x1; //
	MTU2.TCR.BIT.CCLR = 0x1; //
	
	MTU1.TMDR.BIT.MD = 0x2;  // PWM MODE1
	MTU2.TMDR.BIT.MD = 0x2;  // PWM MODE1

	MTU1.TIOR.BIT.IOA = 0x6; //
	MTU1.TIOR.BIT.IOB = 0x5; //
	MTU2.TIOR.BIT.IOA = 0x6; //
	MTU2.TIOR.BIT.IOB = 0x5; //
		
	MTU1.TGRA = C_cycle;
	MTU2.TGRA = C_cycle;
	MTU1.TGRB = 0;
	MTU2.TGRB = 0;
	
	//MTUA.TOER.BIT.OE4A = 1;  //
	
	MTUA.TSTR.BYTE = 0x0F;	// 0000 1111
	
}
		
		
//**************************************************************************************
// SET PWM 	
//**************************************************************************************
void set_pwm (float duty_L, float duty_R)
{
	
    unsigned int dt_L, dt_R;
	
    if(duty_L >100.0) duty_L = 100.0; // duty_L��100�ȏ�ɂ��Ȃ�
    if(duty_L <  0.0) duty_L =   0.0; // duty_L���@0�ȉ��ɂ��Ȃ�
    if(duty_R >100.0) duty_R = 100.0;
    if(duty_R <  0.0) duty_R =   0.0;
	
    /* �f���[�e�B��̎Z�o */
    dt_L = MTU1.TGRA * duty_L / 100.0;//  dt_L = 0.9445*50/100 = 0.5
    dt_R = MTU2.TGRA * duty_R / 100.0;//
	
	/* �f���[�e�B��̃I�[�o�[�t���[�ی� */
    if(dt_L >= MTU1.TGRA)   dt_L = MTU1.TGRA - 1;  // 
    if(dt_R >= MTU2.TGRA)   dt_R = MTU2.TGRA - 1;
	
    /* �f���[�e�B��̐ݒ� */
    MTU1.TGRB = dt_L;
    MTU2.TGRB = dt_R;
}

		
//***************************************************
// display data cycle
//***************************************************
 unsigned char disp_cycle_data[6] =
 
    { 0x01,0x02,0x04,0x08,0x10,0x20 };  // LED cycle
    


//========================================================================
//      �E�F�C�g
//========================================================================
void wait (unsigned long n)        
{ 
	wait_timer = 0;
	while ( wait_timer < n  );  // cnt��n�ɂȂ�܂Ń��[�v
}                  


//**************************************************************************************
// display data 0 �` F high�œ_��
//**************************************************************************************
 unsigned char disp_data[16] =
   { 0x3f, 0x06, 0x5b, 0x4f, 0x66, 0x6d, 0x7d, 0x27,    // LED display data = "0"�`"7" 
     0x7f, 0x6f, 0x77, 0x7c, 0x39, 0x5e, 0x79, 0x71};   // LED display data = "8"�`"F" 


//**************************************************************************************
// ���s���W���[�� MODE0�@�S�[���Ŏ~�܂�Ȃ�
//**************************************************************************************
void mode_0(void)
{

  STBY        = 1;       // ���[�^�[�X�^���o�C����
  SEG_LED     = 0x00;    // �VSEG LED�̕\���N���A
  SEN_DRV     = 1;       // �Z���T�[�pLED�̔��� ON
  
  STR_SPD_NORMAL = 22.0;    // ���i�ʏ�X�s�[�h 22.0
//STR_SPD_LOW    = 19.0;    // ���i�����X�s�[�h
//TURN_SPD_HIGH  = 21.0;    // �^�[���X�s�[�h�� 20.0
//TURN_SPD_LOW   = 19.0;    // �^�[���X�s�[�h�� 20.0
  
  GOAL_FLG2   = 0;
  MARKER_FLG2 = 0; 
  
  //SPEED1 = STR_SPD_NORMAL;      // �ŏ��͒ʏ�X�s�[�h
  //SPEED2 = TURN_SPD_LOW;
  
  cnt = 0;
  while ( cnt < 1000 ) { LED_cycle == 1; beep(); }   //  �b���҂�
  LED_cycle = 0;
  RUN_MODE = 0;
  
  while(1)
	
    {        
	
		     if  (( GOAL_FLG2   == 1 ) && ( START_FLG == 0 ))                   // �@�X�^�[�g���C����؂���
			      { 
				    L_IN1 = 1; L_IN2 = 0; // ���O�i
			        R_IN1 = 1; R_IN2 = 0; // �E�O�i
			        PD7 = 1;              // 7SEG�h�b�g�_��
			        while ( GOAL_FLG1  == 1 ){ set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL); } // �X�^�[�g���C����ʂ�߂���܂Ői��
					//PD7 = 0;  
				    RUN_MODE = 6;
				  }              
			 
			 
			
        else if  (( MARKER_FLG2 == 1 ) || ( GOAL_FLG2  == 1 ) && ( START_FLG == 1 ))                                // �A���C�����o (�ǂ��炩�����o�j���C����ʉ߂���܂Ŏb���i��
		       {    
				    goal_cnt = 0;
		            L_IN1 = 1; L_IN2 = 0; // ���O�i
			        R_IN1 = 1; R_IN2 = 0; // �E�O�i
					
                          if ( MARKER_FLG2 == 1 ) { while  ( MARKER_FLG1 == 1 )          { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // �}�[�J�[���C��(�N���X���C���j��ʂ�߂���܂Ői��
                                                    cnt = 0;
                                                    while  ( cnt < 40 )                  { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ������Ɛi��
                                                  }	
													
								   
				    else  if ( GOAL_FLG2   == 1 ) { while  ( GOAL_FLG1   == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // �S�[���[���C��(�N���X���C���j��ʂ�߂���܂Ői��
					                                cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ���C�������S�ɒʂ�߂���܂Ői��
					                              }	              
					 	 
						     				   
                    if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 1 ))                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0; RUN_MODE = 0;        } // �N���X���C���Ɣ���   �� ���̂܂ܑO�i
               else if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 0 ))                       {                   MARKER_FLG2 = 0; RUN_MODE = 0; beep();  // �}�[�J�[���C���Ɣ��� �� 
			   
                                                                                          //       if (MARKER_FLG3 == 0 ) { SPEED1 = STR_SPD_LOW ;   SPEED2 = TURN_SPD_HIGH; MARKER_FLG3 = 1; } // �ȗ��ω��̊J�n
			                                                                              //  else if (MARKER_FLG3 == 1 ) { SPEED1 = STR_SPD_NORMAL; SPEED2 = TURN_SPD_LOW;  MARKER_FLG3 = 0; } // �ȗ��ω��̏I��
                                                                                                                                                                                            }

               else if (( MARKER_FLG2 == 0 ) && ( GOAL_FLG2 == 1 ) && ( START_FLG == 1 )) {  GOAL_FLG2   = 0;                  RUN_MODE = 0;} // �S�[���[���C���Ɣ��� �� ��~
			   else                                                                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0;              } // �t���O�N���A                                                       
               } 
	         
			 
			                
	   	     if  (( CENTER_L_FLG == 0 ) && ( CENTER_R_FLG == 1 ))     { RUN_MODE = 1; } // �B�E���� L 0
	    else if  (( CENTER_R_FLG == 0 ) && ( CENTER_L_FLG == 1 ))     { RUN_MODE = 2; } // �C������ R 0
		
        else if  ( SIDE_L_FLG   == 1 )                                { RUN_MODE = 4; } // �D���}����
	    else if  ( SIDE_R_FLG   == 1 )                                { RUN_MODE = 3; } // �E�E�}����
		else                                                          { RUN_MODE = 0; } // �F�O�i 
		
		
	      			
	switch (RUN_MODE)
		{
			case 0: // �O�i(�ʏ�j
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (STR_SPD_NORMAL, STR_SPD_NORMAL);  // ���i 22.0 22.0
		 	        break;
		
		 	case 1:	// R_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (20.0 , 0.0);   // �E���� 20.0 0.0
		 	        break;
		
		 	case 2: // L_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // ����i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (0.0 , 20.0);   // ������ 0.0 20.0
		 	        break;
		
			case 3: // R_QTURN:
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i
			        R_IN1 = 0; R_IN2 = 1;   // �E���
			        set_pwm (35.0 ,25.0);   // �E�}���� 30.0 20.0
					break;
		
 		 	case 4: // L_QTURN:
			        L_IN1 = 0; L_IN2 = 1;   // ����i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (25.0 , 35.0);  // ���}���� 20.0 30.0
					break;
			
			
		    case 5: // STOP:�S�[��
			        if (( goal_cnt > 50 ) && ( START_FLG  == 1 ))
					 { 
					   cnt = 0;                                         // �J�E���g�N���A
					   while ( cnt < 500) {set_pwm (18.0 , 18.0);}      // �S�[�������̂Ō������Đi��
			           set_pwm (0.0 , 0.0);                             // ���[�^�[PWM��0�ɂ���
					   PD7 = PD0 = PD3 = 0;                             // 7SEG�h�b�g����
					 		 
                       while(1) SEG_LED  = disp_data[15];               // ��~ "F"�̕\��
					 }
					 		 
				    break;
			
				
            case 6: // �X�^�[�g���C���ʉߏ���
			        START_FLG  = 1;                                    // �X�^�[�g�t���O�𗧂Ă�
					GOAL_FLG2  = 0;                                    // �t���O�N���A
					
			        break;
			

		}
		
  
    }
	
}

//**************************************************************************************
// ���s���W���[�� MODE1 �S�[���Ŏ~�܂�
//**************************************************************************************
void mode_1(void)
{

  STBY        = 1;       // ���[�^�[�X�^���o�C����
  SEG_LED     = 0x00;    // �VSEG LED�̕\���N���A
  SEN_DRV     = 1;       // �Z���T�[�pLED�̔��� ON
  
  STR_SPD_NORMAL = 22.0;    // ���i�ʏ�X�s�[�h 22.0
//STR_SPD_LOW    = 19.0;    // ���i�����X�s�[�h
//TURN_SPD_HIGH  = 21.0;    // �^�[���X�s�[�h�� 20.0
//TURN_SPD_LOW   = 19.0;    // �^�[���X�s�[�h�� 20.0
  
  GOAL_FLG2   = 0;
  MARKER_FLG2 = 0; 
  
  //SPEED1 = STR_SPD_NORMAL;      // �ŏ��͒ʏ�X�s�[�h
  //SPEED2 = TURN_SPD_LOW;
  
  cnt = 0;
  while ( cnt < 1000 ) { LED_cycle == 1; beep(); }   //  �b���҂�
  LED_cycle = 0;
  RUN_MODE = 0;
  
  while(1)
	
    {        
	
		     if  (( GOAL_FLG2   == 1 ) && ( START_FLG == 0 ))                   // �@�X�^�[�g���C����؂���
			      { 
				    L_IN1 = 1; L_IN2 = 0; // ���O�i
			        R_IN1 = 1; R_IN2 = 0; // �E�O�i
			        PD7 = 1;              // 7SEG�h�b�g�_��
			        while ( GOAL_FLG1  == 1 ){ set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL); } // �X�^�[�g���C����ʂ�߂���܂Ői��
					//PD7 = 0;  
				    RUN_MODE = 6;
				  }              
			 
			 
			
        else if  (( MARKER_FLG2 == 1 ) || ( GOAL_FLG2  == 1 ) && ( START_FLG == 1 ))                                // �A���C�����o (�ǂ��炩�����o�j���C����ʉ߂���܂Ŏb���i��
		       {    
				    goal_cnt = 0;
		            L_IN1 = 1; L_IN2 = 0; // ���O�i
			        R_IN1 = 1; R_IN2 = 0; // �E�O�i
					
			              if ( MARKER_FLG2 == 1 ) { while  ( MARKER_FLG1 == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // �}�[�J�[���C��(�N���X���C���j��ʂ�߂���܂Ői��
                                                    cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ������Ɛi��
                                                  }	
													
								   
				    else  if ( GOAL_FLG2   == 1 ) { while  ( GOAL_FLG1   == 1 )           { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // �S�[���[���C��(�N���X���C���j��ʂ�߂���܂Ői��
					                                cnt = 0;
                                                    while  ( cnt < 40 )                   { set_pwm (STR_SPD_NORMAL , STR_SPD_NORMAL );} // ���C�������S�ɒʂ�߂���܂Ői��
					                              }	              
					 	 
						     				   
                        if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 1 ))                   {  GOAL_FLG2   = 0; MARKER_FLG2 = 0; RUN_MODE = 0;        } // �N���X���C���Ɣ���   �� ���̂܂ܑO�i
                   else if (( MARKER_FLG2 == 1 ) && ( GOAL_FLG2 == 0 ))                   {                   MARKER_FLG2 = 0; RUN_MODE = 0; beep();  // �}�[�J�[���C���Ɣ��� �� 
			   
                                                                                          //       if (MARKER_FLG3 == 0 ) { SPEED1 = STR_SPD_LOW ;   SPEED2 = TURN_SPD_HIGH; MARKER_FLG3 = 1; } // �ȗ��ω��̊J�n
			                                                                              //  else if (MARKER_FLG3 == 1 ) { SPEED1 = STR_SPD_NORMAL; SPEED2 = TURN_SPD_LOW;  MARKER_FLG3 = 0; } // �ȗ��ω��̏I��
                                                                                                                                                                                            }

                   else if (( MARKER_FLG2 == 0 ) && ( GOAL_FLG2 == 1 ) && ( START_FLG == 1 )) {  GOAL_FLG2   = 0;                  RUN_MODE = 5;} // �S�[���[���C���Ɣ��� �� ��~
			       else                                                                       {  GOAL_FLG2   = 0; MARKER_FLG2 = 0;              } // �t���O�N���A                                                       
               } 
	                     
						 
						 
						    
	   	else if  (( CENTER_L_FLG == 0 ) && ( CENTER_R_FLG == 1 ))                         { RUN_MODE = 1; } // �B�E���� L 0
		else if  (( CENTER_R_FLG == 0 ) && ( CENTER_L_FLG == 1 ))                         { RUN_MODE = 2; } // �C������ R 0
		
        else if  (( SIDE_L_FLG   == 1 ) && ( SIDE_R_FLG   == 0 ))                         { RUN_MODE = 4; } // �D���}����
	    else if  (( SIDE_R_FLG   == 1 ) && ( SIDE_L_FLG   == 0 ))                         { RUN_MODE = 3; } // �E�E�}����
		else                                                                              { RUN_MODE = 0; } // �F�O�i 
		
		
	      			
	switch (RUN_MODE)
		{
			case 0: // �O�i(�ʏ�j
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (STR_SPD_NORMAL, STR_SPD_NORMAL);  // ���i 22.0 22.0
		 	        break;
		
		 	case 1:	// R_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i L_IN1 = 1; L_IN2 = 0;
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i R_IN1 = 1; R_IN2 = 0;
			        set_pwm (20.0 , 0.0);   // �E���� 20.0 0.0
		 	        break;
		
		 	case 2: // L_TURN:
			        L_IN1 = 1; L_IN2 = 0;   // ����i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (0.0 , 20.0);   // ������ 0.0 20.0
		 	        break;
		
			case 3: // R_QTURN:
			        L_IN1 = 1; L_IN2 = 0;   // ���O�i
			        R_IN1 = 0; R_IN2 = 1;   // �E���
			        set_pwm (35.0 , 25.0);
					break;
		
 		 	case 4: // L_QTURN:
			        L_IN1 = 0; L_IN2 = 1;   // ����i
			        R_IN1 = 1; R_IN2 = 0;   // �E�O�i
			        set_pwm (25.0 , 35.0);  // ���}���� 20.0 30.0
					break;
			
			
		    case 5: // STOP:�S�[��
			        if (( goal_cnt > 50 ) && ( START_FLG  == 1 ))
					 { 
					   cnt = 0;                                         // �J�E���g�N���A
					   while ( cnt < 500) {set_pwm (18.0 , 18.0);}      // �S�[�������̂Ō������Đi��
			           set_pwm (0.0 , 0.0);                             // ���[�^�[PWM��0�ɂ���
					   PD7 = PD0 = PD3 = 0;                             // 7SEG�h�b�g����
					 		 
                       while(1) SEG_LED  = disp_data[15];               // ��~ "F"�̕\��
					 }
					 		 
				    break;
			
				
            case 6: // �X�^�[�g���C���ʉߏ���
			        START_FLG  = 1;                                    // �X�^�[�g�t���O�𗧂Ă�
					GOAL_FLG2  = 0;                                    // �t���O�N���A
					
			        break;
			

		}
		
  
    }
	
}

 

//------------------------------------------------------------------------------------------------------
//    �e���[�h�ł̎��s����
//------------------------------------------------------------------------------------------------------
void start_exe()

{
     wait(1000);                                   // �`���^�����O�΍�
     while(START_SW == SW_OFF);                    // ���s�X�C�b�`���������܂ő҂�
     //beep(4);                                    // �X�^�[�g�X�C�b�`�̍��}
                
                                                   // �e���[�h�ł̎��s
     if      (mode ==  0) mode_0(); // ���[�h0   
     else if (mode ==  1) mode_1(); // ���[�h1   
     else if (mode ==  2) ;// mode_2(); // ���[�h2   
     else if (mode ==  3) ;// mode_3(); // ���[�h3   
     else if (mode ==  4) ;// mode_4(); // ���[�h4  
     else if (mode ==  5) ;// mode_5(); // ���[�h5   
     else if (mode ==  6) ;// ���[�h6  
     else if (mode ==  7) ;// mode_7(); // ���[�h7  
     else if (mode ==  8) ;// mode_8(); // ���[�h8  
     else if (mode ==  9) ;// mode_9(); // ���[�h9  
     else if (mode == 10) ;// mode_10();// ���[�h10 A
     else if (mode == 11) ;// mode_11();// ���[�h11 B
     else if (mode == 12) ;// mode_12();// ���[�h12 C
     else if (mode == 13) ;// mode_13();// ���[�h13 D
     else if (mode == 14) ;// mode_14();// ���[�h14 E
     else if (mode == 15) ;// mode_15();// ���[�h15 F
}

	 
//--------------------------------------------------------------------------
//		���C���v���O���� 												
//--------------------------------------------------------------------------
void  main(void)
{
  
  init();      // ������
  init_CMT();  // CMT�̏�����
  mtu_init();  // MTU0,MTU1�̏�����
  
  beep();   
      
  mode = 0;
  
 
  // ���[�h�ؑ֏��� ----------------------------------------------------------------------------
  while(1)
  { 
           if( MODE_SW == SW_ON)                    // ���[�h�X�C�b�`�͉�����Ă���  
              {                 
                      mode ++;                      // ���[�h��1������
                      
                      beep();                       // ���[�h�ؑ֎��r�[�v���̔���
                      wait(500000);                 // �`���^�����O�΍�
                      while(!MODE_SW == SW_OFF);    // ���[�h�X�C�b�`���������܂ő҂�
              }
                
               SEG_LED  = disp_data[mode];          // 7seg�փ��[�h�\��  
               if( mode == 16 )   mode = 0;         // ���[�h16�܂ŗ�����0�ɖ߂�
               if( START_SW == SW_ON ) start_exe(); // ���s���[�h��
   }                    
     
}    


//--------------------------------------------------------------------------
//		���荞�݃v���O���� CMT0 (SENSOR, LED Cycle, Switch�p�j										
//--------------------------------------------------------------------------
#pragma interrupt (Excep_CMTU0_CMT0(vect=28))
void Excep_CMTU0_CMT0(void)
{
		
	      task ++;                                            // �^�X�N�̍X�V						
      if (task == 9) task = 0;                                // 9�܂ŗ�����O�ɃN���A
      
   switch(task)                                               //�^�X�N�|�C���^�ɏ]���ď������s��			
   {
	
	
	case 0: // 1msec�^�C�}�[�̊m�F�p
            PORTE.DR.BIT.B2 ^= 1;                    // TEST PE2��1mS�̃p���X�𔭐�
            break;                                   // ���m�Ƀp���X���o�Ă��邩�I�V���Ŋm�F�p
	
	// �Z���T�[�̏���
	
	case 1: // GOAL 
	        //SEN_DRV = 1;                            // �Z���T�[�pLED�̔��� ON
			AD0.ADCSR.BYTE =0x60;   			    // A/D�ϊ��J�n
			while(AD0.ADCSR.BIT.ADST == 0);		    // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                            // �Z���T�[�pLED�̔��� OFF
			GOAL = AD0.ADDRA;				        // �f�[�^���擾
			GOAL_LIM = 0x2F;	                    // GOAL�Z���T�[�����臒l   2F
			    if(GOAL > GOAL_LIM) 
	             { GOAL_FLG1 = 1; GOAL_FLG2 = 1; PD0 = 1;} // GOAL���C�������o������t���O�𗧂Ă�
	        else { GOAL_FLG1 = 0;                PD0 = 0;}
	        break;
			
			
	case 2: // SIDE_R
	        //SEN_DRV = 1;                            // �Z���T�[�pLED�̔��� ON
	        AD0.ADCSR.BYTE =0x61;   			    // A/D�ϊ��J�n
			while(AD0.ADCSR.BIT.ADST == 0);		    // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                            // �Z���T�[�pLED�̔��� OFF
			SIDE_R = AD0.ADDRB;				        // �f�[�^���擾
			SIDE_R_LIM = 0x2D;	                    // SIDE_R�Z���T�[�����臒l 2D
			    if(SIDE_R > SIDE_R_LIM)
			     { SIDE_R_FLG = 1; PD2 = 1;}        // SIDE_R ���C�������o������t���O�𗧂Ă�
			else { SIDE_R_FLG = 0; PD2 = 0;}
			break;
	
	case 3: // CENTER_R 
	        //SEN_DRV = 1;                             // �Z���T�[�pLED�̔��� ON
            AD0.ADCSR.BYTE =0x62;   			     // A/D�ϊ��J�n
			while(AD0.ADCSR.BIT.ADST == 0);		     // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                             // �Z���T�[�pLED�̔��� OFF
			CENTER_R = AD0.ADDRC;				     // �f�[�^���擾
			CENTER_R_LIM = 0x2D;	                 // CENTER_R�Z���T�[�����臒l 2D
			    if(CENTER_R > CENTER_R_LIM) 
			     { CENTER_R_FLG = 1; PD1 = 1;}       // SIDE_R ���C�������o������t���O�𗧂Ă� 
			else { CENTER_R_FLG = 0; PD1 = 0;}
            break;
			
	case 4: // CENTER_L
	        //SEN_DRV = 1;                            // �Z���T�[�pLED�̔��� ON
            AD0.ADCSR.BYTE =0x63;   			      // A/D�ϊ��J�n
			while(AD0.ADCSR.BIT.ADST == 0);		      // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                              // �Z���T�[�pLED�̔��� OFF
			CENTER_L = AD0.ADDRD;				      // �f�[�^���擾
			CENTER_L_LIM = 0x2D;	                  // CENTER_L�Z���T�[�����臒l 2D
		         if(CENTER_L > CENTER_L_LIM)
			      { CENTER_L_FLG = 1; PD5 = 1;}       // SIDE_R ���C�������o������t���O�𗧂Ă�
			else  { CENTER_L_FLG = 0; PD5 = 0;}       // 
			break;
				
	case 5: // SIDE_L
	        //SEN_DRV = 1;                            // �Z���T�[�pLED�̔��� ON
            AD1.ADCSR.BYTE =0x60;   			      // A/D�ϊ��J�n
	        while(AD1.ADCSR.BIT.ADST == 0);		      // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                            // �Z���T�[�pLED�̔��� OFF
	        SIDE_L = AD1.ADDRA;				          // �f�[�^���擾
			SIDE_L_LIM = 0x2D;	                      // SIDE_L�Z���T�[�����臒l 2D
			     if(SIDE_L > SIDE_L_LIM)
			      { SIDE_L_FLG = 1; PD4 = 1;}         // SIDE_R ���C�������o������t���O�𗧂Ă� 
			else  { SIDE_L_FLG = 0; PD4 = 0;} 
			break;
				
	case 6: // MARKER 
	        //SEN_DRV = 1;                            // �Z���T�[�pLED�̔��� ON
            AD1.ADCSR.BYTE =0x61;   			      // A/D�ϊ��J�n
			while(AD1.ADCSR.BIT.ADST == 0);		      // ���肪�I������܂ő҂�
			//SEN_DRV = 0;                            // �Z���T�[�pLED�̔��� OFF
			MARKER = AD1.ADDRB;				          // �f�[�^���擾
			MARKER_LIM = 0x2F;	                      // MERKER�Z���T�[�����臒l 2F
			    if(MARKER > MARKER_LIM)
			     { MARKER_FLG1 = 1; MARKER_FLG2 = 1; PD3 = 1;} // SIDE_R ���C�������o������t���O�𗧂Ă� 
	        else { MARKER_FLG1 = 0;                  PD3 = 0;} // ���΂炭������}�[�J�[�t���O�N���A
	        break;
			
			
	case 7: // LED�T�C�N���_������ --------------------------------------------------
  
          /*  if(LED_cycle == 1)
            {
                if (LED_speed >= 200)                           // �_����]�̑��x
                  {
                      LED_speed = 0;
                      SEG_LED = disp_cycle_data[dig];
                      dig ++;
                      if (dig == 6) dig = 0;                    // �U�܂ōs������O�ɖ߂�
                  }

              else{  LED_speed ++;  } 
            } 
			*/
            break;
			
			
	case 8: // �J�E���g
	        cnt ++ ;
			cnt1 ++ ;
			wait_timer++;
			goal_cnt ++;
			
			break;
	
    default: break;
	
	  }
}


//--------------------------------------------------------------------------
//		���荞�݃v���O���� CMT1 (BEEP��p�j										
//--------------------------------------------------------------------------
#pragma interrupt (Excep_CMTU0_CMT0(vect=29))
void Excep_CMTU0_CMT1(void)
{
	
	if ( beep_long < 100 )    // beep_long BEEP���̒���
 	   {
	  		beep_long ++;     //
			BEEP ^= 1;        // ���]���ăp���X�̔���
       }


  if(LED_cycle == 1)
        {
                if (LED_speed >= 200)                           // �_����]�̑��x
                  {
                      LED_speed = 0;
                      SEG_LED = disp_cycle_data[dig];
                      dig ++;
                      if (dig == 6) dig = 0;                    // �U�܂ōs������O�ɖ߂�
                  }

              else{  LED_speed ++;  } 
         } 
 

}











//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif


#ifdef __cplusplus
void abort(void)
{

}
#endif
