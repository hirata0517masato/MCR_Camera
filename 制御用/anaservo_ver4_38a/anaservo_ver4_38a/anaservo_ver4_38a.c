/****************************************************************************/
/* �Ώۃ}�C�R�� R8C/38A                                                     */
/* ̧�ٓ��e     ���[�^�h���C�u���TypeD Ver.1.0�E							*/
/*              ���g�p�����}�C�R���J�[�g���[�X�v���O����                    */
/* �o�[�W����   Ver.0.00                                                    */
/* Date         2017.02.10                                                  */
/* Copyright                            */
/****************************************************************************/

/*
�{�v���O�����́A
�����[�^�h���C�u���TypeTypeD Ver.1.0
��TSL1401
���g�p�����}�C�R���J�[�𓮍삳����v���O�����ł��B
*/

/*======================================*/
/* �C���N���[�h                         */
/*======================================*/
#include <stdio.h>
#include "sfr_r838a.h"                  /* R8C/38A SFR�̒�`�t�@�C��    */
#include "printf_lib.h"                 /* printf�g�p���C�u����         */
#include "types3_beep.h"                /* �u�U�[�ǉ�                   */
#include "microsd_lib.h"                /* microSD���䃉�C�u����        */
#include "data_flash_lib.h"

/*======================================*/
/* �V���{����`                         */
/*======================================*/
/* �萔�ݒ� */
#define     TRC_MOTOR_CYCLE     20000   /* ���O,�E�O���[�^PWM�̎���     */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     TRD_MOTOR_CYCLE     20000   /* ����,�E��,����Ӱ�PWM�̎���   */
                                        /* 50[ns] * 20000 = 1.00[ms]    */
#define     FREE                1       /* ���[�^���[�h�@�t���[         */
#define     BRAKE               0       /* ���[�^���[�h�@�u���[�L       */

#define 	SERVO_MAX 			125	  	/* �n���h���ő�ʒu 115           */

#define 	MAXTIME 			300	  	/* �ő呖�s���� (0.01�b)  1200 = 12s     1250     */


/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void init( void );
unsigned char dipsw_get( void );
unsigned char dipsw_get2( void );
unsigned char pushsw_get( void );
unsigned char cn6_get( void );
void led_out( unsigned char led );
void motor_r( int accele_l, int accele_r );
void motor_f( int accele_l, int accele_r );
void motor2_r( int accele_l, int accele_r );
void motor2_f( int accele_l, int accele_r );
void motor_mode_r( int mode_l, int mode_r );
void motor_mode_f( int mode_l, int mode_r );
void servoPwmOut( int pwm );
unsigned char check_startgate( void );
unsigned char check_crossline( void );
unsigned char check_wideline( void );
unsigned char check_halfline( void );//1 = ��, 2 = �E
int getServoAngle( void );
void servoControl( void );
void servoControl2( void );
int diff( int pwm );
void cam_in(void);
void wait(int);
unsigned long convertBCD_CharToLong( unsigned char hex );
int camera(int, int);
void mode_out(void);
void get_angle_y(void);
void get_angle_x(void);
int angle_check(void);
int date_f_make(int, int , int,int);
int IR_L(void);
int IR_R(void);
void Get_Center_IR(void);
void IRcalibration( void );



/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
int             pattern;                /* �}�C�R���J�[����p�^�[��     */
int             crank_mode;             /* 1:�N�����N���[�h 0:�ʏ�      */
unsigned long   cnt1 = 0;               /* �^�C�}�p                     */
unsigned long   cnt2 = 0;               /* �ő呖�s���ԗp               */
unsigned long   cnt3 = 0;               /* wait �֐��p                  */
unsigned long   cnt4 = 0;               /* �E���`�F�b�N�p               */
unsigned long   cnt5 = 0;               /* ��Z���T�[�`�F�b�N�p         */
unsigned long   cnt6 = 0;               /* �E���`�F�b�N�p(�G���R�[�_)   */
unsigned long   cnt7 = 0;               /* �����ƃJ�[�u�̃J�E���g�p   */
unsigned long   cnt8 = 0;               /* �J�[�u�����p�̃J�E���g�p   */
int 			run = 0;				/* 1 = ���s��					*/
int				mode = 0;				/* 0 = �ʏ� 1 = ��	2 = �E���� 3 = ������	*/
int				flag2 = 0;				/* ���� = �������Ȃ� � = ��������*/
int				out_cnt = 0;			//�E���J�E���g
int 			black_flag = 0;			//
int				Cu_flag = 0;			//0 = ����, 1 = �J�[�u
int 			gate = 0;

/* microSD�֘A�ϐ� */
signed char     msdBuff[ 512 ];         /* �ꎞ�ۑ��o�b�t�@             */
int             msdBuffAddress;         /* �ꎞ�L�^�o�b�t�@�����A�h���X */
int             msdFlag = 0;                /* 1:�f�[�^�L�^ 0:�L�^���Ȃ�    */
int             msdTimer;               /* �擾�Ԋu�v�Z�p               */
unsigned long   msdStartAddress;        /* �L�^�J�n�A�h���X             */
unsigned long   msdEndAddress;          /* �L�^�I���A�h���X             */
unsigned long   msdWorkAddress;         /* ��Ɨp�A�h���X               */
int             msdError = 0;               /* �G���[�ԍ��L�^               */

signed char     msdBuff_ch[ 512 ];         /* �ꎞ�ۑ��o�b�t�@             */
unsigned long   msdStartAddress_ch = 0x0200;        /* �L�^�J�n�A�h���X             */

int				savecnt = 0;				/*�o�͎��̃`�F�b�N�p		*/
int logfin = 0;

/* ���݂̏�ԕۑ��p */
int             handleBuff;             /* ���݂̃n���h���p�x�L�^       */
int             FleftMotorBuff = 0;          /* ���݂̑O�����[�^PWM�l�L�^      */
int             FrightMotorBuff = 0;         /* ���݂̑O�E���[�^PWM�l�L�^      */
int             RleftMotorBuff = 0;          /* ���݂̌㍶���[�^PWM�l�L�^      */
int             RrightMotorBuff= 0;         /* ���݂̌�E���[�^PWM�l�L�^      */


/* �}�C�R�����t���b�V���������֘A */
int				date_f_mode = 0;		//0=�Ȃ� 1=IN 2=OUT
signed char 	date_f_buff[32] ={0};
int			 	date_f_buff_int[16] ={0};
int				date_f_num = 0;
int				Cu_Angle	=		20;		//�J�[�u����Ɏg�p ��������
signed char 	date_f_buff_ch[32] ={0};
int			 	date_f_buff_ch_int[32] ={0};//�������p�^�[���@�������
int				date_f_num_ch = 0;

/*�N�����N�A�n�[�t�����v���p*/
int			 	date_buff_ch_int[32] ={0};//�������p�^�[���@�������
int				date_num_ch = 0;
long            lEncoderTotal_ch = 0;          /* �ώZ�l�ۑ��p                 */

/* �G���R�[�_�֘A */
int             iTimer10 = 0;               /* 10ms�J�E���g�p               */
long            lEncoderTotal = 0;          /* �ώZ�l�ۑ��p                 */
unsigned long   lEncoderTotal2 = 0;          /* �ώZ�l�ۑ��p                 */
int             iEncoder10 = 0;               /* 10ms���̍ŐV�l               */
int             iEncoder5  = 0;               /*  5ms���̍ŐV�l               */
unsigned int    uEncoderBuff  = 0;           /* �v�Z�p�@���荞�ݓ��Ŏg�p     */
long			sp = 0;						/* �����v���p�X�^�[�g�n�_ */
long			sp2 = 0;						/* �����v���p�X�^�[�g�n�_ */
long            SEncoderTotal = 0;          /* �����ώZ�l�ۑ��p                 */

/*  �T�[�{�֘A */
int             iSensorBefore = 0;          /* �O��̃Z���T�l�ۑ�           */
int             iSensorBeforeIR = 0;          /* �O��̃Z���T�l�ۑ�           */
int             iAngleBefore2 = 0;
int             iServoPwm = 0;              /* �T�[�{�o�v�l�l               */
int             iServoPwm2 = 0;
int             iAngle0 = 0;                /* ���S����A/D�l�ۑ�            */
int 			iSetAngle = 0;
int				iAngleBefore = 0;

/* �Z���T�֘A */
int             iSensorPattern;         /* �Z���T��ԕێ��p             */
int  			Center;					/*�J�����Z���^�[	0~127				*/
int			    Wide;					/*������			0~127			*/
int  			Center_old;					/*�J�����Z���^�[				*/
int			    Wide_old;					/*������						*/
int  			Center_offset = 0;		/*�J�����Z���^�[���ړ����񂹂�				*/
int 			angle_y = 0;				/* �W���C���Z���T�[��Y���̒l	*/
int 			angle_x = 0;				/* �W���C���Z���T�[��X���̒l	*/
int 			pre_angle_y[2] = {0};		/* �W���C���Z���T�[��Y���̉ߋ��̒l	*/
int 			pre_angle_x[2] = {0};		/* �W���C���Z���T�[��X���̉ߋ��̒l	*/
int 			Center_IR = 0;
int 			IR_flag = 0;
int 			IR_max[2] = {0};
int				IR_min[2] = {1024,1024};
int 			IR_cnt = 0;
int 			IR_old = 0;


/* TRC���W�X�^�̃o�b�t�@ */
unsigned int    trcgrb_buff;            /* TRCGRB�̃o�b�t�@             */
unsigned int    trcgrd_buff;            /* TRCGRD�̃o�b�t�@             */

/* ���[�^�h���C�u���TypeS Ver.3���LED�A�f�B�b�v�X�C�b�`���� */
unsigned char   types_led;              /* LED�l�ݒ�                    */
unsigned char   types_dipsw;            /* �f�B�b�v�X�C�b�`�l�ۑ�       */

/*	�p�����[�^	*/
//�I�t�Z�b�g
int  		Center_offset_MAX = 4;		/*�J�[�u���J�����Z���^�[���ړ����񂹂� �ŏ��l 0 �ő�l	5			*/
int  		Center_offset_Angle = -20;	/*���̒l�ɂ��P�h�m���Ɋ񂹂�	���FIN�@���FOUT		*/


int			KASOKU = 15;	

//�ʏ�

int			MOTOR_out_base	=		 95;		//�J�[�u�O���p�@�O�����[�^�[�p�p�����[�^�[ 

//////////////////////////////////////////// 0:�֎~ 1��-1�͓���

int		    TOPSPEED	=		50;		//���� 44

//�O��
int			SPEED_DOWN	=		8;		//�p�x�ɂ��TOPSPEED������ �J�[�u�O��
int			MOTOR_out_R	=		 -2;		//�O�����[�^�[�p�p�����[�^�[ -2
int			MOTOR_in_F	=		 1;		//�������[�^�[�p�p�����[�^�[ 2
int			MOTOR_in_R	=		 -2;		//�������[�^�[�p�p�����[�^�[ -2
	
//�㔼
int			SPEED_DOWN_N=		10;		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
int			MOTOR_out_R_N=		1;		//�O�����[�^�[�p�p�����[�^�[ �㔼
int			MOTOR_in_F_N=		2;		//�������[�^�[�p�p�����[�^�[�@�㔼
int			MOTOR_in_R_N=		1;		//�������[�^�[�p�p�����[�^�[�@�㔼


int			S_para		=		1;		//S�����肩�����p�p�����[�^
int			OUT_M_DOWN	=		2;		//�J�[�u�O���u���[�L�p�{��

#define		date_f_brake		650	//�Đ����s���̃u���[�L�g�p�\����(mm)

#define		Cu_FREE_time  		10		//�J�[�u�I�����̌�փt���[�̎���(msec�j

#define		Cu_BRAKE_time  		5		//�J�[�u�i�����̃u���[�L���� (msec)
#define		Cu_BRAKE_SP 		43		//�J�[�u�i�����ɂ��̑��x�ȏ�Ȃ�u���[�L
#define		Cu_BRAKE			-10	//�J�[�u�i�����̃u���[�L

#define		Cu_N_time			200	//Cu_N_time ms �J�[�u�𑖍s����ƌ㔼�ɂȂ� 	

//��
int			saka_max	  =		  1;	//�F���\�ȍ�̐�
#define 	KASA_Encoder1  	400	//��J�n
#define 	KASA_Encoder2  	700	//���I��� 
#define		KASA_Encoder3  	3800	//���I���  1450 2800 3800
#define		KASA_Encoder4  	5000	//����I��� �ʏ�ɂ��ǂ� 4000 5000
//�Ζ�(����)
#define		    TOPSPEED4			48		//����(��j30 33
#define			SPEED_DOWN4			10		//�p�x�ɂ��TOPSPEED������(��j�J�[�u�O��
#define			SPEED_DOWN4_N		10		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out4_R		100	//�O�����[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in4_F			10		//�������[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in4_R			10		//�������[�^�[�p�p�����[�^�[(��j

#define			S_para4				2		//S�����肩�����p�p�����[�^(��j
#define			OUT_M_DOWN4			2		//�J�[�u�O���u���[�L�p�{��(��j
//�Ζ�(���)
#define		    TOPSPEED3			45		//����(��j30 33
#define			SPEED_DOWN3			10		//�p�x�ɂ��TOPSPEED������(��j�J�[�u�O��
#define			SPEED_DOWN3_N		10		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out3_R		100	//�O�����[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in3_F			10		//�������[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in3_R			10		//�������[�^�[�p�p�����[�^�[(��j

#define			S_para3				2		//S�����肩�����p�p�����[�^(��j
#define			OUT_M_DOWN3			2		//�J�[�u�O���u���[�L�p�{��(��j
//��
#define		    TOPSPEED2			37		//����(���j30 33
#define			SPEED_DOWN2			6		//�p�x�ɂ��TOPSPEED������(���j�J�[�u�O��
#define			SPEED_DOWN2_N		6		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out2_R		-1		//�O�����[�^�[�p�p�����[�^�[(���)
#define			MOTOR_in2_F			-3		//�������[�^�[�p�p�����[�^�[(���j
#define			MOTOR_in2_R			-3		//�������[�^�[�p�p�����[�^�[(���j

#define			S_para2				2		//S�����肩�����p�p�����[�^(���j
#define			OUT_M_DOWN2			4		//�J�[�u�O���u���[�L�p�{��(���j


//�N�����N
int		    C_TOPSPEED	=		32;		//�N�����N(��)  25 33
int		    C_TOPSPEED2	=		50;		//�N�����N(�o)	40

int 		C_TOPSPEED4 = 		47;		//�Đ����s���̃u���[�L�O
int		    C_TOPSPEED3	=		40;		//�N�����N(��)  25 33 �Đ����s�p

int			date_f_brake_c	=	500;	//�Đ����s���̃u���[�L�g�p�\����(mm) �N�����N�p
int			date_f_shortcat_c=	260;	//�Đ����s���̃V���[�g�J�b�g����(mm) �N�����N�p 210

int			c_cut_master  	 =	  1;	//�Đ����s���ł����Ă� 0= �Đ����Ȃ� 1= �Đ�����				
int			c_cut_encoder	 =	540;  	//���̋��������̏ꍇ�͍Đ����Ȃ�

int			c_cut;	//0= �Đ����Ȃ� 1= �Đ����� �ҏW���Ӗ�
//int c_cnt = 0; //�N�����N�̉�

//�n�[�t 
int		    H_TOPSPEED	=		50;		//�n�[�t�i�N���j37 �ǂȂ� 47 46
int		    H_TOPSPEED2	=		44;		//�n�[�t(�΂�)  31 �ǂȂ� 45
int		    H_TOPSPEED2_S=		50;		//�n�[�t(�΂�)  �V���[�g�J�b�g�p
int			date_f_brake_h	=	500;	//�Đ����s���̃u���[�L�g�p�\����(mm)�@�n�[�t�p 
int			date_f_shortcat_h=	300;		//�Đ����s���̃V���[�g�J�b�g����(mm)�@�n�[�t�p

int			date_f_plus_h	=	500;		//�Đ����s���̒���̃X�g���[�g�����␳(mm)�@�n�[�t�p  
int			h_cut 			 =	  1;	//�Đ����s���ł����Ă� 0= �Đ����Ȃ� 1= �Đ����� 

///////////////////////////////////

int			BRAKE_MAX	=		-90;	//�u���[�L�̍ő�p���[ 

int			S_flag = 2;				//�⓹�@��������@1 = �������Ȃ�  2 = ��������


int				kp = -18;//- 8  3 -16 -19 -23  -13
int				kd = -110;//-80 20 -130 -190	-110
int 			ki = 0;//-2

int				kp_ir = -20;
int				kd_ir = -110;
int 			ki_ir = 0;


int				kp2 = -10;//�p�x�w��p
int				kd2 = -50;//�p�x�w��p


/*	���̑�	*/
int			topspeed;		
int			speed_down;
int			speed_down_n;
int			motor2_out_R;
int			motor2_in_F;
int			motor2_in_R;
int			s_para;
int			out_m_down;


/************************************************************************/
/* ���C���v���O����                                                     */
/************************************************************************/
void main( void )
{
	int             i, j,old_i;
    unsigned int    u;
    char            c;
	int x,f,r,or,jj = 0;
	int     ret;
	
    /* �}�C�R���@�\�̏����� */
    init();                             /* ������                       */
    initBeepS();                        /* �u�U�[�֘A����               */
	init_uart0_printf( SPEED_9600 );    /* UART0��printf�֘A�̏�����    */
	setMicroSDLedPort( &p6, &pd6, 0 );  /* microSD ���j�^LED�ݒ�        */
   
	asm(" fset I ");                    /* �S�̂̊��荞�݋���           */
	    
	// microSD �������݊J�n�A�h���X// 512�̔{���ɐݒ肷��
    msdStartAddress = 5120;

    // microSD �������ݏI���A�h���X
    // �������݂���������[ms] : x = 10[ms] : 64�o�C�g
    // 60000ms�Ȃ�Ax = 60000 * 64 / 10 = 384000
    // ���ʂ�512�̔{���ɂȂ�悤�ɌJ��グ����B
    msdEndAddress  = 768000;//384000;
    msdEndAddress += msdStartAddress;   /* �X�^�[�g������               */                  

	 /* microSD������ */
    ret = initMicroSD();
    if( ret != 0x00 ) {
        msdError = 1;
		printf("%d\n",ret);
        /* �������ł��Ȃ����3�b�ԁALED�̓_�����@��ς��� */
        cnt1 = 0;
        while( cnt1 < 3000 ) {
            if( cnt1 % 200 < 100 ) {
                led_out( 0x3 );
            } else {
                led_out( 0x0 );
            }
        }
    }
	
    /* �}�C�R���J�[�̏�ԏ����� */
    motor_mode_f( FREE, FREE );
 	//motor_mode_f( BRAKE, BRAKE );
	motor_mode_r( FREE, FREE );
	//motor_mode_r( BRAKE, BRAKE );
	
    motor_f( 0, 0 );
    motor_r( 0, 0 );
    servoPwmOut( 0 );
    setBeepPatternS( 0x8000 );
	wait(10);
	
	topspeed = TOPSPEED;		//�����l
	speed_down = SPEED_DOWN;
	speed_down_n = SPEED_DOWN_N;
	motor2_in_F = MOTOR_in_F;
	motor2_in_R = MOTOR_in_R;
	motor2_out_R = MOTOR_out_R;
	s_para = S_para;
	out_m_down = OUT_M_DOWN;
	
	wait(10);
	while(~p8 == 0xff);//�N������͐��l����������
	wait(10);
	
	iAngle0 = 0;
	iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
	types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
	

	//�����x�Z���T�[�l�̏�����
	angle_y = ad0;
	angle_x = ad1;
	
	
	/* �X�^�[�g���A�}�C�R���f�B�b�v�X�C�b�` 1 = ON   �Đ����[�h�@IN */
	if( (dipsw_get() & 0x01) == 0x01 ) {
		date_f_mode = 1;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
	
	/* �X�^�[�g���A�}�C�R���f�B�b�v�X�C�b�` 2 = ON   �Đ����[�h�@OUT */
	}else if( (dipsw_get() & 0x02) == 0x02 ) {
		date_f_mode = 2;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);	
	}else{
		wait(500);	
	}
	
	
	/* �X�^�[�g���A�}�C�R���f�B�b�v�X�C�b�` 4 = ON�@�f�[�^�]�����[�h*/
    if( (dipsw_get() & 0x08) == 0x08 ) {
        pattern = 101;
        cnt1 = 0;	
			
	/* �X�^�[�g���A�f�B�b�v�X�C�b�` 4 = ON   ��������@1 = �������Ȃ�  2 = ��������*/
	}else if( dipsw_get2() == 0x10 ) {
		S_flag = 1;
				
	/* �X�^�[�g���A�f�B�b�v�X�C�b�` 3 = ON   RX�Ƃ̒ʐM�`�F�b�N���[�h*/
	}else if( dipsw_get2() == 0x20) {
		pattern = 500;
	
	}
 


    while( 1 ) {
		
//	I2CEepromProcess();                 /* I2C EEP-ROM�ۑ�����          */

	mode_out();//��t���O���o��
	
	//�l�̕ۑ�
	Center_old = Center;
	Wide_old = Wide;
	
	Get_Center_IR();//�ԊO���Z���T�[
	cam_in();//�l�̎擾

	
	if(Wide == 0 && mode != 1){//���������� && ��łȂ�
		if(black_flag == 1){//�O�����
			
		}else{
			Center = Center_old;
			Wide = Wide_old;
		}
		black_flag = 1;
	}else{
		black_flag = 0;
	}
	
	if(pattern >= 10){
		led_out(0);
	}else if(pattern > 1){
		led_out(camera(Center,Wide));
	}
	
	//�E���`�F�b�N
	if(pattern < 100){
		if(((check_wideline()) || (iEncoder10 < 3) || (mode != 1 && Wide == 0 && Center == 0  && pattern != 53 && pattern != 63 && pattern != 31 && pattern != 41 && pattern != 32 && pattern != 42)) && (pattern >= 10) && (pattern != 70)){
			out_cnt ++;		
		}else{
			out_cnt = 0;
		}
		
		if((lEncoderTotal > 500) && (out_cnt > 500)){
			pattern = 200;	
		}
	}
	
    switch( pattern ) {	
    case 0:
        
		/* �v�b�V���X�C�b�`�����҂� */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			iAngle0 = 0;
			iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
			
			IRcalibration( );
			
			
 		/*	while(1){
				//printf("%4d  %4d  %4d\n",IR_L(), IR_R(),IR_R() - IR_L());
				printf("%4d  %4d  %4d  %4d\n",IR_R() ,ad7 ,IR_min[1],IR_max[1]);				
			}*/
			
            setBeepPatternS( 0xcc00 );
            cnt1 = 0;
            pattern = 5;
            break;
        }
		
        led_out(camera(Center,Wide));
		
        break;
	case 5:
		/* �v�b�V���X�C�b�`�����҂� */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			iAngle0 = 0;
			iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
			
			
			ret = eraseMicroSD( msdStartAddress, msdEndAddress-1 );
            if( ret != 0x00 ) {
                /* �C���[�Y�ł��� */
                msdError = 2;
            }
            /* microSDProcess�J�n���� */
            ret = microSDProcessStart( msdStartAddress );
            if( ret != 0x00 ) {
                /* �J�n�����ł��� */
                msdError = 3;
            }
			
			if(date_f_mode == 1){
				readDataFlash( 0x3000, date_f_buff, 32 );
				readDataFlash( 0x3400, date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					date_f_buff_int[j] = (int)(date_f_buff[i])*100 + (int)date_f_buff[i+1];
					j++;	
				}
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					date_f_buff_ch_int[j] =(int)date_f_buff_ch[i];
					j++;
					
					date_f_buff_ch_int[j] = (int)(date_f_buff_ch[i+1])*100 + (int)date_f_buff_ch[i+2];
					j++;
				}

	
				
			/*	
					//31�F�E�N�����N 41�F���N�����N 53:���n�[�t 63:�E�n�[�t
					//�N�����N�@����@�߂��F200�@�����F450
					//�n�[�t�@����@350
					 
				date_f_buff_ch_int[4] = 41;
				date_f_buff_ch_int[6] = 41;
				date_f_buff_ch_int[0] = 63;
				date_f_buff_ch_int[2] = 31;
			
				date_f_buff_ch_int[5] = 200 + 550;
				date_f_buff_ch_int[7] = 200 + 600;
				date_f_buff_ch_int[1] = 350 + 350;
				date_f_buff_ch_int[3] = 450 + 50;
			
				
				//����
				date_f_buff_int[0] = 600 * 2;
				date_f_buff_int[1] = 600 * 4;
				date_f_buff_int[2] = 600 * 3 - 100;
				date_f_buff_int[3] = 600 * 6;
				date_f_buff_int[4] = 600 * 2 + 100;
				date_f_buff_int[5] = 600 * 2 - 100;
				date_f_buff_int[6] = 600 * 3;
				date_f_buff_int[7] = 600 * 4;
				date_f_buff_int[8] = 600 * 8;
				*/
			}else if(date_f_mode == 2){
				readDataFlash( 0x3800, date_f_buff, 32 );
				readDataFlash( 0x3c00, date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					date_f_buff_int[j] = (int)(date_f_buff[i])*100 + (int)date_f_buff[i+1];
					j++;	
				}
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					date_f_buff_ch_int[j] =(int)date_f_buff_ch[i];
					j++;
					
					date_f_buff_ch_int[j] = (int)(date_f_buff_ch[i+1])*100 + (int)date_f_buff_ch[i+2];
					j++;
				}
			
				
				/*
					//31�F�E�N�����N 41�F���N�����N 53:���n�[�t 63:�E�n�[�t
					//�N�����N�@����@�߂��F200�@�����F450
					//�n�[�t�@����@350
					 
				date_f_buff_ch_int[0] = 41;
				date_f_buff_ch_int[2] = 41;
				date_f_buff_ch_int[4] = 63;
				date_f_buff_ch_int[6] = 31;
			
				date_f_buff_ch_int[1] = 200 + 550;
				date_f_buff_ch_int[3] = 200 + 600;
				date_f_buff_ch_int[5] = 350 + 350;
				date_f_buff_ch_int[7] = 450 + 50;
			
				
				//����
				date_f_buff_int[0] = 600 * 3 + 100;
				date_f_buff_int[1] = 600 * 2 + 300;
				date_f_buff_int[2] = 600 * 2 - 200;
				date_f_buff_int[3] = 600 * 3;
				date_f_buff_int[4] = 600 * 8 + 100;
				date_f_buff_int[5] = 600 * 4;
				date_f_buff_int[6] = 600 * 3 - 100;
				date_f_buff_int[7] = 600 * 6;
				*/
			}	
			
			
			date_f_num_ch = 0;
			
            setBeepPatternS( 0xcc00 );
            cnt1 = 0;
            pattern = 1;
            break;
        }
		
		if(angle_check() != 1){ //��Z���T�[�`�F�b�N
			setBeepPatternS( 0x8000 );
			
		}
		
        
        if( check_startgate() ) {
             if(((cnt1/100) % 8)%2 == 0)led_out( 0x81);
			 else led_out(camera(Center,Wide));
			 
        }else{
			led_out(camera(Center,Wide));
		}
		break;
    case 1:
        /* �X�^�[�g�o�[�J�҂� */
		servoPwmOut( 0 );
		
		motor_f( 0, 0 );
        motor_r( 0, 0 );
			
		old_i = iAngle0;
		iAngle0 = 0;
        iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
		iAngle0 = (iAngle0 + old_i) >> 1;
		
        if(  (!check_startgate()) && (Wide != 0)) {//�Q�[�g���������@&& ���C����������
		//	iAngle0 = 0;
        //    iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
			//if(cnt1 > 1000)gate = 1;
			gate = 1;
            cnt1 = 0;
			cnt2 = 0;
			cnt7 = 0;
			run = 1;//���s�J�n
			lEncoderTotal = 0; 
			lEncoderTotal2 = 0; 
			sp = 0;
			sp2 = 0;
			
			flag2 = 0;//�⓹�̉�
			
			msdBuffAddress = 0;
            msdWorkAddress = msdStartAddress;
            msdFlag = 1;                /* �f�[�^�L�^�J�n               */
		
            pattern = 10;
            break;
			
        }
		
		/*if( pushsw_get() ) {//�{�^���������ꂽ �Q�[�g�ɋ߂Â��ăX�^�[�g���郂�[�h
			setBeepPatternS( 0x8000 );
			wait(1000);
			pattern = 2;
            break;
		}*/
        led_out( 1 << (cnt1/50) % 8 );
        break;
	/*
	case 2:
		if(  (!check_startgate()) && (Wide != 0)) {//�Q�[�g���������@&& ���C����������
			setBeepPatternS( 0x8000 );
			wait(1000);
			pattern = 3;
            break;
		}
		break;
		
	case 3:

		old_i = iAngle0;
		iAngle0 = 0;
        iAngle0 = getServoAngle();  // 0�x�̈ʒu�L��                
		iAngle0 = (iAngle0 + old_i) >> 1;
		
		//if(Wide_old != 0 && Wide - Wide_old > 6){//���C���������Ȃ������Q�[�g��������
		//if(Wide > 20){//���C���������Ȃ������Q�[�g��������
		 if( check_startgate() ) {
            pattern = 4;
		}
		break;
	
	case 4:

		
		if(Wide < 22){//���C����������
			
            cnt1 = 0;
			cnt2 = 0;
			cnt7 = 0;
			run = 1;//���s�J�n
			lEncoderTotal = 0; 
			lEncoderTotal2 = 0; 
			sp = 0;
			sp2 = 0;
			
			flag2 = 0;//�⓹�̉�
			
			msdBuffAddress = 0;
            msdWorkAddress = msdStartAddress;
            msdFlag = 1;                // �f�[�^�L�^�J�n               
			gate = 1;
            pattern = 10;
		}
		break;
*/
	case 10://�X�^�[�g����
	
		
		if(Center < -10)Center = -10;
		if(Center > 10)Center = 10;
		
		old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;
		
		//if(lEncoderTotal < 200 && Wide > 20){//���s�J�n����͒��� && �Q�[�g�����Ă�
		if(lEncoderTotal < 200){// && gate == 1 ){// && Wide > 28){//���s�J�n����͒��� && �Q�[�g�����Ă�
			mode = 1;//�������������
			
			iSetAngle = 0;
			servoPwmOut( iServoPwm2 );		
		}else{//�ʏ�
			mode = 0;	
			servoPwmOut( iServoPwm / 2 );
		}
		
		if( iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
			x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*2;
				
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
	
		}else{
			motor_f(100 , 100 );
         	motor_r(100 , 100 );
		}
		
		
		if(lEncoderTotal > 700){
			
			mode = 0;//���ɖ߂�
			pattern = 11;
		}
		break;
		
    case 11:
        /* �ʏ�g���[�X */
		 
		 old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;
		
		if(mode != 1 && Wide == 0){//����������
			Center = Center_old;
			Wide = Wide_old;
		}
		
		if(((Center - Center_old) < -15 || 15 < (Center - Center_old)) && ( Wide < 30)){//�}�Ƀ��C�����ω�������
			Center = Center_old;
			Wide = Wide_old;
		}
		
		if(mode == 1
		 && Wide > 28){//�⒆ && ��������
			Center = Center_old;
			Wide = Wide_old;	
		}
		
	
		if(mode == 1){
			if(IR_flag == 0){//�ԊO���ɐ؂�ւ�
				if((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (	-5 < Center_IR  && 	Center_IR < 5)){
					if(KASA_Encoder1 <= (lEncoderTotal-sp2) && (lEncoderTotal-sp2) < KASA_Encoder2){
						IR_cnt++;
						if(IR_cnt > 20){
							IR_flag = 1;
							IR_cnt = 0;
						}	
					}
				}
			}else{//�J�����ɐ؂�ւ�
				if((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (	-5 < Center_IR  && 	Center_IR < 5)){
					if(KASA_Encoder3 <= (lEncoderTotal-sp2)){
						IR_cnt++;
						if(IR_cnt > 20){
							IR_flag = 0;
							IR_cnt = 0;
						}	
					}
				}
			}
		}else{
			if(IR_flag == 1){//��ȊO�ŐԊO�����[�h�Ȃ�J�����ɐ؂�ւ�
				if((-10 < i && i < 10) && (-5 < Center && Center < 5 && Wide != 0) && (	-5 < Center_IR  && 	Center_IR < 5)){
					
					IR_cnt++;
					if(IR_cnt > 20){
						IR_flag = 0;
						IR_cnt = 0;
					}	
				}
			}
		}	
		
		
		if(cnt2 >=  MAXTIME * 10){//���s���ԏI��
			pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			break;
		}
		
		
		
		if(-10 < i && i < 10){
			if(angle_check() == 2 && ( (flag2%2 == 1) || ((lEncoderTotal-sp2) >= 500) && ((lEncoderTotal-sp) >= 1000))){//��Z���T�[�`�F�b�N
			
				cnt5++;
			}else{
				cnt5 = 0;
			}
		}
		
		if(-20 < i && i < 20){
			if(mode == 0){//�⒆�łȂ����
				if(lEncoderTotal > 200 && (lEncoderTotal-sp2) >= 1000 && (lEncoderTotal-sp) >= 100){//�Q�[�g�ɔ������Ȃ��悤�� && ��I�����班���̊Ԃ͖���
				//if(lEncoderTotal > 200 ){//�Q�[�g�ɔ������Ȃ��悤�� 
				
					if(date_f_mode == 0){
						if( check_crossline() ) {       // �N���X���C���`�F�b�N         
            				cnt1 = 0;
            				pattern = 21;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
				
        				}else if(check_halfline() == 1){//�����[���`�F���W�`�F�b�N
							cnt1 = 0;
            				pattern = 51;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
				
						}else if(check_halfline() == 2){//�E���[���`�F���W�`�F�b�N
							cnt1 = 0;
        	    			pattern = 61;
							sp = lEncoderTotal;
							lEncoderTotal_ch = lEncoderTotal;
							mode = 0;
							Center_offset = 0;
							date_f_num_ch++;
							break;
						}
					}else{
						//if( check_wideline() == 1) {       // �����������Ȃ�����      
						if( (check_crossline() || check_halfline() != 0) && iEncoder10 < 60){ 
							if(date_f_buff_ch_int[date_f_num_ch] == 31 || date_f_buff_ch_int[date_f_num_ch] == 41) {       // �N���X���C���`�F�b�N         
            					cnt1 = 0;
            					pattern = 21;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								
								//c_cnt++;
								
								if(c_cut_master == 0){
									c_cut = 0;
								}else{
									if(date_f_buff_ch_int[date_f_num_ch] < c_cut_encoder)c_cut = 0;
									else c_cut = 1;
								}
								/////////////////////////////////////////////////////////////////////////////////////////////////////
							/*	if((date_f_mode == 1 && (c_cnt == 2 || c_cnt == 999) ) || 
									(date_f_mode == 2 && (c_cnt == 999 || c_cnt == 2) )){
									c_cut = 0;
								}else{
									c_cut = 1;	
								}*/
								/////////////////////////////////////////////////////////////////////////////////////////////////////
								break;
				
        					}else if(date_f_buff_ch_int[date_f_num_ch] == 53){//�����[���`�F���W�`�F�b�N
								cnt1 = 0;
            					pattern = 51;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								break;
				
							}else if(date_f_buff_ch_int[date_f_num_ch] == 63){//�E���[���`�F���W�`�F�b�N
								cnt1 = 0;
        		    			pattern = 61;
								sp = lEncoderTotal;
								lEncoderTotal_ch = lEncoderTotal;
								mode = 0;
								Center_offset = 0;
								date_f_num_ch++;
								break;
							}
						}
					}
				}
			}
		}
		
		if(mode == 0){//�ʏ�
		
			if(cnt5 >= 1){//��
				if(flag2 % S_flag == 0 && saka_max > 0){

					saka_max--;
					mode = 1;//�⃂�[�h��
					sp2 = lEncoderTotal;

				}
				flag2++;
				cnt5 = 0;
				
				sp2 = lEncoderTotal;//�`���^�����O�h�~
				
			}
		
		}else if(mode == 1){//��

			if((lEncoderTotal-sp2) >= KASA_Encoder4){//�ʏ�ɖ߂�
			
				mode = 0;
				
				sp2 = lEncoderTotal;//�`���^�����O�h�~
				
				TOPSPEED = topspeed;
				SPEED_DOWN = speed_down;
				SPEED_DOWN_N = speed_down_n;
				MOTOR_out_R = motor2_out_R;
				MOTOR_in_F = motor2_in_F;
				MOTOR_in_R = motor2_in_R;
				S_para = s_para;
				OUT_M_DOWN = out_m_down;
				

			}else if((lEncoderTotal-sp2) >= KASA_Encoder3){// ���� 1000
				TOPSPEED = TOPSPEED4;
				SPEED_DOWN = SPEED_DOWN4;
				SPEED_DOWN_N = SPEED_DOWN4_N;
				MOTOR_out_R = MOTOR_out4_R;
				MOTOR_in_F = MOTOR_in4_F;
				MOTOR_in_R = MOTOR_in4_R;
				S_para = S_para4;
				OUT_M_DOWN = OUT_M_DOWN4;
			
			}else if((lEncoderTotal-sp2) >= KASA_Encoder2){//���x������ɒx�� ��� 550
				TOPSPEED = TOPSPEED2;
				SPEED_DOWN = SPEED_DOWN2;
				SPEED_DOWN_N = SPEED_DOWN2_N;
				MOTOR_out_R = MOTOR_out2_R;
				MOTOR_in_F = MOTOR_in2_F;
				MOTOR_in_R = MOTOR_in2_R;
				S_para = S_para2;
				OUT_M_DOWN = OUT_M_DOWN2;
			
			
			}else if((lEncoderTotal-sp2) >= KASA_Encoder1){//���x��x�� ���
				TOPSPEED = TOPSPEED3;
				SPEED_DOWN = SPEED_DOWN3;
				SPEED_DOWN_N = SPEED_DOWN3_N;
				MOTOR_out_R = MOTOR_out3_R;
				MOTOR_in_F = MOTOR_in3_F;
				MOTOR_in_R = MOTOR_in3_R;
				S_para = S_para3;
				OUT_M_DOWN = OUT_M_DOWN3;
				
				if( (i < -20 || 20 < i) && (lEncoderTotal-sp2) >= KASA_Encoder1+500){//����Ă���r���ŃJ�[�u�͂��肦�Ȃ�
					TOPSPEED = topspeed;
					SPEED_DOWN = speed_down;
					SPEED_DOWN_N = speed_down_n;
					MOTOR_out_R = motor2_out_R;
					MOTOR_in_F = motor2_in_F;
					MOTOR_in_R = motor2_in_R;
					S_para = s_para;
					OUT_M_DOWN = out_m_down;
					mode = 0;
					
					flag2--;//����̕��𖳂��������Ƃ�
					
					sp2 = lEncoderTotal;//�`���^�����O�h�~
				}
			}	
		}
		
		 
		
		if(mode == 1){
			if((lEncoderTotal-sp2) >= KASA_Encoder4)servoPwmOut( iServoPwm );
			else if((lEncoderTotal-sp2) >= KASA_Encoder3)servoPwmOut( iServoPwm );
			else if((lEncoderTotal-sp2) >= KASA_Encoder2)servoPwmOut( iServoPwm );
			else if((lEncoderTotal-sp2) >= KASA_Encoder1)servoPwmOut( iServoPwm /2);
			else servoPwmOut( iServoPwm );
		}else{
			servoPwmOut( iServoPwm );	
		}
        if( i > 12 ){//�n���h���E
			
			if(mode != 1){
				Center_offset = i / Center_offset_Angle ;//�J�[�u�Ŋ񂹂�
				if(Center_offset > Center_offset_MAX )Center_offset = Center_offset_MAX;
				if(Center_offset < -Center_offset_MAX )Center_offset = -Center_offset_MAX;
			}else{
				Center_offset = 0;
			}
			
			
			if((i > 8)&& (i - old_i > 0) && (Cu_flag == 0)){//��������J�[�u�� 
			
				if(cnt7 >= 50 && (lEncoderTotal-sp) >= 200  && (iEncoder10 > Cu_BRAKE_SP)){//���܂蒼���𑖂��Ă��Ȃ����̓u���[�L���Ȃ��悤�� && �N�����N�Ȃǂ̒���͖���
					cnt7 = 0;
				}
				Cu_flag = 1;
			}
			
			if(cnt7 <= Cu_BRAKE_time && (lEncoderTotal-sp) >= 200 && (lEncoderTotal-sp2) >= 100){
				
				servoPwmOut( iServoPwm * 2 );
				
				//motor_f( MOTOR_out_base, Cu_BRAKE*2/3 );
            	//motor_r( Cu_BRAKE*2/3, Cu_BRAKE );
				
				motor_f( Cu_BRAKE , Cu_BRAKE );
            	motor_r( Cu_BRAKE , Cu_BRAKE );
				
			}else if(cnt8 <= Cu_N_time && iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)) && (lEncoderTotal-sp) >= 200) {// �G���R�[�_�ɂ��X�s�[�h����  �J�[�u�O��
			
				x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*2;	
				r = x;
				f = x;
				
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < -5) x = -5;
				if(r < -15) r = -15;
				if(f < -10) f = -10;
				
				motor_f( x, r );
            	motor_r( f, r );			
			
			}else if(iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN_N)) && (lEncoderTotal-sp) >= 200 ) {// �G���R�[�_�ɂ��X�s�[�h����  �J�[�u�㔼
			
				x=((TOPSPEED -(i / SPEED_DOWN_N))-iEncoder10)*2;	
				r = x;
				f = x;
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < 0) x = 0;
				if(r < -10) r = -10;
				if(f < -5) f = -5;
				
				motor_f( x, r );
            	motor_r( f, r );	
					
			}else if(Center + Center_offset < - 10 ) {//���肩����
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((i / MOTOR_in_F) * S_para));
				else f = (MOTOR_out_base - ((i * -MOTOR_in_F) * S_para));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R > 0 )r = (MOTOR_out_base - ((i / MOTOR_in_R) * S_para));
				else r = (MOTOR_out_base - ((i * -MOTOR_in_R) * S_para));
				if(r < 0) r = 0;
				
				
				if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
				else or = (MOTOR_out_base - (i * -MOTOR_out_R));
				
				if(or < 0) or = 0;
				
				motor2_f(f,MOTOR_out_base);
				//motor_r(x,99);
				
				motor2_r(or,r);
					
			}else if((Center  + Center_offset > 10) || (i >= 60 && cnt8 <= Cu_N_time)) {//�O���
				
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((i / MOTOR_in_F) * OUT_M_DOWN));
				else f = (MOTOR_out_base - ((i * -MOTOR_in_F) * OUT_M_DOWN));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R > 0)r = (MOTOR_out_base - ((i / MOTOR_in_R) * OUT_M_DOWN));
				else r = (MOTOR_out_base - ((i * -MOTOR_in_R) * OUT_M_DOWN));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
				else or = (MOTOR_out_base - (i * -MOTOR_out_R));
				
				if(or < 0) or = 0;
					
				motor2_f( MOTOR_out_base, f);
				motor2_r( or, r);
					
        	}else{
				
				if((cnt8 <= Cu_N_time) || (mode == 1) || (i > 80) ){//�J�[�u�O�� || �⃂�[�h || �Ȃ����� || 
					if(MOTOR_in_F > 0)f= (MOTOR_out_base - (i / MOTOR_in_F));
					else f= (MOTOR_out_base - (i * -MOTOR_in_F));
				
					if(MOTOR_in_R > 0)r= (MOTOR_out_base - (i / MOTOR_in_R));
					else r= (MOTOR_out_base - (i * -MOTOR_in_R));


					if(MOTOR_out_R > 0)or = (MOTOR_out_base - (i / MOTOR_out_R));
					else or = (MOTOR_out_base - (i * -MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -5) r = -5;
					if(or < 0) or = 0;
					if(f > MOTOR_out_base) f = MOTOR_out_base;
					if(r > MOTOR_out_base) r = MOTOR_out_base;
					if(or > MOTOR_out_base) or = MOTOR_out_base;
				
					if(angle_y < 250 || 350 < angle_y){
						motor_f( 0, 0);
           				motor_r( 0, 0);
					}else{
           				motor2_f( MOTOR_out_base , f);
           				motor2_r( or, r);
					}
				}else{//�J�[�u�㔼

					if(MOTOR_in_F_N > 0)f= (100 - (i / MOTOR_in_F_N));
					else f= (100 - (i * -MOTOR_in_F_N));
				
					if(MOTOR_in_R_N > 0)r= (100 - (i / MOTOR_in_R_N));
					else r= (100 - (i * -MOTOR_in_R_N));
				
					if(MOTOR_out_R_N > 0)or = (100 - (i / MOTOR_out_R_N));
					else or = (100 - (i * -MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > 100) f = 100;
					if(r > 100) r = 100;
					if(or > 100) or = 100;
				
					if(angle_y < 250 || 350 < angle_y){
						motor_f( 0, 0);
           				motor_r( 0, 0);
					}else{
           				motor2_f( 100, f);
           				motor2_r( or, r);
					}
				}
							
			}
			 		 	 
		}else if( i < -12 ){//�n���h����
			
			if(mode != 1){
				Center_offset = i / Center_offset_Angle ;//�J�[�u�Ŋ񂹂�
				if(Center_offset > Center_offset_MAX )Center_offset = Center_offset_MAX;
				if(Center_offset < -Center_offset_MAX )Center_offset = -Center_offset_MAX;
			}else{
				Center_offset = 0;
			}
			
			
			if(( i < -8) && (i - old_i < 0) && (Cu_flag == 0)){//��������J�[�u�� 
				
				if(cnt7 >= 50 && (lEncoderTotal-sp) >= 200  && (iEncoder10 > Cu_BRAKE_SP)){//���܂蒼���𑖂��Ă��Ȃ����̓u���[�L���Ȃ��悤�� && �N�����N�Ȃǂ̒���͖���
					cnt7 = 0;
				}
				Cu_flag = 1;
			}
			
			if(cnt7 <= Cu_BRAKE_time && (lEncoderTotal-sp) >= 200 && (lEncoderTotal-sp2) >= 100){
				
				servoPwmOut( iServoPwm * 2);
				
				//motor_f( Cu_BRAKE*2/3, MOTOR_out_base );
            	//motor_r( Cu_BRAKE, Cu_BRAKE*2/3);
				
				motor_f( Cu_BRAKE, Cu_BRAKE );
            	motor_r( Cu_BRAKE, Cu_BRAKE );
				
			}else if(cnt8 <= Cu_N_time &&  iEncoder10 >= (TOPSPEED -(-i / SPEED_DOWN)) && (lEncoderTotal-sp) >= 200 ) {  // �G���R�[�_�ɂ��X�s�[�h���� �J�[�u�O��
			
				x=((TOPSPEED -(-i / SPEED_DOWN))-iEncoder10)*2;
				r = x;
				f = x;		
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < -5) x = -5;
				if(r < -15) r = -15;
				if(f < -10) f = -10;
				
				motor_f( r, x );
            	motor_r( r, f );
				
			}else if(iEncoder10 >= (TOPSPEED -(-i / SPEED_DOWN_N)) && (lEncoderTotal-sp) >= 200) {  // �G���R�[�_�ɂ��X�s�[�h���� �J�[�u�㔼
			
				x=((TOPSPEED -(-i / SPEED_DOWN_N))-iEncoder10)*2;
				r = x;
				f = x;	
				//if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(x < 0) x = 0;
				if(r < -10) r = -10;
				if(f < -5) f = -5;
				
				motor_f( r, x );
            	motor_r( r, f );
				
			}else if((Center  + Center_offset < -10) || (i <= -60 && cnt8 <= Cu_N_time) ) {//�O���
				
				if(MOTOR_in_F > 0)f = (MOTOR_out_base - ((-i / MOTOR_in_F) * OUT_M_DOWN));
				else f = (MOTOR_out_base - ((-i * -MOTOR_in_F) * OUT_M_DOWN));
				if(f < 0) f = 0;
					
				if(MOTOR_in_R >0)r = (MOTOR_out_base - ((-i / MOTOR_in_R) * OUT_M_DOWN));
				else r = (MOTOR_out_base - ((-i * -MOTOR_in_R) * OUT_M_DOWN));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
				else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
				if(or < 0) or = 0;
				
				motor2_f(f, MOTOR_out_base);
				motor2_r(r, or);
					
			}else if(Center  + Center_offset > 10) {//���肩����
			
				if(MOTOR_in_F >0)f = (MOTOR_out_base - ((-i / MOTOR_in_F) * S_para));
				else f = (MOTOR_out_base - ((-i * -MOTOR_in_F) * S_para));
				if(f < 0) f = 0;
				
				if(MOTOR_in_R >0)r = (MOTOR_out_base - ((-i / MOTOR_in_R) * S_para));
				else r = (MOTOR_out_base - ((-i * -MOTOR_in_R) * S_para));
				if(r < 0) r = 0;
				
				if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
				else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
				if(or < 0) or = 0;
				
				motor2_f(MOTOR_out_base, f);
			//	motor_r(99, x);
			
				motor2_r(r,or);
					
        	}else{
				
				if((cnt8 <= Cu_N_time) || (mode == 1) || (i < -80) ){//�J�[�u�O�� || �⃂�[�h || ���
					if(MOTOR_in_F >0)f = (MOTOR_out_base - (-i / MOTOR_in_F));
					else f = (MOTOR_out_base - (-i * -MOTOR_in_F)); 
				
					if(MOTOR_in_R >0)r = (MOTOR_out_base - (-i / MOTOR_in_R));
					else r = (MOTOR_out_base - (-i * -MOTOR_in_R));
				
					if(MOTOR_out_R >0)or = (MOTOR_out_base - (-i / MOTOR_out_R));
					else or = (MOTOR_out_base - (-i * -MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -5) r = -5;
					if(or < 0) or = 0;
					if(f > MOTOR_out_base) f = MOTOR_out_base;
					if(r > MOTOR_out_base) r = MOTOR_out_base;
					if(or > MOTOR_out_base) or = MOTOR_out_base;
				
					if(angle_y < 250 || 350 < angle_y){
						motor_f( 0, 0);
           				motor_r( 0, 0);
					}else{
           				motor2_f(f, MOTOR_out_base);
           				motor2_r(r, or );
					}
				}else{//�J�[�u�㔼
				
					if(MOTOR_in_F_N >0)f = (100 - (-i / MOTOR_in_F_N));
					else f = (100 - (-i * -MOTOR_in_F_N)); 
				
					if(MOTOR_in_R_N >0)r = (100 - (-i / MOTOR_in_R_N));
					else r = (100 - (-i * -MOTOR_in_R_N));
				
					if(MOTOR_out_R_N >0)or = (100 - (-i / MOTOR_out_R_N));
					else or = (100 - (-i * -MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > 100) f = 100;
					if(r > 100) r = 100;
					if(or > 100) or = 100;
				
					if(angle_y < 250 || 350 < angle_y){
						motor_f( 0, 0);
           				motor_r( 0, 0);
					}else{
           				motor2_f(f, 100 );
           				motor2_r(r, or );
					}
				}	
				
				
			}
			 	 
		}else{//����
			Center_offset = 0;
			cnt8 = 0;
			
			if((Cu_flag == 1)&&(mode == 0)){//�J�[�u���璼���� && �⒆�ł͂Ȃ�
			
				if(cnt7 >= 60){//���܂�J�[�u�𑖂��Ă��Ȃ����̓t���[�ɂ��Ȃ��悤��
					cnt7 = 0;
				}
				Cu_flag = 0;
			}
			
			if(cnt7 <= Cu_FREE_time && (lEncoderTotal-sp) >= 700){//�J�[�u����̕��A����
				
				if(mode == 0 &&  Center < -10) {//�ԑ̍����
				
					motor_f(95 , 100 );
				
				}else if(mode == 0 &&  Center > 10) {//�ԑ̉E���
				
					motor_f(100 , 95 );
					
				}else{
			
					motor_f(100 , 100 );
				}
				
            	motor_r( 0, 0 );
			
			}else if( (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))) && (date_f_mode == 0 || mode == 1) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
					 
				//�ʏ�u���[�L
				motor_f( 0, 0 );
            	motor_r( 0, 0 );
			
			}else if(mode == 0 && date_f_mode != 0 && ((date_f_buff_int[date_f_num] - date_f_brake)< SEncoderTotal) && (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN)))  ) {// �G���R�[�_�ɂ��X�s�[�h���� 
					 
				//�Đ����s���@�ŏI�u���[�L
				x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*20;
				
				if(x < BRAKE_MAX) x = BRAKE_MAX;
			
				motor_f( x, x );
            	motor_r( x, x );
					
			}else if(mode == 0 && date_f_mode != 0 && (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))+(date_f_buff_int[date_f_num] - SEncoderTotal)/100) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
				
				//�Đ����s���@�u�[�X�g��
				x=(((TOPSPEED -(i / SPEED_DOWN))+(date_f_buff_int[date_f_num] - SEncoderTotal)/100) -iEncoder10)*20;
				
				if(x < -30) x = -30;
				 
				motor_f( x, x );
            	motor_r( x, x );
			
			/*
			}else if(( (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))) && (date_f_mode == 0 || mode == 1) ) ||
						 (mode == 0 && date_f_mode != 0 && ((date_f_buff_int[date_f_num] - date_f_brake)< SEncoderTotal) && (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))) ) || 
						 (mode == 0 && date_f_mode != 0 && (iEncoder10 >= (TOPSPEED -(i / SPEED_DOWN))+(date_f_buff_int[date_f_num] - SEncoderTotal)/80)) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
					 
				if(date_f_mode == 0 || mode == 1)x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)/2;
				else {//�Đ����s���̃u���[�L
					if((date_f_buff_int[date_f_num] - date_f_brake)< SEncoderTotal)x=((TOPSPEED -(i / SPEED_DOWN))-iEncoder10)*10;
					else x=((TOPSPEED -(i / SPEED_DOWN)+(date_f_buff_int[date_f_num] - SEncoderTotal)/80 )-iEncoder10)*5;
				}
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				if(mode != 0 && date_f_mode == 0 && x < -5)x = -5;
				if(mode == 0 && date_f_mode == 0 && x < 0)x = 0;
				motor_f( x, x );
            	motor_r( x, x );
				
			*/	
			}else if(mode == 0 && Center < -10) {//�ԑ̍����
				
				motor_f(90 , 100 );
				motor_r(100 , 100 );
			}else if(mode == 0 &&  Center > 10) {//�ԑ̉E���
				
				motor_f(100 , 90 );
				motor_r(100 , 100 );
				
			}else if(mode == 0 && date_f_mode != 0 && date_f_buff_int[date_f_num] - date_f_brake > SEncoderTotal){//�u�[�X�g
				
				motor_f(100 , 100 );
				motor_r(100 , 100 );
				
			}else{
				motor_f(100 , 100 );
           		motor_r(100 , 100 );
				
			}
		}       
        break;
		
    case 21:
        /* �N���X���C���ʉߏ��� */
      	setBeepPatternS( 0x8000 );

		if(Wide > 30){
			iSetAngle = 0;
			servoPwmOut( iServoPwm2 );
		}else{
			servoPwmOut( iServoPwm  );
		}
		
		//�Đ����s���@�@�u���[�L
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4)){
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			x=(C_TOPSPEED4-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		//�N�����N�@�u���[�L
        }else if( (date_f_mode == 0 && iEncoder10 >= C_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED)   ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
            int x;
			x=(C_TOPSPEED-iEncoder10)*10;
			r = x;
			
			if(r < BRAKE_MAX) r = BRAKE_MAX;
			
			
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
			
			motor_f( x, x );
            motor_r( x, x );
			
			
		//	motor_f( 40, 40 );
         //   motor_r( 40, 40 );
		 
        }else{
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }	

        if( (lEncoderTotal-sp) >= 70 ) {	
           	cnt1 = 0;
            pattern = 22;
			sp = lEncoderTotal;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
        }
        break;

    case 22:
        /* �N���X���C����̃g���[�X�A���p���o���� */
		
		//if(c_cut == 1 && date_f_mode != 0){
		/*if(date_f_mode != 0){
			if(date_f_buff_ch_int[date_f_num_ch-1] == 31){
				
				if(Center < -14){//��肷��
					Center_offset = 9;//�E�Ɋ��
					servoPwmOut( iServoPwm );
				
				}else{
					Center_offset = 12;//�E�Ɋ��
					servoPwmOut( iServoPwm );
				}
			}else{
				if(Center > 14){//��肷��
					Center_offset = -9;//���Ɋ��
					servoPwmOut( iServoPwm );
				}else{
					Center_offset = -12;//���Ɋ��
					servoPwmOut( iServoPwm  );
				}
			}
			
		}else{
			servoPwmOut( iServoPwm * 1.5  );
		}*/
		servoPwmOut( iServoPwm * 1.5  );
		
		if( (lEncoderTotal-sp) >= 250 ) {
			//if((( c_cut == 0 || date_f_mode == 0) && check_halfline() == 2) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 31  && (check_wideline() == 1 || ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch))))){//�E�N�����N
            
			if((date_f_mode == 0 && check_halfline() == 2) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 31  && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch)))
						|| ((c_cut == 0 && date_f_mode != 0) && (date_f_buff_ch_int[date_f_num_ch-1] == 31) && (check_wideline() == 1))){//�E�N�����N
            	cnt1 = 0;
				sp = lEncoderTotal;
				
				if(date_f_mode == 0){//�����v��
					date_buff_ch_int[date_num_ch++] = 31;
					date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
				}
          		pattern = 31;//�E�N�����N
				date_f_num_ch++;
			
				Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(date_f_mode == 0 || c_cut == 0)wait(4);
				wait(5);
				
				if(c_cut == 1 && date_f_mode != 0){
					if(iEncoder10 >= C_TOPSPEED3){
						sp -= (iEncoder10 -  C_TOPSPEED3) * 5;
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;
				
        	}
        	
			//if(((c_cut == 0 || date_f_mode == 0) && check_halfline() == 1) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 41  && (check_wideline() == 1 || ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch))))){//���N�����N
            
			if((date_f_mode == 0 && check_halfline() == 1) || ((c_cut == 1 && date_f_mode != 0) && date_f_buff_ch_int[date_f_num_ch-1] == 41  && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_c )< (lEncoderTotal - lEncoderTotal_ch)))
						|| ((c_cut == 0 && date_f_mode != 0) && (date_f_buff_ch_int[date_f_num_ch-1] == 41) && (check_wideline() == 1))){//���N�����N
            	cnt1 = 0;
				sp = lEncoderTotal;
				
				if(date_f_mode == 0){//�����v��
					date_buff_ch_int[date_num_ch++] = 41;
					date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
				}
				
          		pattern = 41;//���N�����N
				date_f_num_ch++;
			
				Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(date_f_mode == 0 || c_cut == 0)wait(4);
				wait(5);
				
				if(c_cut == 1 && date_f_mode != 0){
					if(iEncoder10 >= C_TOPSPEED3){
						sp -= (iEncoder10 -  C_TOPSPEED3) * 5;
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;	
        	}
		}
		
		old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;
		
		//�Đ����s�� �@�u���[�L
		//if((c_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4){
		if((date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED4){
			
			x=(C_TOPSPEED4-iEncoder10)*2;
			r = x;
			if(r < BRAKE_MAX) r = BRAKE_MAX;
			
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
			motor_f( x, x );
            motor_r( r, r );
			
 		}else if( ((date_f_mode == 0) && iEncoder10 >= C_TOPSPEED) 
				|| ((c_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED3) 
				|| ((c_cut == 0 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_c )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= C_TOPSPEED)   ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
             
			if((i < -10 || 10 < i) && 
				((date_f_mode != 0 && ( date_f_buff_ch_int[date_f_num_ch-1] == 31 || date_f_buff_ch_int[date_f_num_ch-1] == 41) && (lEncoderTotal-sp) >= 300) 
				|| (date_f_mode == 0))){//abs
				//motor_f( 0, 0 );
            	//motor_r( 0, 0 );
				
				if(c_cut == 0 || date_f_mode == 0)x=(C_TOPSPEED -iEncoder10)*20;
				else x=(C_TOPSPEED3 -iEncoder10)*20;
			
				r = x;
				if(r < -10) r = -10;
				
				if(x < BRAKE_MAX+20) x = BRAKE_MAX+20;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				
				motor_f( x, x );
            	motor_r( r, r );
			
			}else{
				if(c_cut == 0 || date_f_mode == 0)x=(C_TOPSPEED -iEncoder10)*20;
				else x=(C_TOPSPEED3 -iEncoder10)*20;
			
				r = x;
				if(r < -50 && (c_cut == 0 || date_f_mode == 0)) r = -50;
				else if(r < BRAKE_MAX) r = BRAKE_MAX;
				
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
				
				motor_f( x, x );
            	motor_r( x, x );
			}
	
        }else{
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }
		
		
	/*	if(angle_check() == 2){ //��Z���T�[�`�F�b�N
			pattern = 11;
			cnt1 = 0;
			mode = 1;//�⃂�[�h��
			sp2 = lEncoderTotal;
			flag2++;
		}*/
		
		if( lEncoderTotal-sp >= 2000 ) {//�듮��`�F�b�N
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			Center_offset = 0;
			date_f_num_ch--;
            pattern = 11;
			//c_cnt--;
			cnt1 = 0;
        }
        break;

    case 31:
        /* �E�N�����N���� */
		setBeepPatternS( 0x8000 );
		
		old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;
		
        if(date_f_mode == 0 || c_cut == 0){
			mode = 3;//������
		
			if((lEncoderTotal-sp) >= 130){
				if(i < 95)iSetAngle = 150;
				else iSetAngle = 110;
				
			}else iSetAngle = 95;
			
			motor_f( 90, -50 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v 85 -40*/
        	motor_r( -50, -60 );          /* �Ōv�Z                        */
			
		
		}else{
			mode = 1;//����͈͂�����
				
				
			if(0 < Center  && (lEncoderTotal-sp) >= 150  || (Wide != 0 && -6 < Center  && (lEncoderTotal-sp) >= 200)){
			
				iSetAngle = -25;
				motor2_f( 0,   80 );         
        		motor2_r( 0,   0 ); 
		
			}else if((lEncoderTotal-sp) >= 350){
				
				iSetAngle = 100;
				motor2_f( 85,   10 );         
        		motor2_r( 30,   0 );  
				
				
			}else if((lEncoderTotal-sp) >= 250){
				
				if(i < 70)iSetAngle = 120;
				else iSetAngle = 90;
				
				motor2_f( 80,   0 );         
        		motor2_r( 0,   0 );  
			
				
			}else if((lEncoderTotal-sp) >= 150){
				
				if(i < 45)iSetAngle = 90;
				else iSetAngle = 75;
				
				motor2_f( 85,   10 );         
        		motor2_r( 20,   0 );
				
			}else if((lEncoderTotal-sp) >= 100){
				
				if(i < 30)iSetAngle = 80;
				else iSetAngle = 50;
				
				motor2_f( 85,   30 );         
        		motor2_r( 25,   0 );
			
			}else if((lEncoderTotal-sp) >= 50){
				
				if(i < 20)iSetAngle = 50;
				else iSetAngle = 35;
				
				motor2_f( 90,   35 );         
        		motor2_r( 35,  0 );
					
			}else{
				iSetAngle = 25;
				motor2_f( 90,   40 );         
        		motor2_r( 40,   0 );
			} 
			
       	}
  
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */
        
		//180 -15 < 25
        if((( c_cut == 0 || date_f_mode == 0) && (lEncoderTotal-sp) >= 150) || ((c_cut == 1 && date_f_mode != 0) && (lEncoderTotal-sp) >= 240)){
			if(Wide != 0){
			//if (((20 < Center)&&(Center < 40)) || ((-15 < Center)&&(Center < 15))) {    /* �Ȃ��I���`�F�b�N           */
			if ( (( c_cut == 0 || date_f_mode == 0) && -25 < Center && Center < 25 && (Wide != 0 && Wide < 30) ) 
				|| ((c_cut == 1 && date_f_mode != 0) && -20 < Center && Center < 0 && (Wide_old == 0 || Wide > Wide_old))
				  || ((c_cut == 1 && date_f_mode != 0) && -25 < Center && Center < 25 && (Wide_old != 0) && (lEncoderTotal-sp) >= 750)) {    /* �Ȃ��I���`�F�b�N           */
				
            	cnt1 = 0;
            	iSensorPattern = 0;
            	
				sp = lEncoderTotal;
            	pattern = 32;
				
				mode = 0;//�ʏ�
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				
				if(date_f_mode != 0 && c_cut == 1){	
					cnt1 = 0;
					sp = lEncoderTotal;
					mode = 0;//����͈͂����ɖ߂�
            		pattern = 11;//�ʏ�g���[�X��
				}	
        	}
			}
		}
        break;

    case 32:
        /* ���肷��܂� (�V���[�J�b�g��32�ɂ͗��Ȃ���)*/
		
        if((lEncoderTotal-sp) >= 90)iSetAngle = 98;
		else iSetAngle = 100;
	
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if( iEncoder10 >= C_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	int x;
			x=(C_TOPSPEED2-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 50 );
           	motor2_r( 70, 60 );
		}
         
		
		if((lEncoderTotal-sp) >= 90){ 
			if(Wide != 0){                       
        		if(((-15 < Center)&&(Center < 20) && getServoAngle() < 120) 
					|| ((-5 < Center)&&(Center < 5) && getServoAngle() < 128)) {    /*  �����ɂȂ�܂�          */
            		cnt1 = 0;
            		iSensorPattern = 0;
            	
           			pattern = 33;
				//	mode = 0;//�ʏ�
					sp = lEncoderTotal;
        		}
			}
		}
	
        break;
	
	case 33://�����҂�	
		//mode = 1;//����͈͂���������
		mode = 0;//����͈͂����ɖ߂�
		
		servoPwmOut( iServoPwm );

		if( iEncoder10 >= TOPSPEED ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
           	int x;
			x=(TOPSPEED-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if( (lEncoderTotal-sp) >= 200 && -30 < getServoAngle() && getServoAngle() < 30 ) {
            cnt1 = 0;
			sp = lEncoderTotal;
			mode = 0;//����͈͂����ɖ߂�
            pattern = 11;//�ʏ�g���[�X��
        }
        break;
		

    case 41:
        /* ���N�����N���� */
        setBeepPatternS( 0x8000 );
			
        if(date_f_mode == 0 || c_cut == 0){
			mode = 2;//�E����
			
			if((lEncoderTotal-sp) >= 130){
				
				if(i > -90)iSetAngle = -150;
				else iSetAngle = -108;
				
			}else iSetAngle = -95;
			
			motor_f( -50, 90 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        	motor_r( -60, -50 );          /* �Ōv�Z                        */
			
		}else{
			mode = 1;//����͈͂�����
			
  
			if((Center < 0 && (lEncoderTotal-sp) >= 150) || (Wide != 0 && Center < 6 && (lEncoderTotal-sp) >= 200)){
				iSetAngle = -25;
				motor2_f( 80,   0 );         
        		motor2_r( 0,   0 ); 
		
			}else if((lEncoderTotal-sp) >= 350){
				iSetAngle = -100;
				motor2_f( 10,   85 );         
        		motor2_r( 0,   30 );  
				
			}else if((lEncoderTotal-sp) >= 250){
				
				if(i > -70)iSetAngle = -120;
				else iSetAngle = -95;
				
				motor2_f( 0,   80 );         
        		motor2_r( 0,   0 );  
			
				
			}else if((lEncoderTotal-sp) >= 150){
				if(i > -45)iSetAngle = -90;
				else iSetAngle = -75;
				
				motor2_f( 10,   85 );         
        		motor2_r( 0,   20 );
				
			}else if((lEncoderTotal-sp) >= 100){
				if(i > -30)iSetAngle = -80;
				else iSetAngle = -50;
				
				motor2_f( 30,   85 );         
        		motor2_r( 0,   25 );
			
			}else if((lEncoderTotal-sp) >= 50){
				if(i > -20)iSetAngle = -50;
				else iSetAngle = -35;
				
				motor2_f( 35,   90 );         
        		motor2_r( 0,  35 );
					
			}else{
				iSetAngle = -25;
				motor2_f( 40,   90 );         
        		motor2_r( 0,   40 );
			}  
       	}
		
		servoPwmOut( iServoPwm2 );        /* �U�肪�ア�Ƃ��͑傫������       */
        
		if(((c_cut == 0 || date_f_mode == 0) && (lEncoderTotal-sp) >= 150) || ((c_cut == 1 && date_f_mode != 0) && (lEncoderTotal-sp) >= 240)){
			if(Wide != 0){ 
			//if( ((-40 < Center)&&(Center < -20)) || ((-15 < Center)&&(Center < 15))) {    /* �Ȃ��I���`�F�b�N           */
	 		if(( (c_cut == 0 || date_f_mode == 0) && -25 < Center && Center < 25 && (Wide != 0 && Wide < 30)) 
				|| ( (c_cut == 1 && date_f_mode != 0) && 0 < Center && Center < 20 && (Wide_old == 0 || Wide < Wide_old)) 
					|| ((c_cut == 1 && date_f_mode != 0) && -25 < Center && Center < 25 && (Wide_old != 0) && (lEncoderTotal-sp) >= 750)){    /* �Ȃ��I���`�F�b�N           */
	 	
            	cnt1 = 0;
            	iSensorPattern = 0;
            
            	sp = lEncoderTotal;
            	pattern = 42;
				
				mode = 0;//�ʏ�
				
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				if(date_f_mode != 0 && c_cut == 1){	
					cnt1 = 0;
					sp = lEncoderTotal;
					mode = 0;//����͈͂����ɖ߂�
            		pattern = 11;//�ʏ�g���[�X��
				}
			}
			}
		}
        break;

    case 42:
		/* ���肷��܂� (�V���[�J�b�g��42�ɂ͗��Ȃ���)*/
		
		if((lEncoderTotal-sp) >= 90)iSetAngle = -98;
		else iSetAngle = -103;
		
		
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if( iEncoder10 >= C_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	int x;
			x=(C_TOPSPEED2-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 50, 100);
           	motor2_r( 60, 70 );
			
		}
         	  
		
		if((lEncoderTotal-sp) >= 50){   
			if(Wide != 0){                      
				if(((-20 < Center)&&(Center < 10) && getServoAngle() > -120) 
					|| ((-5 < Center)&&(Center < 5) && getServoAngle() > -128)) {    /*  �����ɂȂ�܂�          */
            		
            		cnt1 = 0;
      			    iSensorPattern = 0;
            	
           			pattern = 43;
				//	mode = 0;//�ʏ�
					sp = lEncoderTotal;
        		}
			}
		}
		
        break;
		
	case 43://�����҂�	
	//	mode = 1;//����͈͂���������
		mode = 0;//����͈͂����ɖ߂�
		
		servoPwmOut( iServoPwm );

		if( iEncoder10 >= TOPSPEED ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
			
           	int x;
			x=(TOPSPEED-iEncoder10)*10;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if( (lEncoderTotal-sp) >= 200 && -30 < getServoAngle() && getServoAngle() < 30 ) {
            cnt1 = 0;
			sp = lEncoderTotal;
			mode = 0;//����͈͂����ɖ߂�
            pattern = 11;//�ʏ�g���[�X��
        }
        break;
		
	case 51://���n�[�t
		setBeepPatternS( 0x8000 );
	    iSetAngle = 0;
		servoPwmOut( iServoPwm2 );

		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
       
           	int x;
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(date_f_mode == 0 && (check_crossline() || check_halfline() == 2 )) {       // �N���X���C���`�F�b�N         
            cnt1 = 0;
            pattern = 21;	
        }
  
        if( (lEncoderTotal - sp) >= 50 ) {
            cnt1 = 0;
            pattern = 52;
			sp = lEncoderTotal;
        }
        break;

	case 52://�n�[�t���C����
	//	if( date_f_mode != 0 && (lEncoderTotal - sp) >= 100 )mode = 3;//������
		 
		if(Center > 13){//��肷��
			Center_offset = -8;//���Ɋ��
			servoPwmOut( iServoPwm );
		}else{
			Center_offset = -11;//���Ɋ��
			servoPwmOut( iServoPwm  );
		}
		
		
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
           	int x;
			x=(H_TOPSPEED -iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}

	
	//	if((lEncoderTotal-sp) >= 300 ){
	//		if((( h_cut == 0 || date_f_mode == 0)  && Wide == 0) || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //�E���`�F�b�N
        
		if(((h_cut == 0 || date_f_mode == 0)  && Wide == 0 && (lEncoderTotal-sp) >= 250 )  || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //�E���`�F�b�N
            
            cnt1 = 0;
			sp = lEncoderTotal;
				
			if(date_f_mode == 0){//�����v��
				date_buff_ch_int[date_num_ch++] = 53;
				date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
			}
				
           	 pattern = 53;
			date_f_num_ch++;
			mode = 0;//�ʏ�
			Center_offset = 0;//�I�t�Z�b�g��߂�
            break;
		}
		
		if(date_f_mode == 0 && (lEncoderTotal-sp) < 30  && ( check_crossline() || check_halfline() == 2 )) {       // �N���X���C���`�F�b�N         
            cnt1 = 0;
			Center_offset = 0;//�I�t�Z�b�g��߂�
            pattern = 21;	
        }
  
		if( (lEncoderTotal-sp) >= 1500 ) {//�듮��`�F�b�N
			
			date_f_num_ch--;
			mode = 0;//�ʏ�
            pattern = 11;
			cnt1 = 0;
			Center_offset = 0;//�I�t�Z�b�g��߂�
        }	

		break;

	case 53:
		mode = 2;//�E����
		
		if(date_f_mode == 0 || h_cut == 0){
        	//iSetAngle = -45;//-48 -47 �ǂȂ�
			iSetAngle = -45;//
		}else{
			iSetAngle = -18;
		}
		
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 0, 100 );
           		motor_r( 40, 50 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
         
		if( (date_f_mode == 0 && (lEncoderTotal - sp) >= 250 ) || (date_f_mode != 0 && (lEncoderTotal - sp) >= 350 )) {  
        if((-50 < Center)&&(Center < -15) && Wide != 0 ) {    /* �Ȃ��I���`�F�b�N           */
		//	if(Center < -30) {    /* �Ȃ��I���`�F�b�N           */
		
            	cnt1 = 0;
				sp = lEncoderTotal;
            	pattern = 54;
				
				mode = 0;//�ʏ�
				motor_mode_r( BRAKE, BRAKE );
        	}
		}
        break;

	case 54:
		
		if(date_f_mode == 0 || h_cut == 0){
		//	iSetAngle = 5;//5 7
			iSetAngle = -20;//5 7
		}else{
			iSetAngle = -5;//5 7
		}
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 90,  50 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 90,  50 );
			}
		}
        
		
        //if( (15 < Center)&&(Center < 50) ) {    /* �Ȃ��I���`�F�b�N           */
		//if(-10 < Center && Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-7
		if(5 < Center && Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-7

	//	if(20 < Center) {    /* �Ȃ��I���`�F�b�N           */
		
            cnt1 = 0;
			sp = lEncoderTotal;
            pattern = 55;
			
			motor_mode_r( FREE, FREE );
        }
		
        break;

	case 55://���肷��܂�
	
		if(date_f_mode == 0 || h_cut == 0){
			//iSetAngle = 25;//40 37
			iSetAngle = 25;//40 37
		}else{
			iSetAngle = 29;//40 37
		}
		
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 50 );
           		motor_r( 90, 0 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 50 );
           		motor_r( 90, 0 );
			}
		}
          
		if( (lEncoderTotal - sp) >= 100 ) {                        
        if((-23 < Center)&&(Center < 23)&&(Wide != 0)) {    /*  �����ɂȂ�܂�          */
		
            cnt1 = 0;
            iSensorPattern = 0;
          
			pattern = 11;
			
			sp = lEncoderTotal;
        }
		}
        break;
		
	
			
	case 61://�E�n�[�t
		
		setBeepPatternS( 0x8000 );
		iSetAngle = 0;
		servoPwmOut( iServoPwm2 );

		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
           	int x;
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(date_f_mode == 0 && (check_crossline() || check_halfline() == 1)) {       // �N���X���C���`�F�b�N         
            cnt1 = 0;
           	
            pattern = 21;			
        }

        if( (lEncoderTotal-sp) >= 50 ) {
            cnt1 = 0;
            pattern = 62;
			sp=lEncoderTotal;
        }
        break;

	case 62://�n�[�t���C����
		
	//	if( date_f_mode != 0 && (lEncoderTotal - sp) >= 100 )mode = 2;//�E����
		
		if(Center < -14){//
			Center_offset = 9;//�E�Ɋ��
			servoPwmOut( iServoPwm );
				
		}else{
			Center_offset = 12;//�E�Ɋ��
			servoPwmOut( iServoPwm );
		}
		
		
		if((date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )> (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= TOPSPEED)){
			
			x=(TOPSPEED-iEncoder10)*1;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
            motor_r( x, x );
			
		}else if( (date_f_mode == 0 && iEncoder10 >= H_TOPSPEED) || (date_f_mode != 0 && ((date_f_buff_ch_int[date_f_num_ch] - date_f_brake_h )< (lEncoderTotal - lEncoderTotal_ch))&& iEncoder10 >= H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
           	int x;
			x=(H_TOPSPEED-iEncoder10)*15;
			if(x < BRAKE_MAX) x = BRAKE_MAX;
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(((h_cut == 0 || date_f_mode == 0)  && Wide == 0 && (lEncoderTotal-sp) >= 250 )  || (( h_cut == 1 && date_f_mode != 0) && ((date_f_buff_ch_int[date_f_num_ch] - date_f_shortcat_h)< (lEncoderTotal - lEncoderTotal_ch)) )){ //�E���`�F�b�N
            
            cnt1 = 0;
			sp = lEncoderTotal;
				
			if(date_f_mode == 0){//�����v��
				date_buff_ch_int[date_num_ch++] = 63;
				date_buff_ch_int[date_num_ch++] = lEncoderTotal - lEncoderTotal_ch;	
			}
				
           	pattern = 63;
			date_f_num_ch++;
			mode = 0;//�ʏ�
			Center_offset = 0;//�I�t�Z�b�g��߂�
				
            break;
        }
			

		if(date_f_mode == 0 && (lEncoderTotal-sp) < 30  && ( check_crossline() || check_halfline() == 1 )) {       // �N���X���C���`�F�b�N         
            cnt1 = 0;
			Center_offset = 0;//�I�t�Z�b�g��߂�
            pattern = 21;	
        }
  
		if( (lEncoderTotal-sp) >= 1500 ) {//�듮��`�F�b�N
          	
			date_f_num_ch--;
			mode = 0;//�ʏ�
            pattern = 11;
			cnt1 = 0;
			Center_offset = 0;//�I�t�Z�b�g��߂�
        }

		break;

	case 63:
		setBeepPatternS( 0x8000 );

		mode = 3;//������
		
		if(date_f_mode == 0 || h_cut == 0){
       		//iSetAngle = 50;//48 47
			iSetAngle = 48;//48 47
		}else{
			iSetAngle = 20;//48 47
		}
		
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
    	       	motor_r( x, x );
       		}else{
           		motor_f( 100, 0 );
           		motor_r( 50, 40 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
    	       	motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
                
		if( (date_f_mode == 0 && (lEncoderTotal - sp) >= 250 ) || (date_f_mode != 0 && (lEncoderTotal - sp) >= 350 )) {             
        if((15 < Center)&&(Center < 50) && Wide != 0) {    /* �Ȃ��I���`�F�b�N           */
		//if( 30 < Center) {    /* �Ȃ��I���`�F�b�N           */
		
            cnt1 = 0;
            sp = lEncoderTotal;
            pattern = 64;
			
			mode = 0;//�ʏ�
			
			motor_mode_r( BRAKE, BRAKE );
        }
		}
        break;

	case 64:

		if(date_f_mode == 0 || h_cut == 0){
			iSetAngle = 15;//-3
		}else{
			iSetAngle = 5;//-3
		}
		
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 50, 90 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 50, 90 );
			}
		}
             
		//if( (lEncoderTotal - sp) >= 30 ) {                  
        //if((-50 < Center)&&(Center < -15)) {    /* �Ȃ��I���`�F�b�N           */
		if(Center < 0 && Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-5
		//if(Center < -20) {    /* �Ȃ��I���`�F�b�N           */
	
            cnt1 = 0;
            sp = lEncoderTotal;
            pattern = 65;
			
			motor_mode_r( FREE, FREE );
        }
		//}
        break;

	case 65://���肷��܂�
	
		if(date_f_mode == 0 || h_cut == 0){
			iSetAngle = -25;//-40 -35
		}else{
			iSetAngle = -30;//-40 -35
		}
		servoPwmOut( iServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(date_f_mode == 0 || h_cut == 0){
			if( iEncoder10 >= H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 50, 100 );
           		motor_r( 0, 90 );
			}
		}else{
			if( iEncoder10 >= H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           		int x;
				x=(H_TOPSPEED2_S-iEncoder10)*15;
				if(x < BRAKE_MAX) x = BRAKE_MAX;
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 50, 100 );
           		motor_r( 0, 90 );
			}
		}
                      
	    if( (lEncoderTotal - sp) >= 100 ) {    
        if((-23 < Center)&&(Center < 23)&&(Wide != 0)) {    /*  �����ɂȂ�܂�          */
		
            cnt1 = 0;
            iSensorPattern = 0;
        
			pattern = 11;
		
			sp = lEncoderTotal;
        }
		}
        break;
	case 101:
        /* ��~ */
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        setBeepPatternS( 0xc000 );
		
	
		msdFlag = 0;
        if( msdError != 0 ) {
            /* microSD�ɕs����������Ȃ�I�� */
            printf( "microSD Initialize Error!!\n" );
            pattern = 109;
        } else {
            pattern = 102;
            cnt1 = 0;
        }
        break;

    case 102:
		/* �Ō�̃f�[�^�������ނ܂ő҂�*/
        if( checkMicroSDProcess() == 0 ) {
            pattern = 103;               /* �f�[�^�]��������             */
            break;
        }
        if( checkMicroSDProcess() == 11 ) {
            microSDProcessEnd();        /* microSDProcess�I������       */
            while( checkMicroSDProcess() );
            pattern = 103;               /* �f�[�^�]��������             */
        }
		
        break;

   case 103:
		 
        /* 0.5s�҂� && �v�b�V���X�C�b�`�������ꂽ���`�F�b�N*/
        if( cnt1 >= 500 && !pushsw_get()) {
            pattern = 104;
            cnt1 = 0;
        }
        break;

    case 104:
        /* �v�b�V���X�C�b�`�������ꂽ���`�F�b�N */
        led_out( cnt1 / 200 % 2 ? 0x6 : 0x9  );
        if( pushsw_get() ) {
            pattern = 105;
            cnt1 = 0;
        }
        break;

    case 105:
        /* �^�C�g���]���A�]������ */
        printf( "\n" );
        printf( "CarName Data Out\n" ); /* �����̃J�[�l�[�������Ă������� */
        printf( "Pattern, Center, Wide ,�p�x, �T�[�{PWM, " );
        printf( "���OPWM, �E�OPWM, ����PWM, �E��PWM, �G���R�[�_5*2,���[�h,�⓹��,�W���C��Y/10,�W���C��X/10,�ԊO��\n" );
		
		msdWorkAddress = msdStartAddress;   /* �ǂݍ��݊J�n�A�h���X     */
        pattern = 106;
        break;
	case 106:
        /* microSD���f�[�^�ǂݍ��� */
        if( msdWorkAddress >= msdEndAddress ) {
            /* �������ݏI���A�h���X�ɂȂ�����A�I��� */
            pattern = 109;
            break;
        }
        ret = readMicroSD( msdWorkAddress , msdBuff );
        if( ret != 0x00 ) {
            /* �ǂݍ��݃G���[ */
            printf( "\nmicroSD Read Error!!\n" );
            pattern = 109;
            break;
        } else {
            /* �G���[�Ȃ� */
            msdWorkAddress += 512;
            msdBuffAddress = 0;
            pattern = 107;
			i = 0;
        }
        break;
	case 107:
        /* �f�[�^�]�� */
        led_out( 1 << (cnt1/100) % 8 );

		if(msdBuff[msdBuffAddress+0] <= 0 ){ /* �p�^�[����0�ȉ��Ȃ�I�� */
			savecnt++;
		}else{
			savecnt = 0;
		}
		
		if(savecnt > 0) {
			date_f_make(-1,0,0,0);
            printf( "End.\n" );
            pattern = 108;
			cnt1  = 0 ;
			setBeepPatternS( 0xff00 );
            break;
        }

		/* �f�[�^�̓]�� */
        printf( "%d,%4d,%4d,%5d,%5d,%5d,%5d,%5d,%5d,%4d,%4d,%4d,%4d,%4d,%4d\n",
            (int)msdBuff[msdBuffAddress+0],                  /* �p�^�[��     */
            (int)msdBuff[msdBuffAddress+1],					/* �Z���^�[*/
            (unsigned char)msdBuff[msdBuffAddress+2],                  /* ���C�h */
			(int)((unsigned char)msdBuff[msdBuffAddress+3]*0x100 +
	             (unsigned char)msdBuff[msdBuffAddress+4] ),			/* �p�x */
            /* �T�[�{PWM */
	            msdBuff[msdBuffAddress+6],
	            /* ���OPWM */
	            msdBuff[msdBuffAddress+7],
	            /* �E�OPWM */
	            msdBuff[msdBuffAddress+8],
	            /* ����PWM */
	            msdBuff[msdBuffAddress+9],
	            /* �E��PWM */
	            msdBuff[msdBuffAddress+10],
	            /* �G���R�[�_ */
	            msdBuff[msdBuffAddress+11] * 2,
				/* ���[�h */
	            msdBuff[msdBuffAddress+12],
				/* �⓹�� */
	            msdBuff[msdBuffAddress+13],
				/* �����x�Z���T�[Y */
	            msdBuff[msdBuffAddress+14],
				/* �����x�Z���T�[X */
	            msdBuff[msdBuffAddress+15],
				/* �ԊO���Z���T�[�̍� */
	            msdBuff[msdBuffAddress+5]
        );		

        
		if(date_f_mode != 0){
			date_f_make((int)msdBuff[msdBuffAddress+0],(int)((unsigned char)msdBuff[msdBuffAddress+3]*0x100 +
	             (unsigned char)msdBuff[msdBuffAddress+4] ) ,msdBuff[msdBuffAddress+11] ,(int)msdBuff[msdBuffAddress+12]);
		}
		
		 msdBuffAddress += 64;  /* ���̑��M����                 */

        if( msdBuffAddress >= 512 ) {
            pattern = 106;
        }
		
        break;

    case 108:
        /* �]���I�� */
		if(date_f_mode == 1){
			/* �u���b�NA �C���[�Y���܂� */
			blockEraseDataFlash( 0x3000 );
			/* �u���b�NB �C���[�Y���܂� */
			blockEraseDataFlash( 0x3400 );
			
			/* �u���b�NA �������� */
			writeDataFlash( 0x3000, date_f_buff, 32 );
			
			
			if( readMicroSD( msdStartAddress_ch , msdBuff_ch ) != 0x00 ) {
				// �ǂݍ��݃G���[ 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)date_f_buff_ch[i] = msdBuff_ch[i];
			}
			
			/* �u���b�NB �������� */
			writeDataFlash( 0x3400, date_f_buff_ch, 32 );
			printf("date_f_mode == 1\n");
			
		}else if(date_f_mode == 2){
			/* �u���b�NC �C���[�Y���܂� */
			blockEraseDataFlash( 0x3800 );
			/* �u���b�ND �C���[�Y���܂� */
			blockEraseDataFlash( 0x3c00 );
			
			/* �u���b�NC �������� */
			writeDataFlash( 0x3800, date_f_buff, 32 );
			
			if( readMicroSD( msdStartAddress_ch , msdBuff_ch ) != 0x00 ) {
				// �ǂݍ��݃G���[ 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)date_f_buff_ch[i] = msdBuff_ch[i];
			}
		
			/* �u���b�ND �������� */
			writeDataFlash( 0x3c00, date_f_buff_ch, 32 );
		
			printf("date_f_mode == 2\n");
		}
		
		j=0;
		for(i = 0; i < 16; i++){
			//printf("%d%d",date_f_buff[j],date_f_buff[j+1]);
			printf("%d\n",date_f_buff_int[i]);
			
			j+=2;
		}
		
		printf("\n");
		for(i = 0; i < 32; i+=3){
			printf("%d %d%02d\n",date_f_buff_ch[i],date_f_buff_ch[i+1],date_f_buff_ch[i+2]);
		}
		
		
		pattern = 109;
        led_out( 0xff );
        break;
		
	case 109:
		led_out( 0xff );
		break;
	case 200://���s�I��
		setBeepPatternS( 0xff00 );
		
		while(1){
			cam_in();
			if(logfin == 0)led_out(camera(Center,Wide));
			else led_out(0);
			
			if(iEncoder10 < 5 ){
				if(msdFlag == 1)msdFlag = 2;                /* �f�[�^�L�^�I��               */
				servoPwmOut( 0 );
			}else{
				servoPwmOut( iServoPwm );
			}
		
		/*	if( pushsw_get() ) {
				while(pushsw_get());
		 		for(i = 0; i < 32; i+=2){
					printf("%d %d\n",date_buff_ch_int[i],date_buff_ch_int[i+1]);
				}
			}*/
			motor_f( 0, 0 );
            motor_r( 0, 0 );
			if(logfin == 0){
				if(msdFlag == 0){
					//setBeepPatternS( 0xff00 );
					
					servoPwmOut( 0 );
					
					j = 0;
					for(i = 0; i < 32; i+=2){
						//printf("%d %d\n",date_buff_ch_int[i],date_buff_ch_int[i+1]);
						msdBuff_ch[j++] = (signed char)date_buff_ch_int[i];
						msdBuff_ch[j++] = (signed char)(date_buff_ch_int[i+1]/100);
						msdBuff_ch[j++] = (signed char)(date_buff_ch_int[i+1]%100);
					}
					
					while( checkMicroSDProcess() != 11 )wait(2); /* �������݂��I���܂ő҂� */
					// �������ݏ������I���܂ŌJ��Ԃ�
					while( microSDProcessEnd() != 0 )wait(2);
					
					for(i = 0; i < 3; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					/* microSDProcess�J�n���� */
					while(microSDProcessStart( msdStartAddress_ch) != 0x00)wait(2);
            		
					setMicroSDdata( msdBuff_ch );
					
					while( checkMicroSDProcess() != 11 )wait(2); /* �������݂��I���܂ő҂� */
					// �������ݏ������I���܂ŌJ��Ԃ�
					while( microSDProcessEnd() != 0 )wait(2);
			
					
					for(i = 0; i < 5; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					logfin = 1;
				}
			}
		}
		break;

		
	case 500:
		
		if( pushsw_get() ) {
			mode++;
			if(mode >= 4)mode = 0;
			while(pushsw_get());
		}
		cam_in();
		led_out(camera(Center,Wide));
	
		printf("mode = %d  %4d   %4d\n",mode,Center,Wide);
		break;
	
    default:
        break;
    }
    }
}


int date_f_make(int pattern, int angle, int encoder, int rmode){

	static int mode = 0; //S = 0, L = 1, R = 2, C = 3, H = 4, F = 5
	static long Encoder = 0;
	static int buff_num = 0;
	static int buff_num_int = 0;
	static int buff_num_ch = 0;
	static int buff_num_int_ch = 0;
	
	Encoder += encoder;
	
	switch(pattern){
		case -1:
			if(	Encoder >= 500){//���̋����ȉ��͖����i����H�j
				if(mode == 0){//S
					if(Encoder > 1000){//���̋����ȉ��͖���
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
					mode = 5;//F
				}
			}
			break;
		case 10:
		case 11:
		if(	Encoder >= 500){//���̋����ȉ��͖����i����H�j
			if(mode == 0){//S
				if(((rmode == 0 ) && (angle < -Cu_Angle)) || ((rmode != 0 ) && (angle < -(Cu_Angle + 10)))){
					if(Encoder > 1000){//���̋����ȉ��͖���
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
						
					mode = 1;//L
						
				}else if(((rmode == 0) &&(Cu_Angle < angle)) || ((rmode != 0) &&(Cu_Angle < angle+10))){
					if(Encoder > 1000){//���̋����ȉ��͖���
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
					
					Encoder =0;
						
					mode = 2;//R
				}
			}else if(mode == 1){//L�J�[�u
				if(((rmode == 0) && (-Cu_Angle < angle)) || ((rmode != 0) && (-(Cu_Angle+10) < angle)) ){
				
					Encoder =0;
						
					mode = 0;//S
				}
			}else if(mode == 2){//R�J�[�u
				if(((rmode ==0) && (angle < Cu_Angle) || ((rmode !=0) && (angle < Cu_Angle+10)))){
				
					Encoder =0;
						
					mode = 0;//S
				}
			}else{
				if(((rmode == 0) &&(-Cu_Angle < angle && angle < Cu_Angle)) || ((rmode != 0) &&(-(Cu_Angle+10) < angle && angle < Cu_Angle+10))){
					Encoder =0;
					mode = 0;//S
				}
			}

		}
		break;

		case 21:
		case 22:
			if(mode != 3){//C
				if(mode == 0){//S
					if(Encoder > 1000){//���̋����ȉ��͖���
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = Encoder/100;
						buff_num++;
						date_f_buff[buff_num] = Encoder - ((Encoder/100)*100);
						buff_num++;
					}
				}
				Encoder =0;
			}
			mode = 3;//C
				
			break;
		case 31:
			if(mode != 2){//R
				//���̂Ƃ���Encoder���Ȃ���܂ł̋���
				date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
						
				Encoder =0;
			}
			mode = 2;//R
			break;
		case 41:
			if(mode != 1){//L
				//���̂Ƃ���Encoder���Ȃ���܂ł̋���
				date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
				Encoder =0;	
			}
			mode = 1;//L
			break;

		case 51:
		case 61:
		case 52:
		case 62:
			if(mode != 4){//H
				if(mode == 0){//S
					if(Encoder > 1000){//���̋����ȉ��͖���
						date_f_buff_int[buff_num_int] += Encoder;
						buff_num_int++;
						
						date_f_buff[buff_num] = (Encoder/100);
						buff_num++;
						date_f_buff[buff_num] = (Encoder - ((Encoder/100)*100));
						buff_num++;
					}
				}
				Encoder =0;
			}
			mode = 4;//H
			break;
		case 53:
			if(mode != 1){//L
				//���̂Ƃ���Encoder���Ȃ���܂ł̋���
				date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
				
				Encoder =0;
			}
			mode = 1;//L
			break;
		case 63:
			if(mode != 2){//R
				//���̂Ƃ���Encoder���Ȃ���܂ł̋���
				date_f_buff_ch[buff_num_ch] = pattern;
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder/100);
				buff_num_ch++;
				date_f_buff_ch[buff_num_ch] = (Encoder - ((Encoder/100)*100));
				buff_num_ch++;
							
				Encoder =0;
			}
			mode = 2;//R
			break;
			
		default:
			break;
	}
	
	return 0;
}

/************************************************************************/
/* R8C/38A �X�y�V�����t�@���N�V�������W�X�^(SFR)�̏�����                */
/************************************************************************/
void init( void )
{
    int     i;

    /* �N���b�N��XIN�N���b�N(20MHz)�ɕύX */
    prc0  = 1;                          /* �v���e�N�g����               */
    cm13  = 1;                          /* P4_6,P4_7��XIN-XOUT�[�q�ɂ���*/
    cm05  = 0;                          /* XIN�N���b�N���U              */
    for(i=0; i<50; i++ );               /* ���肷��܂ŏ����҂�(��10ms) */
    ocd2  = 0;                          /* �V�X�e���N���b�N��XIN�ɂ���  */
    prc0  = 0;                          /* �v���e�N�gON                 */

    /* �|�[�g�̓��o�͐ݒ� */

    /*  PWM(�\��)       ���OM_PMW       �E�OM_PWM       �u�U�[
        �Z���T���[      �Z���T����      �Z���T�E��      �Z���T�E�[  */
    p0   = 0x00;
    prc2 = 1;                           /* PD0�̃v���e�N�g����          */
    pd0  = 0xf0;

    /*  �Z���T���S      �����ް         RxD0            TxD0
        DIPSW3          DIPSW2          DIPSW1          DIPSW0          */
    pur0 |= 0x04;                       /* P1_3�`P1_0�̃v���A�b�vON     */
    p1  = 0x00;
    pd1 = 0x10;

    /*  �E�OM_����      �X�e�AM_����    �X�e�AM_PWM     �E��M_PWM
        �E��M_����      ����M_PWM       ����M_����      ���OM_����      */
    p2  = 0x00;
    pd2 = 0xff;

	
	/*  7:���[�h�o��1bit           6:��6           5:���[�h�o��2bit            4:��5
        3:��4            2:�G���R�[�_B��      	   1:��3            0:�G���R�[�_A��   */
    p3  = 0x00;
    pd3 = 0xa0;	
	
    /*  XOUT            XIN             �{�[�h���LED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;

	
    /*      �Z���^�[�l          */
	pur1 |= 0x0c;   /* P5_7�`P5_0�̃v���A�b�vON     */
    p5  = 0x00;
    pd5 = 0x00;

    /*      SD�J�[�h 0,1,2,3,4 
			��(2,1,0bit) 7,6,5          */
	pur1 |= 0x30;   /* P6_7�`P6_0�̃v���A�b�vON     */
	//pur1_addr.bit.b5;	/* P6_4 to P6_7 pull-up */
    p6  = 0x00;
    pd6 = 0x1f;
	
    /*  CN6.2����       CN6.3����       CN6.4����       CN6.5����
        none(��۸ޗ\��) �p�xVR          �Z���T_����۸�  �Z���T_�E��۸�  */
    p7  = 0x00;
    pd7 = 0x00;

    /*  DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED
        DIPSWorLED      DIPSWorLED      DIPSWorLED      DIPSWorLED      */
    pur2 |= 0x03;                       /* P8_7�`P8_0�̃v���A�b�vON     */
    p8  = 0x00;
    pd8 = 0x00;

    /*  -               -               �߯������       P8����(LEDorSW)
        �E�OM_Free      ���OM_Free      �E��M_Free      ����M_Free      */
    p9  = 0x00;
    pd9 = 0x1f;
    pu23 = 1;   // P9_4,P9_5���v���A�b�v����
    /* �^�C�}RB�̐ݒ� */
    /* ���荞�ݎ��� = 1 / 20[MHz]   * (TRBPRE+1) * (TRBPR+1)
                    = 1 / (20*10^6) * 200        * 100
                    = 0.001[s] = 1[ms]
    */
    trbmr  = 0x00;                      /* ���샂�[�h�A������ݒ�       */
    trbpre = 200-1;                     /* �v���X�P�[�����W�X�^         */
    trbpr  = 100-1;                     /* �v���C�}�����W�X�^           */
    trbic  = 0x06;                      /* ���荞�ݗD�惌�x���ݒ�       */
    trbcr  = 0x01;                      /* �J�E���g�J�n                 */

    /* A/D�R���o�[�^�̐ݒ� */
    admod   = 0x33;                     /* �J��Ԃ��|�����[�h�ɐݒ�     */
  //  adinsel = 0x90;                     /* ���͒[�qP7��4�[�q��I��      */
	adinsel = 0xb0;                     /* ���͒[�qP7��8�[�q��I��      */
    
    adcon1  = 0x30;                     /* A/D����\                  */
    asm(" nop ");                       /* ��AD��1�T�C�N���E�G�C�g�����*/
    adcon0  = 0x01;                     /* A/D�ϊ��X�^�[�g              */

    /* �^�C�}RG(�ʑ��v�����[�h)�̐ݒ� */
  //  timsr = 0xc0;                       /* TRGCLKA,TRGCLKB�[�q���蓖��  */
    //trgcntc = 0xff;                     /* �ʑ��v��Ӱ�ނ̶��ĕ��@�w��   */
  //  trgmr = 0x82;                       /* TRG�̃J�E���g�J�n            */
	
	/* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ� */
    timsr = 0x40;                       /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
    //trgcr = 0x15;                       /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
	trgcr = 0x05;                       /* TRGCLKA�[�q�̗����オ��G�b�W�ŃJ�E���g*/
    trgmr = 0x80;                       /* TRG�̃J�E���g�J�n            */
   

    /* �^�C�}RC PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^) */
    trcpsr0 = 0x40;                     /* TRCIOA,B�[�q�̐ݒ�           */
    trcpsr1 = 0x33;                     /* TRCIOC,D�[�q�̐ݒ�           */
    trcmr   = 0x0f;                     /* PWM���[�h�I���r�b�g�ݒ�      */
    trccr1  = 0x8e;                     /* �������:f1,�����o�͂̐ݒ�    */
    trccr2  = 0x00;                     /* �o�̓��x���̐ݒ�             */
    trcgra  = TRC_MOTOR_CYCLE - 1;      /* �����ݒ�                     */
    trcgrb  = trcgrb_buff = trcgra;     /* P0_5�[�q��ON��(���O���[�^)   */
    trcgrc  = trcgra;                   /* P0_7�[�q��ON��(�\��)         */
    trcgrd  = trcgrd_buff = trcgra;     /* P0_6�[�q��ON��(�E�O���[�^)   */
    trcic   = 0x07;                     /* ���荞�ݗD�惌�x���ݒ�       */
    trcier  = 0x01;                     /* IMIA������                   */
    trcoer  = 0x01;                     /* �o�͒[�q�̑I��               */
    trcmr  |= 0x80;                     /* TRC�J�E���g�J�n              */

    /* �^�C�}RD ���Z�b�g����PWM���[�h�ݒ�(����Ӱ��A�E��Ӱ��A����Ӱ�) */
    trdpsr0 = 0x08;                     /* TRDIOB0,C0,D0�[�q�ݒ�        */
    trdpsr1 = 0x05;                     /* TRDIOA1,B1,C1,D1�[�q�ݒ�     */
    trdmr   = 0xf0;                     /* �o�b�t�@���W�X�^�ݒ�         */
    trdfcr  = 0x01;                     /* ���Z�b�g����PWM���[�h�ɐݒ�  */
    trdcr0  = 0x20;                     /* �\�[�X�J�E���g�̑I��:f1      */
    trdgra0 = trdgrc0 = TRD_MOTOR_CYCLE - 1;    /* �����ݒ�             */
    trdgrb0 = trdgrd0 = 0;              /* P2_2�[�q��ON��(���ヂ�[�^)   */
    trdgra1 = trdgrc1 = 0;              /* P2_4�[�q��ON��(�E�ヂ�[�^)   */
    trdgrb1 = trdgrd1 = 0;              /* P2_5�[�q��ON��(�T�[�{���[�^) */
    trdoer1 = 0xcd;                     /* �o�͒[�q�̑I��               */
    trdstr  = 0x0d;                     /* TRD0�J�E���g�J�n             */
	
	
}

/************************************************************************/
/* �^�C�}RB ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt /B intTRB(vect=24)
void intTRB( void )
{
    unsigned int i;
	int k;
	//static unsigned int EncoderMod = 0;
	//static int iEncoder5_old = 0; 
	static unsigned int iEncoder1_buf[10] = {0}; 
	//static unsigned long  SEncoderTotal3 = 0;          /* �ώZ�l�ۑ��p                 */
	int iEncoder_buf = 0; 
	int a;
	static int flag = 0,flag20 = 0,flag56 = 0;
	signed char *p;
	
    asm(" fset I ");                    /* �^�C�}RB�ȏ�̊��荞�݋���   */
	
    cnt1++;
	cnt2++;
	cnt3++;
	cnt7++;
	cnt8++;
	
	/* microSD�Ԍ��������ݏ���(1ms���ƂɎ��s)   */
	microSDProcess();

	
	//�����x�Z���T�[
	get_angle_y();
	get_angle_x();

	
    /* �T�[�{���[�^���� */
    servoControl();
	servoControl2();
	
    /* �u�U�[���� */
    beepProcessS();
	
	////////////////////////////////////////////////////////
	
	/*if( pushsw_get() ) {
		 	lEncoderTotal2 = 0;
			lEncoderTotal = 0;	
		}*/
	
	//33*3.14  // 1�p���X*1.03 = 1mm  //1��] 100�p���X
	i = trg;
    iEncoder1_buf[iTimer10] = (i - uEncoderBuff);
	lEncoderTotal += iEncoder1_buf[iTimer10];
	//lEncoderTotal = lEncoderTotal2; 
    uEncoderBuff = i;
	
	iEncoder_buf = 0;
	for(k = 0; k < 10; k++)iEncoder_buf += iEncoder1_buf[k];
	iEncoder10 = iEncoder_buf;
	

    /* 10��1����s���鏈�� */
    iTimer10++;
    switch( iTimer10 ) {

    case 1:
        break;

    case 2:
        /* �X�C�b�`�ǂݍ��ݏ��� */
        p9_4 = 0;                       /* LED�o��OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* �X�C�b�`�ǂݍ��݁ALED�o�� */
        types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
        p8  = types_led;                /* ��ײ�ފ��TypeS Ver.3��LED�֏o��*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED�o��ON                    */
        break;

    case 4:
	case 9:
	/*	iEncoder5_old = iEncoder5 ;
		
		//21*3.14=65.94 200�p���X 200/65.94=3�p���X��1mm
		i = trg;
        iEncoder5       = (i - uEncoderBuff)/3;	
		EncoderMod     = (i - uEncoderBuff)%3;	
        lEncoderTotal += iEncoder5;
		//uEncoderBuff = i ;
        uEncoderBuff = i - EncoderMod ;
		
		iEncoder10 = iEncoder5 + iEncoder5_old ; 
*/		

		iEncoder_buf = 0;
		for(k = 0; k < 5; k++)iEncoder_buf += iEncoder1_buf[(10 + iTimer10 - k) % 10];
		iEncoder5 = iEncoder_buf;
	
		if(date_f_mode != 0 && ( msdFlag == 1 || msdFlag == 2 )){//�Đ����s���[�h
			a = getServoAngle();
			//���� 
			if((pattern == 11 || pattern == 10) && (((mode == 0) &&(-Cu_Angle < a && a < Cu_Angle)) || ((mode != 0) &&(-(Cu_Angle+10) < a && a < Cu_Angle+10)) ) ){
				SEncoderTotal += iEncoder5;//�����v��
				
				if(flag56 == 1){//�n�[�t��̋����␳
					date_f_buff_int[date_f_num] -= date_f_plus_h;
					flag56 = 0;	
				}
				
				if(mode == 0){
					//if(date_f_buff_int[date_f_num] - date_f_brake < SEncoderTotal)flag = 1;//�L�^���������𑖂���
					if(date_f_buff_int[date_f_num] - 600 < SEncoderTotal)flag = 1;//�L�^���������𑖂���
				}else{
					//if(date_f_buff_int[date_f_num] - date_f_brake - 500 < SEncoderTotal)flag = 1;//�L�^���������𑖂���
					if(date_f_buff_int[date_f_num] - 600 - 500 < SEncoderTotal)flag = 1;//�L�^���������𑖂���
				}
				flag20 = 0;
			
			}else if(pattern == 21 || pattern == 22  ){
				if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//���̒����҂����
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//�L�^�ςݒ����I��
				}
				
				if(flag20 == 0){
					flag20 = 1;
					flag56 = 0;
					//SEncoderTotal = 0;
					SEncoderTotal = iEncoder5;
				}else{
					SEncoderTotal += iEncoder5;//�N���X���C���Ȃǂ���̋����v��
				}
				
			}else if( pattern == 51 || pattern == 52 || pattern == 61 || pattern == 62 ){
				if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//���̒����҂����
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//�L�^�ςݒ����I��
				}
				
				if(flag20 == 0){
					flag20 = 1;
					flag56 = 1;
					//SEncoderTotal = 0;
					SEncoderTotal = iEncoder5;
				}else{
					SEncoderTotal += iEncoder5;//�N���X���C���Ȃǂ���̋����v��
				}
				
			}else{//�J�[�u or �N�����N,�n�[�t�Ȃ��蒆 
				 SEncoderTotal = 0;
				 if(flag == 1){
					 flag = 0;
					 if(date_f_num < 15)date_f_num++;//���̒����҂����
					 if(date_f_buff_int[date_f_num] ==0 || date_f_num == 15)flag20 = 99;//�L�^�ςݒ����I��
				 }
			}
		
			if(flag20 == 99){//�L�^�ςݒ����I��
				//�N�����N�A�n�[�t���I��
				if(date_f_buff_ch_int[date_f_num_ch] == 0 && (pattern == 11 || pattern == 10))date_f_mode = 0;//�Đ����s�I��
			}
		}
		
		
		
		/* microSD�L�^���� */
    	if( msdFlag == 1 || msdFlag == 2 ) {
			p = msdBuff + msdBuffAddress;

            /* �o�b�t�@�ɋL�^ �������� */
            *p++ = pattern;             /* �p�^�[��                     */
            *p++ = (char)Center;    
            *p++ = (char)Wide;
			i = getServoAngle();        /* �p�x                         */
			*p++ = i >> 8;
            *p++ = i & 0xff;
			
			*p++ = (char)Center_IR;//�ԊO���Z���T�[
			
			*p++ = handleBuff;  /* �T�[�{PWM�ۑ�        */
            *p++ = FleftMotorBuff;       /* �O�����[�^PWM�l                */
            *p++ = FrightMotorBuff;      /* �O�E���[�^PWM�l                */
			*p++ = RleftMotorBuff;       /* �㍶���[�^PWM�l                */
            *p++ = RrightMotorBuff;      /* ��E���[�^PWM�l                */
			*p++ = iEncoder5;    /* �G���R�[�_                   */
            *p++ = mode;/*���[�h*/
            *p++ = flag2;/*�⓹��*/
            *p++ = (char)(angle_y/10);
            *p++ = (char)(angle_x/10);
			
		//	*p++ = (char)(IR_L());
        //    *p++ = (char)(IR_R());
            
			
            /* �o�b�t�@�ɋL�^ �����܂� */

            msdBuffAddress += 64;       /* RAM�̋L�^�A�h���X������      */


			if( msdBuffAddress >= 512) {	
                /* 512�ɂȂ�����AmicroSD�ɋL�^���� */
                msdBuffAddress = 0;
                setMicroSDdata( msdBuff );
                msdWorkAddress += 512;
				
                if( msdWorkAddress >= msdEndAddress || msdFlag == 2) {
                    /* �L�^�����I�� */
    				msdFlag = 0;
                }
            }
		}
        break;

    case 5:
        break;

    case 6:
        break;

    case 7:
        break;

    case 8:
        break;

  //  case 9:
    //    break;

    case 10:
        /* iTimer10�ϐ��̏��� */
        iTimer10 = 0;
        break;
    }
}

/************************************************************************/
/* �^�C�}RC ���荞�ݏ���                                                */
/************************************************************************/
#pragma interrupt intTRC(vect=7)
void intTRC( void )
{
    trcsr &= 0xfe;

    /* �^�C�}RC�@�f���[�e�B��̐ݒ� */
    trcgrb = trcgrb_buff;
    trcgrd = trcgrd_buff;
}


/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char sw;

    sw = p1 & 0x0f;                     /* P1_3�`P1_0�ǂݍ���           */

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    return types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char sw;

    sw = ~p9_5 & 0x01;

    return sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��CN6�̏�ԓǂݍ���                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0�`15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char data;

    data = p7 >> 4;

    return data;
}


/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out( unsigned char led )
{
	/*
	int i;
	unsigned char LED = 0,l[8];
	
	//	8bit�\���p�ɕϊ�	
	l[0] = led & 0x01;
	l[4] = (led & 0x02) >> 1;
	l[1] = (led & 0x04) >> 2;
	l[5] = (led & 0x08) >> 3;
	l[2] = (led & 0x10) >> 4;
	l[6] = (led & 0x20) >> 5;
	l[3] = (led & 0x40) >> 6;
	l[7] = (led & 0x80) >> 7;
	
	for(i = 7; i > 0; i--){
		LED += l[i];
		LED <<= 1;
	}
	LED += l[0];
	
	// ���ۂ̏o�͂̓^�C�}RB���荞�ݏ����Ŏ��{ 
    types_led = LED;
	*/
	types_led = led;
	
}


/************************************************************************/
/* ��ւ̑��x���� �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_r( int accele_l, int accele_r )
{
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < -100){
		accele_l = -100;
	}
	
	
	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < -100){
		accele_r = -100;
	}
	
	
	RleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    RrightMotorBuff = accele_r;         /* �o�b�t�@�ɕۑ�               */
	

    /* ���ヂ�[�^ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
	/*	if(accele_l == 100){
			p2_2 = 1;
			//trdpsr0 = 0x08;                     // TRDIOB0,C0,D0�[�q�ݒ�        
    		//trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1�[�q�ݒ�     
		}else{
			trdpsr0 = 0x08;                     // TRDIOB0,C0,D0�[�q�ݒ�        
    		trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1�[�q�ݒ�     
		}*/
		
		if(accele_l == 100)trdgrd0 = (long)( TRD_MOTOR_CYCLE);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
		if(accele_l == -100)trdgrd0 = (long)( TRD_MOTOR_CYCLE);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
		if(accele_r == 100)trdgrc1 = (long)( TRD_MOTOR_CYCLE);
        else trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
		
		//trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
		if(accele_r == -100)trdgrc1 = (long)( TRD_MOTOR_CYCLE);
        else trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
		
		//trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}


/************************************************************************/
/* �O�ւ̑��x���� �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_f( int accele_l, int accele_r )
{
	if(accele_l > 100){
		accele_l = 100;
	}else if(accele_l < -100){
		accele_l = -100;
	}
	

	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < -100){
		accele_r = -100;
	}
	
	
	FleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    FrightMotorBuff = accele_r;         /* �o�b�t�@�ɕۑ�               */
	
	
    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = trcgrb_buff = trcgra;
    } else {
		if(accele_l == 100)trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE);
        else trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
		
		//trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = trcgrd_buff = trcgra;
    } else {
		if(accele_r == 100)trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE);
        else trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
		
		//trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}


/************************************************************************/
/* ��ւ̑��x����3 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_r( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < -100)accele_l = -100;
	
	
	if(accele_l == 0 || (RleftMotorBuff > 0 && accele_l < 0) || (RleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - RleftMotorBuff  > KASOKU)accele_l = RleftMotorBuff + KASOKU;
	else if(accele_l - RleftMotorBuff < -KASOKU)accele_l = RleftMotorBuff - KASOKU;
	
	
	if(accele_r > 100)accele_r = 100;
	else if(accele_r < -100)accele_r = -100;
	
	
	if(accele_r == 0 || (RrightMotorBuff > 0 && accele_r < 0) || (RrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - RrightMotorBuff > KASOKU)accele_r = RrightMotorBuff + KASOKU;
	else if(accele_r - RrightMotorBuff < -KASOKU)accele_r = RrightMotorBuff- KASOKU;
	
	
	RleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    RrightMotorBuff = accele_r;          /* �o�b�t�@�ɕۑ�               */

    /* ���ヂ�[�^ */
    if( accele_l >= 0 ) {
        p2_1 = 0;
	/*	if(accele_l == 100){
			p2_2 = 1;
			//trdpsr0 = 0x08;                     // TRDIOB0,C0,D0�[�q�ݒ�        
    		//trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1�[�q�ݒ�     
		}else{
			trdpsr0 = 0x08;                     // TRDIOB0,C0,D0�[�q�ݒ�        
    		trdpsr1 = 0x05;                     // TRDIOA1,B1,C1,D1�[�q�ݒ�     
		}*/
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
		//if(accele_l == -100)p2_2 = 1;
        trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
		//if(accele_r == 100)p2_4 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
		//if(accele_r == -100)p2_4 = 1;
        trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_r ) / 100;
    }
}


/************************************************************************/
/* �O�ւ̑��x����3 �f�B�b�v�X�C�b�`�ɂ͊֌W���Ȃ�motor�֐�              */
/* �����@ �����[�^:-100�`100 , �E���[�^:-100�`100                       */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor2_f( int accele_l, int accele_r )
{
	if(accele_l > 100)accele_l = 100;
	else if(accele_l < -100)accele_l = -100;
	
	
	if(accele_l == 0  || (FleftMotorBuff > 0 && accele_l < 0) || (FleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - FleftMotorBuff  > KASOKU)accele_l = FleftMotorBuff + KASOKU;
	else if(accele_l - FleftMotorBuff < -KASOKU)accele_l = FleftMotorBuff - KASOKU;
	

	if(accele_r > 100)accele_r = 100;
	else if(accele_r < -100)accele_r = -100;
	
	
	if(accele_r == 0 || (FrightMotorBuff > 0 && accele_r < 0) || (FrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - FrightMotorBuff > KASOKU)accele_r = FrightMotorBuff + KASOKU;
	else if(accele_r - FrightMotorBuff < -KASOKU)accele_r = FrightMotorBuff- KASOKU;
	

	FleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    FrightMotorBuff = accele_r;          /* �o�b�t�@�ɕۑ�               */
		
    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = trcgrb_buff = trcgra;
    } else {
		//if(accele_l == 100)p0_5 = 1;
        trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = trcgrd_buff = trcgra;
    } else {
		//if(accele_r == 100)p0_6 = 1;
        trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
    }
}

/************************************************************************/
/* �ヂ�[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_r( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_0 = 1;
    } else {
        p9_0 = 0;
    }
    if( mode_r ) {
        p9_1 = 1;
    } else {
        p9_1 = 0;
    }
}

/************************************************************************/
/* �O���[�^��~����i�u���[�L�A�t���[�j                                 */
/* �����@ �����[�^:FREE or BRAKE , �E���[�^:FREE or BRAKE               */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void motor_mode_f( int mode_l, int mode_r )
{
    if( mode_l ) {
        p9_2 = 1;
    } else {
        p9_2 = 0;
    }
    if( mode_r ) {
        p9_3 = 1;
    } else {
        p9_3 = 0;
    }
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �T�[�{���[�^PWM�F-100�`100                                    */
/*        0�Œ�~�A100�Ő��]100%�A-100�ŋt�]100%                        */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void servoPwmOut( int pwm )
{
	int i = getServoAngle();
	
	if((i < -SERVO_MAX && pwm  > 0) || ( SERVO_MAX < i && pwm  < 0)){//�n���h���Ȃ�����
		pwm = -(pwm / 8);//�t�]				
	}
	
	pwm = -pwm;//
	
	if( pwm >  100 ) pwm =  100;        /* �}�C�R���J�[�����肵����     */
    if( pwm < -100 ) pwm = -100;        /* �����90���炢�ɂ��Ă������� */
    
	handleBuff = pwm;                 /* �o�b�t�@�ɕۑ�               */


    if( pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -pwm ) / 100;
    }
}


/************************************************************************/
/* �T�[�{�p�x�擾                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( ad2 - iAngle0 );
}

/************************************************************************/
/* �X�^�[�g�Q�[�g���o����												*/
/* �߂�l 0:�Ȃ� 1:����													*/
/************************************************************************/
unsigned char check_startgate( void )
{
	unsigned char ret;
	
	ret = 0;
	
	//if(IR_R() > 140){//37
	if(950 < ad7){ 
		ret = 1;
	}
	/*
	if((Wide >= 27) && (Center > -20) && (Center < 20)){
	
		ret = 1;			// �Q�[�g���� 
	}*/
	return ret;
}
 

/************************************************************************/
/* �N���X���C�����o����													*/
/* �߂�l 0:�N���X���C���Ȃ� 1:����										*/
/************************************************************************/
unsigned char check_crossline( void )
{
	unsigned char ret;
	
	//cam_in();//�l�̎擾
	
	ret = 0;
	if( (Wide > 70) || ((Wide >= 50) && (-13 < Center ) && (Center < 13)) ){// || ((Wide >= 30) && (Center > -4) && (Center < 4))){
	
		ret = 1;			/* �N���X���C������ */
	}
	return ret;
}
 

/************************************************************************/
/* �n�[�t���C�����o����                                                 */
/* �߂�l 0:�n�[�t���C���Ȃ� 1:�� 2:�E 3:�N�����N                       */
/************************************************************************/
unsigned char check_halfline( void )
{
    unsigned char ret;
	int center;
	
	ret = 0;
	if(Wide > 36 && Wide < 50){
		if(Center < -7){//�Z���^�[�����
			ret = 1;
			
		}else if(Center > 7){//�Z���^�[�E���
			ret = 2;
			
		}
	}else if(Wide > 32){
		if(Center < -11){//�Z���^�[�����
			ret = 1;
			
		}else if(Center > 11){//�Z���^�[�E���
			ret = 2;
			
		}
	}else if(Wide > 28){
		if(Center < -18){//�Z���^�[�����
			ret = 1;
			
		}else if(Center > 18){//�Z���^�[�E���
			ret = 2;
			
		}
	}
	
	return ret;
}

/************************************************************************/
/* wideline���o����													*/
/* �߂�l 0:�Ȃ� 1:����										*/
/************************************************************************/
unsigned char check_wideline( void )
{
	unsigned char ret;
	
	//cam_in();//�l�̎擾
	
	ret = 0;
	if(Wide >= 40){
	
		ret = 1;			/* wide���C������ */
	}
	return ret;
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� iServoPwm �ɑ��                               */
/************************************************************************/
void servoControl( void )
{
	long     iRet, iP, iD;
	static int iI = 0;

  
    /* �T�[�{���[�^�pPWM�l�v�Z */
	if(IR_flag == 0){	
		iP = kp * (Center + Center_offset);      /* ���                         */
		iI = iI + (iSensorBefore - (Center + Center_offset) );
    	iD = kd * (iSensorBefore - (Center + Center_offset));     /* ����(�ڈ���P��5�`10�{)       */
		iRet = iP - iD - iI * ki;
	}else{
		iP = kp_ir * (Center_IR);      /* ���                         */
		iI = iI + (iSensorBeforeIR - (Center_IR) );
    	iD = kd_ir * (iSensorBeforeIR - (Center_IR));     /* ����(�ڈ���P��5�`10�{)       */
		iRet = iP - iD - iI * ki_ir;
	}
    iRet /= 2;//256 128

    /* PWM�̏���̐ݒ� */
    if( iRet >  100 ) iRet =  100;        /* �}�C�R���J�[�����肵����     */
    if( iRet < -100) iRet = -100;        /* �����90���炢�ɂ��Ă������� */
    iServoPwm = iRet;

    iSensorBefore = (Center + Center_offset );                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
	iSensorBeforeIR = Center_IR;
}

/************************************************************************/
/* ���W���[���� servoControl2                                            */
/* �����T�v     �T�[�{���[�^����                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �O���[�o���ϐ� iServoPwm2 �ɑ��                         */
/************************************************************************/
void servoControl2( void )
{
    int      i,j, iRet, iP, iD;


    i = iSetAngle;              	/* �ڕW�p�x             */
    j = getServoAngle();              	/* �ڕW�p�x�Z���T�l             */

    /* �T�[�{���[�^�pPWM�l�v�Z */
    iP = - kp2 * (j - i);                        /* ���                         */
    iD = - kd2 * (iAngleBefore2 - j);     /* ����(�ڈ���P��5�`10�{)       */
    iRet = iP - iD;
    iRet /= 2;

    /* PWM�̏���̐ݒ� */
    if( iRet >  100 ) iRet =  100;        /* �}�C�R���J�[�����肵����     */
    if( iRet < -100 ) iRet = -100;        /* �����90���炢�ɂ��Ă������� */

    iServoPwm2 = iRet;

    iAngleBefore2 = j;                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/*	wait																*/
/************************************************************************/
void wait(int t)
{
	cnt3 = 0;
	while (cnt3 <= t);
}

/**************************************************************************/
/*	RX62G����̃J�����̒l���擾											  */
/**************************************************************************/
void cam_in(){
	int wide = 0;
		
	wide += ((p6 >> 5 ) & 0x07);//012
	wide += ((p3 << 2 ) & 0x08);//3
	wide += ((p3 << 1 ) & 0x30);//45
	wide += ( p3 & 0x40);//6
	
	Wide = wide;

	
	if(1 < Wide  && Wide < 100){//�J�����̎��t���ŏ�������Ă����̂�
		Center = (p5 & 0x7f) - 64 + 3;	
	}else{
		Center = (p5 & 0x7f) - 64;
	}
}

/**************************************************************************/
/*	RX62G�փ��[�h�̒l���o��											  */
/**************************************************************************/
void mode_out(){
	
	p3_5 = mode & 0x01;//LOW
	p3_7 = (mode & 0x02) >> 1;//HIGH
}

/**************************************************************************/
/*	LED�\���p�̒l���v�Z													  */
/**************************************************************************/
int camera(int center, int wide){
	
	int start = 17,end = 111,i,led = 0,cnt = 1;
	// (end - start)/7 = 13
	int led_bit[8] = {999,42,52,62,999,66,76,86};//4�͕s��
	
	center += 64;
	wide /= 2;
	
	for(i = 0; i < 8; i++){
		if(center - wide < led_bit[i] && led_bit[i] < center + wide){
			led += cnt;
		}
		cnt *= 2;
	}

	return led;
}

/************************************************************************/
/* char�^�f�[�^�̒l��long�^�ϐ���2�i���ŕϊ�                            */
/* �����@ unsigned char �ϊ�����8bit�f�[�^                              */
/* �߂�l unsigned long �ϊ���̕ϐ�(0�`11111111) ��0��1��������܂���  */
/************************************************************************/
unsigned long convertBCD_CharToLong( unsigned char hex )
{
    int             i;
    unsigned long   l = 0;

    for( i=0; i<8; i++ ) {
        l *= 10;
        if( hex & 0x80 ) l += 1;
        hex <<= 1;
    }

    return l;
}

/**************************************************************************/
/*	�W���C���Z���T�[��Y���̒l���擾										  */
/**************************************************************************/
void get_angle_y(){
	
	pre_angle_y[0] = pre_angle_y[1];
	pre_angle_y[1] = angle_y;
	
	angle_y = (angle_y * 9/10) + (ad0 * 1/10);
	
	if(((pre_angle_y[0] < pre_angle_y[1]) && (pre_angle_y[1] < angle_y))  || ((angle_y < pre_angle_y[1]) && (pre_angle_y[1] < pre_angle_y[0])))
		angle_y = pre_angle_y[1];
		
	else if(((pre_angle_y[1] < pre_angle_y[0]) && (pre_angle_y[0] < angle_y))  || ((angle_y < pre_angle_y[0]) && (pre_angle_y[0] < pre_angle_y[1])))
		angle_y = pre_angle_y[0];
}

/**************************************************************************/
/*	�W���C���Z���T�[��X���̒l���擾													  */
/**************************************************************************/
void get_angle_x(){
	pre_angle_x[0] = pre_angle_x[1];
	pre_angle_x[1] = angle_x;
	
	angle_x = (angle_x * 85/100) + (ad1 * 15/100);
	
	if(((pre_angle_x[0] < pre_angle_x[1]) && (pre_angle_x[1] < angle_x))  || ((angle_x < pre_angle_x[1]) && (pre_angle_x[1] < pre_angle_x[0])))
		angle_x = pre_angle_x[1];
		
	else if(((pre_angle_x[1] < pre_angle_x[0]) && (pre_angle_x[0] < angle_x))  || ((angle_x < pre_angle_x[0]) && (pre_angle_x[0] < pre_angle_x[1])))
		angle_x = pre_angle_x[0];
}

/**************************************************************************/
/*	�W���C���Z���T�[��X���̒l���`�F�b�N													  */
/**************************************************************************/
int angle_check(){

	if(230 < angle_y && angle_y < 350){
		if(angle_x <= 190)return 2;//��
		if(angle_x > 440)return 0;//��
	}
	return 1;//�ω�����
}



/**************************************************************************/
/*	IR_L												  */
/**************************************************************************/
int IR_L(){
	
	return 100 * (ad6 - IR_min[0]) / (IR_max[0] - IR_min[0]);
}
/**************************************************************************/
/*	IR_R												  */
/**************************************************************************/
int IR_R(){
	
	return 100 * (ad7 - IR_min[1]) / (IR_max[1] - IR_min[1]);
}
/**************************************************************************/
/*	Get_Center_IR											  */
/**************************************************************************/
void Get_Center_IR(){
	int l = IR_L(),r = IR_R();
	IR_old = Center_IR;
	
	Center_IR = (r-l)/5;
	
	if(l < 15 && r < 15){
		if(	IR_old < 0)Center_IR = -30;
		else Center_IR = 30;
	}
	
}
/**************************************************************************/
/*	�ԊO���Z���T�[�̃L�����u���[�V����												  */
/**************************************************************************/
void IRcalibration( ){
	int i ,j,l,r,p = 20,a = 60;

	iAngle0 = 0;
	iAngle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
	i = getServoAngle();
	
	for(j = 0; j < 2;j++){
		while(-a < i){
			i = getServoAngle();
			servoPwmOut(p);
			r = ad7;
			if(IR_max[1] < r)IR_max[1] = r;
			if(r < IR_min[1])IR_min[1] = r;
			l = ad6;
			if(IR_max[0] < l)IR_max[0] = l;
			if(l < IR_min[0])IR_min[0] = l;
		}
	
		while(i < a){
			i = getServoAngle();
			servoPwmOut(-p);
			r = ad7;
			if(IR_max[1] < r)IR_max[1] = r;
			if(r < IR_min[1])IR_min[1] = r;
			l = ad6;
			if(IR_max[0] < l)IR_max[0] = l;
			if(l < IR_min[0])IR_min[0] = l;
		}
	}
	while(0 < i){
		i = getServoAngle();
		servoPwmOut(p);
		r = ad7;
		if(IR_max[1] < r)IR_max[1] = r;
		if(r < IR_min[1])IR_min[1] = r;
		l = ad6;
		if(IR_max[0] < l)IR_max[0] = l;
		if(l < IR_min[0])IR_min[0] = l;
	}
	servoPwmOut( 0 );
}

/************************************************************************/
/* end of file                                                          */
/************************************************************************/



/*
�����o��

2011.06.01 Ver.1.00 �쐬
2012.02.23 Ver.1.01 ���[�^�h���C�u���TypeS Ver.3��
                    �A�i���O�Z���T���TypeS Ver.2�̃R�����g�ύX
2013.05.10 Ver.2.00 ���[�^�h���C�u���TypeS Ver.4�ɑΉ�
*/
