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

#define 	MAXTIME 			1110 //1100	  	/* �ő呖�s���� (0.01�b)  1200 = 12s     1250     */


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
unsigned char check_halfline_forC( void );//1 = ��, 2 = �E

int getServoAngle( void );
void servoControl( void );
void servoControl2( void );
void cam_in(void);
void wait(int);
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
int max(int,int);


/*======================================*/
/* �O���[�o���ϐ��̐錾                 */
/*======================================*/
int             i_pattern;                /* �}�C�R���J�[����p�^�[��     */
unsigned long   ul_cnt_1ms = 0;            /* �^�C�}�p                     */
unsigned long   ul_cnt_running_1ms = 0;    /* �ő呖�s���ԗp               */
unsigned long   ul_cnt_for_wait = 0;       /* wait �֐��p                  */
unsigned long   ul_cnt_saka = 0;           /* ��Z���T�[�`�F�b�N�p         */
unsigned long   ul_cnt_straight_time_1ms = 0;/* �����ƃJ�[�u�̃J�E���g�p   */
unsigned long   ul_cnt_curve_time_1ms = 0;   /* �J�[�u�����p�̃J�E���g�p   */
char 			c_running_flag = 0;			/* 1 = ���s��					*/
char			c_mode = 0;				/* 0 = �ʏ� 1 = ��	2 = �E���� 3 = ������	*/
char			c_saka_cnt = 0;				/* ���� = �������Ȃ� � = ��������*/
int				i_out_cnt = 0;			//�E���J�E���g
char			c_out_flag = 0;			//�E��flag 1=�R�[�X�A�E�g
char 			c_black_flag = 0;			//
char			c_Cu_flag = 0;			//0 = ����, 1 = �J�[�u
char			c_c_short_mode = 0; //�N�����N�@0:long 1:short
char			c_c_cut;	//0= �Đ����Ȃ� 1= �Đ����� �ҏW���Ӗ�

/* microSD�֘A�ϐ� */
signed char     c_msdBuff[ 512 ];         /* �ꎞ�ۑ��o�b�t�@             */
int             i_msdBuffAddress;         /* �ꎞ�L�^�o�b�t�@�����A�h���X */
int             i_msdFlag = 0;                /* 1:�f�[�^�L�^ 0:�L�^���Ȃ�    */
unsigned long   ul_msdStartAddress;        /* �L�^�J�n�A�h���X             */
unsigned long   ul_msdEndAddress;          /* �L�^�I���A�h���X             */
unsigned long   ul_msdWorkAddress;         /* ��Ɨp�A�h���X               */
int             i_msdError = 0;               /* �G���[�ԍ��L�^               */

signed char     c_msdBuff_ch[ 512 ];         /* �ꎞ�ۑ��o�b�t�@             */
unsigned long   ul_msdStartAddress_ch = 0x0200;        /* �L�^�J�n�A�h���X             */


char c_logfin = 0;

/* ���݂̏�ԕۑ��p */
int             i_handleBuff;             /* ���݂̃n���h���p�x�L�^       */
int             i_FleftMotorBuff = 0;          /* ���݂̑O�����[�^PWM�l�L�^      */
int             i_FrightMotorBuff = 0;         /* ���݂̑O�E���[�^PWM�l�L�^      */
int             i_RleftMotorBuff = 0;          /* ���݂̌㍶���[�^PWM�l�L�^      */
int             i_RrightMotorBuff= 0;         /* ���݂̌�E���[�^PWM�l�L�^      */


/* �}�C�R�����t���b�V���������֘A */
int				i_date_f_mode = 0;		//0=�Ȃ� 1=IN 2=OUT
signed char 	c_date_f_buff[32] ={0};
int			 	i_date_f_buff_int[16] ={0};
int				i_date_f_num = 0;
int				i_Cu_Angle	=		20;		//�J�[�u����Ɏg�p ��������
int				i_Cu_Angle_saka	=	45;		//�J�[�u����Ɏg�p �������� ��p
signed char 	c_date_f_buff_ch[32] ={0};
int			 	i_date_f_buff_ch_int[32] ={0};//�������p�^�[���@�������
int				i_date_f_num_ch = 0;

/*�N�����N�A�n�[�t�����v���p*/
int			 	i_date_buff_ch_int[32] ={0};//�������p�^�[���@�������
int				i_date_num_ch = 0;
long            l_EncoderTotal_ch = 0;          /* �ώZ�l�ۑ��p                 */

/* �G���R�[�_�֘A */
int             i_Timer10 = 0;               /* 10ms�J�E���g�p               */
long            l_EncoderTotal = 0;          /* �ώZ�l�ۑ��p                 */
int             i_Encoder10 = 0;               /* 10ms���̍ŐV�l               */
int             i_Encoder5  = 0;               /*  5ms���̍ŐV�l               */
unsigned int    ui_EncoderBuff  = 0;           /* �v�Z�p�@���荞�ݓ��Ŏg�p     */
long			l_startPoint = 0;			/* �����v���p�X�^�[�g�n�_ */
long			l_startPoint_saka = 0;		/* �����v���p�X�^�[�g�n�_ ��H */
long			l_startPoint_curve = 0;		/* �����v���p�X�^�[�g�n�_ �J�[�u�I���ʒu*/
long            l_straight_EncoderTotal = 0;   /* �����ώZ�l�ۑ��p                 */

/*  �T�[�{�֘A */
int             i_SensorBefore = 0;          /* �O��̃Z���T�l�ۑ�           */
int             i_SensorBeforeIR = 0;          /* �O��̃Z���T�l�ۑ�           */
int             i_AngleBefore2 = 0;
int             i_ServoPwm = 0;              /* �T�[�{�o�v�l�l               */
int             i_ServoPwm2 = 0;
int             i_Angle0 = 0;                /* ���S����A/D�l�ۑ�            */
int 			i_SetAngle = 0;
int				i_AngleBefore = 0;

/* �Z���T�֘A */
int             i_SensorPattern;         /* �Z���T��ԕێ��p             */
int  			i_Center;					/*�J�����Z���^�[	0~127				*/
int			    i_Wide;					/*������			0~127			*/
int  			i_Center_old;					/*�J�����Z���^�[				*/
int			    i_Wide_old;					/*������						*/
int  			i_Center_offset = 0;		/*�J�����Z���^�[���ړ����񂹂�				*/
int 			i_angle_y = 0;				/* �W���C���Z���T�[��Y���̒l	*/
int 			i_angle_x = 0;				/* �W���C���Z���T�[��X���̒l	*/
int 			i_pre_angle_y[2] = {0};		/* �W���C���Z���T�[��Y���̉ߋ��̒l	*/
int 			i_pre_angle_x[2] = {0};		/* �W���C���Z���T�[��X���̉ߋ��̒l	*/
int 			i_Center_IR = 0;
char 			c_IR_flag = 0;
int 			i_IR_max[2] = {0};
int				i_IR_min[2] = {1024,1024};
int 			i_IR_cnt = 0;
int 			i_IR_old = 0;


/* TRC���W�X�^�̃o�b�t�@ */
unsigned int    ui_trcgrb_buff;            /* TRCGRB�̃o�b�t�@             */
unsigned int    ui_trcgrd_buff;            /* TRCGRD�̃o�b�t�@             */

/* ���[�^�h���C�u���TypeS Ver.3���LED�A�f�B�b�v�X�C�b�`���� */
unsigned char   uc_types_led;              /* LED�l�ݒ�                    */
unsigned char   uc_types_dipsw;            /* �f�B�b�v�X�C�b�`�l�ۑ�       */



/*	�p�����[�^	*/
/*************************************************************************************************

���[�^�̏o�͂��グ�邱�Ƃ��^�C���̌���ɒ�������킯�ł͂Ȃ�

�ɒ[�ȍ��o�͐ݒ�͗\�I�i�t���[���s�j�̂Ƃ������ɂ��邱��
���肵�Ȃ��̂Ńg�[�i�����g�ŃR�[�X�A�E�g����

�J�[�u�Ŋ���Ȃ����Ƃ��ŏd�v

**************************************************************************************************/
//�I�t�Z�b�g
int  		i_Center_offset_MAX = 8;		/*�J�[�u���J�����Z���^�[���ړ����񂹂� �ŏ��l 0 	*/
int  		i_Center_offset_Angle = -3;	/*���̒l�ɂ��P�h�m���Ɋ񂹂�	���FIN�@���FOUT		*/

int			i_KASOKU = 15;


#define		MOTOR_OUT_BASE			90		//�J�[�u�O���p�@�O�����[�^�[�p�p�����[�^�[ 

#define		MOTOR_OUT_BASE_N		100		//�J�[�u�㔼�p�@�O�����[�^�[�p�p�����[�^�[ 

#define		MAX_TOPSPEED	65	//�u�[�X�g���ł����̑��x�ȏ�͏o�Ȃ��悤�ɐ������� JMCR�w�胂�[�^�����̍ő呬�x�Ɠ����ɐݒ肷��

int		    i_TOPSPEED	=		50;		//���� 

/////////////////////////////////////////////////////////////////////////////////////// 0:�֎~ 1��-1�͓���
//�O��
int			i_SPEED_DOWN	=		4;		//�p�x�ɂ��i_TOPSPEED������ �J�[�u�O�� 8 6
int			i_MOTOR_out_R	=		 1;		//�O�����[�^�[�p�p�����[�^�[ 1	-2
int			i_MOTOR_in_F	=		 4;		//�������[�^�[�p�p�����[�^�[ 	2 	1
int			i_MOTOR_in_R	=		 -2;		//�������[�^�[�p�p�����[�^�[ -2	-3
	
//�㔼
int			i_SPEED_DOWN_N=		6;		//�p�x�ɂ��i_TOPSPEED������  �J�[�u�㔼 11 10
int			i_MOTOR_out_R_N=		3;		//�O�����[�^�[�p�p�����[�^�[ �㔼	5	5
int			i_MOTOR_in_F_N=		7;		//�������[�^�[�p�p�����[�^�[�@�㔼	6	6
int			i_MOTOR_in_R_N=		4;		//�������[�^�[�p�p�����[�^�[�@�㔼	3	3


#define		date_f_brake		400	//�Đ����s�� �ʏ푖�s�Ɠ��l�̑��x���������鋗�� 400
#define		date_f_brake2		65	//�Đ����s���@�c�苗��/date_f_brake2 �������x������グ�� ���l��傫�����������x���Ȃ�(0�ɂ͂��Ȃ����Ɓj

#define		Cu_FREE_time  		25		//�J�[�u�I�����̌�փt���[�̎���(msec�j

#define		Cu_BRAKE_time  		10		//�J�[�u�i�����̃u���[�L���� (msec)
#define		Cu_BRAKE_SP 		30		//�J�[�u�i�����ɂ��̑��x�ȏ�Ȃ�u���[�L
#define		Cu_BRAKE			-80		//�J�[�u�i�����̃u���[�L�i��ցj 
#define		Cu_BRAKE_out		 0		//�J�[�u�i�����̃u���[�L(�O��OUT�j 
#define		Cu_BRAKE_Fin		-25		//�J�[�u�i�����̃u���[�L(�O��IN�j 


#define		Cu_N_time			200	//Cu_N_time ms �J�[�u�𑖍s����ƌ㔼�ɂȂ� 	

#define		Cu_BRAKE_N			50	//�J�[�u�O�����̃u���[�L�̑O������

//////////////////////////////////////////////////////////////////////////////////////////////
//��
int			i_S_flag = 2;				//�⓹�@��������@1 = �������Ȃ�  2 = ��������
int			i_saka_max	  =		  1;	//�F���\�ȍ�̐�
#define 	KASA_Encoder1  	100	//��J�n	50
#define 	KASA_Encoder2  	400	//���r�� �I��� 300
#define 	KASA_Encoder3  	1500	//���I��� 

#define		KASA_Encoder4  	3800	//���I���  2500
#define		KASA_Encoder5  	4400	//����I��� �ʏ�ɂ��ǂ� 

#define		KASA_Encoder4_2  3000	//���I���(�Ō�̍⓹)2500 3000
#define		KASA_Encoder5_2  3600	//����I��� �ʏ�ɂ��ǂ�(�Ō�̍⓹) 3000 3200 3500 4500


//�Ζ�(���)
#define		    TOPSPEED2			44		//����(��j30 33
#define			SPEED_DOWN2			20		//�p�x�ɂ��TOPSPEED������(��j�J�[�u�O��
#define			SPEED_DOWN2_N		20		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out2_R		20 		//�O�����[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in2_F			20		//�������[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in2_R			20		//�������[�^�[�p�p�����[�^�[(��j


//�Ζ�(���,����t�߁@��ђ��˖h�~)
#define		    TOPSPEED3			31		//����(��j30 33
#define			SPEED_DOWN3			6		//�p�x�ɂ��TOPSPEED������(��j�J�[�u�O��
#define			SPEED_DOWN3_N		6		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out3_R		2	//�O�����[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in3_F			4		//�������[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in3_R			1		//�������[�^�[�p�p�����[�^�[(��j


//��
#define		    TOPSPEED4			50		//����(���j30 33
#define			SPEED_DOWN4			6		//�p�x�ɂ��TOPSPEED������(���j�J�[�u�O��
#define			SPEED_DOWN4_N		6		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out4_R		1		//�O�����[�^�[�p�p�����[�^�[(���)
#define			MOTOR_in4_F			4		//�������[�^�[�p�p�����[�^�[(���j
#define			MOTOR_in4_R			-2		//�������[�^�[�p�p�����[�^�[(���j


//�Ζ�(����)
#define		    TOPSPEED5			50		//����(��j30 33
#define			SPEED_DOWN5			20		//�p�x�ɂ��TOPSPEED������(��j�J�[�u�O��
#define			SPEED_DOWN5_N		20		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼
#define			MOTOR_out5_R		20	//�O�����[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in5_F			20		//�������[�^�[�p�p�����[�^�[(��j
#define			MOTOR_in5_R			20		//�������[�^�[�p�p�����[�^�[(��j


//////////////////////////////////////////////////////////////////////////////////////////////

//�N�����N�A�n�[�t����̐ݒ�l	�J�[�u�̃p�����[�^�ύX���N�����N�A�n�[�t�ɉe�����Ȃ��悤�ɂ��邽��
#define		    TOPSPEED_CH_Len		600		//�N�����N�A�n�[�t����̂��̋��������͈ȉ��̐ݒ�l�ő���@���ӁF�J�[�u�O���̂ݗL��

#define			MOTOR_OUT_BASE_CH	85		//�J�[�u�O���p�@�O�����[�^�[�p�p�����[�^�[ �N�����N�A�n�[�t�̒���

#define		    TOPSPEED_CH			50		//����
#define			SPEED_DOWN_CH		 6		//�p�x�ɂ��TOPSPEED������  �J�[�u�O��
#define			SPEED_DOWN_CH_N		 14		//�p�x�ɂ��TOPSPEED������  �J�[�u�㔼�i���������j

#define			MOTOR_out_CH_R		 1		//�O�����[�^�[�p�p�����[�^�[
#define			MOTOR_in_CH_F		 4		//�������[�^�[�p�p�����[�^�[
#define			MOTOR_in_CH_R		-2		//�������[�^�[�p�p�����[�^�[


//�N�����N
int		    i_C_TOPSPEED	=		28;		//�N�����N(��)  25 33
int		    i_C_TOPSPEED2	=		50;		//�N�����N(�o)	40

int 		i_C_TOPSPEED4 = 		47;		//�Đ����s���̃u���[�L�O
int		    i_C_TOPSPEED3	=		40;		//�Đ����s�p�N�����N(��)  25 33 

int			i_C_short_len =		600;	//���̋��������̓V���[�g�A�ȏ�̓����O
#define		C_TOPSPEED_SHORT	2		//(i_Encoder10 > C_TOPSPEED + C_TOPSPEED_SHORT )�̂Ƃ��@�������ł��Ă��Ȃ��ꍇ�̓V���[�g
#define		C_TOPSPEED_SHORT_NG	1		//(i_Encoder10 < C_TOPSPEED - C_TOPSPEED_SHORT_NG )�̂Ƃ��@�������̓V���[�g�ł����x���x���Ƃ��̓����O

int			i_date_f_brake_c	=	600;	//�Đ����s���̃u���[�L�g�p�\����(mm) �N�����N�p 600
int			i_date_f_shortcat_c=	280;	//�Đ����s���̃V���[�g�J�b�g����(mm) �N�����N�p 210

char		c_c_cut_master  	 =	  1;	//�Đ����s���ł����Ă� 0= �Đ����Ȃ� 1= �Đ����� 	
int			i_c_cut_encoder	 =	540;  	//���̋��������̏ꍇ�͍Đ����Ȃ�


//�n�[�t 
//#define HWall 							//�ǂ���̎��ɗL�������邱��

#ifdef  HWall //�ǂ���
int		    i_H_TOPSPEED	=		45;		//�n�[�t�i�N���j
int		    i_H_TOPSPEED2	=		42;		//�n�[�t(�΂�)
#else //�ǂȂ�
int		    i_H_TOPSPEED	=		50;		//�n�[�t�i�N���j
int		    i_H_TOPSPEED2	=		45;		//�n�[�t(�΂�) �u���[�L��������Ȃ��悤�ɒl����������
#endif
  
int		    i_H_TOPSPEED2_S=		50;		//�n�[�t(�΂�)  �V���[�g�J�b�g�p
int			i_date_f_brake_h	=	700;	//�Đ����s���̃u���[�L�g�p�\����(mm)�@�n�[�t�p 
int			i_date_f_shortcat_h=	400;		//�Đ����s���̃V���[�g�J�b�g����(mm)�@�n�[�t�p

int			i_date_f_plus_h	=	300;		//�Đ����s���̒���̃X�g���[�g�����␳(mm)�@�n�[�t�p  

#ifdef  HWall
char		c_h_cut 			 =	  0;	//�ǂ���̎��̓V���[�g�J�b�g���Ȃ�
#else
char		c_h_cut 			 =	  1;	//�Đ����s���ł����Ă� 0= �Đ����Ȃ� 1= �Đ�����
#endif
//////////////////////////////////////////////////////////////////////////////////////////////

#define			BRAKE_MAX			-100	//�u���[�L�̍ő�p���[ 
#define			BRAKE_MAX_R			-90	//�u���[�L�̍ő�p���[ ���A�p


int				i_kp = -18;//- 8  3 -16 -19 -23  -13
int				i_kd = -105;//-80 20 -130 -190	-110
int 			i_ki = 0;//-2

int				i_kp_ir = -20;
int				i_kd_ir = -110;
int 			i_ki_ir = 0;


int				i_kp2 = -10;//�p�x�w��p
int				i_kd2 = -50;//�p�x�w��p


/*	���̑�	*/
int			i_topspeed;		
int			i_speed_down;
int			i_speed_down_n;
int			i_motor2_out_R;
int			i_motor2_in_F;
int			i_motor2_in_R;
int			i_MOTOR_out_base = MOTOR_OUT_BASE;

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
    ul_msdStartAddress = 5120;

    // microSD �������ݏI���A�h���X
    // �������݂���������[ms] : x = 10[ms] : 64�o�C�g
    // 60000ms�Ȃ�Ax = 60000 * 64 / 10 = 384000
    // ���ʂ�512�̔{���ɂȂ�悤�ɌJ��グ����B
    ul_msdEndAddress  = 768000;//384000;
    ul_msdEndAddress += ul_msdStartAddress;   /* �X�^�[�g������               */                  

	 /* microSD������ */
    ret = initMicroSD();
    if( ret != 0x00 ) {
        i_msdError = 1;
		printf("%d\n",ret);
        /* �������ł��Ȃ����3�b�ԁALED�̓_�����@��ς��� */
        ul_cnt_1ms = 0;
        while( ul_cnt_1ms < 3000 ) {
            if( ul_cnt_1ms % 200 < 100 ) {
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
	
	i_topspeed = i_TOPSPEED;		//�����l
	i_speed_down = i_SPEED_DOWN;
	i_speed_down_n = i_SPEED_DOWN_N;
	i_motor2_in_F = i_MOTOR_in_F;
	i_motor2_in_R = i_MOTOR_in_R;
	i_motor2_out_R = i_MOTOR_out_R;

	wait(10);
	while(~p8 == 0xff);//�N������͐��l����������
	wait(10);
	
	i_Angle0 = 0;
	i_Angle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
	uc_types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
	

	//�����x�Z���T�[�l�̏�����
	i_angle_y = ad0;
	i_angle_x = ad1;
	
	
	/* �X�^�[�g���A�}�C�R���f�B�b�v�X�C�b�` 1 = ON   �Đ����[�h�@IN */
	if( (dipsw_get() & 0x01) == 0x01 ) {
		i_date_f_mode = 1;
		
		setBeepPatternS( 0x8000 );
		wait(500);
		setBeepPatternS( 0x8000 );
		wait(500);
	
	/* �X�^�[�g���A�}�C�R���f�B�b�v�X�C�b�` 2 = ON   �Đ����[�h�@OUT */
	}else if( (dipsw_get() & 0x02) == 0x02 ) {
		i_date_f_mode = 2;
		
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
        i_pattern = 101;
        ul_cnt_1ms = 0;	
			
	// �X�^�[�g���A�f�B�b�v�X�C�b�` 4 = ON   ��������@1 = �������Ȃ�  2 = ��������
/*	}else if( dipsw_get2() == 0x10 ) {
		i_S_flag = 1;
				
	// �X�^�[�g���A�f�B�b�v�X�C�b�` 3 = ON   RX�Ƃ̒ʐM�`�F�b�N���[�h
	}else if( dipsw_get2() == 0x20) {
		i_pattern = 500;
*/	
	}
 


    while( 1 ) {
		
//	I2CEepromProcess();                 /* I2C EEP-ROM�ۑ�����          */

	mode_out();//��t���O���o��
	
	//�l�̕ۑ�
	i_Center_old = i_Center;
	i_Wide_old = i_Wide;
	
	Get_Center_IR();//�ԊO���Z���T�[
	cam_in();//�l�̎擾

	
	if(i_Wide == 0 && c_mode != 1){//���������� && ��łȂ�
		if(c_black_flag == 1){//�O�����
			
		}else{
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		c_black_flag = 1;
	}else{
		c_black_flag = 0;
	}
	
	if(i_pattern >= 10){
		led_out(0);
	}else if(i_pattern > 1){
		led_out(camera(i_Center,i_Wide));
	}
		
	//�E���`�F�b�N
	if(10 <= i_pattern && i_pattern < 100){
		if(((check_wideline()) || (i_Encoder10 < 3) || (c_mode != 1 && i_Wide == 0 && i_Center == 0  && i_pattern != 53 && i_pattern != 63 && i_pattern != 31 && i_pattern != 41 && i_pattern != 32 && i_pattern != 42)) && (i_pattern >= 10) && (i_pattern != 70)){
			i_out_cnt ++;		
		}else{
			i_out_cnt = 0;
		}
		
		if((l_EncoderTotal > 500) && ((i_pattern != 22 && i_out_cnt > 500) || (i_pattern != 22 && i_out_cnt > 2000 ) )){
			i_pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			c_out_flag = 1;
		}
	}
	
    switch( i_pattern ) {	
    case 0:
        
		/* �v�b�V���X�C�b�`�����҂� */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			i_Angle0 = 0;
			i_Angle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
			
			IRcalibration( );
			
			/*
 			while(1){
				printf("%4d  %4d  %4d -- \t",IR_L(), IR_R(),IR_R() - IR_L());
				printf("%4d  %4d \n",ad6 ,ad7 );				
			}*/
			
            setBeepPatternS( 0xcc00 );
            ul_cnt_1ms = 0;
            i_pattern = 5;
            break;
        }
		
        led_out(camera(i_Center,i_Wide));
		
        break;
	case 5:
		/* �v�b�V���X�C�b�`�����҂� */
        servoPwmOut( 0 );
        if( pushsw_get() ) {
			setBeepPatternS( 0x8000 );
			i_Angle0 = 0;
			i_Angle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
			
			
			ret = eraseMicroSD( ul_msdStartAddress, ul_msdEndAddress-1 );
            if( ret != 0x00 ) {
                /* �C���[�Y�ł��� */
                i_msdError = 2;
            }
            /* microSDProcess�J�n���� */
            ret = microSDProcessStart( ul_msdStartAddress );
            if( ret != 0x00 ) {
                /* �J�n�����ł��� */
                i_msdError = 3;
            }
			
			if(i_date_f_mode == 1){
				readDataFlash( 0x3000, c_date_f_buff, 32 );
				readDataFlash( 0x3400, c_date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					i_date_f_buff_int[j] = (int)(c_date_f_buff[i])*100 + (int)c_date_f_buff[i+1];
					j++;	
				}
				
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					i_date_f_buff_ch_int[j] =(int)c_date_f_buff_ch[i];
					j++;
					
					i_date_f_buff_ch_int[j] = (int)(c_date_f_buff_ch[i+1])*100 + (int)c_date_f_buff_ch[i+2];
					j++;
				}


				
			/*	
					//31�F�E�N�����N 41�F���N�����N 53:���n�[�t 63:�E�n�[�t
					//�N�����N�@����@�߂��F200�@�����F450
					//�n�[�t�@����@350
					 
				i_date_f_buff_ch_int[0] = 63;
				i_date_f_buff_ch_int[2] = 31;
				i_date_f_buff_ch_int[4] = 41;
				i_date_f_buff_ch_int[6] = 31;
				i_date_f_buff_ch_int[8] = 53;
			
				i_date_f_buff_ch_int[1] = 1200;
				i_date_f_buff_ch_int[3] = 580;
				i_date_f_buff_ch_int[5] = 500;
				i_date_f_buff_ch_int[7] = 880;
				i_date_f_buff_ch_int[9] = 700;
				
				//����
				i_date_f_buff_int[0] = 1800;
				i_date_f_buff_int[1] = 1000;
				i_date_f_buff_int[2] = 1500;
				i_date_f_buff_int[3] = 1300;
				i_date_f_buff_int[4] = 3200;
				i_date_f_buff_int[5] = 2800;
				i_date_f_buff_int[6] = 1600;
				i_date_f_buff_int[7] = 1600;
				i_date_f_buff_int[8] = 2000;
				i_date_f_buff_int[9] = 4500;
				i_date_f_buff_int[10] = 1000;
				
			
				
				*/
			}else if(i_date_f_mode == 2){
				readDataFlash( 0x3800, c_date_f_buff, 32 );
				readDataFlash( 0x3c00, c_date_f_buff_ch, 32 );
				
				j = 0;
				for(i = 0; i < 32;i+=2){
					i_date_f_buff_int[j] = (int)(c_date_f_buff[i])*100 + (int)c_date_f_buff[i+1];
					j++;	
				}
				
				j = 0;
				for(i = 0; i < 32; i+=3){
					i_date_f_buff_ch_int[j] =(int)c_date_f_buff_ch[i];
					j++;
					
					i_date_f_buff_ch_int[j] = (int)(c_date_f_buff_ch[i+1])*100 + (int)c_date_f_buff_ch[i+2];
					j++;
				}
		

				/*
					//31�F�E�N�����N 41�F���N�����N 53:���n�[�t 63:�E�n�[�t
					//�N�����N�@����@�߂��F200�@�����F450
					//�n�[�t�@����@350
					 
				i_date_f_buff_ch_int[0] = 41;
				i_date_f_buff_ch_int[2] = 31;
				i_date_f_buff_ch_int[4] = 53;
				i_date_f_buff_ch_int[6] = 63;
			    i_date_f_buff_ch_int[8] = 31;
			
				i_date_f_buff_ch_int[1] = 500;
				i_date_f_buff_ch_int[3] = 880;
				i_date_f_buff_ch_int[5] = 700;
				i_date_f_buff_ch_int[7] = 1200;
				i_date_f_buff_ch_int[9] = 580;
				
				//����
				i_date_f_buff_int[0] = 1600;
				i_date_f_buff_int[1] = 1600;
				i_date_f_buff_int[2] = 1600;
				i_date_f_buff_int[3] = 2000;
				i_date_f_buff_int[4] = 4500;
				i_date_f_buff_int[5] = 1000;
				i_date_f_buff_int[6] = 1500;
				i_date_f_buff_int[7] = 1300;
				i_date_f_buff_int[8] = 3200;
				i_date_f_buff_int[9] = 2800;
				i_date_f_buff_int[10] = 1600;
				*/
			}	
			
			
			i_date_f_num_ch = 0;
			
            setBeepPatternS( 0xcc00 );
            ul_cnt_1ms = 0;
            i_pattern = 1;
			
			wait(1500);////////////////////////////////////////////////////////////////////////////
			
            break;
        }
/*		
		if(angle_check() != 1){ //��Z���T�[�`�F�b�N
			setBeepPatternS( 0x8000 );
			
		}
*/		
        
        if( check_startgate() ) {
             if(((ul_cnt_1ms/100) % 8)%2 == 0)led_out( 0x81);
			 else led_out(camera(i_Center,i_Wide));
			 
        }else{
			led_out(camera(i_Center,i_Wide));
		}
		break;
    case 1:
        /* �X�^�[�g�o�[�J�҂� */
		servoPwmOut( 0 );
		
		motor_f( 0, 0 );
        motor_r( 0, 0 );
			
		old_i = i_Angle0;
		i_Angle0 = 0;
        i_Angle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
		i_Angle0 = (i_Angle0 + old_i) >> 1;
		
        if(  (!check_startgate()) && (i_Wide != 0)) {//�Q�[�g���������@&& ���C����������
		
            ul_cnt_1ms = 0;
			ul_cnt_running_1ms = 0;
			ul_cnt_straight_time_1ms = 0;
			c_running_flag = 1;//���s�J�n
			l_EncoderTotal = 0;  
			l_startPoint = 0;
			l_startPoint_saka = 0;
			
			c_saka_cnt = 0;//�⓹�̉�
			
			i_msdBuffAddress = 0;
            ul_msdWorkAddress = ul_msdStartAddress;
            i_msdFlag = 1;                /* �f�[�^�L�^�J�n               */
		
            i_pattern = 10;
            break;
			
        }
		
        led_out( 1 << (ul_cnt_1ms/50) % 8 );
        break;

	case 10://�X�^�[�g����
		
		if(i_Center < -10)i_Center = -10;
		if(i_Center > 10)i_Center = 10;
		
		old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;
		

		if(l_EncoderTotal < 200){// && i_Wide > 28){//���s�J�n����͒��� && �Q�[�g�����Ă�
			c_mode = 1;//�������������
			
			i_SetAngle = 0;
			servoPwmOut( i_ServoPwm2 );		
		}else{//�ʏ�
			c_mode = 0;	
			servoPwmOut( i_ServoPwm / 2 );
		}
		
		if( i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
			x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;
				
	
			motor_f( x, x );
            motor_r( x, x );
	
		}else{
			motor_f(100 , 100 );
         	motor_r(100 , 100 );
		}
		
		
		if(l_EncoderTotal > 700){
			
			c_mode = 0;//���ɖ߂�
			
			i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
		}
		break;
		
    case 11:
        /* �ʏ�g���[�X */
		 
		old_i = i;//�O��̊p�x���L��
        i = getServoAngle();//�n���h���p�x�擾
		i = (i +old_i) >> 1;
		
		if(c_mode != 1 && i_Wide == 0){//�⒆�ł͂Ȃ��@�����@����������
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		
		if(((i_Center - i_Center_old) < -15 || 15 < (i_Center - i_Center_old)) && ( i_Wide < 30)){//�}�Ƀ��C�����ω�������
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;
		}
		
		if(c_mode == 1
		 && i_Wide > 28){//�⒆ && ��������
			i_Center = i_Center_old;
			i_Wide = i_Wide_old;	
		}
		
	
		if(c_mode == 1){
			if(c_IR_flag == 0){//�ԊO���ɐ؂�ւ�
				if(((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (-5 < i_Center_IR  && 	i_Center_IR < 5) || (KASA_Encoder2 <= (l_EncoderTotal-l_startPoint_saka)) )){
					//if(KASA_Encoder1 <= (l_EncoderTotal-l_startPoint_saka) && (l_EncoderTotal-l_startPoint_saka) < KASA_Encoder2 + 300){
					if(KASA_Encoder1 <= (l_EncoderTotal-l_startPoint_saka)){
						i_IR_cnt++;
						if(i_IR_cnt > 10){
							c_IR_flag = 1;
							i_IR_cnt = 0;
						}	
					}
				}
			}else{//�J�����ɐ؂�ւ�
				if((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (	-5 < i_Center_IR  && 	i_Center_IR < 5)){
				//	if(KASA_Encoder4 <= (l_EncoderTotal-l_startPoint_saka)){
					
					if(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4)) || 
						((i_saka_max == 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4_2))  ){
					
						i_IR_cnt++;
						if(i_IR_cnt > 10){
							c_IR_flag = 0;
							i_IR_cnt = 0;
						}	
					}
				}
			}
		}else{
			if(c_IR_flag == 1){//��ȊO�ŐԊO�����[�h�Ȃ�J�����ɐ؂�ւ�
				if((-10 < i && i < 10) && (-5 < i_Center && i_Center < 5 && i_Wide != 0) && (	-5 < i_Center_IR  && 	i_Center_IR < 5)){
					
					i_IR_cnt++;
					if(i_IR_cnt > 20){
						c_IR_flag = 0;
						i_IR_cnt = 0;
						c_mode = 0;
					}	
				}
			}
		}	
		
		
		if(ul_cnt_running_1ms >=  MAXTIME * 10){//���s���ԏI��
			i_pattern = 200;
			motor_mode_f( BRAKE, BRAKE );
    		motor_mode_r( BRAKE, BRAKE );
			break;
		}
		
		
//if(c_saka_cnt == 0){//�Q��ڂ͐�΂Ɍ��o���Ȃ��@�ꎞ�I�ȑ΍�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B�B		
		if(-15 < i && i < 15){
			if(c_mode == 0){//�ʏ�
				//if(angle_check() == 2 && ( (c_saka_cnt%2 == 1) || ((l_EncoderTotal-l_startPoint_saka) >= 1000) && ((l_EncoderTotal-l_startPoint ) >= 1000) )){//��Z���T�[�`�F�b�N l_startPoint =�N�����N�I���ʒu
				//if(angle_check() == 2 && ( ((l_EncoderTotal-l_startPoint_saka) >= 600) && ((l_EncoderTotal-l_startPoint ) >= 1000) )){//��Z���T�[�`�F�b�N l_startPoint =�N�����N�I���ʒu
				if(angle_check() == 2 &&  (((l_EncoderTotal-l_startPoint_saka) >= 600) || ((c_saka_cnt%2 == 1) && ((l_EncoderTotal-l_startPoint_saka) >= 300))) && ((l_EncoderTotal-l_startPoint ) >= 1000) ){//��Z���T�[�`�F�b�N l_startPoint =�N�����N�I���ʒu
					ul_cnt_saka++;
				}else{
					ul_cnt_saka = 0;
				}
			}else{
				ul_cnt_saka = 0;
			}
		}
//}		
		//if(-20 < i && i < 20){
		if(-50 < i && i < 50){
			if(c_mode == 0){//�⒆�łȂ����
				if(l_EncoderTotal > 200 && (l_startPoint_saka == 0 || (l_EncoderTotal-l_startPoint_saka) >= 500) && (l_startPoint == 0 || (l_EncoderTotal-l_startPoint ) >= 150) && (l_startPoint_curve == 0 || (l_EncoderTotal-l_startPoint_curve) >= 0)){//�Q�[�g�ɔ������Ȃ��悤�� && ��I�����班���̊Ԃ͖��� && �N�����N�A�n�[�t�I���㏭������ && �J�[�u����͖���
				
					if(i_date_f_mode == 0){
						if( check_crossline() ) {       // �N���X���C���`�F�b�N         
            				ul_cnt_1ms = 0;
            				i_pattern = 21;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
				
        				}else if(check_halfline() == 1){//�����[���`�F���W�`�F�b�N
							ul_cnt_1ms = 0;
            				i_pattern = 51;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
				
						}else if(check_halfline() == 2){//�E���[���`�F���W�`�F�b�N
							ul_cnt_1ms = 0;
        	    			i_pattern = 61;
							l_startPoint = l_EncoderTotal;
							l_EncoderTotal_ch = l_EncoderTotal;
							c_mode = 0;
							i_Center_offset = 0;
							i_date_f_num_ch++;
							break;
						}
					}else{
						//if( check_wideline() == 1) {       // �����������Ȃ�����      
						//if( (check_crossline() || check_halfline() != 0 ||  check_wideline() == 1) && i_Encoder10 < 60){ 		
						if( (check_crossline() || check_halfline() != 0 ) && i_Encoder10 < 60){ 
							
							if(i_date_f_buff_ch_int[i_date_f_num_ch] == 31 || i_date_f_buff_ch_int[i_date_f_num_ch] == 41) {       // �N���X���C���`�F�b�N         
            					ul_cnt_1ms = 0;
            					i_pattern = 21;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								
								
								if(c_c_cut_master == 0){
									c_c_cut = 0;
								}else{
									if(i_date_f_buff_ch_int[i_date_f_num_ch] < i_c_cut_encoder)c_c_cut = 0;
									else c_c_cut = 1;
								}
						
								break;
				
        					}else if(i_date_f_buff_ch_int[i_date_f_num_ch] == 53){//�����[���`�F���W�`�F�b�N
								ul_cnt_1ms = 0;
            					i_pattern = 51;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								break;
				
							}else if(i_date_f_buff_ch_int[i_date_f_num_ch] == 63){//�E���[���`�F���W�`�F�b�N
								ul_cnt_1ms = 0;
        		    			i_pattern = 61;
								l_startPoint = l_EncoderTotal;
								l_EncoderTotal_ch = l_EncoderTotal;
								c_mode = 0;
								i_Center_offset = 0;
								i_date_f_num_ch++;
								break;
							}
						}
					}
				}
			}
		}
		
		if(c_mode == 0){//�ʏ�
		
			if(ul_cnt_saka >= 5){//��
			
				if(c_saka_cnt % i_S_flag == 0 && i_saka_max > 0){

					i_saka_max--;
					c_mode = 1;//�⃂�[�h��
					l_startPoint_saka = l_EncoderTotal;

				}
				c_saka_cnt++;
				ul_cnt_saka = 0;
				
				l_startPoint_saka = l_EncoderTotal;//�`���^�����O�h�~
				
			}
		
		}else if(c_mode == 1){//��

			if(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder5)) || 
				((i_saka_max <= 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder5_2))  ){//�ʏ�ɖ߂�
			
				c_mode = 0;
				
				l_startPoint_saka = l_EncoderTotal;//�`���^�����O�h�~
				
				i_TOPSPEED = i_topspeed;
				i_SPEED_DOWN = i_speed_down;
				i_SPEED_DOWN_N = i_speed_down_n;
				i_MOTOR_out_R = i_motor2_out_R;
				i_MOTOR_in_F = i_motor2_in_F;
				i_MOTOR_in_R = i_motor2_in_R;
				
				
			}else if(((i_saka_max > 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4)) || 
				((i_saka_max <= 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder4_2))  ){// ����J�n
				
				
				i_TOPSPEED = TOPSPEED5;
				i_SPEED_DOWN = SPEED_DOWN5;
				i_SPEED_DOWN_N = SPEED_DOWN5_N;
				i_MOTOR_out_R = MOTOR_out5_R;
				i_MOTOR_in_F = MOTOR_in5_F;
				i_MOTOR_in_R = MOTOR_in5_R;
				
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder3){// ���
				i_TOPSPEED = TOPSPEED4;
				i_SPEED_DOWN = SPEED_DOWN4;
				i_SPEED_DOWN_N = SPEED_DOWN4_N;
				i_MOTOR_out_R = MOTOR_out4_R;
				i_MOTOR_in_F = MOTOR_in4_F;
				i_MOTOR_in_R = MOTOR_in4_R;
			
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2){//���x������ɒx�� ���W�����v�h�~
				i_TOPSPEED = TOPSPEED3;
				i_SPEED_DOWN = SPEED_DOWN3;
				i_SPEED_DOWN_N = SPEED_DOWN3_N;
				i_MOTOR_out_R = MOTOR_out3_R;
				i_MOTOR_in_F = MOTOR_in3_F;
				i_MOTOR_in_R = MOTOR_in3_R;
			
			}else if((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1){//���x��x�� ���J�n
				i_TOPSPEED = TOPSPEED2;
				i_SPEED_DOWN = SPEED_DOWN2;
				i_SPEED_DOWN_N = SPEED_DOWN2_N;
				i_MOTOR_out_R = MOTOR_out2_R;
				i_MOTOR_in_F = MOTOR_in2_F;
				i_MOTOR_in_R = MOTOR_in2_R;
			
				
				//if( (i < -20 || 20 < i) && (l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1+500){//����Ă���r���ŃJ�[�u�͂��肦�Ȃ�
		/*		if( i < -20 || 20 < i){//����Ă���r���ŃJ�[�u�͂��肦�Ȃ�
					i_TOPSPEED = i_topspeed;
					SPEED_DOWN = i_speed_down;
					SPEED_DOWN_N = i_speed_down_n;
					i_MOTOR_out_R = i_motor2_out_R;
					i_MOTOR_in_F = i_motor2_in_F;
					i_MOTOR_in_R = i_motor2_in_R;

					c_mode = 0;
					
					c_saka_cnt--;//����̕��𖳂��������Ƃ�
					i_saka_max++;
					l_startPoint_saka = l_EncoderTotal;//�`���^�����O�h�~
				}*/
			}	
		}
		
		 
		
		if(c_mode == 1){
			if(((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder1) && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder2)servoPwmOut( i_ServoPwm /2);
			else servoPwmOut( i_ServoPwm );
		}else{
			servoPwmOut( i_ServoPwm );

		}
		
		if(c_mode != 1){//�⒆�̐ݒ肪�㏑������Ȃ��悤�ɂ���
			if(l_EncoderTotal > 1000 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//�N�����N�A�n�[�t����͐ݒ�l��ς���
				i_MOTOR_out_base = MOTOR_OUT_BASE_CH;
			
				i_TOPSPEED = TOPSPEED_CH;
				i_SPEED_DOWN = SPEED_DOWN_CH;
				i_SPEED_DOWN_N = SPEED_DOWN_CH_N;
				i_MOTOR_out_R = MOTOR_out_CH_R;
				i_MOTOR_in_F = MOTOR_in_CH_F;
				i_MOTOR_in_R = MOTOR_in_CH_R;

				
			}else{//�N�����N�A�n�[�t����łȂ���Βʏ�ݒ�ɂ���
				i_MOTOR_out_base = MOTOR_OUT_BASE;
			
				i_TOPSPEED = i_topspeed;
				i_SPEED_DOWN = i_speed_down;
				i_SPEED_DOWN_N = i_speed_down_n;
				i_MOTOR_out_R = i_motor2_out_R;
				i_MOTOR_in_F = i_motor2_in_F;
				i_MOTOR_in_R = i_motor2_in_R;

			}
		}
		
        if(  i > 12 ){//�n���h���E
		
			if(c_mode != 1){//�⒆�łȂ����
				i_Center_offset = (i-12) / i_Center_offset_Angle ;//�J�[�u�Ŋ񂹂�
				if(i_Center_offset > i_Center_offset_MAX )i_Center_offset = i_Center_offset_MAX;
				if(i_Center_offset < -i_Center_offset_MAX )i_Center_offset = -i_Center_offset_MAX;
			}else{
				i_Center_offset = 0;
			}
			
			
			if((i - old_i > 0) && (c_Cu_flag == 0)){//��������J�[�u�� 
			
				if(ul_cnt_straight_time_1ms >= 30 && (l_EncoderTotal-l_startPoint ) >= 100  && (i_Encoder10 > Cu_BRAKE_SP)){//���܂蒼���𑖂��Ă��Ȃ����̓u���[�L���Ȃ��悤�� && �N�����N�Ȃǂ̒���͖���
					if(l_EncoderTotal > 500 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//�N�����N�A�n�[�t����̓u���[�L���Ȃ�
						ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//�N�����N����̃J�[�u�͂܂��N�����N�̔����Ȃ̂Ńu���[�L�s�v
					}else{
						ul_cnt_straight_time_1ms = 0;
					}
				}
				c_Cu_flag = 1;
			
			}
			
			l_startPoint_curve = l_EncoderTotal;//�J�[�u�I���ʒu�p
				
			if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (l_EncoderTotal-l_startPoint ) >= 100 && (l_EncoderTotal-l_startPoint_saka) >= 100){//�J�[�u�i�����̃u���[�L
				
				servoPwmOut( i_ServoPwm * 2 );
				
				motor_f( Cu_BRAKE_out , Cu_BRAKE_Fin );
            	motor_r( Cu_BRAKE , Cu_BRAKE );
				
			}else if(ul_cnt_curve_time_1ms <= Cu_N_time && i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)) && (l_EncoderTotal-l_startPoint ) >= 200) {// �G���R�[�_�ɂ��X�s�[�h����  �J�[�u�O��
			
				if(c_mode == 1 && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder3 - 300){//��o�蒆�̃u���[�L�͑S�֋ϓ��ɂ���
					x = (i_TOPSPEED -i_Encoder10)*10;
				
					r = x;
					f = x;
				}else if(ul_cnt_curve_time_1ms <= Cu_BRAKE_N && c_mode != 1){//�u���[�L�O�� && �⃂�[�h�łȂ����
				
					x = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*5;	
					r = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*40;
					f = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*20;
				
					if(x < -10) x = -10;
					if(r < -70) r = -70;
					if(f < -25) f = -25;
				}else{
					x = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;	
					r = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*5;
					f = ((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*2;
				
					if(x < -5) x = -5;
					if(r < -15) r = -15;
					if(f < -15) f = -15;	
				}
				motor_f( x, f );
            	motor_r( r, r );			
			
			}else if(i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN_N)) && (l_EncoderTotal-l_startPoint ) >= 200 ) {// �G���R�[�_�ɂ��X�s�[�h����  �J�[�u�㔼
			
				x=((i_TOPSPEED -(i / i_SPEED_DOWN_N))-i_Encoder10)*2;	
				r = x;
				f = x;

				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( x, r );
            	motor_r( f, r );	
					
        	}else{
				
				if((ul_cnt_curve_time_1ms <= Cu_N_time) || (c_mode == 1) || (i > 95) ){//�J�[�u�O�� || �⃂�[�h || �Ȃ����� || 
					if(i_MOTOR_in_F > 0)f= (i_MOTOR_out_base - (i / i_MOTOR_in_F));
					else f= (i_MOTOR_out_base - (i * -i_MOTOR_in_F));
				
					if(i_MOTOR_in_R > 0)r= (i_MOTOR_out_base - (i / i_MOTOR_in_R));
					else r= (i_MOTOR_out_base - (i * -i_MOTOR_in_R));


					if(i_MOTOR_out_R > 0)or = (i_MOTOR_out_base - (i / i_MOTOR_out_R));
					else or = (i_MOTOR_out_base - (i * -i_MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -10) r = -10;
					if(or < 0) or = 0;
					if(f > i_MOTOR_out_base) f = i_MOTOR_out_base;
					if(r > i_MOTOR_out_base) r = i_MOTOR_out_base;
					if(or > i_MOTOR_out_base) or = i_MOTOR_out_base;
				
					motor2_f( i_MOTOR_out_base , f);
           			motor2_r( or, r);
					
				}else{//�J�[�u�㔼

					if(i_MOTOR_in_F_N > 0)f= (MOTOR_OUT_BASE_N - (i / i_MOTOR_in_F_N));
					else f= (MOTOR_OUT_BASE_N - (i * -i_MOTOR_in_F_N));
				
					if(i_MOTOR_in_R_N > 0)r= (MOTOR_OUT_BASE_N - (i / i_MOTOR_in_R_N));
					else r= (MOTOR_OUT_BASE_N - (i * -i_MOTOR_in_R_N));
				
					if(i_MOTOR_out_R_N > 0)or = (MOTOR_OUT_BASE_N - (i / i_MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (i * -i_MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > MOTOR_OUT_BASE_N) f = MOTOR_OUT_BASE_N;
					if(r > MOTOR_OUT_BASE_N) r = MOTOR_OUT_BASE_N;
					if(or > MOTOR_OUT_BASE_N) or = MOTOR_OUT_BASE_N;
				
					motor2_f( MOTOR_OUT_BASE_N, f);
           			motor2_r( or, r);
					
				}
							
			}
			 		 	 
		}else if( i < -12 ){//�n���h����
			
		
			if(c_mode != 1){
				i_Center_offset = (i+12) / i_Center_offset_Angle ;//�J�[�u�Ŋ񂹂�
				if(i_Center_offset > i_Center_offset_MAX )i_Center_offset = i_Center_offset_MAX;
				if(i_Center_offset < -i_Center_offset_MAX )i_Center_offset = -i_Center_offset_MAX;
			}else{
				i_Center_offset = 0;
			}
			
			
			if( (i - old_i < 0) && (c_Cu_flag == 0)){//��������J�[�u�� 
				
				if(ul_cnt_straight_time_1ms >= 30 && (l_EncoderTotal-l_startPoint ) >= 100  && (i_Encoder10 > Cu_BRAKE_SP)){//���܂蒼���𑖂��Ă��Ȃ����̓u���[�L���Ȃ��悤�� && �N�����N�Ȃǂ̒���͖���
					if(l_EncoderTotal > 500 && (l_EncoderTotal-l_startPoint ) < TOPSPEED_CH_Len ){//�N�����N�A�n�[�t����̓u���[�L���Ȃ�
						ul_cnt_straight_time_1ms = Cu_BRAKE_time + 1;//�N�����N����̃J�[�u�͂܂��N�����N�̔����Ȃ̂Ńu���[�L�s�v
					}else{
						ul_cnt_straight_time_1ms = 0;
					}
				}
				
				c_Cu_flag = 1;
		
			}
			
			l_startPoint_curve = l_EncoderTotal;//�J�[�u�I���ʒu�p

			if(ul_cnt_straight_time_1ms <= Cu_BRAKE_time && (l_EncoderTotal-l_startPoint ) >= 100 && (l_EncoderTotal-l_startPoint_saka) >= 100){//�J�[�u�i�����̃u���[�L
				
				servoPwmOut( i_ServoPwm * 2);
				
				motor_f( Cu_BRAKE_Fin, Cu_BRAKE_out );
            	motor_r( Cu_BRAKE, Cu_BRAKE );
				
			}else if(ul_cnt_curve_time_1ms <= Cu_N_time &&  i_Encoder10 >= (i_TOPSPEED -(-i / i_SPEED_DOWN)) && (l_EncoderTotal-l_startPoint ) >= 200 ) {  // �G���R�[�_�ɂ��X�s�[�h���� �J�[�u�O��
				
				if(c_mode == 1 && (l_EncoderTotal-l_startPoint_saka) <= KASA_Encoder3 - 300){//��o�蒆�̃u���[�L�͑S�֋ϓ��ɂ���
					x = (i_TOPSPEED -i_Encoder10)*10;
	
					r = x;
					f = x;
				}else if(ul_cnt_curve_time_1ms <= Cu_BRAKE_N && c_mode != 1){//�u���[�L�O�� && �⃂�[�h�łȂ����
					x = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*5;
					r = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*40;
					f = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*20;
							
		
					if(x < -10) x = -10;
					if(r < -70) r = -70;
					if(f < -25) f = -25;
				}else{
					x = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*2;
					r = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*5;
					f = ((i_TOPSPEED -(-i / i_SPEED_DOWN))-i_Encoder10)*2;
							

					if(x < -5) x = -5;
					if(r < -15) r = -15;
					if(f < -15) f = -15;
				}	
				motor_f( f, x );
            	motor_r( r, r );
				
			}else if(i_Encoder10 >= (i_TOPSPEED -(-i / i_SPEED_DOWN_N)) && (l_EncoderTotal-l_startPoint ) >= 200) {  // �G���R�[�_�ɂ��X�s�[�h���� �J�[�u�㔼
			
				x=((i_TOPSPEED -(-i / i_SPEED_DOWN_N))-i_Encoder10)*2;
				r = x;
				f = x;	
	
				if(x < -5) x = -5;
				if(r < -5) r = -5;
				if(f < -5) f = -5;
				
				motor_f( r, x );
            	motor_r( r, f );
				
        	}else{
				
				if((ul_cnt_curve_time_1ms <= Cu_N_time) || (c_mode == 1) || (i < -95) ){//�J�[�u�O�� || �⃂�[�h || ���
					if(i_MOTOR_in_F >0)f = (i_MOTOR_out_base - (-i / i_MOTOR_in_F));
					else f = (i_MOTOR_out_base - (-i * -i_MOTOR_in_F)); 
				
					if(i_MOTOR_in_R >0)r = (i_MOTOR_out_base - (-i / i_MOTOR_in_R));
					else r = (i_MOTOR_out_base - (-i * -i_MOTOR_in_R));
				
					if(i_MOTOR_out_R >0)or = (i_MOTOR_out_base - (-i / i_MOTOR_out_R));
					else or = (i_MOTOR_out_base - (-i * -i_MOTOR_out_R));
					
					if(f < 0) f = 0;
					if(r < -10) r = -10;
					if(or < 0) or = 0;
					if(f > i_MOTOR_out_base) f = i_MOTOR_out_base;
					if(r > i_MOTOR_out_base) r = i_MOTOR_out_base;
					if(or > i_MOTOR_out_base) or = i_MOTOR_out_base;
				
					motor2_f(f, i_MOTOR_out_base);
           			motor2_r(r, or );
					
				}else{//�J�[�u�㔼
				
					if(i_MOTOR_in_F_N >0)f = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_in_F_N));
					else f = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_in_F_N)); 
				
					if(i_MOTOR_in_R_N >0)r = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_in_R_N));
					else r = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_in_R_N));
				
					if(i_MOTOR_out_R_N >0)or = (MOTOR_OUT_BASE_N - (-i / i_MOTOR_out_R_N));
					else or = (MOTOR_OUT_BASE_N - (-i * -i_MOTOR_out_R_N));
					
					if(f < 0) f = 0;
					if(r < 0) r = 0;
					if(or < 0) or = 0;
					if(f > MOTOR_OUT_BASE_N) f = MOTOR_OUT_BASE_N;
					if(r > MOTOR_OUT_BASE_N) r = MOTOR_OUT_BASE_N;
					if(or > MOTOR_OUT_BASE_N) or = MOTOR_OUT_BASE_N;
				
					motor2_f(f, MOTOR_OUT_BASE_N );
           			motor2_r(r, or );
					
				}	
				
				
			}
			 	 
		}else{//����
			i_Center_offset = 0;
			
			ul_cnt_curve_time_1ms = 0;
						
			if((c_Cu_flag == 1)&&(c_mode == 0)){//�J�[�u���璼���� && �⒆�ł͂Ȃ�
			
				if(ul_cnt_straight_time_1ms >= 60){//�J�[�u����莞�ԑ����Ă����灁���܂�J�[�u�𑖂��Ă��Ȃ����i��֊����j�̓t���[�ɂ������Ȃ�
					ul_cnt_straight_time_1ms = 0;
				}
				c_Cu_flag = 0;
			}
			
			if(ul_cnt_straight_time_1ms <= Cu_FREE_time && (l_EncoderTotal-l_startPoint ) >= 700){//�J�[�u�I��肩���莞�ԓ��@�����@�N�����N�A�n�[�t����̕��A����ł͂Ȃ�
				
				if(c_mode == 0 &&  i_Center < -10) {//�ԑ̍����
				
					motor_f(98 , 100 );
				
				}else if(c_mode == 0 &&  i_Center > 10) {//�ԑ̉E���
				
					motor_f(100 , 98 );
					
				}else{
			
					motor_f(100 , 100 );
				}
				
            	motor_r( 0, 0 );
			
			}else if( (i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN))) && (i_date_f_mode == 0 || c_mode == 1) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
				
				if(c_mode == 1){//��
					x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*10;
					r=x;
			
					motor_f( x, x );
            		motor_r( r, r );
				}else{
					//�ʏ�u���[�L
					motor_f( 0, 0 );
            		motor_r( 0, 0 );
				}
		//�Đ����s���@�c�苗����date_f_brake�ȉ��ɂȂ�����ʏ푖�s�̐������x�܂Ńu���[�L
			}else if(c_mode == 0 && i_date_f_mode != 0 && ((i_date_f_buff_int[i_date_f_num] - date_f_brake)< l_straight_EncoderTotal) && (i_Encoder10 >= (i_TOPSPEED -(i / i_SPEED_DOWN)))  ) {// �G���R�[�_�ɂ��X�s�[�h���� 
					 
				//�Đ����s���@�ŏI�u���[�L
				x=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*20;
				r=((i_TOPSPEED -(i / i_SPEED_DOWN))-i_Encoder10)*10;
			
				motor_f( x, x );
            	motor_r( r, r );
					
			//�Đ����s���@�c�苗���ɉ����đ��x������ύX����
			}else if(c_mode == 0 && i_date_f_mode != 0 && (i_Encoder10 >= max(i_TOPSPEED,i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) ) {// �G���R�[�_�ɂ��X�s�[�h���� 
				
				//�Đ����s���@�u�[�X�g��
				x=(max(i_TOPSPEED,(i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) -i_Encoder10)*10;
				r=(max(i_TOPSPEED,(i_TOPSPEED+(i_date_f_buff_int[i_date_f_num] - l_straight_EncoderTotal - date_f_brake)/date_f_brake2)) -i_Encoder10)*5;
				
				//if(x < -40) x = -40;
				//r = x;
				 
				motor_f( x, x );
            	motor_r( r, r );
			
			}else if(i_Encoder10 >= MAX_TOPSPEED){//�u�[�X�g���ł�MAX_TOPSPEED�ȏ�͏o�Ȃ��悤�ɐ�������
				
				motor_f( 0, 0 );
            	motor_r( 0, 0 );
					
		
			}else if(c_mode == 0 && i_Center < -10) {//�ԑ̍����
				
				motor_f(95 , 100 );
				motor_r(100 , 100 );
			}else if(c_mode == 0 &&  i_Center > 10) {//�ԑ̉E���
				
				motor_f(100 , 95 );
				motor_r(100 , 100 );
				
			}else{
				
				//if((c_mode == 1) &&  (i_Wide == 0) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2) && ((l_EncoderTotal-l_startPoint_saka) < KASA_Encoder3)){//�Ⓒ��t�߂̂Ƃ�
				if((c_mode == 1) && ((l_EncoderTotal-l_startPoint_saka) >= KASA_Encoder2) && ((l_EncoderTotal-l_startPoint_saka) < KASA_Encoder3)){//�Ⓒ��t�߂̂Ƃ�
					motor_f(50 , 50 );
           			motor_r(0 , 0 );		//��ђ��ˌy���̂��ߌ�ւ̓t���[
				}else{
					motor_f(100 , 100 );
           			motor_r(100 , 100 );
				}	
			}
		}       
        break;
		
    case 21:
	
        /* �N���X���C���ʉߏ��� */
      	//setBeepPatternS( 0x8000 );

		if(i_Wide > 20 || i_Center < -20 || i_Center > 20 ){//�N���X���C���������Ă���Ƃ�
			i_SetAngle = 0;
			servoPwmOut( i_ServoPwm2 );
		}else{
			servoPwmOut( i_ServoPwm  );
		}
		
		//�Đ����s���@�@�u���[�L
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4)){
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			x=(i_C_TOPSPEED4-i_Encoder10)*1;
	
			motor_f( x, x );
            motor_r( x, x );
			
		//�N�����N�@�u���[�L
        }else if( (i_date_f_mode == 0 && i_Encoder10 >= i_C_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED)   ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
          
			x = (i_C_TOPSPEED-i_Encoder10)*2;
			r = (i_C_TOPSPEED-i_Encoder10)*1;
			
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
			
			motor_f( x, x );
            motor_r( r, r );
		 
        }else{
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }	

        if( (l_EncoderTotal-l_startPoint ) >= 70 ) {	
           	ul_cnt_1ms = 0;
            i_pattern = 22;
			l_startPoint = l_EncoderTotal;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
        }
        break;

    case 22:	
        /* �N���X���C����̃g���[�X�A���p���o���� */
		

		servoPwmOut( i_ServoPwm * 15 / 10   );
		
		if( (l_EncoderTotal-l_startPoint ) >= 250 ) {
			//if((( c_c_cut == 0 || i_date_f_mode == 0) && check_halfline() == 2) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31  && (check_wideline() == 1 || ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch))))){//�E�N�����N
            
			if((i_date_f_mode == 0 && check_halfline_forC() == 2) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31  && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch)))
						|| ((c_c_cut == 0 && i_date_f_mode != 0) && (i_date_f_buff_ch_int[i_date_f_num_ch-1] == 31) && (check_wideline() == 1))){//�E�N�����N
            	ul_cnt_1ms = 0;
				l_startPoint = l_EncoderTotal;
				
				if(i_date_f_mode == 0){//�����v��
					i_date_buff_ch_int[i_date_num_ch++] = 31;
					i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
				}
				
				//if((i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch )|| (i_date_f_mode == 0 && (i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT))){//�������Z���ƌ������ł��Ă��Ȃ��\�������� || �ݒ葬�x�܂Ō����ł��Ă��Ȃ��Ƃ�
				if( (i_date_f_mode != 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch - max(0,i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c) )  ||  (i_date_f_mode == 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch ) 
					 || ( i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT) ){//�������Z���ƌ������ł��Ă��Ȃ��\�������� || �ݒ葬�x�܂Ō����ł��Ă��Ȃ��Ƃ�
					
					//if((i_date_f_mode == 0 )&& (i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//�����̓V���[�g�ł��i�����x���x���̂Ń����O�Ƃ���
					if(i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG ){//�����̓V���[�g�ł��i�����x���x���̂Ń����O�Ƃ���
						c_c_short_mode = 0;
					}else{
						c_c_short_mode = 1;	
					}
				}else{
					c_c_short_mode = 0;
				}
				
          		i_pattern = 31;//�E�N�����N
				i_date_f_num_ch++;
			
				i_Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(i_date_f_mode == 0 || c_c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_c_cut == 1 && i_date_f_mode != 0){//�V���[�g�J�b�g����
					if(i_Encoder10 >= i_C_TOPSPEED3){//�w�葬�x�𒴂��Ă����ꍇ��
						l_startPoint -= (i_Encoder10 -  i_C_TOPSPEED3) * 5;//������␳���A��葬���Ȃ���悤�ɂ���
					}
				}
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				break;
				
        	}
        	
			//if(((c_c_cut == 0 || i_date_f_mode == 0) && check_halfline() == 1) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41  && (check_wideline() == 1 || ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch))))){//���N�����N
            
			if((i_date_f_mode == 0 && check_halfline_forC() == 1) || ((c_c_cut == 1 && i_date_f_mode != 0) && i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41  && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_c )< (l_EncoderTotal - l_EncoderTotal_ch)))
						|| ((c_c_cut == 0 && i_date_f_mode != 0) && (i_date_f_buff_ch_int[i_date_f_num_ch-1] == 41) && (check_wideline() == 1))){//���N�����N
            	ul_cnt_1ms = 0;
				l_startPoint = l_EncoderTotal;
				
				if(i_date_f_mode == 0){//�����v��
					i_date_buff_ch_int[i_date_num_ch++] = 41;
					i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
				}
				
				//if((i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch) || (i_date_f_mode == 0 && (i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT))){//�������Z���ƌ������ł��Ă��Ȃ��\�������� || �ݒ葬�x�܂Ō����ł��Ă��Ȃ��Ƃ�){//�������Z���ƌ������ł��Ă��Ȃ��\��������
				if( (i_date_f_mode != 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch - max(0,i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c) )  ||  (i_date_f_mode == 0 && i_C_short_len > l_EncoderTotal - l_EncoderTotal_ch ) 
					 || ( i_Encoder10 > i_C_TOPSPEED + C_TOPSPEED_SHORT) ){//�������Z���ƌ������ł��Ă��Ȃ��\�������� || �ݒ葬�x�܂Ō����ł��Ă��Ȃ��Ƃ�
				
					//if((i_date_f_mode == 0) && (i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG) ){//�����̓V���[�g�ł��i�����x���x���̂Ń����O�Ƃ���
					if(i_Encoder10 < i_C_TOPSPEED - C_TOPSPEED_SHORT_NG ){//�����̓V���[�g�ł��i�����x���x���̂Ń����O�Ƃ���
						c_c_short_mode = 0;
					}else{
						c_c_short_mode = 1;	
					}
				}else{
					c_c_short_mode = 0;
				}
				
          		i_pattern = 41;//���N�����N
				i_date_f_num_ch++;
			
				i_Center_offset = 0;
				
				motor_mode_f( BRAKE, BRAKE );
				motor_mode_r( BRAKE, BRAKE );
			
				servoPwmOut( 0);
				motor_f( -90, -90 );
            	motor_r( -80, -80 );
				
				//if(i_date_f_mode == 0 || c_c_cut == 0)wait(4);
				wait(0);//5
				
				if(c_c_cut == 1 && i_date_f_mode != 0){//�V���[�g�J�b�g����
					if(i_Encoder10 >= i_C_TOPSPEED3){//�w�葬�x�𒴂��Ă����ꍇ��
						l_startPoint -= (i_Encoder10 -  i_C_TOPSPEED3) * 5;//������␳���A��葬���Ȃ���悤�ɂ���
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
		//if((c_c_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4){
		if((i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED4){ //�Đ����s���@�u���[�L�\�\�����ɓ��B���Ă��Ȃ��Ƃ���������������
			
			x=(i_C_TOPSPEED4-i_Encoder10)*2;
			r=(i_C_TOPSPEED4-i_Encoder10)*2;
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
			motor_f( x, x );
            motor_r( r, r );
		
      	
 		}else if( ((i_date_f_mode == 0) && i_Encoder10 >= i_C_TOPSPEED) //�ʏ탂�[�h�@�w�葬�x�𒴂����Ƃ�
				|| ((c_c_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED3) //�Đ����s���[�h �u���[�L�\�����ő��x�𒴂����Ƃ�
				|| ((c_c_cut == 0 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_c )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_C_TOPSPEED)   ) {  //�Đ����s���[�h�@�N�����N�͍Đ����Ȃ����[�h�@�u���[�L�\�����ő��x�𒴂����Ƃ�        // �G���R�[�_�ɂ��X�s�[�h���� 
              
		
			//�ʏ�
			if(c_c_cut == 0 || i_date_f_mode == 0)x=(i_C_TOPSPEED -i_Encoder10)*20;
			else x=(i_C_TOPSPEED3 -i_Encoder10)*30;
			
			if(c_c_cut == 0 || i_date_f_mode == 0)r=(i_C_TOPSPEED -i_Encoder10)*20;
			else r=(i_C_TOPSPEED3 -i_Encoder10)*30;
		
		/*		
			if(i < -10 || 10 < i){//abs
		
				if(r < -30)r = -30;
			}
		*/	 
			motor_mode_f( BRAKE, BRAKE );
			motor_mode_r( BRAKE, BRAKE );
				
			motor_f( x, x );
            motor_r( r, r );
			
	
        }else{
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
				
            motor_f( 100, 100 );
            motor_r( 100, 100 );
        }
		
		
		if( l_EncoderTotal-l_startPoint >= 2000 ) {//�듮��`�F�b�N
			
			motor_mode_f( FREE, FREE );
			motor_mode_r( FREE, FREE );
			
			i_Center_offset = 0;
			i_date_f_num_ch--;
            i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
        }

        break;

    case 31:
        /* �E�N�����N���� */
		//setBeepPatternS( 0x8000 );
		
		old_i = i;//�O��̊p�x���L��
		
        i = getServoAngle();//�n���h���p�x�擾
		
		i = (i +old_i) >> 1;     
			
        if(i_date_f_mode == 0 || c_c_cut == 0){//�ʏ�
			c_mode = 3;//������
		
			if(c_c_short_mode == 1){//short
				if((l_EncoderTotal-l_startPoint ) >= 130){
					if(i < 95)i_SetAngle = 125;
					else i_SetAngle = 115;
				
				}else i_SetAngle = 110;
			
				motor_f( 15, 0 );  //10,0        /* ���̕����́u�p�x�v�Z(4WD��).xls�v 85 -40*/
        		motor_r( -40, -40 ); //-15.-15         /* �Ōv�Z                        */
			
			}else{//long
				if((l_EncoderTotal-l_startPoint ) >= 200){
					if(i < 95)i_SetAngle = 130;
					else i_SetAngle = 100;
				
				}else i_SetAngle = 95;
			
				motor_f( 85,  -5 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v 85 -40*/
        		motor_r( -30,  -30 );          /* �Ōv�Z                        */
			}
			
		
		}else{
			c_mode = 1;//����͈͂�����
				
				
			if(0 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 150  || (i_Wide != 0 && -6 < i_Center  && (l_EncoderTotal-l_startPoint ) >= 200)){
			
				i_SetAngle = -30;
				motor2_f( -20,   50 );  //0 80    
        		motor2_r( 0,   0 );   //0 0
				
			//	l_startPoint += 2; //��O�ŋȂ��肷���Ă��邽�ߋ����������̂΂�
				
		
			}else if((l_EncoderTotal-l_startPoint ) >= 350){
				
				i_SetAngle = 100;
				motor2_f( 40,   5 );  //85 5       
        		motor2_r( 8,   -5 );  //25 0
				
				
			}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
				if(i < 70)i_SetAngle = 120;
				else i_SetAngle = 98;
				
				motor2_f( 30,   0 );  //80 0       
        		motor2_r( -10,   -10 );   //0 0
			
				
			}else if((l_EncoderTotal-l_startPoint ) >= 150){
				
				if(i < 45)i_SetAngle = 100;
				else i_SetAngle = 80;
				
				motor2_f( 40,   0 );  //85 5      
        		motor2_r( 0,   -10 );  //15 0
				
			}else if((l_EncoderTotal-l_startPoint ) >= 100){
				
				if(i < 30)i_SetAngle = 80;
				else i_SetAngle = 50;
				
				motor2_f( 45,   2 ); //85 30        
        		motor2_r( 10,   -10 );  //25 0
			
			}else if((l_EncoderTotal-l_startPoint ) >= 50){
				
				if(i < 20)i_SetAngle = 50;
				else i_SetAngle = 35;
				
				motor2_f( 50,   5 ); //90 35        
        		motor2_r( 15,  -5 );   //35 0
					
			}else{
				i_SetAngle = 25;
				motor2_f( 50,   10 );  //90 40       
        		motor2_r( 20,   -5 );   //40  0	
			} 
			
       	}
  
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */
        
		//180 -15 < 25
        if((( c_c_cut == 0 || i_date_f_mode == 0) && (l_EncoderTotal-l_startPoint ) >= 150) || ((c_c_cut == 1 && i_date_f_mode != 0) && (l_EncoderTotal-l_startPoint ) >= 240 )  ){
			if(i_Wide != 0){
			//if (((20 < i_Center)&&(i_Center < 40)) || ((-15 < i_Center)&&(i_Center < 15))) {    /* �Ȃ��I���`�F�b�N           */
			if ( (( c_c_cut == 0 || i_date_f_mode == 0) && 15 < i_Center && i_Center < 35 && (i_Wide != 0 && i_Wide < 12) ) 
				|| ((c_c_cut == 1 && i_date_f_mode != 0) && -20 < i_Center && i_Center < 0 && (i_Wide_old == 0 || i_Wide_old == 127 || i_Wide > i_Wide_old))
			//	|| ((c_c_cut == 1 && i_date_f_mode != 0) && -20 < i_Center && i_Center < 0 && (i_Wide != 0 && i_Wide < 30))
				  || ((c_c_cut == 1 && i_date_f_mode != 0) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 750)) {    /* �Ȃ��I���`�F�b�N           */
				
            	ul_cnt_1ms = 0;
            	i_SensorPattern = 0;
            	
				l_startPoint = l_EncoderTotal;
            	i_pattern = 32;
				
			//	c_mode = 0;//�ʏ�
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				
				if(i_date_f_mode != 0 && c_c_cut == 1){	
					ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
					c_mode = 0;//����͈͂����ɖ߂�
            		i_pattern = 11;//�ʏ�g���[�X��
					ul_cnt_curve_time_1ms = 0;
				}	
        	}
			}
		}
        break;

    case 32:
        /* ���肷��܂� (�V���[�J�b�g��32�ɂ͗��Ȃ���)*/
		
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			c_mode = 0;//����͈͂����ɖ߂�
		}
		
		if(c_c_short_mode == 1){//short
        	if((l_EncoderTotal-l_startPoint ) >= 110)i_SetAngle = 90;
			else i_SetAngle = 95;
		}else{//long
			if((l_EncoderTotal-l_startPoint ) >= 100)i_SetAngle = 90;
			else i_SetAngle = 95;
		}
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if( i_Encoder10 >= i_C_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           
			x=(i_C_TOPSPEED2-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_c_short_mode == 1){//short
				motor2_f( 50, 40 );
           		motor2_r( 0, 0 );
		
		
			}else{//long
				motor2_f( 80, 40 );
           		motor2_r( 40, 10 );
			}
		}
         
		
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			if(i_Wide != 0){                       
        		if(((-15 < i_Center)&&(i_Center < 15) && getServoAngle() < 120) 
					|| ((-5 < i_Center)&&(i_Center < 5) && getServoAngle() < 128)) {    /*  �����ɂȂ�܂�          */
            		ul_cnt_1ms = 0;
            		i_SensorPattern = 0;
            	
           			i_pattern = 33;
					c_mode = 0;//�ʏ�
					l_startPoint = l_EncoderTotal;
        		}
			}
		}
	
        break;
	
	case 33://�����҂�	
		//c_mode = 1;//����͈͂���������
		c_mode = 0;//����͈͂����ɖ߂�
		
		servoPwmOut( i_ServoPwm );

		if( i_Encoder10 >= i_TOPSPEED ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
          
			x=(i_TOPSPEED-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if( ((l_EncoderTotal-l_startPoint ) >= 100 && -30 < getServoAngle() && getServoAngle() < 30 ) || ((l_EncoderTotal-l_startPoint ) >= 300)  ) {
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
			c_mode = 0;//����͈͂����ɖ߂�
            i_pattern = 11;//�ʏ�g���[�X��
			ul_cnt_curve_time_1ms = 0;
        }
        break;
		

    case 41:
        /* ���N�����N���� */
        //setBeepPatternS( 0x8000 );
			
        if(i_date_f_mode == 0 || c_c_cut == 0){//�ʏ�
			c_mode = 2;//�E����
			
			if(c_c_short_mode == 1){//short
				if((l_EncoderTotal-l_startPoint ) >= 130){
				
					if(i > -95)i_SetAngle = -125;
					else i_SetAngle = -115;
				
				}else i_SetAngle = -110;
			
				motor_f( 0, 15);    //0,10      /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        		motor_r( -40, -40 );   //-15.-15       /* �Ōv�Z                        */
				
			}else{//long
				
				if((l_EncoderTotal-l_startPoint ) >= 200){
				
					if(i > -95)i_SetAngle = -130;
					else i_SetAngle = -100;
				
				}else i_SetAngle = -95;
			
				motor_f(  -5, 85 );          /* ���̕����́u�p�x�v�Z(4WD��).xls�v*/
        		motor_r(  -30, -30 );          /* �Ōv�Z                        */
			}
		}else{
			c_mode = 1;//����͈͂�����
			
			 
			if((i_Center < 0 && (l_EncoderTotal-l_startPoint ) >= 150) || (i_Wide != 0 && i_Center < 6 && (l_EncoderTotal-l_startPoint ) >= 200)){
				i_SetAngle = 30;

				motor2_f( 50,   -20 );  //80 0       
        		motor2_r( 0,   0 );  //0 0
		
				//l_startPoint += 2; //��O�ŋȂ��肷���Ă��邽�ߋ����������̂΂�
				
			}else if((l_EncoderTotal-l_startPoint ) >= 350){
				i_SetAngle = -100;

				motor2_f( 5,   40 ); //5 85        
        		motor2_r( -5,   8 );  //0 25
				
			}else if((l_EncoderTotal-l_startPoint ) >= 250){
				
				if(i > -70)i_SetAngle = -120;
				else i_SetAngle = -98;

				motor2_f( 0,   30 ); //0 80        
        		motor2_r( -10,   -10 );  //0 0
			
				
			}else if((l_EncoderTotal-l_startPoint ) >= 150){
				if(i > -45)i_SetAngle = -100;
				else i_SetAngle = -80;
				
				motor2_f( 0,   40 ); //5 85        
        		motor2_r( -10,   0 ); //0 15
				
			}else if((l_EncoderTotal-l_startPoint ) >= 100){
				if(i > -30)i_SetAngle = -80;
				else i_SetAngle = -50;
				
				motor2_f( 2,   45 ); //30 85        
        		motor2_r( -10,   10 ); //0 25
			
			}else if((l_EncoderTotal-l_startPoint ) >= 50){
				if(i > -20)i_SetAngle = -50;
				else i_SetAngle = -35;
				
				motor2_f( 5,   50 ); //35 90       
        		motor2_r( -5,  15 );   //0 35
					
			}else{
				i_SetAngle = -25;
				motor2_f( 10,   50 );  //40 90    
        		motor2_r( -5,   20 );   // 0 40
			}  
       	}
		
		servoPwmOut( i_ServoPwm2 );        /* �U�肪�ア�Ƃ��͑傫������       */
        
		if(((c_c_cut == 0 || i_date_f_mode == 0) && (l_EncoderTotal-l_startPoint ) >= 150) || ((c_c_cut == 1 && i_date_f_mode != 0) && (l_EncoderTotal-l_startPoint ) >= 240 ) ){
			if(i_Wide != 0){ 
			//if( ((-40 < i_Center)&&(i_Center < -20)) || ((-15 < i_Center)&&(i_Center < 15))) {    /* �Ȃ��I���`�F�b�N           */
	 		if(( (c_c_cut == 0 || i_date_f_mode == 0) && -35 < i_Center && i_Center < -15 && (i_Wide != 0 && i_Wide < 10)) 
				|| ( (c_c_cut == 1 && i_date_f_mode != 0) && 0 < i_Center && i_Center < 20 && (i_Wide_old == 0 || i_Wide_old == 127 ||  i_Wide < i_Wide_old)) 
				//|| ( (c_c_cut == 1 && i_date_f_mode != 0) && 0 < i_Center && i_Center < 20 && (i_Wide != 0 && i_Wide < 30)) 
					|| ((c_c_cut == 1 && i_date_f_mode != 0) && -25 < i_Center && i_Center < 25 && (i_Wide_old != 0) && (l_EncoderTotal-l_startPoint ) >= 750)){    /* �Ȃ��I���`�F�b�N           */
	 	
            	ul_cnt_1ms = 0;
            	i_SensorPattern = 0;
            
            	l_startPoint = l_EncoderTotal;
            	i_pattern = 42;
				
				//c_mode = 0;//�ʏ�
				
				motor_mode_f( FREE, FREE );
				motor_mode_r( FREE, FREE );
				
				if(i_date_f_mode != 0 && c_c_cut == 1){	
					ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
					c_mode = 0;//����͈͂����ɖ߂�
            		i_pattern = 11;//�ʏ�g���[�X��
					ul_cnt_curve_time_1ms = 0;
				}
			}
			}
		}
        break;

    case 42:
	
		if((l_EncoderTotal-l_startPoint ) >= 50){ 
			c_mode = 0;//����͈͂����ɖ߂�
		}
		
		/* ���肷��܂� (�V���[�J�b�g��42�ɂ͗��Ȃ���)*/
		if(c_c_short_mode == 1){//short
			if((l_EncoderTotal-l_startPoint ) >= 110)i_SetAngle = -90;
			else i_SetAngle = -95;
		}else{//long
			if((l_EncoderTotal-l_startPoint ) >= 100)i_SetAngle = -90;
			else i_SetAngle = -95;
		}
		
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if( i_Encoder10 >= i_C_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           
			x=(i_C_TOPSPEED2-i_Encoder10)*10;

			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			if(c_c_short_mode == 1){//short
				motor2_f( 40, 50);
           		motor2_r( 0, 0 );
			}else{//long
				motor2_f( 40, 80);
           		motor2_r( 10, 40 );
			}
		}
         	  
		
		if((l_EncoderTotal-l_startPoint ) >= 50){   
			if(i_Wide != 0){                      
				if(((-15 < i_Center)&&(i_Center < 15) && getServoAngle() > -120) 
					|| ((-5 < i_Center)&&(i_Center < 5) && getServoAngle() > -128)) {    /*  �����ɂȂ�܂�          */
            		
            		ul_cnt_1ms = 0;
      			    i_SensorPattern = 0;
            	
           			i_pattern = 43;
					c_mode = 0;//�ʏ�
					l_startPoint = l_EncoderTotal;
        		}
			}
		}
		
        break;
		
	case 43://�����҂�	
		//c_mode = 1;//����͈͂���������
		c_mode = 0;//����͈͂����ɖ߂�
		
		servoPwmOut( i_ServoPwm );

		if( i_Encoder10 >= i_TOPSPEED ) {          // �G���R�[�_�ɂ��X�s�[�h���� 
			
			x=(i_TOPSPEED-i_Encoder10)*10;
	
			motor_f( x, x );
           	motor_r( x, x );
       	}else{
			motor2_f( 100, 100 );
           	motor2_r( 70, 70 );
		}
			
		if(( (l_EncoderTotal-l_startPoint ) >= 100 && -30 < getServoAngle() && getServoAngle() < 30) || ((l_EncoderTotal-l_startPoint ) >= 300) ) {
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
			c_mode = 0;//����͈͂����ɖ߂�
            i_pattern = 11;//�ʏ�g���[�X��
			ul_cnt_curve_time_1ms = 0;
        }
        break;
		
	case 51://���n�[�t
		//setBeepPatternS( 0x8000 );
	    i_SetAngle = 0;
		servoPwmOut( i_ServoPwm2 );

		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
       
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(i_date_f_mode == 0 && (check_crossline() || check_halfline() == 2 )) {       // �N���X���C���`�F�b�N         
            ul_cnt_1ms = 0;
            i_pattern = 21;	
        }
  
        if( (l_EncoderTotal - l_startPoint ) >= 100 ) {
            ul_cnt_1ms = 0;
            i_pattern = 52;
			l_startPoint = l_EncoderTotal;
        }
        break;

	case 52://�n�[�t���C����
	//	if( i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 100 )c_mode = 3;//������
	
#ifdef HWall //�ǂ���	 
		if(i_Center > 11){//��肷��
			i_Center_offset = -6;//���Ɋ��
			servoPwmOut( i_ServoPwm );
		}else{
			i_Center_offset = -9;//���Ɋ��
			servoPwmOut( i_ServoPwm  );
		}
#else //�ǖ���
		if(i_Center > 12){//��肷��
			i_Center_offset = -7;//���Ɋ��
			servoPwmOut( i_ServoPwm );
		}else{
			i_Center_offset = -10;//���Ɋ��
			servoPwmOut( i_ServoPwm  );
		}
#endif		
		
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
			
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}

	
	//	if((l_EncoderTotal-l_startPoint ) >= 300 ){
	//		if((( c_h_cut == 0 || i_date_f_mode == 0)  && i_Wide == 0) || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //�E���`�F�b�N
        
		if(((c_h_cut == 0 || i_date_f_mode == 0)  && (i_Wide == 0 || check_crossline()) && (l_EncoderTotal-l_startPoint ) >= 250 )  || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //�E���`�F�b�N
            
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
				
			if(i_date_f_mode == 0){//�����v��
				i_date_buff_ch_int[i_date_num_ch++] = 53;
				i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
			}
				
           	 i_pattern = 53;
			i_date_f_num_ch++;
			c_mode = 0;//�ʏ�
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
            break;
		}
		
		if(i_date_f_mode == 0 && (l_EncoderTotal-l_startPoint ) < 30  && ( check_crossline() || check_halfline() == 2 )) {       // �N���X���C���`�F�b�N         
            ul_cnt_1ms = 0;
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
            i_pattern = 21;	
        }
  
		if( (l_EncoderTotal-l_startPoint ) >= 1500 ) {//�듮��`�F�b�N
			
			i_date_f_num_ch--;
			c_mode = 0;//�ʏ�
            i_pattern = 11;
			//ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
        }	

		break;

	case 53:
		c_mode = 2;//�E����
		
		if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //�ǂ���    	
			i_SetAngle = -65;
#else //�ǖ���
			i_SetAngle = -50;
#endif

		}else{
			i_SetAngle = -20;
		}
		
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
				
				motor_f( x, 10 );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //�ǂ���    	
           		motor_f( 0, 100 );
           		motor_r( 10, 20 );
#else //�ǖ���

           		motor_f( 10, 90 );
           		motor_r( 70, 70 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
         
		if( ((i_date_f_mode == 0 || c_h_cut == 0) && (l_EncoderTotal - l_startPoint ) >= 200 ) || (i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 350 )) {  
	        if((-50 < i_Center)&&(i_Center < -10) && i_Wide != 0 ) {    /* �Ȃ��I���`�F�b�N           */
			//	if(i_Center < -30) {    /* �Ȃ��I���`�F�b�N           */
		
	            	ul_cnt_1ms = 0;
					l_startPoint = l_EncoderTotal;
	            	i_pattern = 54;
				
					c_mode = 0;//�ʏ�
					motor_mode_r( BRAKE, BRAKE );
	        	}
			}
        break;

	case 54:
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
			
			if((i_Center == 0)&&(i_Wide == 0)){ //�C���ɗ�������
				i_SetAngle = -50;
			}else{
#ifdef HWall  //�ǂ���    	
				i_SetAngle = -30;
#else //�ǖ���
				i_SetAngle = 0; // -20
#endif
			}
		}else{
			i_SetAngle = -5;//5 7
		}
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
	
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //�ǂ���    	
           		motor_f( 100, 100 );
           		motor_r( 60,  60 );
#else //�ǖ���
           		motor_f( 90, 80 );
           		motor_r( 70,  40 );
#endif


			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
	
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 90,  50 );
			}
		}
        
		
        //if( (15 < i_Center)&&(i_Center < 50) ) {    /* �Ȃ��I���`�F�b�N           */
		//if(-10 < i_Center && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-7
		
#ifdef HWall  //�ǂ���    	
		if(10 < i_Center && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-7
#else //�ǖ���
		if(0 < i_Center && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-7
#endif
	//	if(20 < i_Center) {    /* �Ȃ��I���`�F�b�N           */
		
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
            i_pattern = 55;
			
			motor_mode_r( FREE, FREE );
        }
		
        break;

	case 55://���肷��܂�
	
		if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //�ǂ���    	
			i_SetAngle = 20;
#else //�ǖ���
			i_SetAngle = 15;
#endif
			servoPwmOut( i_ServoPwm2 );          
		
			//servoPwmOut( i_ServoPwm  );
		}else{
			i_SetAngle = 20;//40 37
			servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		}
		
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //�ǂ���    	
           		motor_f( 100, 30 );
           		motor_r( 90, 0 );
#else //�ǖ���
				if(i_Center > 10){//�O�ɗ�������
           			motor_f( 90, 0 );
				}else{
					motor_f( 90, 30 );
				}
				motor_r( 80, 0 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;

				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 50 );
           		motor_r( 90, 0 );
			}
		}
          
		if( (l_EncoderTotal - l_startPoint ) >= 100 ) {  

#ifdef HWall  //�ǂ���  
	        if((-35 < i_Center)&&(i_Center < 35)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
#else //�ǖ���
			//if((-23 < i_Center)&&(i_Center < 23)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
			if((-18 < i_Center)&&(i_Center < 18)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
#endif	
            ul_cnt_1ms = 0;
            i_SensorPattern = 0;
          
			i_pattern = 11;
			ul_cnt_curve_time_1ms = 0;
			
			l_startPoint = l_EncoderTotal;
        }
		}
        break;
		
	
			
	case 61://�E�n�[�t
		
		//setBeepPatternS( 0x8000 );
		i_SetAngle = 0;
		servoPwmOut( i_ServoPwm2 );

		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
		
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
			
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(i_date_f_mode == 0 && (check_crossline() || check_halfline() == 1)) {       // �N���X���C���`�F�b�N         
            ul_cnt_1ms = 0;
           	
            i_pattern = 21;			
        }

        if( (l_EncoderTotal-l_startPoint ) >= 100 ) {
            ul_cnt_1ms = 0;
            i_pattern = 62;
			l_startPoint =l_EncoderTotal;
        }
        break;

	case 62://�n�[�t���C����
		
	//	if( i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 100 )c_mode = 2;//�E����

#ifdef HWall //�ǂ���
		if(i_Center < -11){//
			i_Center_offset = 6;//�E�Ɋ��
			servoPwmOut( i_ServoPwm );
				
		}else{
			i_Center_offset = 9;//�E�Ɋ��
			servoPwmOut( i_ServoPwm );
		}
#else //�ǖ���		
		if(i_Center < -12){//
			i_Center_offset = 7;//�E�Ɋ��
			servoPwmOut( i_ServoPwm );
				
		}else{
			i_Center_offset = 10;//�E�Ɋ��
			servoPwmOut( i_ServoPwm );
		}
#endif		
		
		if((i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )> (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_TOPSPEED)){
			
			x=(i_TOPSPEED-i_Encoder10)*1;
			r=(i_TOPSPEED-i_Encoder10)*1;
	
			motor_f( x, x );
            motor_r( r, r );
			
		}else if( (i_date_f_mode == 0 && i_Encoder10 >= i_H_TOPSPEED) || (i_date_f_mode != 0 && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_brake_h )< (l_EncoderTotal - l_EncoderTotal_ch))&& i_Encoder10 >= i_H_TOPSPEED)) {          // �G���R�[�_�ɂ��X�s�[�h���� 
    
			x=(i_H_TOPSPEED-i_Encoder10)*15;
			r=(i_H_TOPSPEED-i_Encoder10)*5;
		
			motor_f( x, x );
           	motor_r( r, r );
       	}else{
           	motor_f( 100, 100 );
           	motor_r( 100, 100 );
		}
        
		if(((c_h_cut == 0 || i_date_f_mode == 0)  && (i_Wide == 0 || check_crossline()) && (l_EncoderTotal-l_startPoint ) >= 250 )  || (( c_h_cut == 1 && i_date_f_mode != 0) && ((i_date_f_buff_ch_int[i_date_f_num_ch] - i_date_f_shortcat_h)< (l_EncoderTotal - l_EncoderTotal_ch)) )){ //�E���`�F�b�N
            
            ul_cnt_1ms = 0;
			l_startPoint = l_EncoderTotal;
				
			if(i_date_f_mode == 0){//�����v��
				i_date_buff_ch_int[i_date_num_ch++] = 63;
				i_date_buff_ch_int[i_date_num_ch++] = l_EncoderTotal - l_EncoderTotal_ch;	
			}
				
           	i_pattern = 63;
			i_date_f_num_ch++;
			c_mode = 0;//�ʏ�
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
				
            break;
        }
			

		if(i_date_f_mode == 0 && (l_EncoderTotal-l_startPoint ) < 30  && ( check_crossline() || check_halfline() == 1 )) {       // �N���X���C���`�F�b�N         
            ul_cnt_1ms = 0;
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
            i_pattern = 21;	
        }
  
		if( (l_EncoderTotal-l_startPoint ) >= 1500 ) {//�듮��`�F�b�N
          	
			i_date_f_num_ch--;
			c_mode = 0;//�ʏ�
            i_pattern = 11;
			//ul_cnt_curve_time_1ms = 0;
			ul_cnt_1ms = 0;
			i_Center_offset = 0;//�I�t�Z�b�g��߂�
        }

		break;

	case 63:
		//setBeepPatternS( 0x8000 );

		c_mode = 3;//������
		
		if(i_date_f_mode == 0 || c_h_cut == 0){
 
#ifdef HWall  //�ǂ���    	
			i_SetAngle = 68;
#else //�ǖ���
			i_SetAngle = 50;
#endif			
		}else{
			i_SetAngle = 20;//48 47
		}
		
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
          
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
			
				motor_f( 10, x );
    	       	motor_r( x, x );
       		}else{
#ifdef HWall  //�ǂ���    	
				motor_f( 100, 0 );
           		motor_r( 20, 10 );
#else //�ǖ���
				motor_f( 90, 10 );
           		motor_r( 70, 70 );
#endif	

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
    	       	motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 100, 100 );
			}
		}
                
		if( ((i_date_f_mode == 0 || c_h_cut == 0) && (l_EncoderTotal - l_startPoint ) >= 200 ) || (i_date_f_mode != 0 && (l_EncoderTotal - l_startPoint ) >= 350 )) {             
	        if((10 < i_Center)&&(i_Center < 50) && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           */
			//if( 30 < i_Center) {    /* �Ȃ��I���`�F�b�N           */
		
	            ul_cnt_1ms = 0;
	            l_startPoint = l_EncoderTotal;
	            i_pattern = 64;
			
				c_mode = 0;//�ʏ�
			
				motor_mode_r( BRAKE, BRAKE );
	        }
		}
        break;

	case 64:

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if((i_Center == 0)&&(i_Wide == 0)){ //�C���ɗ�������
				i_SetAngle = 50;
			}else{
#ifdef HWall  //�ǂ���    	
				i_SetAngle = 30;//-3
#else //�ǖ���
				i_SetAngle = -10;//-3 20
#endif
			}
		}else{
			i_SetAngle = 5;//-3
		}
		
		servoPwmOut( i_ServoPwm2 );          /* �U�肪�ア�Ƃ��͑傫������       */

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
          
				x=(i_H_TOPSPEED2-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //�ǂ���    	
				motor_f( 100, 100 );
           		motor_r( 60, 60 );
#else //�ǖ���
				motor_f( 80, 90 );
           		motor_r( 40, 70 );
#endif
           		
			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
		
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 100, 100 );
           		motor_r( 50, 90 );
			}
		}
             
		//if( (l_EncoderTotal - l_startPoint ) >= 30 ) {                  
        //if((-50 < i_Center)&&(i_Center < -15)) {    /* �Ȃ��I���`�F�b�N           */

#ifdef HWall  //�ǂ���    	
		if(i_Center < -10 && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-5
#else //�ǖ���
		if(i_Center < 0 && i_Wide != 0) {    /* �Ȃ��I���`�F�b�N           *///-5
#endif				
		//if(i_Center < -20) {    /* �Ȃ��I���`�F�b�N           */
	
            ul_cnt_1ms = 0;
            l_startPoint = l_EncoderTotal;
            i_pattern = 65;
			
			motor_mode_r( FREE, FREE );
        }
		//}
        break;

	case 65://���肷��܂�
	
		if(i_date_f_mode == 0 || c_h_cut == 0){

#ifdef HWall  //�ǂ���    	
			i_SetAngle = -20;
#else //�ǖ���
			i_SetAngle = -15;
			
#endif

			servoPwmOut( i_ServoPwm2 );
			
		//	servoPwmOut( i_ServoPwm  );
		}else{
			i_SetAngle = -20;//-40 -35
			
			servoPwmOut( i_ServoPwm2 );
		}
		       

		if(i_date_f_mode == 0 || c_h_cut == 0){
			if( i_Encoder10 >= i_H_TOPSPEED2 ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           	
				x=(i_H_TOPSPEED2-i_Encoder10)*15;

				motor_f( x, x );
           		motor_r( x, x );
       		}else{

#ifdef HWall  //�ǂ���    	
				motor_f( 30, 100 );
           		motor_r( 0, 90 );
#else //�ǖ���
				if(i_Center < -10){//�O�ɗ�������
					motor_f( 0, 90 );
				}else{
					motor_f( 30, 90 );					
				}
           		motor_r( 0, 80 );
#endif

			}
		}else{
			if( i_Encoder10 >= i_H_TOPSPEED2_S ) {          /* �G���R�[�_�ɂ��X�s�[�h���� */
           
				x=(i_H_TOPSPEED2_S-i_Encoder10)*15;
				
				motor_f( x, x );
           		motor_r( x, x );
       		}else{
           		motor_f( 50, 100 );
           		motor_r( 0, 90 );
			}
		}
                      
	    if( (l_EncoderTotal - l_startPoint ) >= 100 ) {  
#ifdef HWall  //�ǂ���  
	        if((-35 < i_Center)&&(i_Center < 35)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
#else //�ǖ���
			//if((-23 < i_Center)&&(i_Center < 23)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
			if((-18 < i_Center)&&(i_Center < 18)&&(i_Wide != 0)) {    /*  �����ɂȂ�܂�          */
#endif		
	            ul_cnt_1ms = 0;
	            i_SensorPattern = 0;
        
				i_pattern = 11;
				ul_cnt_curve_time_1ms = 0;
		
				l_startPoint = l_EncoderTotal;
	        }
		}
        break;
	case 101:
        /* ��~ */
        servoPwmOut( 0 );
        motor_f( 0, 0 );
        motor_r( 0, 0 );
        setBeepPatternS( 0xc000 );
		
	
		i_msdFlag = 0;
        if( i_msdError != 0 ) {
            /* microSD�ɕs����������Ȃ�I�� */
            printf( "microSD Initialize Error!!\n" );
            i_pattern = 109;
        } else {
            i_pattern = 102;
            ul_cnt_1ms = 0;
        }
        break;

    case 102:
		/* �Ō�̃f�[�^�������ނ܂ő҂�*/
        if( checkMicroSDProcess() == 0 ) {
            i_pattern = 103;               /* �f�[�^�]��������             */
            break;
        }
        if( checkMicroSDProcess() == 11 ) {
            microSDProcessEnd();        /* microSDProcess�I������       */
            while( checkMicroSDProcess() );
            i_pattern = 103;               /* �f�[�^�]��������             */
        }
		
        break;

   case 103:
		 
        /* 0.5s�҂� && �v�b�V���X�C�b�`�������ꂽ���`�F�b�N*/
        if( ul_cnt_1ms >= 500 && !pushsw_get()) {
            i_pattern = 104;
            ul_cnt_1ms = 0;
        }
        break;

    case 104:
        /* �v�b�V���X�C�b�`�������ꂽ���`�F�b�N */
        led_out( ul_cnt_1ms / 200 % 2 ? 0x6 : 0x9  );
        if( pushsw_get() ) {
            i_pattern = 105;
            ul_cnt_1ms = 0;
        }
        break;

    case 105:
        /* �^�C�g���]���A�]������ */
        printf( "\n" );
        printf( "CarName Data Out\n" ); /* �����̃J�[�l�[�������Ă������� */
        printf( "i_pattern, i_Center, i_Wide ,�p�x, �T�[�{PWM, " );
        printf( "���OPWM, �E�OPWM, ����PWM, �E��PWM, �G���R�[�_5*2,���[�h,�⓹��,�W���C��Y/10,�W���C��X/10,�ԊO��\n" );
		
		ul_msdWorkAddress = ul_msdStartAddress;   /* �ǂݍ��݊J�n�A�h���X     */
        i_pattern = 106;
        break;
	case 106:
        /* microSD���f�[�^�ǂݍ��� */
        if( ul_msdWorkAddress >= ul_msdEndAddress ) {
            /* �������ݏI���A�h���X�ɂȂ�����A�I��� */
            i_pattern = 109;
            break;
        }
        ret = readMicroSD( ul_msdWorkAddress , c_msdBuff );
        if( ret != 0x00 ) {
            /* �ǂݍ��݃G���[ */
            printf( "\nmicroSD Read Error!!\n" );
            i_pattern = 109;
            break;
        } else {
            /* �G���[�Ȃ� */
            ul_msdWorkAddress += 512;
            i_msdBuffAddress = 0;
            i_pattern = 107;
			i = 0;
        }
        break;
	case 107:
        /* �f�[�^�]�� */
        led_out( 1 << (ul_cnt_1ms/100) % 8 );

		if(c_msdBuff[i_msdBuffAddress+0] <= 0 ){ /* �p�^�[����0�ȉ��Ȃ�I�� */
			date_f_make(-1,0,0,0);
            printf( "End.\n" );
            i_pattern = 108;
			ul_cnt_1ms  = 0 ;
			setBeepPatternS( 0xff00 );
            break;
        }

	//	if( (dipsw_get() & 0x04) == 0x00 ) {//���O�o�͂Ȃ��A�����R�[�X�L�����������p
			/* �f�[�^�̓]�� */
	        printf( "%d,%4d,%4d,%5d,%5d,%5d,%5d,%5d,%5d,%4d,%4d,%4d,%4d,%4d,%4d\n",
	            (int)c_msdBuff[i_msdBuffAddress+0],                  /* �p�^�[��     */
	            (int)c_msdBuff[i_msdBuffAddress+1],					/* �Z���^�[*/
	            (unsigned char)c_msdBuff[i_msdBuffAddress+2],                  /* ���C�h */
				(int)((unsigned char)c_msdBuff[i_msdBuffAddress+3]*0x100 +
		             (unsigned char)c_msdBuff[i_msdBuffAddress+4] ),			/* �p�x */
	            /* �T�[�{PWM */
		            c_msdBuff[i_msdBuffAddress+6],
		            /* ���OPWM */
		            c_msdBuff[i_msdBuffAddress+7],
		            /* �E�OPWM */
		            c_msdBuff[i_msdBuffAddress+8],
		            /* ����PWM */
		            c_msdBuff[i_msdBuffAddress+9],
		            /* �E��PWM */
		            c_msdBuff[i_msdBuffAddress+10],
		            /* �G���R�[�_ */
		            c_msdBuff[i_msdBuffAddress+11] * 2,
					/* ���[�h */
		            c_msdBuff[i_msdBuffAddress+12],
					/* �⓹�� */
		            c_msdBuff[i_msdBuffAddress+13],
					/* �����x�Z���T�[Y */
		            c_msdBuff[i_msdBuffAddress+14],
					/* �����x�Z���T�[X */
		            c_msdBuff[i_msdBuffAddress+15],
					/* �ԊO���Z���T�[�̍� */
		            c_msdBuff[i_msdBuffAddress+5]
        );		
	//	}
        
		if(i_date_f_mode != 0){
			date_f_make((int)c_msdBuff[i_msdBuffAddress+0],(int)((unsigned char)c_msdBuff[i_msdBuffAddress+3]*0x100 +
	             (unsigned char)c_msdBuff[i_msdBuffAddress+4] ) ,c_msdBuff[i_msdBuffAddress+11] ,(int)c_msdBuff[i_msdBuffAddress+12]);
		}
		
		 i_msdBuffAddress += 64;  /* ���̑��M����                 */

        if( i_msdBuffAddress >= 512 ) {
            i_pattern = 106;
        }
		
        break;

    case 108:
        /* �]���I�� */
		if(i_date_f_mode == 1){
			/* �u���b�NA �C���[�Y���܂� */
			blockEraseDataFlash( 0x3000 );
			/* �u���b�NB �C���[�Y���܂� */
			blockEraseDataFlash( 0x3400 );
			
			/* �u���b�NA �������� */
			writeDataFlash( 0x3000, c_date_f_buff, 32 );
			
			
			if( readMicroSD( ul_msdStartAddress_ch , c_msdBuff_ch ) != 0x00 ) {
				// �ǂݍ��݃G���[ 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)c_date_f_buff_ch[i] = c_msdBuff_ch[i];
			}
			
			/* �u���b�NB �������� */
			writeDataFlash( 0x3400, c_date_f_buff_ch, 32 );
			printf("i_date_f_mode == 1\n");
			
		}else if(i_date_f_mode == 2){
			/* �u���b�NC �C���[�Y���܂� */
			blockEraseDataFlash( 0x3800 );
			/* �u���b�ND �C���[�Y���܂� */
			blockEraseDataFlash( 0x3c00 );
			
			/* �u���b�NC �������� */
			writeDataFlash( 0x3800, c_date_f_buff, 32 );
			
			if( readMicroSD( ul_msdStartAddress_ch , c_msdBuff_ch ) != 0x00 ) {
				// �ǂݍ��݃G���[ 
				printf( "microSD Read Error!!\n" );
		
			}else{
				for(i = 0; i < 32; i++)c_date_f_buff_ch[i] = c_msdBuff_ch[i];
			}
		
			/* �u���b�ND �������� */
			writeDataFlash( 0x3c00, c_date_f_buff_ch, 32 );
		
			printf("i_date_f_mode == 2\n");
		}
		
		j=0;
		for(i = 0; i < 16; i++){
			//printf("%d%d",c_date_f_buff[j],c_date_f_buff[j+1]);
			printf("%d\n",i_date_f_buff_int[i]);
			
			j+=2;
		}
		
		printf("\n");
		for(i = 0; i < 32; i+=3){
			printf("%d %d%02d\n",c_date_f_buff_ch[i],c_date_f_buff_ch[i+1],c_date_f_buff_ch[i+2]);
		}
		
		
		i_pattern = 109;
        led_out( 0xff );
        break;
		
	case 109:
		led_out( 0xff );
		break;
	case 200://���s�I��
		setBeepPatternS( 0xff00 );
		
		while(1){
			cam_in();
			if(c_logfin == 0)led_out(camera(i_Center,i_Wide));
			else led_out(0);
			
			if(i_Encoder10 < 5 ){
				if(i_msdFlag == 1)i_msdFlag = 2;                /* �f�[�^�L�^�I��               */
				servoPwmOut( 0 );
			}else{
				if(c_out_flag == 1){//�R�[�X�A�E�g��
					servoPwmOut( 0 );//�Փˎ��̔j����h�����߃T�[�{�ɗ͂����Ȃ�
				}else{
					servoPwmOut( i_ServoPwm );					
				}
			}
		
		/*	if( pushsw_get() ) {
				while(pushsw_get());
		 		for(i = 0; i < 32; i+=2){
					printf("%d %d\n",i_date_buff_ch_int[i],i_date_buff_ch_int[i+1]);
				}
			}*/
			motor_f( 0, 0 );
            motor_r( 0, 0 );
			if(c_logfin == 0){
				if(i_msdFlag == 0){
					//setBeepPatternS( 0xff00 );
					
					servoPwmOut( 0 );
					
					j = 0;
					for(i = 0; i < 32; i+=2){
						//printf("%d %d\n",i_date_buff_ch_int[i],i_date_buff_ch_int[i+1]);
						c_msdBuff_ch[j++] = (signed char)i_date_buff_ch_int[i];
						c_msdBuff_ch[j++] = (signed char)(i_date_buff_ch_int[i+1]/100);
						c_msdBuff_ch[j++] = (signed char)(i_date_buff_ch_int[i+1]%100);
					}
					
					while( checkMicroSDProcess() != 11 )wait(2); /* �������݂��I���܂ő҂� */
					// �������ݏ������I���܂ŌJ��Ԃ�
					while( microSDProcessEnd() != 0 )wait(2);
					
					for(i = 0; i < 3; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					/* microSDProcess�J�n���� */
					while(microSDProcessStart( ul_msdStartAddress_ch) != 0x00)wait(2);
            		
					setMicroSDdata( c_msdBuff_ch );
					
					while( checkMicroSDProcess() != 11 )wait(2); /* �������݂��I���܂ő҂� */
					// �������ݏ������I���܂ŌJ��Ԃ�
					while( microSDProcessEnd() != 0 )wait(2);
			
					
					for(i = 0; i < 5; i++){
						setBeepPatternS( 0x8000 );
						wait(500);
					}
					
					c_logfin = 1;
				}
			}
		}
		break;

		
	case 500:
		
		if( pushsw_get() ) {
			c_mode++;
			if(c_mode >= 4)c_mode = 0;
			while(pushsw_get());
		}
		
		
		cam_in();
		led_out(camera(i_Center,i_Wide));
	
		printf("c_mode = %d  %4d   %4d\n",c_mode,i_Center,i_Wide);
		break;
	
    default:
        break;
    }
    }
}


int date_f_make(int i_pattern, int i_angle, int i_encoder, int i_rmode){

	static int si_mode = 0; //S = 0, L = 1, R = 2, C = 3, H = 4, F = 5
	static long sl_Encoder = 0;
	static int si_buff_num = 0;
	static int si_buff_num_int = 0;
//	static int si_buff_num_ch = 0;
//	static int si_buff_num_int_ch = 0;
	
	sl_Encoder += i_encoder;
	
	switch(i_pattern){
		case -1:
			//if(	sl_Encoder >= 500){//���̋����ȉ��͖����i����H�j
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//���̋����ȉ��͖���
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
					si_mode = 5;//F
				}
		//	}
			break;
		case 10:
		case 11:
		//if(	sl_Encoder >= 500){//���̋����ȉ��͖����i����H�j
			if(si_mode == 0){//S
				if(((i_rmode == 0 ) && (i_angle < -i_Cu_Angle)) || ((i_rmode != 0 ) && (i_angle < -(i_Cu_Angle_saka)))){
					if(sl_Encoder > 1000){//���̋����ȉ��͖���
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
						
					si_mode = 1;//L
						
				}else if(((i_rmode == 0) &&(i_Cu_Angle < i_angle)) || ((i_rmode != 0) &&(i_Cu_Angle_saka < i_angle))){
					if(sl_Encoder > 1000){//���̋����ȉ��͖���
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
					
					sl_Encoder =0;
						
					si_mode = 2;//R
				}
			}else if(si_mode == 1){//L�J�[�u
				//if(((i_rmode == 0) && (-i_Cu_Angle < i_angle)) || ((i_rmode != 0) && (-(i_Cu_Angle_saka) < i_angle)) ){
				if(-i_Cu_Angle < i_angle){
					sl_Encoder =0;
											
					si_mode = 0;//S
				}
			}else if(si_mode == 2){//R�J�[�u
				//if(((i_rmode ==0) && (i_angle < i_Cu_Angle) || ((i_rmode !=0) && (i_angle < i_Cu_Angle_saka)))){
				if(i_angle < i_Cu_Angle){
				
					sl_Encoder =0;
						
					si_mode = 0;//S
				}
			}else{
				if(((i_rmode == 0) &&(-i_Cu_Angle < i_angle && i_angle < i_Cu_Angle)) || ((i_rmode != 0) &&(-(i_Cu_Angle_saka) < i_angle && i_angle < i_Cu_Angle_saka))){
					sl_Encoder =0;
					si_mode = 0;//S
				}
			}

		//}
		break;

		case 21:
		case 22:
			if(si_mode != 3){//C
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//���̋����ȉ��͖���
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = sl_Encoder/100;
						si_buff_num++;
						c_date_f_buff[si_buff_num] = sl_Encoder - ((sl_Encoder/100)*100);
						si_buff_num++;
					}
				}
				sl_Encoder =0;
			}
			si_mode = 3;//C
				
			break;
		case 31:
			if(si_mode != 2){//R
				//���̂Ƃ���sl_Encoder���Ȃ���܂ł̋���
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/			
				sl_Encoder =0;
			}
			si_mode = 2;//R
			break;
		case 41:
			if(si_mode != 1){//L
				//���̂Ƃ���sl_Encoder���Ȃ���܂ł̋���
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/
				sl_Encoder =0;	
			}
			si_mode = 1;//L
			break;

		case 51:
		case 61:
		case 52:
		case 62:
			if(si_mode != 4){//H
				if(si_mode == 0){//S
					if(sl_Encoder > 1000){//���̋����ȉ��͖���
						i_date_f_buff_int[si_buff_num_int] += sl_Encoder;
						si_buff_num_int++;
						
						c_date_f_buff[si_buff_num] = (sl_Encoder/100);
						si_buff_num++;
						c_date_f_buff[si_buff_num] = (sl_Encoder - ((sl_Encoder/100)*100));
						si_buff_num++;
					}
				}
				sl_Encoder =0;
			}
			si_mode = 4;//H
			break;
		case 53:
			if(si_mode != 1){//L
				//���̂Ƃ���sl_Encoder���Ȃ���܂ł̋���
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/	
				sl_Encoder =0;
			}
			si_mode = 1;//L
			break;
		
		case 55:
			sl_Encoder =0;
			si_mode = 0;//S
			break;
			
		case 63:
			if(si_mode != 2){//R
				//���̂Ƃ���sl_Encoder���Ȃ���܂ł̋���
			/*	c_date_f_buff_ch[si_buff_num_ch] = i_pattern;
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder/100);
				si_buff_num_ch++;
				c_date_f_buff_ch[si_buff_num_ch] = (sl_Encoder - ((sl_Encoder/100)*100));
				si_buff_num_ch++;
			*/				
				sl_Encoder =0;
			}
			si_mode = 2;//R
			break;
		
		case 65:
			sl_Encoder =0;
			si_mode = 0;//S
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

	
	/*  7:���[�h�o��1bit           6:��6           5:���[�h�o��0bit            4:��5
        3:��4            2:�G���R�[�_B��      	   1:��3            0:�G���R�[�_A��   */
    p3  = 0x00;
    pd3 = 0xa0;	
	
    /*  XOUT            XIN             �{�[�h���LED   none
        none            VREF            none            none            */
    p4  = 0x20;                         /* P4_5��LED:�����͓_��         */
    pd4 = 0xb8;

	
    /*      �Z���^�[�l   P5_6�`P5_0       */
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
    timsr = 0xc0;                       /* TRGCLKA,TRGCLKB�[�q���蓖��  */
    trgcntc = 0xff;                     /* �ʑ��v��Ӱ�ނ̶��ĕ��@�w��   */
    trgmr = 0x82;                       /* TRG�̃J�E���g�J�n            */
	
	/* �^�C�}RG �^�C�}���[�h(���G�b�W�ŃJ�E���g)�̐ݒ� */
    //timsr = 0x40;                       /* TRGCLKA�[�q P3_0�Ɋ��蓖�Ă� */
    //trgcr = 0x15;                       /* TRGCLKA�[�q�̗��G�b�W�ŃJ�E���g*/
	//trgcr = 0x05;                       /* TRGCLKA�[�q�̗����オ��G�b�W�ŃJ�E���g*/
    //trgmr = 0x80;                       /* TRG�̃J�E���g�J�n            */
   

    /* �^�C�}RC PWM���[�h�ݒ�(���O���[�^�A�E�O���[�^) */
    trcpsr0 = 0x40;                     /* TRCIOA,B�[�q�̐ݒ�           */
    trcpsr1 = 0x33;                     /* TRCIOC,D�[�q�̐ݒ�           */
    trcmr   = 0x0f;                     /* PWM���[�h�I���r�b�g�ݒ�      */
    trccr1  = 0x8e;                     /* �������:f1,�����o�͂̐ݒ�    */
    trccr2  = 0x00;                     /* �o�̓��x���̐ݒ�             */
    trcgra  = TRC_MOTOR_CYCLE - 1;      /* �����ݒ�                     */
    trcgrb  = ui_trcgrb_buff = trcgra;     /* P0_5�[�q��ON��(���O���[�^)   */
    trcgrc  = trcgra;                   /* P0_7�[�q��ON��(�\��)         */
    trcgrd  = ui_trcgrd_buff = trcgra;     /* P0_6�[�q��ON��(�E�O���[�^)   */
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
	//static unsigned int sui_EncoderMod = 0;
	//static int si_Encoder5_old = 0; 
	static int si_Encoder1_buf[10] = {0}; 
	int i_Encoder_buf = 0; 
	int a;
	static int si_flag = 0,si_flag20 = 0,si_flag56 = 0;
	signed char *p;
	
    asm(" fset I ");                    /* �^�C�}RB�ȏ�̊��荞�݋���   */
	
    ul_cnt_1ms++;
	ul_cnt_running_1ms++;
	ul_cnt_for_wait++;
	ul_cnt_straight_time_1ms++;
	ul_cnt_curve_time_1ms++;
	
	
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
	

	//33*3.14  // 1�p���X*1.03 = 1mm  //1��] 100�p���X
	i = trg;
	si_Encoder1_buf[i_Timer10] = (i - ui_EncoderBuff)/2;
	l_EncoderTotal += si_Encoder1_buf[i_Timer10];
	if(l_EncoderTotal < 0)l_EncoderTotal = 0;
	ui_EncoderBuff = i;
	
	i_Encoder_buf = 0;
	for(k = 0; k < 10; k++)i_Encoder_buf += si_Encoder1_buf[k];
	i_Encoder10 = i_Encoder_buf;
	

    /* 10��1����s���鏈�� */
    i_Timer10++;
    switch( i_Timer10 ) {

    case 1:
        break;

    case 2:
        /* �X�C�b�`�ǂݍ��ݏ��� */
        p9_4 = 0;                       /* LED�o��OFF                   */
        pd8  = 0x00;
        break;

    case 3:
        /* �X�C�b�`�ǂݍ��݁ALED�o�� */
        uc_types_dipsw = ~p8;              /* ��ײ�ފ��TypeS Ver.3��SW�ǂݍ���*/
        p8  = uc_types_led;                /* ��ײ�ފ��TypeS Ver.3��LED�֏o��*/
        pd8 = 0xff;
        p9_4 = 1;                       /* LED�o��ON                    */
        break;

    case 4:
	case 9:
	/*	si_Encoder5_old = i_Encoder5 ;
		
		//21*3.14=65.94 200�p���X 200/65.94=3�p���X��1mm
		i = trg;
        i_Encoder5       = (i - ui_EncoderBuff)/3;	
		sui_EncoderMod     = (i - ui_EncoderBuff)%3;	
        l_EncoderTotal += i_Encoder5;
		//ui_EncoderBuff = i ;
        ui_EncoderBuff = i - sui_EncoderMod ;
		
		i_Encoder10 = i_Encoder5 + si_Encoder5_old ; 
*/		

		i_Encoder_buf = 0;
		for(k = 0; k < 5; k++)i_Encoder_buf += si_Encoder1_buf[(10 + i_Timer10 - k) % 10];
		i_Encoder5 = i_Encoder_buf;
	
		if(i_date_f_mode != 0 && ( i_msdFlag == 1 || i_msdFlag == 2 )){//�Đ����s���[�h
			a = getServoAngle();
			//���� 
			if((i_pattern == 11 || i_pattern == 10) && (((c_mode == 0) &&(-i_Cu_Angle < a && a < i_Cu_Angle)) || ((c_mode != 0) &&(-(i_Cu_Angle_saka) < a && a < i_Cu_Angle_saka)) ) ){
				l_straight_EncoderTotal += i_Encoder5;//�����v��
				
				if(si_flag56 == 1){//�n�[�t��̋����␳
					i_date_f_buff_int[i_date_f_num] -= i_date_f_plus_h;
					si_flag56 = 0;	
				}
				
				if(c_mode == 0){
					//if(i_date_f_buff_int[i_date_f_num] - date_f_brake < l_straight_EncoderTotal)si_flag = 1;//�L�^���������𑖂���
					if(i_date_f_buff_int[i_date_f_num] - 500 < l_straight_EncoderTotal)si_flag = 1;//�L�^���������𑖂���
				}else{
					//if(i_date_f_buff_int[i_date_f_num] - date_f_brake - 500 < l_straight_EncoderTotal)si_flag = 1;//�L�^���������𑖂���
					if(i_date_f_buff_int[i_date_f_num] - 500 - 600 < l_straight_EncoderTotal)si_flag = 1;//�L�^���������𑖂���
				}
				si_flag20 = 0;
			
			}else if(i_pattern == 21 || i_pattern == 22  ){
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//���̒����҂����
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//�L�^�ςݒ����I��
				}
				
				if(si_flag20 == 0){
					si_flag20 = 1;
					si_flag56 = 0;
					//l_straight_EncoderTotal = 0;
					l_straight_EncoderTotal = i_Encoder5;
				}else{
					l_straight_EncoderTotal += i_Encoder5;//�N���X���C���Ȃǂ���̋����v��
				}
				
			}else if( i_pattern == 51 || i_pattern == 52 || i_pattern == 61 || i_pattern == 62 ){
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//���̒����҂����
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//�L�^�ςݒ����I��
				}
				
				if(si_flag20 == 0){
					si_flag20 = 1;
					si_flag56 = 1;
					//l_straight_EncoderTotal = 0;
					l_straight_EncoderTotal = i_Encoder5;
				}else{
					l_straight_EncoderTotal += i_Encoder5;//�N���X���C���Ȃǂ���̋����v��
				}
				
			}else{//�J�[�u or �N�����N,�n�[�t�Ȃ��蒆 
			
				l_straight_EncoderTotal = 0;
				
				if(si_flag == 1){
					 si_flag = 0;
					 if(i_date_f_num < 15)i_date_f_num++;//���̒����҂����
					 if(i_date_f_buff_int[i_date_f_num] ==0 || i_date_f_num == 15)si_flag20 = 99;//�L�^�ςݒ����I��
				}
			}
		
			if(si_flag20 == 99){//�L�^�ςݒ����I��
				//�N�����N�A�n�[�t���I��
				if(i_date_f_buff_ch_int[i_date_f_num_ch] == 0 && (i_pattern == 11 || i_pattern == 10))i_date_f_mode = 0;//�Đ����s�I��
			}
		}
		
		
		
		/* microSD�L�^���� */
    	if( i_msdFlag == 1 || i_msdFlag == 2 ) {
			p = c_msdBuff + i_msdBuffAddress;

            /* �o�b�t�@�ɋL�^ �������� */
            *p++ = i_pattern;             /* �p�^�[��                     */
            *p++ = (char)i_Center;    
            *p++ = (char)i_Wide;
			i = getServoAngle();        /* �p�x                         */
			*p++ = i >> 8;
            *p++ = i & 0xff;
			
			*p++ = (char)i_Center_IR;//�ԊO���Z���T�[
			
			*p++ = i_handleBuff;  /* �T�[�{PWM�ۑ�        */
            *p++ = i_FleftMotorBuff;       /* �O�����[�^PWM�l                */
            *p++ = i_FrightMotorBuff;      /* �O�E���[�^PWM�l                */
			*p++ = i_RleftMotorBuff;       /* �㍶���[�^PWM�l                */
            *p++ = i_RrightMotorBuff;      /* ��E���[�^PWM�l                */
			*p++ = i_Encoder5;    /* �G���R�[�_                   */
            *p++ = c_mode;/*���[�h*/
            *p++ = c_saka_cnt;/*�⓹��*/
            *p++ = (char)(i_angle_y/10);
            *p++ = (char)(i_angle_x/10);
			
		//	*p++ = (char)(IR_L());
        //    *p++ = (char)(IR_R());
            
			
            /* �o�b�t�@�ɋL�^ �����܂� */

            i_msdBuffAddress += 64;       /* RAM�̋L�^�A�h���X������      */


			if( i_msdBuffAddress >= 512) {	
                /* 512�ɂȂ�����AmicroSD�ɋL�^���� */
                i_msdBuffAddress = 0;
                setMicroSDdata( c_msdBuff );
                ul_msdWorkAddress += 512;
				
                if( ul_msdWorkAddress >= ul_msdEndAddress || i_msdFlag == 2) {
                    /* �L�^�����I�� */
    				i_msdFlag = 0;
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
        /* i_Timer10�ϐ��̏��� */
        i_Timer10 = 0;
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
    trcgrb = ui_trcgrb_buff;
    trcgrd = ui_trcgrd_buff;
}


/************************************************************************/
/* �}�C�R���{�[�h��̃f�B�b�v�X�C�b�`�l�ǂݍ���                         */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`15                                              */
/************************************************************************/
unsigned char dipsw_get( void )
{
    unsigned char uc_sw;

    uc_sw = p1 & 0x0f;                     /* P1_3�`P1_0�ǂݍ���           */

    return uc_sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃f�B�b�v�X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0�`255                                             */
/************************************************************************/
unsigned char dipsw_get2( void )
{
    /* ���ۂ̓��͂̓^�C�}RB���荞�ݏ����Ŏ��{ */
    return uc_types_dipsw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��̃v�b�V���X�C�b�`�l�ǂݍ���          */
/* �����@ �Ȃ�                                                          */
/* �߂�l �X�C�b�`�l 0:OFF 1:ON                                         */
/************************************************************************/
unsigned char pushsw_get( void )
{
    unsigned char uc_sw;

    uc_sw = ~p9_5 & 0x01;

    return uc_sw;
}

/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��CN6�̏�ԓǂݍ���                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l 0�`15                                                         */
/************************************************************************/
unsigned char cn6_get( void )
{
    unsigned char uc_data;

    uc_data = p7 >> 4;

    return uc_data;
}


/************************************************************************/
/* ���[�^�h���C�u���TypeS Ver.3��LED����                               */
/* �����@ 8��LED���� 0:OFF 1:ON                                       */
/* �߂�l �Ȃ�                                                          */
/************************************************************************/
void led_out( unsigned char uc_led )
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
    uc_types_led = LED;
	*/
	uc_types_led = uc_led;
	
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
	}else if(accele_l < BRAKE_MAX_R){
		accele_l = BRAKE_MAX_R;
	}
	
	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < BRAKE_MAX_R){
		accele_r = BRAKE_MAX_R;
	}
	
	
	i_RleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    i_RrightMotorBuff = accele_r;         /* �o�b�t�@�ɕۑ�               */
	

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
		
		if(accele_l == 100)trdgrd0 = (long)( TRD_MOTOR_CYCLE +2);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_l / 100;
    } else {
        p2_1 = 1;
		if(accele_l == -100)trdgrd0 = (long)( TRD_MOTOR_CYCLE + 2);
        else trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
		
		//trdgrd0 = (long)( TRD_MOTOR_CYCLE - 2 ) * ( -accele_l ) / 100;
    }

    /* �E�ヂ�[�^ */
    if( accele_r >= 0 ) {
        p2_3 = 0;
		if(accele_r == 100)trdgrc1 = (long)( TRD_MOTOR_CYCLE  +2);
        else trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
		
		//trdgrc1 = (long)( TRD_MOTOR_CYCLE - 2 ) * accele_r / 100;
    } else {
        p2_3 = 1;
		if(accele_r == -100)trdgrc1 = (long)( TRD_MOTOR_CYCLE  +2);
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
	}else if(accele_l < BRAKE_MAX){
		accele_l = BRAKE_MAX;
	}
	

	if(accele_r > 100){
		accele_r = 100;
	}else if(accele_r < BRAKE_MAX){
		accele_r = BRAKE_MAX;
	}
	
	
	i_FleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    i_FrightMotorBuff = accele_r;         /* �o�b�t�@�ɕۑ�               */
	
	
    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = ui_trcgrb_buff = trcgra;
    } else {
		if(accele_l == 100)ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
		
		//ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = ui_trcgrd_buff = trcgra;
    } else {
		if(accele_r == 100)ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE  +2);
        else ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
		
		//ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
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
	else if(accele_l < BRAKE_MAX_R)accele_l = BRAKE_MAX_R;
	
	
	if(accele_l == 0 || (i_RleftMotorBuff > 0 && accele_l < 0) || (i_RleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - i_RleftMotorBuff  > i_KASOKU)accele_l = i_RleftMotorBuff + i_KASOKU;
	else if(accele_l - i_RleftMotorBuff < -i_KASOKU)accele_l = i_RleftMotorBuff - i_KASOKU;
	
	
	if(accele_r > 100)accele_r = 100;
	else if(accele_r < BRAKE_MAX_R)accele_r = BRAKE_MAX_R;
	
	
	if(accele_r == 0 || (i_RrightMotorBuff > 0 && accele_r < 0) || (i_RrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - i_RrightMotorBuff > i_KASOKU)accele_r = i_RrightMotorBuff + i_KASOKU;
	else if(accele_r - i_RrightMotorBuff < -i_KASOKU)accele_r = i_RrightMotorBuff- i_KASOKU;
	
	
	i_RleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    i_RrightMotorBuff = accele_r;          /* �o�b�t�@�ɕۑ�               */

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
	else if(accele_l < BRAKE_MAX)accele_l = BRAKE_MAX;
	
	
	if(accele_l == 0  || (i_FleftMotorBuff > 0 && accele_l < 0) || (i_FleftMotorBuff < 0 && accele_l > 0))accele_l = 0;
	else if(accele_l - i_FleftMotorBuff  > i_KASOKU)accele_l = i_FleftMotorBuff + i_KASOKU;
	else if(accele_l - i_FleftMotorBuff < -i_KASOKU)accele_l = i_FleftMotorBuff - i_KASOKU;
	

	if(accele_r > 100)accele_r = 100;
	else if(accele_r < BRAKE_MAX)accele_r = BRAKE_MAX;
	
	
	if(accele_r == 0 || (i_FrightMotorBuff > 0 && accele_r < 0) || (i_FrightMotorBuff < 0 && accele_r > 0))accele_r = 0;
	else if(accele_r - i_FrightMotorBuff > i_KASOKU)accele_r = i_FrightMotorBuff + i_KASOKU;
	else if(accele_r - i_FrightMotorBuff < -i_KASOKU)accele_r = i_FrightMotorBuff- i_KASOKU;
	

	i_FleftMotorBuff  = accele_l;          /* �o�b�t�@�ɕۑ�               */
    i_FrightMotorBuff = accele_r;          /* �o�b�t�@�ɕۑ�               */
		
    /* ���O���[�^ */
    if( accele_l >= 0 ) {
        p2_0 = 0;
    } else {
        p2_0 = 1;
        accele_l = -accele_l;
    }
    if( accele_l <= -1 ) {//5
        trcgrb = ui_trcgrb_buff = trcgra;
    } else {
		//if(accele_l == 100)p0_5 = 1;
        ui_trcgrb_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_l / 100;
    }

    /* �E�O���[�^ */
    if( accele_r >= 0 ) {
        p2_7 = 0;
    } else {
        p2_7 = 1;
        accele_r = -accele_r;
    }
    if( accele_r <= -1 ) {//5
        trcgrd = ui_trcgrd_buff = trcgra;
    } else {
		//if(accele_r == 100)p0_6 = 1;
        ui_trcgrd_buff = (unsigned long)(TRC_MOTOR_CYCLE-2) * accele_r / 100;
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
void servoPwmOut( int i_pwm )
{
	int i_angle = getServoAngle();
	
	if((i_angle < -SERVO_MAX && i_pwm  > 0) || ( SERVO_MAX < i_angle && i_pwm  < 0)){//�n���h���Ȃ�����
		i_pwm = -(i_pwm / 8);//�t�]				
	}
	
	i_pwm = -i_pwm;//
	
	if( i_pwm >  100 ) i_pwm =  100;        /* �}�C�R���J�[�����肵����     */
    if( i_pwm < -100 ) i_pwm = -100;        /* �����90���炢�ɂ��Ă������� */
    
	i_handleBuff = i_pwm;                 /* �o�b�t�@�ɕۑ�               */


    if( i_pwm >= 0 ) {
        p2_6 = 0;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE - 2 ) * i_pwm / 100;
    } else {
        p2_6 = 1;
        trdgrd1 = (long)( TRD_MOTOR_CYCLE- 2 ) * ( -i_pwm ) / 100;
    }
}


/************************************************************************/
/* �T�[�{�p�x�擾                                                       */
/* �����@ �Ȃ�                                                          */
/* �߂�l ����ւ���̒l                                                */
/************************************************************************/
int getServoAngle( void )
{
    return( ad2 - i_Angle0 );
}

/************************************************************************/
/* �X�^�[�g�Q�[�g���o����												*/
/* �߂�l 0:�Ȃ� 1:����													*/
/************************************************************************/
unsigned char check_startgate( void )
{
	unsigned char uc_ret;
	
	uc_ret = 0;
	
	//if(IR_R() > 140){//37
	if(850 < ad7){ 
		uc_ret = 1;
	}
	/*
	if((i_Wide >= 27) && (i_Center > -20) && (i_Center < 20)){
	
		uc_ret = 1;			// �Q�[�g���� 
	}*/
	return uc_ret;
}
 

/************************************************************************/
/* �N���X���C�����o����													*/
/* �߂�l 0:�N���X���C���Ȃ� 1:����										*/
/************************************************************************/
unsigned char check_crossline( void )
{
	unsigned char uc_ret;
	
	//cam_in();//�l�̎擾

	uc_ret = 0;
	if( (i_Wide > 60) || ((i_Wide >= 40) && (-10 < i_Center ) && (i_Center < 10))   || ((i_Wide >= 35) && (i_Center > -5) && (i_Center < 5)) ){
	
		uc_ret = 1;			/* �N���X���C������ */

	}
	return uc_ret;
}
 

/************************************************************************/
/* �n�[�t���C�����o����                                                 */
/* �߂�l 0:�n�[�t���C���Ȃ� 1:�� 2:�E 3:�N�����N                       */
/************************************************************************/
unsigned char check_halfline( void )
{
    unsigned char uc_ret;

	uc_ret = 0;
	if(i_Wide > 44 && i_Wide < 50){
		if(i_Center < -12){//�Z���^�[�����
			uc_ret = 1;
			
		}else if(i_Center > 12){//�Z���^�[�E���
			uc_ret = 2;
			
		}
	}else if(i_Wide > 34){
		if(i_Center < -10){//�Z���^�[�����
			uc_ret = 1;
			
		}else if(i_Center > 10){//�Z���^�[�E���
			uc_ret = 2;
			
		}
	
	}else if(i_Wide > 25){
		if(i_Center < -5){//�Z���^�[�����
			uc_ret = 1;
			
		}else if(i_Center > 5){//�Z���^�[�E���
			uc_ret = 2;
			
		}
	}
	
	return uc_ret;
}

/************************************************************************/
/* �n�[�t���C�����o����  �N�����N�p                                     */
/* �߂�l 0:�n�[�t���C���Ȃ� 1:�� 2:�E 3:�N�����N                       */
/************************************************************************/
unsigned char check_halfline_forC( void ) //�N�����N���̒��p�m�F�p
{
    unsigned char uc_ret;
	
	uc_ret = 0;
	if(i_Wide > 24){
		if(i_Center < -1){//�Z���^�[�����
			uc_ret = 1;
			
		}else if(i_Center > 1){//�Z���^�[�E���
			uc_ret = 2;
			
		}
	}
	
	return uc_ret;
}

/************************************************************************/
/* wideline���o����													*/
/* �߂�l 0:�Ȃ� 1:����										*/
/************************************************************************/
unsigned char check_wideline( void )
{
	unsigned char uc_ret;
	
	//cam_in();//�l�̎擾
	
	uc_ret = 0;
	if(i_Wide >= 28){
	
		uc_ret = 1;			/* wide���C������ */
	}
	return uc_ret;
}

/************************************************************************/
/* �T�[�{���[�^����                                                     */
/* �����@ �Ȃ�                                                          */
/* �߂�l �O���[�o���ϐ� i_ServoPwm �ɑ��                               */
/************************************************************************/
void servoControl( void )
{
	long     l_Ret, l_P, l_D;
	static int si_I = 0;

  
    /* �T�[�{���[�^�pPWM�l�v�Z */
	if(c_IR_flag == 0){	
		l_P = i_kp * (i_Center + i_Center_offset);      /* ���                         */
		si_I = si_I + (i_SensorBefore - (i_Center + i_Center_offset) );
    	l_D = i_kd * (i_SensorBefore - (i_Center + i_Center_offset));     /* ����(�ڈ���P��5�`10�{)       */
		l_Ret = l_P - l_D - si_I * i_ki;
	}else{
		l_P = i_kp_ir * (i_Center_IR);      /* ���                         */
		si_I = si_I + (i_SensorBeforeIR - (i_Center_IR) );
    	l_D = i_kd_ir * (i_SensorBeforeIR - (i_Center_IR));     /* ����(�ڈ���P��5�`10�{)       */
		l_Ret = l_P - l_D - si_I * i_ki_ir;
	}
    l_Ret /= 2;//256 128

    /* PWM�̏���̐ݒ� */
    if( l_Ret >  100 ) l_Ret =  100;        /* �}�C�R���J�[�����肵����     */
    if( l_Ret < -100) l_Ret = -100;        /* �����90���炢�ɂ��Ă������� */
    i_ServoPwm = l_Ret;

    i_SensorBefore = (i_Center + i_Center_offset );                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
	i_SensorBeforeIR = i_Center_IR;
}

/************************************************************************/
/* ���W���[���� servoControl2                                            */
/* �����T�v     �T�[�{���[�^����                                        */
/* ����         �Ȃ�                                                    */
/* �߂�l       �O���[�o���ϐ� i_ServoPwm2 �ɑ��                         */
/************************************************************************/
void servoControl2( void )
{
    int      i,j, i_Ret, i_P, i_D;


    i = i_SetAngle;              	/* �ڕW�p�x             */
    j = getServoAngle();              	/* �ڕW�p�x�Z���T�l             */

    /* �T�[�{���[�^�pPWM�l�v�Z */
    i_P = - i_kp2 * (j - i);                        /* ���                         */
    i_D = - i_kd2 * (i_AngleBefore2 - j);     /* ����(�ڈ���P��5�`10�{)       */
    i_Ret = i_P - i_D;
    i_Ret /= 2;

    /* PWM�̏���̐ݒ� */
    if( i_Ret >  100 ) i_Ret =  100;        /* �}�C�R���J�[�����肵����     */
    if( i_Ret < -100 ) i_Ret = -100;        /* �����90���炢�ɂ��Ă������� */

    i_ServoPwm2 = i_Ret;

    i_AngleBefore2 = j;                  /* ����͂��̒l��1ms�O�̒l�ƂȂ�*/
}

/************************************************************************/
/*	wait																*/
/************************************************************************/
void wait(int t)
{
	ul_cnt_for_wait = 0;
	while (ul_cnt_for_wait <= t);
}

/**************************************************************************/
/*	RX62G����̃J�����̒l���擾											  */
/**************************************************************************/
void cam_in(){
	int i_wide = 0;
	
	/*	
	//��z��
	i_wide += ((p6 >> 5 ) & 0x07);//012
	i_wide += ((p3 << 2 ) & 0x08);//3
	i_wide += ((p3 << 1 ) & 0x30);//45
	i_wide += ( p3 & 0x40);//6
	*/
	
	//NEW
	i_wide += ((p3 >> 6 ) & 0x01);
	i_wide += ((p3 >> 3 ) & 0x02);
	i_wide += ((p3 >> 1 ) & 0x04);
	i_wide += ((p3 << 2 ) & 0x08);
	i_wide += ((p6 >> 3 ) & 0x10);
	i_wide += ((p6 >> 1 ) & 0x20);
	i_wide += ((p6 << 1 ) & 0x40);
	
	
	i_Wide = i_wide;

	//i_Center = (p5 & 0x7f);
	
	if(1 < i_Wide  && i_Wide < 100){//�J�����̎��t���ŏ�������Ă����̂�
		i_Center = (p5 & 0x7f) - 64 + 3;	
	}else{
		i_Center = (p5 & 0x7f) - 64;
	}
}

/**************************************************************************/
/*	RX62G�փ��[�h�̒l���o��											  */
/**************************************************************************/
void mode_out(){
	
	p3_5 = c_mode & 0x01;//LOW
	p3_7 = (c_mode & 0x02) >> 1;//HIGH
}

/**************************************************************************/
/*	LED�\���p�̒l���v�Z													  */
/**************************************************************************/
int camera(int i_center, int i_wide){
	
	int i_start = 17,i_end = 111,i,i_led = 0,i_cnt = 1;
	// (end - start)/7 = 13
	int i_led_bit[8] = {999,48,55,61,999,67,74,81};//4�͕s��
	
	i_center += 64;
	i_wide /= 2;
	
	for(i = 0; i < 8; i++){
		if(i_center - i_wide < i_led_bit[i] && i_led_bit[i] < i_center + i_wide){
			i_led += i_cnt;
		}
		i_cnt *= 2;
	}

	return i_led;
}


/**************************************************************************/
/*	�W���C���Z���T�[��Y���̒l���擾										  */
/**************************************************************************/
void get_angle_y(){
	
	i_pre_angle_y[0] = i_pre_angle_y[1];
	i_pre_angle_y[1] = i_angle_y;
	
	i_angle_y = (i_angle_y * 9/10) + (ad0 * 1/10);
	
	if(((i_pre_angle_y[0] < i_pre_angle_y[1]) && (i_pre_angle_y[1] < i_angle_y))  || ((i_angle_y < i_pre_angle_y[1]) && (i_pre_angle_y[1] < i_pre_angle_y[0])))
		i_angle_y = i_pre_angle_y[1];
		
	else if(((i_pre_angle_y[1] < i_pre_angle_y[0]) && (i_pre_angle_y[0] < i_angle_y))  || ((i_angle_y < i_pre_angle_y[0]) && (i_pre_angle_y[0] < i_pre_angle_y[1])))
		i_angle_y = i_pre_angle_y[0];
}

/**************************************************************************/
/*	�W���C���Z���T�[��X���̒l���擾													  */
/**************************************************************************/
void get_angle_x(){
	i_pre_angle_x[0] = i_pre_angle_x[1];
	i_pre_angle_x[1] = i_angle_x;
	
	i_angle_x = (i_angle_x * 85/100) + (ad1 * 15/100);
	
	if(((i_pre_angle_x[0] < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[1]) && (i_pre_angle_x[1] < i_pre_angle_x[0])))
		i_angle_x = i_pre_angle_x[1];
		
	else if(((i_pre_angle_x[1] < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_angle_x))  || ((i_angle_x < i_pre_angle_x[0]) && (i_pre_angle_x[0] < i_pre_angle_x[1])))
		i_angle_x = i_pre_angle_x[0];
}

/**************************************************************************/
/*	�W���C���Z���T�[��X���̒l���`�F�b�N									  */
/**************************************************************************/
int angle_check(){

	if(200 < i_angle_y && i_angle_y < 400){
		if(i_angle_x <= 170)return 2;//�� 170
		if(i_angle_x > 440)return 0;//��
	}
	return 1;//�ω�����
}



/**************************************************************************/
/*	IR_L												  */
/**************************************************************************/
int IR_L(){
	
	return 100 * (ad6 - i_IR_min[0]) / (i_IR_max[0] - i_IR_min[0]);
}
/**************************************************************************/
/*	IR_R												  */
/**************************************************************************/
int IR_R(){
	
	return 100 * (ad7 - i_IR_min[1]) / (i_IR_max[1] - i_IR_min[1]);
}
/**************************************************************************/
/*	Get_Center_IR											  */
/**************************************************************************/
void Get_Center_IR(){
	int i_l = IR_L(),i_r = IR_R();
	i_IR_old = i_Center_IR;
	
	i_Center_IR = (i_r - i_l)/5;
	
	if(i_l < 15 && i_r < 15){//�ԊO���������Ȃ�����O��l����
		if(	i_IR_old < 0)i_Center_IR = -30;
		else i_Center_IR = 30;
	}
	
}
/**************************************************************************/
/*	�ԊO���Z���T�[�̃L�����u���[�V����												  */
/**************************************************************************/
void IRcalibration( ){
	int i ,j,l,r,p = 20,a = 60;

	i_Angle0 = 0;
	i_Angle0 = getServoAngle();  /* 0�x�̈ʒu�L��                */
	i = getServoAngle();
	
	for(j = 0; j < 2;j++){
		while(-a < i){
			i = getServoAngle();
			servoPwmOut(p);
			r = ad7;
			if(i_IR_max[1] < r)i_IR_max[1] = r;
			if(r < i_IR_min[1])i_IR_min[1] = r;
			l = ad6;
			if(i_IR_max[0] < l)i_IR_max[0] = l;
			if(l < i_IR_min[0])i_IR_min[0] = l;
		}
	
		while(i < a){
			i = getServoAngle();
			servoPwmOut(-p);
			r = ad7;
			if(i_IR_max[1] < r)i_IR_max[1] = r;
			if(r < i_IR_min[1])i_IR_min[1] = r;
			l = ad6;
			if(i_IR_max[0] < l)i_IR_max[0] = l;
			if(l < i_IR_min[0])i_IR_min[0] = l;
		}
	}
	while(0 < i){
		i = getServoAngle();
		servoPwmOut(p);
		r = ad7;
		if(i_IR_max[1] < r)i_IR_max[1] = r;
		if(r < i_IR_min[1])i_IR_min[1] = r;
		l = ad6;
		if(i_IR_max[0] < l)i_IR_max[0] = l;
		if(l < i_IR_min[0])i_IR_min[0] = l;
	}
	servoPwmOut( 0 );
}

int max(int i_a,int i_b){
	if(i_a < i_b)return i_b;
	return i_a;	
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
