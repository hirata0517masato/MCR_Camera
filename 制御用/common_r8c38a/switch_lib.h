/*======================================*/
/* �萔�ݒ�                             */
/*======================================*/
#define     SW_0    (0x01 << 0)         /* getSwNow�֐���bit0�̈ʒu     */
#define     SW_1    (0x01 << 1)         /* getSwNow�֐���bit1�̈ʒu     */
#define     SW_2    (0x01 << 2)         /* getSwNow�֐���bit2�̈ʒu     */
#define     SW_3    (0x01 << 3)         /* getSwNow�֐���bit3�̈ʒu     */
#define     SW_4    (0x01 << 4)         /* getSwNow�֐���bit4�̈ʒu     */

/*======================================*/
/* �v���g�^�C�v�錾                     */
/*======================================*/
void initSwitch( void );
unsigned char getSwNow( void );
unsigned char getSwFlag( unsigned char flag );
void switchProcess( void );
