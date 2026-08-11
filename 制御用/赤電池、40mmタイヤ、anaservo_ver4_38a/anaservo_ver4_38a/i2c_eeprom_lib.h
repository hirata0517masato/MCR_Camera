/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void initI2CEeprom( void );
void selectI2CEepromAddress( unsigned char address );
signed char readI2CEeprom( unsigned long address );
void writeI2CEeprom( unsigned long address, signed char write );
void setPageWriteI2CEeprom( unsigned long address, int count, signed char* data );
void I2CEepromProcess( void );
void clearI2CEeprom( char *led_port , int bit );
int checkI2CEeprom( void );
