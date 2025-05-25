/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
int initMicroSD( void );
int getMsdType( void );
unsigned long getMsdSize( void );
int readMicroSD( unsigned long address, signed char *read );
int writeMicroSD( unsigned long address, signed char *write );
int getMicroSD_CSD( unsigned char *p );
int eraseMicroSD( unsigned long st_address, unsigned long ed_address );
int setMicroSDdata( signed char *p );
int microSDProcessStart( unsigned long address );
int microSDProcessEnd( void );
void microSDProcess( void );
int checkMicroSDProcess( void );
void setMicroSDLedPort( char *p, char *pd, int bit );

int mountMicroSD_FAT32( void );
int writeFile( const char *s, unsigned long fileSize );

void convertDecimalToStr( int value, int keta, signed char *p );
void convertHexToStr( unsigned int value, int keta, signed char *p );
void convertBinaryToStr( unsigned char value, int keta, signed char *p );

int getCompileYear( const char *p );
int getCompileMonth( const char *p );
int getCompileDay( const char *p );
int getCompileHour( const char *p );
int getCompilerMinute( const char *p );
int getCompilerSecond( const char *p );
void setDateStamp( int y, int m, int d );
void setTimeStamp( int h, int m, int s );

int readMicroSDNumber( void );
int writeMicroSDNumber( int number );

int msdPrintf(char _far *format, ...);
int microSDPrintfProcess( void );
int checkMsdPrintf( void );

/*======================================*/
/* 定数設定                             */
/*======================================*/
#define MSD_TYPE_SD100  1
#define MSD_TYPE_SD200  2
#define MSD_TYPE_SDHC   3
