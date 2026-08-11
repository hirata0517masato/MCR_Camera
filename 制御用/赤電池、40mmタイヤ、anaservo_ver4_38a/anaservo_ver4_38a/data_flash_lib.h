/*======================================*/
/* プロトタイプ宣言                     */
/*======================================*/
void readDataFlash( unsigned int r_address, signed char *w_address, int count );
int writeDataFlash( unsigned int w_address, signed char *r_address, int count );
int blockEraseDataFlash( unsigned int address );
