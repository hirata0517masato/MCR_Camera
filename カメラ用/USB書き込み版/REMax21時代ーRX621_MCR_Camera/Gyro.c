#include "MPU9250.h"
#include"Gyro.h"
#include "rspi.h"
#include "wait.h"	


int gyro= 0;        /* ジャイロセンサーの値  左＋　右ー   */
long long gyro_base = 0;
long long gyro_sum = 0;  //左＋　右ー  
int Gyro_kp = 5,Gyro_kd = 65;



void GyroSum_reset(){
  gyro_sum = 0;
}

void GyroSum_add(long long a){
  gyro_sum += a;
}

long long GyroSum_get(){
  return gyro_sum;
}

int Gyro(){
	return gyro;
}

long long Gyro_get(){
	read_gyro();
	return (long long)get_gyro_data(2);
}


void Gyro_update(){
 
  //long long g = (((long long)gyro * 7) + (((long long)Gyro_get() - gyro_base) * 3)) / 10;
  //gyro = (long long)Gyro_get() - gyro_base;

 /* int g = (Gyro_get() - gyro_base);
  
  if(g < 0)gyro = (g * 105) / 100;
  else gyro = g;
*/
  gyro = (Gyro_get() - gyro_base)/2;
  gyro_sum += (long long)gyro ; 
}

int gyro_powor_L(){
  return ((gyro_sum * (long long)Gyro_kp))/100  + ((long long)(gyro *Gyro_kd)) / 100;
}

void Gyro_init(){
	int i;
	
  //pinMode(CS, OUTPUT);
  SPI_init();//SPI.begin();

  delay_ITP(100);
  MPU_init();
  delay_ITP(100);
   
  calib_acc();
  calib_mag();
	
  delay_ITP(1000);
  
  gyro_base = 0;
  for(i = 0;i < 256;i++){
    gyro_base += Gyro_get();
    delay_ITP(1);
  }
  gyro_base >>= 8;
  gyro_sum = 0;
  //gyro = gyro_base;
}
