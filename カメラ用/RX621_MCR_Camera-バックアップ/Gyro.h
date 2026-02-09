#ifndef GYRO_H
#define GYRO_H

long long Gyro_get(void);
void GyroSum_reset(void);
void GyroSum_add(long long);
long long GyroSum_get(void);
void Gyro_update(void);
int gyro_powor_L(void);
void Gyro_init(void);
int Gyro(void);


#endif