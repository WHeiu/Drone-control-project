#ifndef __AHRS_

void CalibrateToZero(void);
float invSqrt(float number);
void CountTurns(float *newdata,float *olddata,short *turns);
void CalYaw(float *yaw,short *turns);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);


#endif