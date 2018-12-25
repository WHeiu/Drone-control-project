#ifndef MahonyAHRS_h
#define MahonyAHRS_h

typedef enum
{
	FlyStop  = 0,
	FlyIdle  = 1,
	FlyStart = 2
}FLY_STATE;


void MPU9250_Calibration(void);
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
void Mahony_Init(float sampleFrequency);
void Mahony_computeAngles(void);
float getRoll(void);
float getPitch(void);
float getYaw(void);
float getRollRadians(void);
float getPitchRadians(void);
float getYawRadians(void);
void attitude_quat_5ms_task(void);
void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw);
void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt);
#endif
