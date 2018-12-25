//=============================================================================================
// MahonyAHRS.c
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// Algorithm paper:
// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
//
//=============================================================================================

//-------------------------------------------------------------------------------------------
// Header files

#include "MahonyAHRS.h"
#include <math.h>
#include "usart.h"
#include "string.h"
//-------------------------------------------------------------------------------------------
// Definitions

float twoKi;		// 2 * integral gain (Ki)
float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
float invSampleFreq;
extern float roll, pitch, yaw;
char anglesComputed;



//#define twoKpDef	(100.0f*0.5f)	// 2 * proportional gain 比例   通过调节Kp,Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度
//#define twoKiDef	(1*1.0f)    	// 2 * integral gain    积分      
#define twoKpDef	(200.0f*0.5f)	// 2 * proportional gain 比例     通过调节Kp,Ki两个参数，可以控制加速度计修正陀螺仪积分姿态的速度
#define twoKiDef	(1*1.0f)    	// 2 * integral gain    积分     
void Mahony_Init(float sampleFrequency)
{
	twoKi = twoKiDef;	// 2 * integral gain (Ki)
	q0 = 1.0f;
	q1 = 0.0f;
	q2 = 0.0f;
	q3 = 0.0f;
	integralFBx = 0.0f;
	integralFBy = 0.0f;
	integralFBz = 0.0f;
	anglesComputed = 0;
	invSampleFreq = 1.0f / sampleFrequency;
}
float Mahony_invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

#if 1
//在Mahony算法中首先对加速度值进行了归一化，故加速度计的输出单位不会对算法本身产生影响。而Mahony算法中，关于角度使用的是弧度制，我们在使用陀螺仪的输出数据之前，需要进行转换
void Mahony_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Convert gyroscope degrees/sec to radians/sec
//	gx *= 0.0174533f;
//	gy *= 0.0174533f;
//	gz *= 0.0174533f;
  char TEST1[20] = {0};
	char TEST2[20] = {0};
	char TEST3[20] = {0};
	char TEST4[20] = {0};
	char TEST5[20] = {0};
	char TEST6[20] = {0};
	char TEST7[20] = {0};
	char TEST8[20] = {0};
	char TEST9[20] = {0};	
	
	 printff(USART3, "\r\n转换后的浮点数:\r\n",strlen("\r\n转换后的浮点数:\r\n"));
		sprintf( TEST1, "%f", ax); 
		sprintf( TEST2, "%f", ay); 
		sprintf( TEST3, "%f", az); 
		sprintf( TEST4, "%f", gx); 
		sprintf( TEST5, "%f", gy); 
		sprintf( TEST6, "%f", gz); 
		sprintf( TEST7, "%f", mx); 
		sprintf( TEST8, "%f", my); 
		sprintf( TEST9, "%f", mz); 
	  printff(USART3, "\r\nACC:\r\n",strlen("\r\nACC:\r\n"));
		printff(USART3, TEST1,strlen(TEST1));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST2,strlen(TEST2));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST3,strlen(TEST3));   
  	printff(USART3, "\r\nGYO:\r\n",strlen("\r\nGYO:\r\n"));
		printff(USART3, TEST4,strlen(TEST4));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST5,strlen(TEST5));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST6,strlen(TEST6));   
  	printff(USART3, "\r\nMAG:\r\n",strlen("\r\nMAG:\r\n"));
		printff(USART3, TEST7,strlen(TEST7));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST8,strlen(TEST8));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST9,strlen(TEST9));   
		printff(USART3, "\r\n",strlen("\r\n"));
		printff(USART3, "\r\n",strlen("\r\n"));
	

	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float recipNorm;
	//float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float hx, hy, bx, bz;
	float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
	float halfex, halfey, halfez;
	float qa, qb, qc;
	// Convert gyroscope degrees/sec to radians/sec
//	gx *= 0.0174533f;
//	gy *= 0.0174533f;
//	gz *= 0.0174533f;
  char TEST1[20] = {0};
	char TEST2[20] = {0};
	char TEST3[20] = {0};
	char TEST4[20] = {0};
	char TEST5[20] = {0};
	char TEST6[20] = {0};
	char TEST7[20] = {0};
	char TEST8[20] = {0};
	char TEST9[20] = {0};	
	
			// Auxiliary variables to avoid repeated arithmetic
	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;
	
	 printff(USART3, "\r\n转换后的浮点数:\r\n",strlen("\r\n转换后的浮点数:\r\n"));
		sprintf( TEST1, "%f", ax); 
		sprintf( TEST2, "%f", ay); 
		sprintf( TEST3, "%f", az); 
		sprintf( TEST4, "%f", gx); 
		sprintf( TEST5, "%f", gy); 
		sprintf( TEST6, "%f", gz); 
		sprintf( TEST7, "%f", mx); 
		sprintf( TEST8, "%f", my); 
		sprintf( TEST9, "%f", mz); 
	  printff(USART3, "\r\nACC:\r\n",strlen("\r\nACC:\r\n"));
		printff(USART3, TEST1,strlen(TEST1));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST2,strlen(TEST2));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST3,strlen(TEST3));   
  	printff(USART3, "\r\nGYO:\r\n",strlen("\r\nGYO:\r\n"));
		printff(USART3, TEST4,strlen(TEST4));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST5,strlen(TEST5));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST6,strlen(TEST6));   
  	printff(USART3, "\r\nMAG:\r\n",strlen("\r\nMAG:\r\n"));
		printff(USART3, TEST7,strlen(TEST7));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST8,strlen(TEST8));   printff(USART3, "\t",strlen("\t"));
		printff(USART3, TEST9,strlen(TEST9));   
		printff(USART3, "\r\n",strlen("\r\n"));
		printff(USART3, "\r\n",strlen("\r\n"));
	


	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = Mahony_invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = Mahony_invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Reference direction of Earth's magnetic field
		hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
		hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
		bx = sqrtf(hx * hx + hy * hy);
		bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

		// Estimated direction of gravity and magnetic field
		halfvx = q1q3 - q0q2;
		halfvy = q0q1 + q2q3;
		halfvz = q0q0 - 0.5f + q3q3;
		halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
		halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
		halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

		// Error is sum of cross product between estimated direction
		// and measured direction of field vectors
		halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
		halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
		halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

		// Compute and apply integral feedback if enabled
		if(twoKi > 0.0f) {
			// integral error scaled by Ki
			integralFBx += twoKi * halfex * invSampleFreq;
			integralFBy += twoKi * halfey * invSampleFreq;
			integralFBz += twoKi * halfez * invSampleFreq;
			gx += integralFBx;	// apply integral feedback
			gy += integralFBy;
			gz += integralFBz;
		} else {
			integralFBx = 0.0f;	// prevent integral windup
			integralFBy = 0.0f;
			integralFBz = 0.0f;
		}

		// Apply proportional feedback
		gx += twoKpDef * halfex;
		gy += twoKpDef * halfey;
		gz += twoKpDef * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * invSampleFreq);		// pre-multiply common factors
	gy *= (0.5f * invSampleFreq);
	gz *= (0.5f * invSampleFreq);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = Mahony_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
	anglesComputed = 0;
}
#endif



void Mahony_computeAngles()
{
	roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
	anglesComputed = 1;
}
float getRoll() {
	if (!anglesComputed) Mahony_computeAngles();
	return roll * 57.29578f;            //范围从-180°到180°
}
float getPitch() {
	if (!anglesComputed) Mahony_computeAngles();
	return pitch * 57.29578f;           //范围从-180°到180°
}
float getYaw() {
	if (!anglesComputed) Mahony_computeAngles();
	return yaw * 57.29578f ;    //范围从0°到360°
}
//============================================================================================
// END OF CODE
//============================================================================================
