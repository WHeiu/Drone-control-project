#include "Posture_Parsing.h"
#include <math.h>
#include "usart.h"
#include "string.h"
#include "mpu9250.h"
#include <stdbool.h>
bool GRY_Stable_Flag=false;     // 陀螺仪数据稳定标志
FLY_STATE fly_state=FlyStop;     //飞行状态
float_XYZ acce_f,gyro_f;        //IMU 数据
float_RPY Q_ANGLE,Q_Rad;

float IMU_P = 0.8;

float q0=1, q1=0, q2=0, q3=0;
static float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void sensfusion6UpdateQ(float gx, float gy, float gz, float ax, float ay, float az, float dt)
{ 
	float twoKp ;
  
	float recipNorm;
	float halfvx, halfvy, halfvz;
	float halfex, halfey, halfez;
	float qa, qb, qc;                
  
	gx = gx * 0.0174f;         //转化为弧度
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;
  
	twoKp=IMU_P;      //互补滤波比例系数
	
  /*
	加速度计数据归一化（单位化），把加速度计的三维向量转换为单位向量， 因为是单位矢量到参考性的投影， 
  所以要把加速度计数据单位化，其实归一化改变的只是这三个向量的长度，
  也就是只改变了相同的倍数，方向并没有改变，也是为了与单位四元数对应
  */
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
  /*
  把四元数换算成“方向余弦矩阵”中的第三列的三个元素。根据余弦矩阵和欧拉角的定义，
  地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。所以这里的halfvx、halfvy、halfvz 
  其实就是当前的机体坐标参照系上换算出来的重力单位向量的一半。(用表示机体姿态的四元数进行换算)
  */
	halfvx = q1 * q3 - q0 * q2;  //参考系X轴与载体系 x 轴之间方向余弦向量 
	halfvy = q0 * q1 + q2 * q3;   //参考系Y轴与载体系 y 轴之间方向余弦向量 
	halfvz = q0 * q0 - 0.5f + q3 * q3; //参考系Z轴与载体系 z 轴之间方向余弦向量 
	
 /*
 这里说明一点，加速度计由于噪声比较大，而且在飞行过程中，受机体振动影响比陀螺仪明显，短时间内的可靠性不高。
 陀螺仪噪声小，但是由于积分是离散的，长时间的积分会出现漂移的情况，因此需要将用加速度计求得的姿态来矫正陀螺仪积分姿态的漂移。
 解释： 
 ax,ay,az是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
 vx,vy,vz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
 那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
 向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，ex,ey,ez就是两个重力向量的叉积。
 这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，
 而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。
 由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。

 计算由当前姿态的重力在三个轴上的分量与加速度计测得的重力在三个轴上的分量的差,这里采用三维空间的差积(向量积)方法求差.

 向量叉积得出姿态误差，通过加速度计测得的重力坐标系下的单位向量与上一次四元数转换成的单位向量进行叉乘，
 以此得到其误差量外积在相减得到差分就是误差，把叉积等同于角速度误差
 */ 
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);        //机体系 下


  /*
  将该误差输入 PID 控制器后与本次姿态更新周期中陀螺仪测得的角速度相加，
  最终得到一个修正的角速度值,将其输入四元数微分方程，更新四元数
  */
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;



  //解四元数微分方程用到的是一阶毕卡法，当然也可以用龙格库塔法更新四元数，dt：陀螺采样的间隔 
	gx *= (0.5f * dt);   // pre-multiply common factors
	gy *= (0.5f * dt);
	gz *= (0.5f * dt);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);
  
	// 四元数归一化（单位化）
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


 //四元数转欧拉角 
void sensfusion6GetEulerRPY(float* roll, float* pitch,float* yaw)
{
	float gx, gy, gz;    
  char test1[10] = {0},test2[10] = {0};
	gx = 2.0f * (q1*q3 - q0*q2);
	gy = 2.0f * (q0*q1 + q2*q3);
	gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

	*pitch = atanf(gx / sqrt(gy*gy + gz*gz)) * 57.3f;
	*roll  = atanf(gy / sqrt(gx*gx + gz*gz)) * 57.3f;
//	*yaw   = atan2f(2.0f*q1*q2 - 2.0f*q0*q3, 2.0f*q0*q0 + 2.0f*q1*q1 - 1) * -57.3f;
	sprintf( test1, "%f", *pitch); 
	sprintf( test2, "%f", *roll); 
	printff(USART3, "\r\nPitch:\r\n",strlen("\r\nPitch:\r\n"));
	printff(USART3, test1,strlen(test1));   
	printff(USART3, "\r\nRoll:\r\n",strlen("\r\nRoll:\r\n"));
	printff(USART3, test2,strlen(test2));   
}

void attitude_quat_5ms_task(void)
{
//		if (fly_state!=FlyStart && GRY_Stable_Flag == true)       //根据飞机状态 更新互补滤波系数
//		{                
			IMU_P = 10.0f;        //未起飞且IMU数据稳定时加大互补滤波P 加快 加速度数据融合比例
//		} 
//    else
//    {                         
//			  IMU_P = 0.6f;      //起飞后减小加速度数据融合比例
//		}			
//			
		sensfusion6UpdateQ(gyro_f.X, gyro_f.Y, gyro_f.Z,acce_f.X*100.0f,acce_f.Y*100.0f,acce_f.Z*100.0f, 0.005f);    //6Axie IMU更新四元数
		sensfusion6GetEulerRPY(&Q_ANGLE.Roll,&Q_ANGLE.Pitch,&Q_ANGLE.Yaw);   //四元数转欧拉交		
}





