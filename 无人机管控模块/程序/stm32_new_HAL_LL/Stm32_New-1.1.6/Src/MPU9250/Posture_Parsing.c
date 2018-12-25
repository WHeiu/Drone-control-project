#include "Posture_Parsing.h"
#include <math.h>
#include "usart.h"
#include "string.h"
#include "mpu9250.h"
#include <stdbool.h>
bool GRY_Stable_Flag=false;     // �����������ȶ���־
FLY_STATE fly_state=FlyStop;     //����״̬
float_XYZ acce_f,gyro_f;        //IMU ����
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
  
	gx = gx * 0.0174f;         //ת��Ϊ����
	gy = gy * 0.0174f;
	gz = gz * 0.0174f;
  
	twoKp=IMU_P;      //�����˲�����ϵ��
	
  /*
	���ٶȼ����ݹ�һ������λ�������Ѽ��ٶȼƵ���ά����ת��Ϊ��λ������ ��Ϊ�ǵ�λʸ�����ο��Ե�ͶӰ�� 
  ����Ҫ�Ѽ��ٶȼ����ݵ�λ������ʵ��һ���ı��ֻ�������������ĳ��ȣ�
  Ҳ����ֻ�ı�����ͬ�ı���������û�иı䣬Ҳ��Ϊ���뵥λ��Ԫ����Ӧ
  */
	recipNorm = invSqrt(ax * ax + ay * ay + az * az);
	ax *= recipNorm;
	ay *= recipNorm;
	az *= recipNorm;
	
  /*
  ����Ԫ������ɡ��������Ҿ����еĵ����е�����Ԫ�ء��������Ҿ����ŷ���ǵĶ��壬
  ��������ϵ������������ת����������ϵ��������������Ԫ�ء����������halfvx��halfvy��halfvz 
  ��ʵ���ǵ�ǰ�Ļ����������ϵ�ϻ��������������λ������һ�롣(�ñ�ʾ������̬����Ԫ�����л���)
  */
	halfvx = q1 * q3 - q0 * q2;  //�ο�ϵX��������ϵ x ��֮�䷽���������� 
	halfvy = q0 * q1 + q2 * q3;   //�ο�ϵY��������ϵ y ��֮�䷽���������� 
	halfvz = q0 * q0 - 0.5f + q3 * q3; //�ο�ϵZ��������ϵ z ��֮�䷽���������� 
	
 /*
 ����˵��һ�㣬���ٶȼ����������Ƚϴ󣬶����ڷ��й����У��ܻ�����Ӱ������������ԣ���ʱ���ڵĿɿ��Բ��ߡ�
 ����������С���������ڻ�������ɢ�ģ���ʱ��Ļ��ֻ����Ư�Ƶ�����������Ҫ���ü��ٶȼ���õ���̬�����������ǻ�����̬��Ư�ơ�
 ���ͣ� 
 ax,ay,az�ǻ����������ϵ�ϣ����ٶȼƲ����������������Ҳ����ʵ�ʲ����������������
 vx,vy,vz�����ݻ��ֺ����̬����������������������Ƕ��ǻ����������ϵ�ϵ�����������
 ������֮�������������������ݻ��ֺ����̬�ͼӼƲ��������̬֮�����
 ������������������������Ҳ�������������ˣ�����ʾ��ex,ey,ez�����������������Ĳ����
 �����������Ծ���λ�ڻ�������ϵ�ϵģ������ݻ������Ҳ���ڻ�������ϵ��
 ���Ҳ���Ĵ�С�����ݻ����������ȣ����������������ݡ�
 ���������ǶԻ���ֱ�ӻ��֣����Զ����ݵľ�������ֱ�������ڶԻ�������ϵ�ľ�����

 �����ɵ�ǰ��̬���������������ϵķ�������ٶȼƲ�õ��������������ϵķ����Ĳ�,���������ά�ռ�Ĳ��(������)�������.

 ��������ó���̬��ͨ�����ٶȼƲ�õ���������ϵ�µĵ�λ��������һ����Ԫ��ת���ɵĵ�λ�������в�ˣ�
 �Դ˵õ�����������������õ���־������Ѳ����ͬ�ڽ��ٶ����
 */ 
	halfex = (ay * halfvz - az * halfvy);
	halfey = (az * halfvx - ax * halfvz);
	halfez = (ax * halfvy - ay * halfvx);        //����ϵ ��


  /*
  ����������� PID ���������뱾����̬���������������ǲ�õĽ��ٶ���ӣ�
  ���յõ�һ�������Ľ��ٶ�ֵ,����������Ԫ��΢�ַ��̣�������Ԫ��
  */
	gx += twoKp * halfex;
	gy += twoKp * halfey;
	gz += twoKp * halfez;



  //����Ԫ��΢�ַ����õ�����һ�ױϿ�������ȻҲ���������������������Ԫ����dt�����ݲ����ļ�� 
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
  
	// ��Ԫ����һ������λ����
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


 //��Ԫ��תŷ���� 
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
//		if (fly_state!=FlyStart && GRY_Stable_Flag == true)       //���ݷɻ�״̬ ���»����˲�ϵ��
//		{                
			IMU_P = 10.0f;        //δ�����IMU�����ȶ�ʱ�Ӵ󻥲��˲�P �ӿ� ���ٶ������ںϱ���
//		} 
//    else
//    {                         
//			  IMU_P = 0.6f;      //��ɺ��С���ٶ������ںϱ���
//		}			
//			
		sensfusion6UpdateQ(gyro_f.X, gyro_f.Y, gyro_f.Z,acce_f.X*100.0f,acce_f.Y*100.0f,acce_f.Z*100.0f, 0.005f);    //6Axie IMU������Ԫ��
		sensfusion6GetEulerRPY(&Q_ANGLE.Roll,&Q_ANGLE.Pitch,&Q_ANGLE.Yaw);   //��Ԫ��תŷ����		
}





