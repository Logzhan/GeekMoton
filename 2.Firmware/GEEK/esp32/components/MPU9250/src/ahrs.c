#include "ahrs.h"
#include "math.h"
#include <stdint.h>
float _sqrtf(float x) {
	return 1.0f / invSqrt(x);
}
float GET_NOWTIME(void) {
	float temp = 0;
	static uint32_t now;
	if (now == 0)
		now = WDEV_NOW();

	now = WDEV_NOW() - now;
	temp = (float)((float) now / 2000000.0f); //转换为ms halfT
	if (temp < 0) //防止溢出
		temp = 0.0f;
	now = WDEV_NOW();
	return temp;
}

/**********************************************************************************************************
 * Function Name  : AHRSupdate
 * 输入           : 三轴加速度计、陀螺仪、磁力计的值
 * 描述           ：加速度计、陀螺仪、磁力计融合得到的欧拉角

 1）这个算法基于标准笛卡尔右手坐标系，z轴朝上，x轴朝人，y轴朝右
 2）传感器的方向需要变换到和改算法定义的坐标轴相同。

 * 修改           ：2016/11/5 : 根据磁力计采样率比较低的特点，设置了2种模式：1、融合磁力计 2、不融合磁力计
 2016/11/6 ：改进了四元数更新算法，原来的四元数更新算法会导致累计误差。
 2016/11/9 : 规定了本算法运行的坐标轴，重写了四元数到欧拉角变换的代码。
 ***********************************************************************************************************/

float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;
float exInt = 0.0f;
float eyInt = 0.0f;
float ezInt = 0.0f;

#define  Kp   2.5f                                                                     /*加速度计和磁力计对陀螺仪的比例修正参数		 */
#define  Ki   0.005f                                                                   /*加速度计和磁力计对陀螺仪的积分修正参数		 */
#define  Kd   0.0f                                                                    

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az,
		float mx, float my, float mz, char useMag) {

	static int8_t AHRS_Init = 0;
	/*原算法的定义*/
	float norm, halfT;
	float hx, hy, hz, bz, by;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	float qw, qx, qy, qz;
	float AccelAjust, MagAjust;
	/*方便之后的程序使用，减少计算时间*/
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	if (AHRS_Init > 100) //如果已经初始化
			{
		halfT = GET_NOWTIME(); /*得到每次姿态更新的周期的一半t = (1/SampleFrq) * 0.5		           */

	} else {
		halfT = 0.1;
		gx = gy = gz = 0;
		AHRS_Init++;
	}

	norm = invSqrt(ax * ax + ay * ay + az * az); /*把加速度计向量转换为单位向量*/
	if ((1.0f / norm) > 1.6 || (1.0f / norm) < 0.4) {
		AccelAjust = 0;
		MagAjust = 1.2f;

	} else {
		AccelAjust = 1.0f;
		MagAjust = 1.0f;
	}

	ax = ax * norm;
	ay = ay * norm;
	az = az * norm; /*把磁力计向量转换为单位向量  */
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;

	hx = 2.0f
			* (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3)
					+ mz * (q1q3 + q0q2)); /*把(mx,my,mz)转换到地理坐标系的(hx,hy,hz),利用H = Q^-1 * M            */
	hy = 2.0f
			* (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3)
					+ mz * (q2q3 - q0q1));
	hz = 2.0f
			* (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1)
					+ mz * (0.5f - q1q1 - q2q2));

	/*计的数值为(bx,by,0),所以我们不关注hx的数值                           */
	// bx = sqrtf((hx*hx) + (hy*hy));												   /*使磁力计正交化*/
	by = sqrt((hx * hx) + (hy * hy));
	bz = hz;

	/*v代表的是把地理坐标系的加速度(0,0,1g)转换到机体坐标系的加            */
	vx = 2.0f * (q1q3 - q0q2); /*速度计(ax,ay,az),其实就是用R*(0,0,1),R为旋转矩阵，此矩阵可			  */
	vy = 2 * (q0q1 + q2q3); /*由四元数转换得到													  */
	vz = q0q0 - q1q1 - q2q2 + q3q3;

	wx = 2.0f * (by * (q1q2 + q0q3) + bz * (q1q3 - q0q2)); /*把正交化的H即B从地理坐标系转换到飞行器坐标系，利用W = Q * B          */
	wy = 2.0f * (by * (0.5f - q1q1 - q3q3) + bz * (q0q1 + q2q3)); /*这里认为bx = 0                                                   */
	wz = 2.0f * (by * (q2q3 - q0q1) + bz * (0.5f - q1q1 - q2q2));

	if (useMag != 0) /*如果使用磁力计，则融合磁力计*/
	{
		ex = AccelAjust * (ay * vz - az * vy) + MagAjust * (my * wz - mz * wy); /*用当前姿态向量和加速度的姿态向量做叉乘，乘积越小说明两个向量方向越相同*/
		ey = AccelAjust * (az * vx - ax * vz) + MagAjust * (mz * wx - mx * wz);
		ez = AccelAjust * (ax * vy - ay * vx) + MagAjust * (mx * wy - my * wx);
	} else {

		ex = AccelAjust * (ay * vz - az * vy);
		ey = AccelAjust * (az * vx - ax * vz);
		ez = AccelAjust * (ax * vy - ay * vx);
		/*因为加速度计没有校准，ez不靠谱*/
	}

	if (ex != 0.0f && ey != 0.0f && ez != 0.0f) {
		/*这里使用了PID调节的方式，对角速度修正,Ki 是积分修正，Kp是直接修正    */
		exInt = exInt + ex * Ki * halfT; /*对误差进行积分                                                       */
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;
		gx = gx + Kp * (1 + Kd * fabs(ex)) * ex + exInt; /*使用比例和积分综合对陀螺仪进行修正，由于存在积分修正，所以才能确保回 */
		gy = gy + Kp * (1 + Kd * fabs(ey)) * ey + eyInt; /*到期望回到的位置                                                     */
		gz = gz + Kp * (1 + Kd * fabs(ez)) * ez + ezInt;
	}

	qw = (-q1 * gx - q2 * gy - q3 * gz) * halfT; /*利用一阶龙格库塔法对四元数进行更新*/
	qx = (q0 * gx + q2 * gz - q3 * gy) * halfT;
	qy = (q0 * gy - q1 * gz + q3 * gx) * halfT;
	qz = (q0 * gz + q1 * gy - q2 * gx) * halfT;

	q0 = q0 + qw;
	q1 = q1 + qx;
	q2 = q2 + qy;
	q3 = q3 + qz;

	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3); /*对四元数进行规范化，即四元数模为1*/

	q0 = q0 * norm;       //w
	q1 = q1 * norm;       //x
	q2 = q2 * norm;       //y
	q3 = q3 * norm;       //z

	/*对原变换公式进行重写，避免了以前坐标轴混乱的问题，按照标准数学右手坐标*/
//	Pitch = asin(-2.0f*(q3*q1-q0*q2))* (180.0f /3.141592f);							    /*轴进行四元数到欧拉角的转换                                            */
//	Yaw   = atan2(q2*q1 + q0*q3,0.5f - q2*q2 - q3*q3)* (180.0f /3.141592f);
//	Roll  = atan2(q2*q3 + q0*q1,0.5f - q2*q2 - q1*q1)* (180.0f /3.141592f);
}

/*******************************************************************************
 * Function Name  : invSqrt
 * Description    : 快速计算 1/Sqrt(x)，源自雷神3，神奇的0x5f3759df
 * 输入           : x
 * 输出           ：1/sqrt(x)
 *******************************************************************************/
float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*) &y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*) &i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}

