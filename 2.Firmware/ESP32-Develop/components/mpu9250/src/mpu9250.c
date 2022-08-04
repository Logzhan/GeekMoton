/*----------------------------------------------------------------------------/
* Description  : MPU9250 diriver for esp32
/----------------------------------------------------------------------------*/
#include <math.h>
#include <stdio.h>
#include "mpu9250.h"
#include "mpu9250_iic.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "geek_shell_api.h"

#define MOUNT_POINT      "/sdcard"
static const char *TAG = "MPU9250";
float q0 = 1.0f; 
float q1 = 0.0f; 
float q2 = 0.0f; 
float q3 = 0.0f;
MPU mpu9250;
FILE* fp = NULL;

/**----------------------------------------------------------------------
* Function    : init_mpu9250
* Description : 初始化配置MPU9250
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/31 zhanli
*---------------------------------------------------------------------**/
void init_mpu9250(){
	// 配置MPU9250 IO口
    init_mpu9250_iic_gpio();

	uint8_t id = 0;
	while(id != MPU9250_Device_ID)
	{
		id = i2c_read_one_byte(MPU9250_I2C_ADDR, (MPU9250_WHO_AM_I));
		vTaskDelay(200 / portTICK_PERIOD_MS);
	}
    if(id == MPU9250_Device_ID)
	{  
		ESP_LOGI(TAG, "read device id sucess. 0x%.2x\r\n", id);
        i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_PWR_MGMT_1,   0x00);			// 唤醒mpu9250
	    i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_CONFIG,       0x06);    	    // 低通滤波5hz				 
	    i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_GYRO_CONFIG,  0x18);			// 不自检，2000deg/s		  
	    i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_ACCEL_CONFIG, 0x00);			// (0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)
	    i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_INT_PIN_CFG,  0x02);
	    i2c_write_one_byte(MPU9250_I2C_ADDR,MPU9250_USER_CTRL,    0x00);			//使能I2C 
    }else{
		ESP_LOGI(TAG, "read device id fail.0x%.2x\r\n", id);
	}
	//vTaskDelete(NULL);
}

void IIC_Get_MPU6500Data(int16_t *ax,int16_t *ay,int16_t *az,
                         int16_t *gx,int16_t *gy,int16_t *gz,int16_t *temp)
{
	uint8_t buf[20];
	i2c_read_bytes(MPU9250_I2C_ADDR,MPU9250_ACCEL_XOUT_H,
				   buf, 14);
	// 加速度
	*ax = (buf[0] << 8) + buf[1];
	*ay = (buf[2] << 8) + buf[3];
	*az = (buf[4] << 8) + buf[5];
	// 温度
	*temp =(buf[6] <<8) + buf[7];
	// 陀螺仪
	*gx = (buf[8]  << 8) + buf[9];
	*gy = (buf[10] << 8) + buf[11];
	*gz = (buf[12] << 8) + buf[13];
}

// void imu_sample_task(){

// }

// int imu_detect_static(){
// 	return 0;
// }

void GetMPU9250Data()
{
	float ax,ay,az,gx,gy,gz;
	ESP_LOGI(TAG, "Get data....\n");
	/*获取加速度计和陀螺仪的数据*/
	IIC_Get_MPU6500Data(&mpu9250.acc_x,&mpu9250.acc_y,&mpu9250.acc_z,&mpu9250.gyro_x,
	                    &mpu9250.gyro_y,&mpu9250.gyro_z,&mpu9250.temp);

	/*对加速度计和陀螺仪数据进行缩放以及进行坐标轴变换           */
	gx = (float) (M_PI*(mpu9250.gyro_x)* GYRO_KEN / 180.00f);
	gy = (float) (M_PI*(mpu9250.gyro_y)* GYRO_KEN / 180.00f);
	gz = (float) (M_PI*(mpu9250.gyro_z)* GYRO_KEN / 180.00f);
	
    /* 获取原始的加速度计的数值并按照单位进行缩放*/	
	ax = (float) (mpu9250.acc_x * ACC_KEN); 
	ay = (float) (mpu9250.acc_y * ACC_KEN); 
	az = (float) (mpu9250.acc_z * ACC_KEN); 

	AHRSupdate(gx, gy, gz, ax, ay, az);
}

void GetMPU9250Data_Euler(float* yaw,float* roll, float* pitch)
{
	float ax,ay,az,gx,gy,gz;

	/*获取加速度计和陀螺仪的数据*/
	IIC_Get_MPU6500Data(&mpu9250.acc_x,&mpu9250.acc_y,&mpu9250.acc_z,&mpu9250.gyro_x,
	                    &mpu9250.gyro_y,&mpu9250.gyro_z,&mpu9250.temp);

	/*对加速度计和陀螺仪数据进行缩放以及进行坐标轴变换           */
	gx = (float) (M_PI*(mpu9250.gyro_x)* GYRO_KEN / 180.00f);
	gy = (float) (M_PI*(mpu9250.gyro_y)* GYRO_KEN / 180.00f);
	gz = (float) (M_PI*(mpu9250.gyro_z)* GYRO_KEN / 180.00f);
	
    /* 获取原始的加速度计的数值并按照单位进行缩放*/	
	ax = (float) (mpu9250.acc_x * ACC_KEN); 
	ay = (float) (mpu9250.acc_y * ACC_KEN); 
	az = (float) (mpu9250.acc_z * ACC_KEN); 

	mpu9250.acc[0] = ax;
	mpu9250.acc[1] = ay;
	mpu9250.acc[2] = az;

	mpu9250.gyr[0] = gx;
	mpu9250.gyr[1] = gy;
	mpu9250.gyr[2] = gz;

	if(fp != NULL ){
		fprintf(fp, "%f,%f,%f,%f,%f,%f\n", ax,ay,az,gx,gy,gz);
		printf("%f,%f,%f,%f,%f,%f\r\n", ax,ay,az,gx,gy,gz);
	}
	AHRSupdate(gx, gy, gz, ax, ay, az);
	
	*yaw   = mpu9250.yaw;   
	*roll  = mpu9250.roll;  
	*pitch = mpu9250.pitch; 
}

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) 
{	
	float halfT = 0.5 * (10.0f / 1000.0f);
	float Kp = 2.0f;
	
   	float norm = 1.0 / sqrtf(ax*ax + ay*ay + az*az);	
	
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;																	 
       	                                                                      
	float vx = 2.0f*(q1*q3 - q0*q2);														 
	float vy = 2.0f*(q0*q1 + q2*q3);															 
	float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
	
	float ex = (ay*vz - az*vy);
	float ey = (az*vx - ax*vz);
	float ez = (ax*vy - ay*vx);
		                                                                        
   	if(ex != 0.0f && ey != 0.0f && ez != 0.0f){																			
      	gx = gx + Kp * ex;														
      	gy = gy + Kp * ey;														                                              
      	gz = gz + Kp * ez;
   	}
	         
	float qw = (-q1*gx - q2*gy - q3*gz) * halfT;											  
	float qx = ( q0*gx + q2*gz - q3*gy) * halfT;
	float qy = ( q0*gy - q1*gz + q3*gx) * halfT;	
	float qz = ( q0*gz + q1*gy - q2*gx) * halfT; 

	q0 = q0 + qw;											
   	q1 = q1 + qx;
   	q2 = q2 + qy;
   	q3 = q3 + qz;  

   	norm = 1.0 / sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);										

   	q0 = q0 * norm;       //w
   	q1 = q1 * norm;       //x
   	q2 = q2 * norm;       //y
   	q3 = q3 * norm;       //z
																	
    float Pitch = asin((float)(-2.0f*(q3*q1 - q0*q2))); // * (180.0f / 3.141592f);							                                            */
    float Yaw  = atan2(q2*q1 + q0*q3,0.5f - q2*q2 - q3*q3); // * (180.0f /3.141592f);
    float Roll  = atan2(q2*q3 + q0*q1,0.5f - q2*q2 - q1*q1); //* (180.0f /3.141592f);

    Pitch *= (180.0f / 3.141592f);
    Yaw   *= (180.0f / 3.141592f);
    Roll  *= (180.0f / 3.141592f);
	
	mpu9250.yaw   = Yaw;
	mpu9250.roll  = Roll;
	mpu9250.pitch = Pitch;

}

void init_quaternion(float q[4]){
	mpu9250.q[0] = 1.0f;
	mpu9250.q[1] = 0.0f;
	mpu9250.q[2] = 0.0f;
	mpu9250.q[3] = 0.0f;
}

/**----------------------------------------------------------------------
* Function    : data_record_task
* Description : 启动IMU数据采集任务
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
int data_record_task(){

	if(fp != NULL){
		fclose(fp);
		fp = NULL;
	}
	fp = fopen(MOUNT_POINT"/imu.txt", "w");
	if(fp == NULL){
		printf("open file fail\r\n");
		return -1;
	}
	return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), data_sample_task, data_record_task, save imu data);


/**----------------------------------------------------------------------
* Function    : data_record_task
* Description : 停止IMU数据采集任务
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
int stop_record_task(){

	if(fp != NULL){
		fclose(fp);
		fp = NULL;
		return 0;
	}
	return -1;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), stop_sample_task, stop_record_task, stop save imu data);