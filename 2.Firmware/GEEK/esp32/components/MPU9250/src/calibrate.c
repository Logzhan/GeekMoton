/************************************************************************
Filename    :   calibrate.c
Content     :   Sensor calibration store, fetch, and apply
Created     :   2015/10/3
*************************************************************************/
#include "calibrate.h"
#include "mpu9250.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "spi_flash_storage.h"
#include "esp_log.h"
//#include <math.h>

//float Gyro_Offset[3];
float Gyro_Xout,Gyro_Yout,Gyro_Zout;      //陀螺仪零漂值
#define CALI_FALG 0xa3
unsigned short int Buffer[13]={0};
/***********************************************************
*函  数：calibrate_init(void)
*功  能：初始化追踪器的所有校准数据
*时  间：2015/10/3
*作  者: JustFeng
***********************************************************/
void calibrate_init(void)
{
	/*初始化陀螺仪零偏数据*/
	Init_Gyro_Offset();
}

/***********************************************************
*函  数：Read_Gyro_Offset()
*功  能：从STM32的EEROM中读取陀螺仪零偏值
*时  间：2015/10/3
*作  者: Laputa_logzhan
***********************************************************/
void Init_Gyro_Offset()
{
	if(plug_param.Gyro_Cali_flag!=CALI_FALG)
	{
		// os_printf("Gyro_Cali_Start");
		int i=0;
		for (i=0;i<100;i++)
			vTaskDelay(50 / portTICK_PERIOD_MS);
		Compute_Gyro_Offset(); /*获取陀螺仪零偏*/
		Store_Gyro_Offset();   /*存储零偏数据到EEPROM中*/
	}

}
/************************************************************
*函  数：Compute_Gyro_Offset(void)
*功  能：在静止时候读取MPU6050的陀螺仪数值，计算陀螺仪的零偏
*时  间：2015/10/3
*作  者: Laputa_logzhan
************************************************************/
/*通过求平均来计算偏移*/
void Compute_Gyro_Offset(void)
{
  	unsigned short int i;
  	float gyro_x=0, gyro_y=0, gyro_z=0;
  	unsigned int count=0;
	signed short int accel[3],gyro[3],temperature;
  	for(i=0;i<3000;i++)
  	{
		IIC_Get_MPU6500Data(&accel[0],&accel[1],&accel[2],&gyro[0],
	                        &gyro[1],&gyro[2],&temperature);
        /*进行原始数据的校准*/
		gyro_x 	+= gyro[0];
		gyro_y	+= gyro[1];
		gyro_z	+= gyro[2];
		count++;
		// system_soft_wdt_feed();
  	}
  	Gyro_Xout = (gyro_x / count);
  	Gyro_Yout = (gyro_y / count);
  	Gyro_Zout = (gyro_z / count);
	count = 0;	gyro_x=0; gyro_y=0; gyro_z=0;
}

/************************************************************
*函  数：Store_Gyro_Offset(void)
*功  能：把误差参数存到eeprom中
*时  间：2015/10/3
*作  者: Laputa_logzhan
************************************************************/
void Store_Gyro_Offset(void)
{
	plug_param.Gyro_Cali_flag=CALI_FALG;
	plug_param.Gyro_Offset[0]=Gyro_Xout;
	plug_param.Gyro_Offset[1]=Gyro_Yout;
	plug_param.Gyro_Offset[2]=Gyro_Zout;

	esp_err_t espRc = save_param();
	if (espRc == ESP_OK) {
		ESP_LOGI("calibration", "IIC Gyro Cali Store successfully");
	} else {
		ESP_LOGE("calibration", "IIC Gyro Cali Store failed. code: 0x%.2X", espRc);
	}
}
/************************************************************
*函  数：Clean_Gyro_Offset(void)
*功  能：从EEPROM中擦除零偏数据
*时  间：2015/10/3
*作  者: Laputa_logzhan
************************************************************/
void Clean_Gyro_Offset(void)
{
	uint8_t i;
	plug_param.Gyro_Cali_flag=0;
	for(i=0;i<3;i++)
		plug_param.Gyro_Offset[i]=0;

	esp_err_t espRc = save_param();
	if (espRc == ESP_OK) {
		ESP_LOGI("calibration", "IIC Clean Gyro Offset successfully");
	} else {
		ESP_LOGE("calibration", "IIC Clean Gyro Offset failed. code: 0x%.2X", espRc);
	}
}

