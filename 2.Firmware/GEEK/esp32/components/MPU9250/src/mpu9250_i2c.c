#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/i2c.h"
#include "esp_log.h"

#include "MPU9250.h"
#include "mpu9250_i2c.h"
#include "ahrs.h"
#define tag "MPU9250"
MPU mpu9250;
int16_t MPU_temp;
float ax, ay, az, gx, gy, gz, mx, my, mz;
//extern uint8_t packet_buf[64];
//extern int NumOfSensor ;
float MagASA[3];
void IIC_Get_AK8963MagAdjData(float* mx, float* my, float *mz) //axis sensitivity adjustment value
{
	uint8_t buf[3];
	//IIC_Write_OneByte(AK8963_I2C_ADDR,AK8963_CNTL1,0x11);//每读一次数据，ak8963会自动进入powerdown模式,这里需要重新设定为单测量模式
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, AK8963_I2C_ADDR, true);
	i2c_master_write_byte(cmd, AK8963_ASAX, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, AK8963_I2C_ADDR | I2C_MASTER_READ, true);
	i2c_master_read(cmd,(uint8_t *)buf, 3,I2C_MASTER_LAST_NACK );
	// i2c_master_write_byte(cmd, &REG_data, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "IIC Get_AK8963 Mag Adj Data successfully");
	} else {
		//ESP_LOGE(tag, "IIC Get_AK8963 Mag Adj Data failed. code: 0x%.2X", espRc);
	}

	*mx = (float) (((float) (buf[0] - 0x80)) / 256 + 1);
	*my = (float) (((float) (buf[1] - 0x80)) / 256 + 1);
	*mz = (float) (((float) (buf[2] - 0x80)) / 256 + 1);//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13
	// i2c_driver_delete(I2C_NUM_1);
}

void IIC_Write_OneByte(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t REG_data) {
 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, REG_Address, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, REG_data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		ESP_LOGI("MPU9250", "IIC configured successfully");
	} else {
		ESP_LOGE("MPU9250", "IIC Write OneByte failed. code: 0x%.2X", espRc);
	}
	// i2c_driver_delete(I2C_NUM_1);
}

uint8_t IIC_Read_OneByte(uint8_t SlaveAddress, uint8_t REG_Address) {
	uint8_t REG_data;
 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, REG_Address, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &REG_data, I2C_MASTER_LAST_NACK );
	// i2c_master_write_byte(cmd, &REG_data, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		// ESP_LOGI("MPU9250", "IIC Read Byte successfully");
	} else {
		ESP_LOGE("MPU9250", "IIC Read Byte failed. code: 0x%.2X", espRc);
	}
	// i2c_master_read(cmd, data_rd, RW_TEST_LENGTH, ACK_VAL);
    // i2c_master_read_byte(cmd, data_rd + RW_TEST_LENGTH, NACK_VAL);	
	// i2c_driver_delete(I2C_NUM_1);
	return REG_data;
}

void IIC_Read_Bytes(uint8_t SlaveAddress, uint8_t REG_Address, uint8_t *data, uint8_t len) {

 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, REG_Address, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, SlaveAddress | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK );
	// i2c_master_write_byte(cmd, &REG_data, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		// ESP_LOGI("MPU9250", "IIC Read Byte successfully");
	} else {
		ESP_LOGE("MPU9250", "IIC Read Bytes failed. code: 0x%.2X", espRc);
	}
	// i2c_master_read(cmd, data_rd, RW_TEST_LENGTH, ACK_VAL);
    // i2c_master_read_byte(cmd, data_rd + RW_TEST_LENGTH, NACK_VAL);	
	// i2c_driver_delete(I2C_NUM_1);
	// return REG_data;
}

void mpu9250_i2c_master_init(MPU9250_t * dev, int16_t sda, int16_t scl, int16_t reset)
{
	int i2c_master_port = 1;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = sda,         // select GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = scl,         // select GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 200000,  // select frequency specific to your project
		.clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};
	
	esp_err_t espRc = i2c_param_config(i2c_master_port, &conf);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "MPU9250 i2c_param_config successfully");
	} else {
		ESP_LOGE(tag, "MPU9250 i2c_param_config failed. code: 0x%.2X", espRc);
	}


	espRc = i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "MPU9250 i2c_driver_install successfully");
	} else {
		ESP_LOGE(tag, "MPU9250 i2c_driver_install failed. code: 0x%.2X", espRc);
	}
	if (reset >= 0) {
		gpio_pad_select_gpio(reset);
		gpio_set_direction(reset, GPIO_MODE_OUTPUT);
		gpio_set_level(reset, 0);
		vTaskDelay(50 / portTICK_PERIOD_MS);
		gpio_set_level(reset, 1);
	}
	dev->_address = MPU9250_I2C_ADDR;
	dev->_mag_address = AK8963_I2C_ADDR;
	mpu9250_i2c_init(dev);
}

void mpu9250_i2c_init(MPU9250_t * dev) {
	uint8_t addr = IIC_Read_OneByte(MPU9250_I2C_ADDR, MPU9250_WHO_AM_I);
	ESP_LOGW(tag, "MPU9250 addr: 0x%.2X", addr);

	// i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	// i2c_master_start(cmd);
	// i2c_master_write_byte(cmd, MPU9250_I2C_ADDR, true);
	// /* 唤醒mpu9250                */
	// i2c_master_write_byte(cmd, MPU9250_PWR_MGMT_1, true);
	// i2c_master_write_byte(cmd, 0x00, true);
	// /* 低通滤波5hz				  */
	// i2c_master_write_byte(cmd, MPU9250_CONFIG, true);
	// i2c_master_write_byte(cmd, 0x06, true);
	// /* 不自检，2000deg/s		  */
	// i2c_master_write_byte(cmd, MPU9250_GYRO_CONFIG, true);
	// i2c_master_write_byte(cmd, 0x18, true);
	// //(0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)	
	// i2c_master_write_byte(cmd, MPU9250_ACCEL_CONFIG, true);
	// i2c_master_write_byte(cmd, 0x00, true);
	// //使能I2C
	// i2c_master_write_byte(cmd, MPU9250_USER_CTRL, true);
	// i2c_master_write_byte(cmd, 0x00, true);
	// i2c_master_stop(cmd);
	// esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd);
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(tag, "MPU9250 configured successfully");
	// } else {
	// 	ESP_LOGE(tag, "MPU9250 configuration failed. code: 0x%.2X", espRc);
	// }
	// vTaskDelay(1000 / portTICK_PERIOD_MS);	

	// cmd = i2c_cmd_link_create();
	// i2c_master_start(cmd);
	// i2c_master_write_byte(cmd, AK8963_I2C_ADDR, true);
	// i2c_master_write_byte(cmd, AK8963_CNTL2, true);
	// i2c_master_write_byte(cmd, 0x01, true);
	// i2c_master_stop(cmd);
	// espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd);
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(tag, "AK8963_CNTL2 configured successfully");
	// } else {
	// 	ESP_LOGE(tag, "AK8963_CNTL2 configuration failed. code: 0x%.2X", espRc);
	// }
	// vTaskDelay(50 / portTICK_PERIOD_MS);	

	// cmd = i2c_cmd_link_create();	
	// i2c_master_start(cmd);
	// i2c_master_write_byte(cmd, AK8963_I2C_ADDR, true);
	// i2c_master_write_byte(cmd, AK8963_CNTL1, true);
	// i2c_master_write_byte(cmd, 0x0f, true);
	// i2c_master_stop(cmd);
	// espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd);
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(tag, "AK8963_CNTL1 configured successfully");
	// } else {
	// 	ESP_LOGE(tag, "AK8963_CNTL1 configuration failed. code: 0x%.2X", espRc);
	// }
	// vTaskDelay(500 / portTICK_PERIOD_MS);	
	// //Power-down mode
	// cmd = i2c_cmd_link_create();
	// i2c_master_start(cmd);
	// i2c_master_write_byte(cmd, AK8963_I2C_ADDR, true);
	// i2c_master_write_byte(cmd, AK8963_CNTL1, true);
	// i2c_master_write_byte(cmd, 0x00, true);
	// i2c_master_stop(cmd);
	// espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	// i2c_cmd_link_delete(cmd);
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(tag, "AK8963_CNTL1 configured  Power-down mode successfully");
	// } else {
	// 	ESP_LOGE(tag, "AK8963_CNTL1 configuration Power-down mode failed. code: 0x%.2X", espRc);
	// }
	// vTaskDelay(50 / portTICK_PERIOD_MS);	

	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_PWR_MGMT_1, 0x00); 
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_PWR_MGMT_1);
	ESP_LOGW(tag, "MPU9250_PWR_MGMT_1: 0x00 0x%.2X", addr);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_CONFIG, 0x06); /* 低通滤波5hz				  */
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_CONFIG);
	ESP_LOGW(tag, "MPU9250_CONFIG: 0x06 0x%.2X", addr);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_GYRO_CONFIG, 0x18); /* 不自检，2000deg/s		  */
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_GYRO_CONFIG);
	ESP_LOGW(tag, "MPU9250_GYRO_CONFIG: 0x18 0x%.2X", addr);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_ACCEL_CONFIG, 0x00);//(0x00 +-2g;)  ( 0x08 +-4g;)  (0x10 +-8g;)  (0x18 +-16g)
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_ACCEL_CONFIG);
	ESP_LOGW(tag, "MPU9250_ACCEL_CONFIG: 0x00 0x%.2X", addr);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_INT_PIN_CFG, 0x02);
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_INT_PIN_CFG);
	ESP_LOGW(tag, "MPU9250_INT_PIN_CFG: 0x02 0x%.2X", addr);
	vTaskDelay(10 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(MPU9250_I2C_ADDR, MPU9250_USER_CTRL, 0x00);		//使能I2C
	addr = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_USER_CTRL);
	ESP_LOGW(tag, "MPU9250_USER_CTRL: 0x00 0x%.2X", addr);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(AK8963_I2C_ADDR, AK8963_CNTL2, 0x01);
	addr = IIC_Read_OneByte(AK8963_I2C_ADDR,AK8963_CNTL2);
	ESP_LOGW(tag, "AK8963_CNTL2: 0x01 0x%.2X", addr);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	// os_delay_us(50000);
	IIC_Write_OneByte(AK8963_I2C_ADDR, AK8963_CNTL1, 0x0f);	//Fuse ROM access mode
	addr = IIC_Read_OneByte(AK8963_I2C_ADDR,AK8963_CNTL1);
	ESP_LOGW(tag, "AK8963_CNTL1: 0x0f 0x%.2X", addr);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	// os_delay_us(50000);
	IIC_Get_AK8963MagAdjData(&MagASA[0], &MagASA[1], &MagASA[2]);
	// os_delay_us(50000);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	IIC_Write_OneByte(AK8963_I2C_ADDR, AK8963_CNTL1, 0x00);	//Power-down mode
	// os_delay_us(50000);
	vTaskDelay(50 / portTICK_PERIOD_MS);
	
	IIC_Write_OneByte(AK8963_I2C_ADDR, AK8963_CNTL1, 0x16);		//连续测量模式+16位精度
	addr=0;
	addr = IIC_Read_OneByte(AK8963_I2C_ADDR, AK8963_WIA);
	ESP_LOGW(tag, "AK8963 AK8963_WIA: 0x%.2X", addr);
	addr = IIC_Read_OneByte(AK8963_I2C_ADDR, AK8963_INFO);
	ESP_LOGW(tag, "AK8963 AK8963_INFO: 0x%.2X", addr);


	// espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(tag, "AK8963 configured successfully");
	// } else {
	// 	ESP_LOGE(tag, "AK8963 configuration failed. code: 0x%.2X", espRc);
	// }
}

/********************************************************************************************
 *  描  述  ： 获取传感器0的数据并解算
 *********************************************************************************************/
void GetMPU9250Data() {

	int16_t temp;
	char useMag = 0;
	static char MagTime = 0;
	float tax, tay, taz, tgx, tgy, tgz, tmx, tmy, tmz;
	/*获取加速度计和陀螺仪的数据*/
	IIC_Get_MPU6500Data(&mpu9250.acc_x, &mpu9250.acc_y, &mpu9250.acc_z,
			&mpu9250.gyro_x, &mpu9250.gyro_y, &mpu9250.gyro_z, &mpu9250.temp);

	/*对加速度计和陀螺仪数据进行缩放以及进行坐标轴变换           */
	gx = (float) (M_PI * (mpu9250.gyro_x - plug_param.Gyro_Offset[0]) * GYRO_KEN
			/ 180.00f);
	gy = (float) (M_PI * (mpu9250.gyro_y - plug_param.Gyro_Offset[1]) * GYRO_KEN
			/ 180.00f);
	gz = (float) (M_PI * (mpu9250.gyro_z - plug_param.Gyro_Offset[2]) * GYRO_KEN
			/ 180.00f);

	/* 获取原始的加速度计的数值并按照单位进行缩放*/
	ax = (float) (mpu9250.acc_x * ACC_KEN);
	ay = (float) (mpu9250.acc_y * ACC_KEN);
	az = (float) (mpu9250.acc_z * ACC_KEN);
	// ESP_LOGW(tag, "IIC Get MPU6500 ax %1.4f", ax);
	// ESP_LOGW(tag, "IIC Get MPU6500 ay %1.4f", ay);
	// ESP_LOGW(tag, "IIC Get MPU6500 az %1.4f", az);
	/*默认不使用磁力计*/
	useMag = 0;
	MagTime++;
	/*估计读取了10次后，才去读取磁力计数据*/
	if (MagTime == 2) {
		MagTime = 0; /*到了15次，重新计数*/
		IIC_Get_AK8963MagData(&mpu9250.mag_x, &mpu9250.mag_y, &mpu9250.mag_z,
				&useMag); /* 尝试读取磁力数据,如果成功读取，useMag = 1 */
		if (useMag == 1) {
			mx = mpu9250.mag_x;
			my = mpu9250.mag_y;
			mz = mpu9250.mag_z;
			/*获取原始磁力计数值，*/
			MagCalibrate(&mx, &my, &mz);
			MagFilter(&mx, &my, &mz, 0.75);
		}
	}
	tax = -az;
	tay = -ay;
	taz = -ax;
	tgx = -gz;
	tgy = -gy;
	tgz = -gx;
	tmx = -mz;
	tmy = -my;
	tmz = -mx;

//	tax = - ay;  tay = ax; taz = az;
//	tgx = - gy;  tgy = gx; tgz = gz;
//	tmx = - my;  tmy = mx; tmz = mz;

	AHRSupdate(tgx, tgy, tgz, tax, tay, taz, tmx, tmy, tmz, useMag);
}

void IIC_Get_MPU6500Data(int16_t *ax, int16_t *ay, int16_t *az, int16_t *gx, int16_t *gy, int16_t *gz, int16_t *temp) {
	// uint8_t buf[14];
	int16_t buf[7];
	// uint8_t buf2[15];
	uint8_t buf0[15];
	// uint8_t p = MPU9250_ACCEL_XOUT_H;
	// for(uint8_t i=0;i<14;i++){
	// 	buf2[i] = IIC_Read_OneByte(MPU9250_I2C_ADDR,p+i);
	// }
	// buf2[14] = IIC_Read_OneByte(MPU9250_I2C_ADDR,MPU9250_WHO_AM_I);
	
	// IIC_Read_Bytes(MPU9250_I2C_ADDR, MPU9250_ACCEL_XOUT_H,(uint8_t *)buf,14 )

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU9250_I2C_ADDR, true);
	i2c_master_write_byte(cmd, MPU9250_ACCEL_XOUT_H, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, MPU9250_I2C_ADDR | I2C_MASTER_READ, true);
	i2c_master_read(cmd, (uint8_t *)buf0, 14, I2C_MASTER_LAST_NACK );
	// i2c_master_write_byte(cmd, &REG_data, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 20/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		// ESP_LOGI(tag, "IIC Get MPU6500 Data successfully");
	} else {
		ESP_LOGE(tag, "IIC Get MPU6500 Data failed. code: 0x%.2X", espRc);
	}
	
	// //加速度
	// *ax = (buf[0] << 8) + buf[1];
	// *ay = (buf[2] << 8) + buf[3];
	// *az = (buf[4] << 8) + buf[5];
	// //温度
	// *temp = (buf[6] << 8) + buf[7];
	// //陀螺仪
	// *gx = (buf[8] << 8) + buf[9];
	// *gy = (buf[10] << 8) + buf[11];
	// *gz = (buf[12] << 8) + buf[13];

	// for (int i=0;i<6;i++){
	// 	ESP_LOGW(tag, "IIC Get MPU6500 buf 0x%.2X", buf2[i]);
	// }
	// ESP_LOGI(tag, "IIC Get MPU6500 buf 0x%.4X",  ((int16_t)buf0[0] <<8) | buf0[1]);
	// ESP_LOGI(tag, "IIC Get MPU6500 buf 0x%.4X",  ((int16_t)buf0[2] <<8) | buf0[3]);
	// ESP_LOGI(tag, "IIC Get MPU6500 buf 0x%.4X",  ((int16_t)buf0[4] <<8) | buf0[5]);

	for (int i=0;i<7;i++){
		buf[i] = buf0[2*i];
		buf[i] = (buf[i] <<8) | buf0[2*i+1];
		// ESP_LOGE(tag, "IIC Get MPU6500 buf 0x%.4X", (uint16_t)buf[i]);
	}

	//加速度
	*ax = buf[0];
	*ay = buf[1];
	*az = buf[2];
	//温度
	*temp = buf[3];
	//陀螺仪
	*gx = buf[4];
	*gy = buf[5];
	*gz = buf[6];
	// for (int i=0;i<7;i++){
	// 	ESP_LOGW(tag, "IIC Get MPU6500 buf 0x%.4X", (uint16_t)buf[i]);
	// }
}

void MagCalibrate(float *mx, float *my, float *mz) {
	//static float _mc[6]=	{-101.435211, 223.581635, 152.207245, 0.795118, 0.792904, 0.765773};
//	if (plug_param.Mag_Cali_flag = 0x55) {
//		*mx = (*mx - plug_param.Mag_Offset[0]) * plug_param.Mag_Offset[3];
//		*my = (*my - plug_param.Mag_Offset[1]) * plug_param.Mag_Offset[4];
//		*mz = (*mz - plug_param.Mag_Offset[2]) * plug_param.Mag_Offset[5];
//	}

}

void IIC_Get_AK8963MagData(int16_t *mx, int16_t *my, int16_t *mz, char* usemag) {
	uint8_t buf[8];
	int16_t buf2[3];
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, AK8963_I2C_ADDR, true);
	i2c_master_write_byte(cmd, AK8963_ST1, true);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, AK8963_I2C_ADDR | I2C_MASTER_READ, true);
	i2c_master_read(cmd,(uint8_t *)buf, 8, I2C_MASTER_LAST_NACK );
	// i2c_master_write_byte(cmd, &REG_data, true);
	i2c_master_stop(cmd);
	esp_err_t espRc = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (espRc == ESP_OK) {
		// ESP_LOGI(tag, "IIC Get_AK8963 Mag Data successfully");
	} else {
		//ESP_LOGE(tag, "IIC Get_AK8963 Mag Data failed. code: 0x%.2X", espRc);
	}
	if (((buf[0] & AK8963_ST1_DOR) == 0x00)	|| ((buf[7] & AK8963_ST2_HOFL) == 0)) {
//		*my=((buf[1+1]<<8)|buf[0+1]);		//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13
//		*mx=((buf[3+1]<<8)|buf[2+1]);	
//		*mz=((buf[5+1]<<8)|buf[4+1]);	
		// *my = ((buf[1 + 1] << 8) | buf[0 + 1]) ;//* (((MagASA[0] - 128) >> 8) + 1);//灵敏度纠正 公式见/RM-MPU-9250A-00 PDF/ 5.13
		// *mx = ((buf[3 + 1] << 8) | buf[2 + 1]) ;//* (((MagASA[1] - 128) >> 8) + 1);
		// *mz = ((buf[5 + 1] << 8) | buf[4 + 1]) ;//* (((MagASA[2] - 128) >> 8) + 1);
		//这里读的时候传进去是uin8_t* 所以寄存器数据取出来需要调整一下
		buf2[0]=buf[2];
		buf2[0]=(buf2[0]<<8)|buf[1];
		buf2[1]=buf[4];
		buf2[1]=(buf2[1]<<8)|buf[3];
		buf2[2]=buf[6];
		buf2[2]=(buf2[1]<<8)|buf[5];

		*my = buf2[0];
		*mx = buf2[1] ;
		*mz = buf2[2] ;
		*mz = -*mz;
		*usemag = 1;
	}
}

void MagFilter(float *mx, float *my, float *mz, float alpha) {
	static float Mx = 0, My = 0, Mz = 0;

	if (Mx != 0 && My != 0 && Mz != 0) {
		*mx = Mx * alpha + (*mx) * (1 - alpha);
		*my = My * alpha + (*my) * (1 - alpha);
		*mz = Mz * alpha + (*mz) * (1 - alpha);
		Mx = *mx;
		My = *my;
		Mz = *mz;

	} else {
		Mx = *mx;
		My = *my;
		Mz = *mz;
	}
}

void Init_AHRS() {

	int16_t temp;
	uint8_t i;
	char useMag = 0;
	float tax, tay, taz, tmx, tmy, tmz;

	/*获取加速度计和陀螺仪的数据*/
	IIC_Get_MPU6500Data(&mpu9250.acc_x, &mpu9250.acc_y, &mpu9250.acc_z,
			&mpu9250.gyro_x, &mpu9250.gyro_y, &mpu9250.gyro_z, &temp);

	/*对加速度计和陀螺仪数据进行缩放以及进行坐标轴变换           */
	gx = (float) (M_PI * (mpu9250.gyro_x - plug_param.Gyro_Offset[0]) * GYRO_KEN
			/ 180.00f);
	gy = (float) (M_PI * (mpu9250.gyro_y - plug_param.Gyro_Offset[1]) * GYRO_KEN
			/ 180.00f);
	gz = (float) (M_PI * (mpu9250.gyro_z - plug_param.Gyro_Offset[2]) * GYRO_KEN
			/ 180.00f);

	/* 获取原始的加速度计的数值并按照单位进行缩放*/
	ax = (float) (mpu9250.acc_x * ACC_KEN);
	ay = (float) (mpu9250.acc_y * ACC_KEN);
	az = (float) (mpu9250.acc_z * ACC_KEN);

	/*默认不使用磁力计*/
	while (useMag != 1) {
		// system_soft_wdt_feed();
		useMag = 0;
		IIC_Get_AK8963MagData(&mpu9250.mag_x, &mpu9250.mag_y, &mpu9250.mag_z,
				&useMag); /* 尝试读取磁力数据,如果成功读取，useMag = 1 */
		if (useMag == 1) {
			mx = MagASA[0]*mpu9250.mag_x;
			my = MagASA[1]*mpu9250.mag_y;
			mz = MagASA[2]*mpu9250.mag_z;
			/*获取原始磁力计数值，*/
			MagCalibrate(&mx, &my, &mz);
			MagFilter(&mx, &my, &mz, 0.75);
		}
	}
	tax = -az;
	tay = -ay;
	taz = -ax;
	tmx = -mz;
	tmy = -my;
	tmz = -mx;

	for (i = 0; i < 100; i++) {
		AHRSupdate(0, 0, 0, tax, tay, taz, tmx, tmy, tmz, useMag);
		// system_soft_wdt_feed();
	}
}
// i2cget -c 0x0c -r 0x00 -l 1
// i2cget -c 0x68 -r 0x75 -l 1

// i2cget -c 0x68 -r 0x6a -l 1
// i2cget -c 0x68 -r 0x6b -l 1
// i2cset -c 0x68 -r 0x6b 0x00
// i2cset -c 0x68 -r 0x6a 0x00
// 00001100
// 00011000
// i2cget -c 0x18 -r 0x00 -l 1