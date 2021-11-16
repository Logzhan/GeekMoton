#ifndef __MPU9250_H
#define __MPU9250_H
#include "stdint.h"

#define M_PI 3.14159265358979323846F	/*定义pi*/

#define GYRO_KEN 0.06103515625
#define ACC_KEN 0.00006103515625


/* ---- MPU9250 Reg In MPU9250 ---------------------------------------------- */

#define MPU9250_I2C_ADDR            ((uint8_t)0xD0)
#define MPU9250_Device_ID           ((uint8_t)0x71)  // In MPU9250

#define MPU9250_SELF_TEST_XG        ((uint8_t)0x00)
#define MPU9250_SELF_TEST_YG        ((uint8_t)0x01)
#define MPU9250_SELF_TEST_ZG        ((uint8_t)0x02)
#define MPU9250_SELF_TEST_XA        ((uint8_t)0x0D)
#define MPU9250_SELF_TEST_YA        ((uint8_t)0x0E)
#define MPU9250_SELF_TEST_ZA        ((uint8_t)0x0F)
#define MPU9250_XG_OFFSET_H         ((uint8_t)0x13)
#define MPU9250_XG_OFFSET_L         ((uint8_t)0x14)
#define MPU9250_YG_OFFSET_H         ((uint8_t)0x15)
#define MPU9250_YG_OFFSET_L         ((uint8_t)0x16)
#define MPU9250_ZG_OFFSET_H         ((uint8_t)0x17)
#define MPU9250_ZG_OFFSET_L         ((uint8_t)0x18)
#define MPU9250_SMPLRT_DIV          ((uint8_t)0x19)
#define MPU9250_CONFIG              ((uint8_t)0x1A)
#define MPU9250_GYRO_CONFIG         ((uint8_t)0x1B)
#define MPU9250_ACCEL_CONFIG        ((uint8_t)0x1C)
#define MPU9250_ACCEL_CONFIG_2      ((uint8_t)0x1D)
#define MPU9250_LP_ACCEL_ODR        ((uint8_t)0x1E)
#define MPU9250_MOT_THR             ((uint8_t)0x1F)
#define MPU9250_FIFO_EN             ((uint8_t)0x23)
#define MPU9250_I2C_MST_CTRL        ((uint8_t)0x24)
#define MPU9250_I2C_SLV0_ADDR       ((uint8_t)0x25)
#define MPU9250_I2C_SLV0_REG        ((uint8_t)0x26)
#define MPU9250_I2C_SLV0_CTRL       ((uint8_t)0x27)
#define MPU9250_I2C_SLV1_ADDR       ((uint8_t)0x28)
#define MPU9250_I2C_SLV1_REG        ((uint8_t)0x29)
#define MPU9250_I2C_SLV1_CTRL       ((uint8_t)0x2A)
#define MPU9250_I2C_SLV2_ADDR       ((uint8_t)0x2B)
#define MPU9250_I2C_SLV2_REG        ((uint8_t)0x2C)
#define MPU9250_I2C_SLV2_CTRL       ((uint8_t)0x2D)
#define MPU9250_I2C_SLV3_ADDR       ((uint8_t)0x2E)
#define MPU9250_I2C_SLV3_REG        ((uint8_t)0x2F)
#define MPU9250_I2C_SLV3_CTRL       ((uint8_t)0x30)
#define MPU9250_I2C_SLV4_ADDR       ((uint8_t)0x31)
#define MPU9250_I2C_SLV4_REG        ((uint8_t)0x32)
#define MPU9250_I2C_SLV4_DO         ((uint8_t)0x33)
#define MPU9250_I2C_SLV4_CTRL       ((uint8_t)0x34)
#define MPU9250_I2C_SLV4_DI         ((uint8_t)0x35)
#define MPU9250_I2C_MST_STATUS      ((uint8_t)0x36)
#define MPU9250_INT_PIN_CFG         ((uint8_t)0x37)
#define MPU9250_INT_ENABLE          ((uint8_t)0x38)
#define MPU9250_INT_STATUS          ((uint8_t)0x3A)
#define MPU9250_ACCEL_XOUT_H        ((uint8_t)0x3B)
#define MPU9250_ACCEL_XOUT_L        ((uint8_t)0x3C)
#define MPU9250_ACCEL_YOUT_H        ((uint8_t)0x3D)
#define MPU9250_ACCEL_YOUT_L        ((uint8_t)0x3E)
#define MPU9250_ACCEL_ZOUT_H        ((uint8_t)0x3F)
#define MPU9250_ACCEL_ZOUT_L        ((uint8_t)0x40)
#define MPU9250_TEMP_OUT_H          ((uint8_t)0x41)
#define MPU9250_TEMP_OUT_L          ((uint8_t)0x42)
#define MPU9250_GYRO_XOUT_H         ((uint8_t)0x43)
#define MPU9250_GYRO_XOUT_L         ((uint8_t)0x44)
#define MPU9250_GYRO_YOUT_H         ((uint8_t)0x45)
#define MPU9250_GYRO_YOUT_L         ((uint8_t)0x46)
#define MPU9250_GYRO_ZOUT_H         ((uint8_t)0x47)
#define MPU9250_GYRO_ZOUT_L         ((uint8_t)0x48)
#define MPU9250_EXT_SENS_DATA_00    ((uint8_t)0x49)
#define MPU9250_EXT_SENS_DATA_01    ((uint8_t)0x4A)
#define MPU9250_EXT_SENS_DATA_02    ((uint8_t)0x4B)
#define MPU9250_EXT_SENS_DATA_03    ((uint8_t)0x4C)
#define MPU9250_EXT_SENS_DATA_04    ((uint8_t)0x4D)
#define MPU9250_EXT_SENS_DATA_05    ((uint8_t)0x4E)
#define MPU9250_EXT_SENS_DATA_06    ((uint8_t)0x4F)
#define MPU9250_EXT_SENS_DATA_07    ((uint8_t)0x50)
#define MPU9250_EXT_SENS_DATA_08    ((uint8_t)0x51)
#define MPU9250_EXT_SENS_DATA_09    ((uint8_t)0x52)
#define MPU9250_EXT_SENS_DATA_10    ((uint8_t)0x53)
#define MPU9250_EXT_SENS_DATA_11    ((uint8_t)0x54)
#define MPU9250_EXT_SENS_DATA_12    ((uint8_t)0x55)
#define MPU9250_EXT_SENS_DATA_13    ((uint8_t)0x56)
#define MPU9250_EXT_SENS_DATA_14    ((uint8_t)0x57)
#define MPU9250_EXT_SENS_DATA_15    ((uint8_t)0x58)
#define MPU9250_EXT_SENS_DATA_16    ((uint8_t)0x59)
#define MPU9250_EXT_SENS_DATA_17    ((uint8_t)0x5A)
#define MPU9250_EXT_SENS_DATA_18    ((uint8_t)0x5B)
#define MPU9250_EXT_SENS_DATA_19    ((uint8_t)0x5C)
#define MPU9250_EXT_SENS_DATA_20    ((uint8_t)0x5D)
#define MPU9250_EXT_SENS_DATA_21    ((uint8_t)0x5E)
#define MPU9250_EXT_SENS_DATA_22    ((uint8_t)0x5F)
#define MPU9250_EXT_SENS_DATA_23    ((uint8_t)0x60)
#define MPU9250_I2C_SLV0_DO         ((uint8_t)0x63)
#define MPU9250_I2C_SLV1_DO         ((uint8_t)0x64)
#define MPU9250_I2C_SLV2_DO         ((uint8_t)0x65)
#define MPU9250_I2C_SLV3_DO         ((uint8_t)0x66)
#define MPU9250_I2C_MST_DELAY_CTRL  ((uint8_t)0x67)
#define MPU9250_SIGNAL_PATH_RESET   ((uint8_t)0x68)
#define MPU9250_MOT_DETECT_CTRL     ((uint8_t)0x69)
#define MPU9250_USER_CTRL           ((uint8_t)0x6A)
#define MPU9250_PWR_MGMT_1          ((uint8_t)0x6B)
#define MPU9250_PWR_MGMT_2          ((uint8_t)0x6C)
#define MPU9250_FIFO_COUNTH         ((uint8_t)0x72)
#define MPU9250_FIFO_COUNTL         ((uint8_t)0x73)
#define MPU9250_FIFO_R_W            ((uint8_t)0x74)
#define MPU9250_WHO_AM_I            ((uint8_t)0x75)	// ID = 0x71 In MPU9250
#define MPU9250_XA_OFFSET_H         ((uint8_t)0x77)
#define MPU9250_XA_OFFSET_L         ((uint8_t)0x78)
#define MPU9250_YA_OFFSET_H         ((uint8_t)0x7A)
#define MPU9250_YA_OFFSET_L         ((uint8_t)0x7B)
#define MPU9250_ZA_OFFSET_H         ((uint8_t)0x7D)
#define MPU9250_ZA_OFFSET_L         ((uint8_t)0x7E)

/* ---- AK8963 Reg In MPU9250 ----------------------------------------------- */

#define AK8963_I2C_ADDR             ((uint8_t)0x18)
#define AK8963_Device_ID            ((uint8_t)0x48)

// Read-only Reg
#define AK8963_WIA                  ((uint8_t)0x00)
#define AK8963_INFO                 ((uint8_t)0x01)
#define AK8963_ST1                  ((uint8_t)0x02)
#define AK8963_ST1_DOR                   0x02
#define AK8963_ST1_DRDY                  0x01 //Data Ready
#define AK8963_HXL                  ((uint8_t)0x03)
#define AK8963_HXH                  ((uint8_t)0x04)
#define AK8963_HYL                  ((uint8_t)0x05)
#define AK8963_HYH                  ((uint8_t)0x06)
#define AK8963_HZL                  ((uint8_t)0x07)
#define AK8963_HZH                  ((uint8_t)0x08)
#define AK8963_ST2                  ((uint8_t)0x09)
#define AK8963_ST2_HOFL             ((uint8_t)0x08)
// Write/Read Reg
#define AK8963_CNTL1                ((uint8_t)0x0A)
#define AK8963_CNTL2                ((uint8_t)0x0B)
#define AK8963_ASTC                 ((uint8_t)0x0C)
#define AK8963_TS1                  ((uint8_t)0x0D)
#define AK8963_TS2                  ((uint8_t)0x0E)
#define AK8963_I2CDIS               ((uint8_t)0x0F)
// Read-only Reg ( ROM )
#define AK8963_ASAX                 ((uint8_t)0x10)
#define AK8963_ASAY                 ((uint8_t)0x11)
#define AK8963_ASAZ                 ((uint8_t)0x12)
typedef struct MPU
{
	short acc_x;
	short acc_y;
	short acc_z;
	short temp;
	short gyro_x;
	short gyro_y;
	short gyro_z;
	short mag_x;
	short mag_y;
	short mag_z;
}MPU;
extern float ax, ay, az, gx, gy, gz, mx, my, mz;
extern int16_t MPU_temp;
extern MPU mpu9250;

typedef struct MPU9250_t{
	int _address;
	int _mag_address;
} MPU9250_t;
typedef struct plug_saved_param {
    uint8_t status;
    uint8_t pad[3];
    uint8_t Gyro_Cali_flag;
    uint8_t Mag_Cali_flag;
    float Gyro_Offset[3];
    float Mag_Offset[6];
    uint8_t part;

}plug_saved_param;
extern plug_saved_param plug_param;

void IIC_Get_MPU6500Data(short *ax,short *ay,short *az,short *gx,short *gy,short *gz,short *temp);
void IIC_MPU9250_init();
void GetMPU9250Data();
void Init_AHRS();
void MagCalibrate(float *mx,float *my,float *mz);
void IIC_Get_AK8963MagData(short* mx,short* my ,short *mz,char * usemag);
void MagFilter(float *mx,float *my,float *mz,float alpha);

#endif

