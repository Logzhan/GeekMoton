#ifndef __MPU9250_I2C_H
#define __MPU9250_I2C_H
#include "driver/i2c.h"
#include "stdint.h"
// typedef uint64_t u64;
// typedef uint32_t u32;
// typedef uint16_t u16;
// typedef uint8_t u8;
// typedef int64_t s64;
// typedef int32_t s32;
// typedef int16_t s16;
// typedef int8_t s8;
#include "esp_log.h"
void mpu9250_i2c_master_init(MPU9250_t * dev, int16_t sda, int16_t scl, int16_t reset);
void mpu9250_i2c_init(MPU9250_t * dev);
#define ACK_CHECK_EN   0x1     /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS  0x0     /*!< I2C master will not check ack from slave */
#define ACK_VAL    0x0         /*!< I2C ack value */
#define NACK_VAL   0x1         /*!< I2C nack value */
#endif
