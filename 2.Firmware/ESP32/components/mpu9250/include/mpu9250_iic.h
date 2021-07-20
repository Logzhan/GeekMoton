/*----------------------------------------------------------------------------/
/ MPU9250 IIC include file            
/----------------------------------------------------------------------------*/
#ifndef _DEF_MPU9250_IIC_H_
#define _DEF_MPU9250_IIC_H_
/*---------------------------------------------------------------------------*/
void i2c_start(void);

void i2c_stop(void);

void i2c_ack(void);

void i2c_send_byte(unsigned char txd);

unsigned char i2c_read_byte(unsigned char ack);

unsigned char i2c_wait_ack();

/* System Configurations */
int init_mpu9250_iic_gpio(void);

// 往I2C设备写一个字节
void i2c_write_one_byte(unsigned char slave_addr,unsigned char reg_addr,unsigned char reg_data);

//从I2C设备读取一个字节数据
unsigned char i2c_read_one_byte(unsigned char slave_addr,unsigned char reg_addr);

#endif /* MPU9250 */
