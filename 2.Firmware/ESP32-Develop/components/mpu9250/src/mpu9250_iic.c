/******************** (C) COPYRIGHT 2020 GEEKIMU *******************************
* File Name          : mpu9250_iic.c
* Current Version    : V2.0  & EPS-IDF v4.3
* Author             : zhanli 719901725@qq.com.
* Date of Issued     : 2021.10.26 zhanli : Create
* Comments           : MPU9250 i2c驱动   
********************************************************************************/

#include "mpu9250_iic.h"
// ESP32 IO操作支持
#include "driver/gpio.h"
// 用于支持i2c延时函数
#include "esp32/rom/ets_sys.h"
#include "driver/i2c.h"
#include "mpu9250.h"
#include "esp_log.h"

static const char *tag = "MPU9250";

#define USE_HARDWARE_I2C 1

#define MPU9250_SDA_PIN  22
#define MPU9250_SCL_PIN  21
#define IIC_DELAY_TIME   2000

#define I2C_MASTER_SCL_IO MPU9250_SCL_PIN           /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO MPU9250_SDA_PIN           /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM 1                        /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */

#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */


// MPU9250的IIC端口初始化
int init_mpu9250_iic_gpio(){
#if USE_HARDWARE_I2C == 0
    gpio_pad_select_gpio(MPU9250_SDA_PIN);
    gpio_set_direction(MPU9250_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(MPU9250_SCL_PIN);
    gpio_set_direction(MPU9250_SCL_PIN, GPIO_MODE_OUTPUT);

    // 设置SDA和SDL的初始电平都为1
	gpio_set_level(MPU9250_SDA_PIN, 1);
    gpio_set_level(MPU9250_SCL_PIN, 1);
#else
    int i2c_master_port = I2C_MASTER_NUM;
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = I2C_MASTER_SDA_IO,         // select GPIO specific to your project
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = I2C_MASTER_SCL_IO,         // select GPIO specific to your project
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 200000,  // select frequency specific to your project
		.clk_flags = 0,          /*!< Optional, you can use I2C_SCLK_SRC_FLAG_* flags to choose i2c source clock here. */
	};

 	esp_err_t ret = i2c_param_config(i2c_master_port, &conf);
	if (ret != ESP_OK) {
		ESP_LOGE(tag, "MPU9250 i2c_param_config failed. code: 0x%.2X", ret);
	}
	ret = i2c_driver_install(i2c_master_port, I2C_MODE_MASTER, 0, 0, 0);
	if (ret != ESP_OK) {
		ESP_LOGE(tag, "MPU9250 i2c_driver_install failed. code: 0x%.2X", ret);
	}
#endif
	return 0;
}

#if USE_HARDWARE_I2C == 0
//模拟IIC延时函数
static void i2c_delay(){
	// 延时1ms
	//ets_delay_us(3);
	uint8_t i = 500;
	while(i--);
}

// 配置SDA输出模式
static void sda_out(){
	gpio_pad_select_gpio(MPU9250_SDA_PIN);
	gpio_set_direction(MPU9250_SDA_PIN, GPIO_MODE_OUTPUT);
}
// 配置SDA输入模式
static void sda_in(){
	gpio_pad_select_gpio(MPU9250_SDA_PIN);
	gpio_set_direction(MPU9250_SDA_PIN, GPIO_MODE_INPUT);
}
// 读取SDA端口状态
static int read_sda(){
	sda_in();
	return gpio_get_level(MPU9250_SDA_PIN);
}
// 设置SDA端口电平
static void iic_sda(int level){
	sda_out();
	gpio_set_level(MPU9250_SDA_PIN, level);
}
// 设置SCL端口电平
static void iic_scl(int level){
	gpio_pad_select_gpio(MPU9250_SCL_PIN);
	gpio_set_direction(MPU9250_SCL_PIN, GPIO_MODE_OUTPUT);
	gpio_set_level(MPU9250_SCL_PIN, level);
}

/**----------------------------------------------------------------------
* Function    : i2c_start
* Description : 产生IIC起始信号
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
void i2c_start(void)
{
	sda_out();   
	iic_sda(1);	  	  
	iic_scl(1);
	i2c_delay();
 	iic_sda(0);
	i2c_delay();
	//钳住I2C总线，准备发送或接收数据 
	iic_scl(0);
}	  

/**----------------------------------------------------------------------
* Function    : i2c_stop
* Description : 产生IIC停止信号
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
void i2c_stop(void)
{
	sda_out();
	iic_scl(0);
	iic_sda(0);
 	i2c_delay();
	iic_scl(1); 
	iic_sda(1);
	i2c_delay();							   	
}

/**----------------------------------------------------------------------
* Function    : i2c_wait_ack
* Description : 等待应答信号到来
* Return      : 1，接收应答失败  0，接收应答成功
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
unsigned char i2c_wait_ack()
{
	unsigned char ucErrTime = 0;
	// SDA设置为输入  
	sda_in();      
	iic_sda(1);i2c_delay();	   
	iic_scl(1);i2c_delay();	 
	while(read_sda())
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{
			i2c_stop();
			return 1;
		}
	}
	// 时钟输出0 
	iic_scl(0);	   
	return 0;  
} 

/**----------------------------------------------------------------------
* Function    : i2c_ack
* Description : 产生ACK应答
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
void i2c_ack(void)
{
	iic_sda(0);
	i2c_delay();
	iic_scl(1);
	i2c_delay();
	iic_scl(0);
	i2c_delay();
	iic_sda(1);
}
/**----------------------------------------------------------------------
* Function    : i2c_no_ack
* Description : 不产生ACK应答		  
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/  
static void i2c_no_ack(void)
{
	iic_sda(1);
	i2c_delay();
	iic_scl(1);
	i2c_delay();
	iic_scl(0);
	i2c_delay();
}					 				     	
/**----------------------------------------------------------------------
* Function    : i2c_send_byte
* Description : IIC发送一个字节	  
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/    
void i2c_send_byte(unsigned char txd)
{                        
    unsigned char t;   
	sda_out(); 	    
	// 拉低时钟开始数据传输
    iic_scl(0);
    for(t = 0;t < 8; t++)
    {              
        // iic_sda=(txd&0x80)>>7;
		if((txd & 0x80) >> 7)
			iic_sda(1);
		else
			iic_sda(0);
		txd<<=1; 	  
		i2c_delay(); 
		iic_scl(1);
		i2c_delay(); 
		iic_scl(0);	
		i2c_delay();
    }	 
} 	    

/**----------------------------------------------------------------------
* Function    : i2c_read_byte
* Description : i2c读1个字节, ack=1时，发送ACK，ack=0，发送nACK  
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/  
unsigned char i2c_read_byte(unsigned char ack)
{
	unsigned char i,receive=0;
	// SDA设置为输入
	sda_in();
    for(i=0;i<8;i++ )
	{
        iic_scl(0); 
        i2c_delay();
		iic_scl(1);
        receive<<=1;
        if(read_sda())receive++;   
		i2c_delay(); 
    }					 
    if (!ack)
        i2c_no_ack();//发送nACK
    else
        i2c_ack(); //发送ACK   
    return receive;
}
#endif
/**----------------------------------------------------------------------
* Function    : i2c_write_one_byte
* Description : i2c写一个字节
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/ 
void i2c_write_one_byte(unsigned char slave_addr,unsigned char reg_addr,unsigned char reg_data)
{
#if USE_HARDWARE_I2C == 0
	// 起始信号
	i2c_start();          
	// 发送设备地址+写信号         
    i2c_send_byte(slave_addr);   
	i2c_wait_ack();	   
	// 内部寄存器地址
    i2c_send_byte(reg_addr);    
	i2c_wait_ack(); 	 			
	// 内部寄存器数据							  		   
    i2c_send_byte(reg_data);       
	i2c_wait_ack(); 	 
	 // 发送停止信号										  		   
    i2c_stop();    
#else               
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_data, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret == ESP_OK) {
		ESP_LOGI("MPU9250", "write dev:0x%.2X reg:0x%.2X data:0x%.2X\r\n", 
			slave_addr,reg_addr, reg_data);
	} else {
		ESP_LOGE("MPU9250", "IIC Write OneByte failed. code: 0x%.2X", ret);
	}
#endif
}

/**----------------------------------------------------------------------
* Function    : i2c_read_one_byte
* Description : i2c读一个字节
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/ 
unsigned char i2c_read_one_byte(unsigned char slave_addr,unsigned char reg_addr)
{
	unsigned char reg_data = 0;
#if USE_HARDWARE_I2C == 0
	// // 起始信号
	i2c_start();           
	// 发送设备地址+写信号      
	i2c_send_byte(slave_addr);  
	reg_data=i2c_wait_ack();	   
	// 发送存储单元地址，从0开始
	i2c_send_byte(reg_addr);   
	// 起始信号
	reg_data=i2c_wait_ack();	   
	i2c_start();                  
	// 发送设备地址+读信号
	i2c_send_byte(slave_addr+1);
	reg_data=i2c_wait_ack();
	if(reg_data == 0){
		ESP_LOGE("MPU9250", "mpu9250 ack");
	}else{
		ESP_LOGE("MPU9250", "mpu9250 no ack");
	}
	// 读取一个字节,不继续再读,发送NAK,读出寄存器数据	   
	reg_data=i2c_read_byte(0);	

	ESP_LOGE("MPU9250", "IIC Read Byte code: 0x%.2X", reg_data);	
	// 停止信号
	i2c_stop();	
#else
 	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read_byte(cmd, &reg_data, I2C_MASTER_LAST_NACK );
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10 / portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		ESP_LOGE("MPU9250", "IIC Read Byte failed. code: 0x%.2X", ret);
	}
#endif
	return reg_data;
}

/**----------------------------------------------------------------------
* Function    : i2c_read_one_byte
* Description : i2c读多个字节
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/ 
void i2c_read_bytes(uint8_t slave_addr,uint8_t reg_addr,
					uint8_t *data, uint8_t len)
{
#if USE_HARDWARE_I2C == 0
	// 起始信号	                         
	i2c_start();
	// 发送设备地址+写信号                          
	i2c_send_byte(slave_addr);       
	i2c_wait_ack();	   
	// 发送存储单元地址，从0开始
	i2c_send_byte(reg_addr);   
	i2c_wait_ack();	
	// 起始信号   
	i2c_start();
	// 发送设备地址+读信号                           
	i2c_send_byte(slave_addr+1);     
	i2c_wait_ack();
	for(uint8_t i = 0;i < len;i++){
		// 读取一个字节,不继续再读,发送NACK  
		if(i==len-1)data[i]=i2c_read_byte(0);     
		else data[i]=i2c_read_byte(1);	      
	}
	// 产生一个停止条件
	i2c_stop();					         
#else
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, slave_addr | I2C_MASTER_READ, ACK_CHECK_EN);
	i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK );
	i2c_master_stop(cmd);
	esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_1, cmd, 10/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
	if (ret != ESP_OK) {
		ESP_LOGE("MPU9250", "IIC Read Bytes failed. code: 0x%.2X\r\n", ret);
	}
#endif
}








