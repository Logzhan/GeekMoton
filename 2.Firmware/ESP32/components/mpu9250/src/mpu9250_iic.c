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

#define LED              2
#define MPU9250_SDA_PIN  22
#define MPU9250_SCL_PIN  21
#define IIC_DELAY_TIME   2000

static void i2c_delay(void);

// MPU9250的IIC端口初始化
int init_mpu9250_iic_gpio(){
    // 设置MPU9250和SDA和SDL输出模式且低电平
    gpio_pad_select_gpio(MPU9250_SDA_PIN);
    gpio_set_direction(MPU9250_SDA_PIN, GPIO_MODE_OUTPUT);
    gpio_pad_select_gpio(MPU9250_SCL_PIN);
    gpio_set_direction(MPU9250_SCL_PIN, GPIO_MODE_OUTPUT);

    // 设置SDA和SDL的初始电平都为0
    gpio_set_level(MPU9250_SDA_PIN, 1);
    gpio_set_level(MPU9250_SCL_PIN, 1);

	return 0;
}

// 模拟IIC延时函数
static void i2c_delay(){
	// 延时1ms
	ets_delay_us(1);
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
	return gpio_get_level(MPU9250_SDA_PIN);
}
// 设置SDA端口电平
static void iic_sda(int level){
	gpio_set_level(MPU9250_SDA_PIN, level);
}
// 设置SCL端口电平
static void iic_scl(int level){
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
	iic_scl(0);
	sda_out();
	iic_sda(0);
	i2c_delay();
	iic_scl(1);
	i2c_delay();
	iic_scl(0);
}
/**----------------------------------------------------------------------
* Function    : i2c_no_ack
* Description : 不产生ACK应答		  
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/  
static void i2c_no_ack(void)
{
	iic_scl(0);
	sda_out();
	iic_sda(1);
	i2c_delay();
	iic_scl(1);
	i2c_delay();
	iic_scl(0);
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

/**----------------------------------------------------------------------
* Function    : i2c_write_one_byte
* Description : i2c写一个字节
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/ 
void i2c_write_one_byte(unsigned char slave_addr,unsigned char reg_addr,unsigned char reg_data)
{
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
}

/**----------------------------------------------------------------------
* Function    : i2c_read_one_byte
* Description : i2c读一个字节
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/ 
unsigned char i2c_read_one_byte(unsigned char slave_addr,unsigned char reg_addr)
{
	unsigned char reg_data;
	// 起始信号
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
	// 读取一个字节,不继续再读,发送NAK,读出寄存器数据	   
	reg_data=i2c_read_byte(0);		
	// 停止信号
	i2c_stop();	                  
	return reg_data;
}











