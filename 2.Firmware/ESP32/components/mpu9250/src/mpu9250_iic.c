/*----------------------------------------------------------------------------/
* Description  : iic diriver for esp32
/----------------------------------------------------------------------------*/

#include "mpu9250_iic.h"
#include "driver/gpio.h"
// 用于支持i2c延时函数
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp32/rom/ets_sys.h"

#define LED              2
#define MPU9250_SDA_PIN  22
#define MPU9250_SCL_PIN  21
#define IIC_DELAY_TIME   2000

static void i2c_delay(void);

// MPU9250的IIC端口初始化
int init_mpu9250_iic_gpio(){
    // 设置MPU9250和SDA和SDL为低电平
    gpio_pad_select_gpio(MPU9250_SDA_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(MPU9250_SDA_PIN, GPIO_MODE_OUTPUT);

    // 设置MPU9250和SDA和SDL为低电平
    gpio_pad_select_gpio(MPU9250_SCL_PIN);
    /* Set the GPIO as a push/pull output */
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


//产生IIC起始信号
void i2c_start(void)
{
	sda_out();     //sda线输出
	iic_sda(1);	  	  
	iic_scl(1);
	i2c_delay();
 	iic_sda(0);//START:when CLK is high,DATA change form high to low 
	i2c_delay();
	iic_scl(0);//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void i2c_stop(void)
{
	sda_out();//sda线输出
	iic_scl(0);
	iic_sda(0);//STOP:when CLK is high DATA change form low to high
 	i2c_delay();
	iic_scl(1); 
	iic_sda(1);//发送I2C总线结束信号
	i2c_delay();							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
unsigned char i2c_wait_ack()
{
	unsigned char ucErrTime = 0;
	sda_in();      //SDA设置为输入  
	iic_sda(1);i2c_delay();	   
	iic_scl(1);i2c_delay();	 
	while(read_sda())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			i2c_stop();
			return 1;
		}
	}
	iic_scl(0);//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
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
//不产生ACK应答		    
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
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void i2c_send_byte(unsigned char txd)
{                        
    unsigned char t;   
	sda_out(); 	    
    iic_scl(0);//拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {              
        //iic_sda=(txd&0x80)>>7;
		if((txd&0x80)>>7)
			iic_sda(1);
		else
			iic_sda(0);
		txd<<=1; 	  
		i2c_delay();   //对TEA5767这三个延时都是必须的
		iic_scl(1);
		i2c_delay(); 
		iic_scl(0);	
		i2c_delay();
    }	 
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
unsigned char i2c_read_byte(unsigned char ack)
{
	unsigned char i,receive=0;
	sda_in();//SDA设置为输入
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

void i2c_write_one_byte(unsigned char slave_addr,unsigned char reg_addr,unsigned char reg_data)
{
	i2c_start();                   //起始信号
    i2c_send_byte(slave_addr);   //发送设备地址+写信号
	i2c_wait_ack();	   
    i2c_send_byte(reg_addr);    //内部寄存器地址
	i2c_wait_ack(); 	 										  		   
    i2c_send_byte(reg_data);       //内部寄存器数据
	i2c_wait_ack(); 	 										  		   
    i2c_stop();                    //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
unsigned char i2c_read_one_byte(unsigned char slave_addr,unsigned char reg_addr)
{
	unsigned char reg_data;
	i2c_start();                  //起始信号
	i2c_send_byte(slave_addr);  //发送设备地址+写信号
	reg_data=i2c_wait_ack();	   
	i2c_send_byte(reg_addr);   //发送存储单元地址，从0开始
	reg_data=i2c_wait_ack();	   
	i2c_start();                  //起始信号
	i2c_send_byte(slave_addr+1);//发送设备地址+读信号
	reg_data=i2c_wait_ack();	   
	reg_data=i2c_read_byte(0);		//读取一个字节,不继续再读,发送NAK,读出寄存器数据
	i2c_stop();	                  //停止信号
	return reg_data;
}











