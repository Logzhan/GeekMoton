#include "lcd_driver.h"
#include "driver/gpio.h"
// 用于支持i2c延时函数
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include <stdlib.h>
#include <string.h>
#include "esp32/rom/ets_sys.h"

spi_device_handle_t spi;

#define LCD_HOST        VSPI_HOST
#define DMA_CHAN        2
#define PARALLEL_LINES  16

#define PIN_NUM_MISO    25
#define PIN_NUM_MOSI    23
#define PIN_NUM_CLK     19
#define PIN_NUM_CS      4
#define PIN_NUM_DC      27
#define PIN_NUM_RST     18
#define PIN_NUM_BCKL    5


/**----------------------------------------------------------------------
* Function    : lcd_spi
* Description : lcd的SPI函数函数,调用ESP32的硬件SPI函数.
                * data     : 传输的数据
				* len      : 数据长度
				* cmd_flag : 1:命令 0:数据
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
static void lcd_spi(spi_device_handle_t spi, const uint8_t *data, int len, int cmd_flag)
{
    esp_err_t ret;
    spi_transaction_t t;
    if (len==0) return;       
    memset(&t, 0, sizeof(t));       
    t.length=len*8;                
    t.tx_buffer=data;               
	if(cmd_flag > 0){
		t.user=(void*)1;
	}else{
		t.user=(void*)0;
	}
    ret=spi_device_polling_transmit(spi, &t);  
    assert(ret==ESP_OK);           
}

/**----------------------------------------------------------------------
* Function    : lcd_write_data_u8
* Description : lcd 写8位数据
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_data_u8(u8 dat){
	lcd_spi(spi, &dat, 1, 1);
}

/**----------------------------------------------------------------------
* Function    : lcd_write_data
* Description : lcd 写16位数据
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_data(u16 dat){
	u8 data[2] = {dat >> 8, dat};
	lcd_spi(spi, data, 2, 1);
}

/**----------------------------------------------------------------------
* Function    : lcd_write_reg
* Description : lcd 写寄存器
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_reg(u8 dat){
	lcd_spi(spi, &dat, 1, 0);
}

/**----------------------------------------------------------------------
* Function    : lcd_address_set
* Description : 设置起始和结束地址
                * x1,x2 设置列的起始和结束地址
                * y1,y2 设置行的起始和结束地址
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_address_set(u16 x1,u16 y1,u16 x2,u16 y2)
{
	if(USE_HORIZONTAL==0)
	{
		lcd_write_reg(0x2a);//列地址设置
		lcd_write_data(x1+52);
		lcd_write_data(x2+52);
		lcd_write_reg(0x2b);//行地址设置
		lcd_write_data(y1+40);
		lcd_write_data(y2+40);
		lcd_write_reg(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==1)
	{
		lcd_write_reg(0x2a);//列地址设置
		lcd_write_data(x1+53);
		lcd_write_data(x2+53);
		lcd_write_reg(0x2b);//行地址设置
		lcd_write_data(y1+40);
		lcd_write_data(y2+40);
		lcd_write_reg(0x2c);//储存器写
	}
	else if(USE_HORIZONTAL==2)
	{
		lcd_write_reg(0x2a);//列地址设置
		lcd_write_data(x1+40);
		lcd_write_data(x2+40);
		lcd_write_reg(0x2b);//行地址设置
		lcd_write_data(y1+53);
		lcd_write_data(y2+53);
		lcd_write_reg(0x2c);//储存器写
	}
	else
	{
		lcd_write_reg(0x2a);//列地址设置
		lcd_write_data(x1+40);
		lcd_write_data(x2+40);
		lcd_write_reg(0x2b);//行地址设置
		lcd_write_data(y1+52);
		lcd_write_data(y2+52);
		lcd_write_reg(0x2c);//储存器写
	}
}

void lcd_spi_pre_transfer_callback(spi_transaction_t *t){
    int dc=(int)t->user;
    gpio_set_level(PIN_NUM_DC, dc);
}

/**----------------------------------------------------------------------
* Function    : lcd_init_spi
* Description : lcd 配置硬件SPI
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_init_spi(void){
	spi_bus_config_t buscfg = {
		.miso_io_num   = PIN_NUM_MISO,
		.mosi_io_num   = PIN_NUM_MOSI,
		.sclk_io_num   = PIN_NUM_CLK,
		.quadwp_io_num = -1,
		.quadhd_io_num = -1,
		.max_transfer_sz = PARALLEL_LINES*320*2+8
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz = 26*1000*1000,                   // 配置SPI速度为26Mhz,可选10Mhz
		.mode           = 0,                              // SPI mode 0
		.spics_io_num   = PIN_NUM_CS,                     // CS pin
		.queue_size     = 7,                              // We want to be able to queue 7 transactions at a time
		.pre_cb         = lcd_spi_pre_transfer_callback,  // Specify pre-transfer callback to handle D/C line
	};

	// 初始化SPI总线
	esp_err_t ret = spi_bus_initialize(LCD_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	// 将LCD连接到总线
	ret = spi_bus_add_device(LCD_HOST, &devcfg, &spi);
}
/**----------------------------------------------------------------------
* Function    : lcd_reset
* Description : lcd 复位
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_reset(){
    // LCD复位
    gpio_set_level(PIN_NUM_RST, 0);
    vTaskDelay(100 / portTICK_RATE_MS);
    gpio_set_level(PIN_NUM_RST, 1);
    vTaskDelay(100 / portTICK_RATE_MS);
}
/**----------------------------------------------------------------------
* Function    : lcd_backlight_ctrl
* Description : lcd 背光控制, 0: 关闭背光 1：开启背光
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_backlight_ctrl(int status){
	gpio_set_level(PIN_NUM_BCKL, status);
}

/**----------------------------------------------------------------------
* Function    : lcd_config_reg
* Description : lcd 配置寄存器,设置lcd的显示参数
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_config_reg(){
	// LCD 初始化code配置
	int cmd = 0;
	while (st_init_cmds[cmd].databytes != 0xff) {
		// 选择寄存器
		lcd_write_reg(st_init_cmds[cmd].cmd);
		// 写数据
		lcd_spi(spi, st_init_cmds[cmd].data, st_init_cmds[cmd].databytes&0x1F, 1);
        if (st_init_cmds[cmd].databytes == 0x80) {
            vTaskDelay(1 / portTICK_RATE_MS);
        }
        cmd++;
    }
}
/**----------------------------------------------------------------------
* Function    : lcd_config_gpio
* Description : GPIO 配置DC、RST、BCKL
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_config_gpio(){
    // GPIO 配置DC、RST、BCKL
    gpio_set_direction(PIN_NUM_DC, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_RST, GPIO_MODE_OUTPUT);
    gpio_set_direction(PIN_NUM_BCKL, GPIO_MODE_OUTPUT);
}

/**----------------------------------------------------------------------
* Function    : lcd_init
* Description : lcd 初始化配置
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_init(void)
{
	// LCD 初始化硬件SPI
	lcd_init_spi();
	// 配置LCD相关操作GPIO
	lcd_config_gpio();
	// LCD复位
	lcd_reset();
	// LCD 初始化code配置
	lcd_config_reg();
	// lcd 开启背光
    lcd_backlight_ctrl(1);
} 








