#ifndef __LCD_DRIVER_H
#define __LCD_DRIVER_H

#include <stdint.h>

#define USE_HORIZONTAL 2  

#if USE_HORIZONTAL==0||USE_HORIZONTAL==1
    #define LCD_W 135
    #define LCD_H 240
#else
    #define LCD_W 240
    #define LCD_H 135
#endif

#define u8  unsigned char
#define u16 unsigned int

typedef struct {
    uint8_t cmd;
    uint8_t data[16];
    uint8_t databytes; //No of data in data; bit 7 = delay after set; 0xFF = end of cmds.
} lcd_init_cmd_t;

static const lcd_init_cmd_t st_init_cmds[]={
    {0x11, {0x00}, 0x80},
    /* Memory Data Access Control, MX=MV=1, MY=ML=MH=0, RGB=0 */
    {0x36, {0x70}, 1},
    /* Interface Pixel Format, 16bits/pixel for RGB/MCU interface */
    {0x3A, {0x05}, 1},
    /* Porch Setting */
    {0xB2, {0x0c, 0x0c, 0x00, 0x33, 0x33}, 5},
    /* Gate Control, Vgh=13.65V, Vgl=-10.43V */
    {0xB7, {0x35}, 1},
    /* VCOM Setting, VCOM=1.175V */
    {0xBB, {0x19}, 1},
    /* LCM Control, XOR: BGR, MX, MH */
    {0xC0, {0x2C}, 1},
    /* VDV and VRH Command Enable, enable=1 */
    {0xC2, {0x01}, 1},
    /* VRH Set, Vap=4.4+... */
    {0xC3, {0x12}, 1},
    /* VDV Set, VDV=0 */
    {0xC4, {0x20}, 1},
    /* Frame Rate Control, 60Hz, inversion=0 */
    {0xC6, {0x0f}, 1},
    /* Power Control 1, AVDD=6.8V, AVCL=-4.8V, VDDS=2.3V */
    {0xD0, {0xA4, 0xA1}, 2},
    /* Positive Voltage Gamma Control */
    {0xE0, {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}, 14},
    /* Negative Voltage Gamma Control */
    {0xE1, {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}, 14},
    /* Sleep Out */
    {0x21, {0}, 0x80},
    /* Display On */
    {0x29, {0}, 0x80},
    /* Init Finish */
    {0x00, {0}, 0xff}
};

void lcd_gpio_init(void);
/**----------------------------------------------------------------------
* Function    : lcd_write_data_u8
* Description : lcd 写8位数据
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_data_u8(u8 dat);
/**----------------------------------------------------------------------
* Function    : lcd_write_data
* Description : lcd 写16位数据
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_data(u16 dat);
/**----------------------------------------------------------------------
* Function    : lcd_write_reg
* Description : lcd 写寄存器
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_write_reg(u8 dat);
/**----------------------------------------------------------------------
* Function    : lcd_address_set
* Description : 设置起始和结束地址
                * x1,x2 设置列的起始和结束地址
                * y1,y2 设置行的起始和结束地址
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_address_set(u16 x1,u16 y1,u16 x2,u16 y2);
/**----------------------------------------------------------------------
* Function    : lcd_init
* Description : lcd 初始化配置
* Author      : zhanli&719901725@qq.com
* Date        : 2021/11/15 zhanli
*---------------------------------------------------------------------**/
void lcd_init(void);
#endif




