/*----------------------------------------------------------------------------/
/ LED include file               (C)ChaN, 2019
/----------------------------------------------------------------------------*/
#ifndef DEF_LED_H_
#define DEF_LED_H_
/*---------------------------------------------------------------------------*/
/* System Configurations */
int init_led_gpio(void);

// 设置准备指示灯的状态，0低电平，1高电平
void set_ready_led_status(int sta);

// 设置碰撞指示灯的状态，0低电平，1高电平
void set_knock_led_status(int sta);

// 设置状态灯的状态，0低电平，1高电平
void set_status_led_status(int sta);

#endif /* LED */
