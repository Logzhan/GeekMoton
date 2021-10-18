/*----------------------------------------------------------------------------/

/----------------------------------------------------------------------------*/
#include "driver/gpio.h"
#include "led.h"

#define LED_READY_PIN   2        
#define LED_KNOCK_PIN   13

int init_led_gpio(){

	gpio_pad_select_gpio(LED_READY_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_READY_PIN, GPIO_MODE_OUTPUT);

    // 设置MPU9250和SDA和SDL为低电平
    gpio_pad_select_gpio(LED_KNOCK_PIN);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(LED_KNOCK_PIN, GPIO_MODE_OUTPUT);

	gpio_set_level(LED_READY_PIN, 0);
    gpio_set_level(LED_KNOCK_PIN, 0);

	return 0;
}

// 设置准备指示灯的状态，0低电平，1高电平
void set_ready_led_status(int sta){
	gpio_set_level(LED_READY_PIN, (sta >= 1));
}

// 设置碰撞指示灯的状态，0低电平，1高电平
void set_knock_led_status(int sta){
	gpio_set_level(LED_KNOCK_PIN, (sta >= 1));
}