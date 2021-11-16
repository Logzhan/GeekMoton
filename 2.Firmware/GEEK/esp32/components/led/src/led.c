#include <stdio.h>
#include "sdkconfig.h"
#include "led.h"
#include "driver/gpio.h"
#include "esp_log.h"
#define BLINK_GPIO CONFIG_BLINK_GPIO
/* Use project configuration menu (idf.py menuconfig) to choose the GPIO to blink,
   or you can edit the following line and set a number here.
*/

//static uint8_t s_led_state = 0;
static const char *TAG = "example/led";


void blink_led(uint8_t s_led_state)
{
    /* Set the GPIO level according to the state (LOW or HIGH)*/
    gpio_set_level(BLINK_GPIO, s_led_state);
}

void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink GPIO LED!");
    gpio_reset_pin(BLINK_GPIO);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
}
