/* Blink Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "led.h"
#include "ssd1306.h"
#include "MPU9250.h"
#include "mpu9250_i2c.h"
#include "ahrs.h"
#include "spi_flash_storage.h"


static const char *TAG = "example";
static uint8_t s_led_state = 0;

SSD1306_t oledDev;
MPU9250_t imuDev;

#define SSD1306_SCL_GPIO 13
#define SSD1306_SDA_GPIO 14
static const char *OLED_TAG = "oled_Task";
#define OLED_ADDR 0x3C
#define CONFIG_BLINK_PERIOD 40
#define MPU9250_SCL_GPIO 21
#define MPU9250_SDA_GPIO 22

plug_saved_param plug_param;

void app_main(void)
{
    double Pitch,Yaw,Roll;
    float w,x,y,z;
    int i = 0;
    char s[16];
    // init storage
    // esp_err_t espRc = init_flash_storage();
	// if (espRc == ESP_OK) {
	// 	ESP_LOGI(TAG, "IIC Get Init Flash Storage successfully");
	// } else {
	// 	ESP_LOGE(TAG, "IIC Get Init Flash Storage. code: 0x%.2X", espRc);
	// }
    mpu9250_i2c_master_init(&imuDev, MPU9250_SDA_GPIO, MPU9250_SCL_GPIO, -1);
    Init_AHRS() ;
    /* Configure the peripheral according to the LED type */
    configure_led();
    oled_ssd1306_i2c_master_init(&oledDev, SSD1306_SDA_GPIO, SSD1306_SCL_GPIO, -1);
    oledDev._flip = true;
    oledDev._address = OLED_ADDR;
    ssd1306_init(&oledDev, 128, 32);
    ESP_LOGI(OLED_TAG, "init seccess");
    ssd1306_clear_screen(&oledDev, false);
    ssd1306_contrast(&oledDev, 0xff);
    ssd1306_display_text(&oledDev, 0, "Hello-JustFeng", 14, true);

    ESP_LOGI(TAG, "2 c the LED %s!", s_led_state == true ? "ON" : "OFF");
    while (1)
    {
        i++;
        // ESP_LOGI(TAG, "Turning the LED %s!", s_led_state == true ? "ON" : "OFF");
        blink_led(s_led_state);
        /* Toggle the LED state */
        s_led_state = !s_led_state;
        vTaskDelay(CONFIG_BLINK_PERIOD / portTICK_PERIOD_MS);
        GetMPU9250Data();
        if (i==30){
            i=0;
            w = q0;
            x = q1;
            y = q2;
            z = q3;
            Pitch = asin(-2.0f*(z*x-w*y))* (180.0f /M_PI);
            Yaw   = atan2(y*x + w*z,0.5f - y*y - z*z)* (180.0f /M_PI);
            Roll  = atan2(y*z + w*x,0.5f - y*y - x*x)* (180.0f /M_PI);
            ESP_LOGW(TAG, "Pitch:%f",Pitch);
            ESP_LOGW(TAG, "Yaw:%f",Yaw);
            ESP_LOGW(TAG, "Roll:%f",Roll);
            ssd1306_clear_line(&oledDev, 1, false);            
            sprintf(s, "ax:%6d", mpu9250.acc_x);
            ssd1306_display_text(&oledDev, 1, s, strlen(s), false);
            sprintf(s, "ay:%6d", mpu9250.acc_y);
            ssd1306_display_text(&oledDev, 2, s, strlen(s), false);
            sprintf(s, "az:%6d", mpu9250.acc_z);
            ssd1306_display_text(&oledDev, 3, s, strlen(s), false);
        }

    }
}
