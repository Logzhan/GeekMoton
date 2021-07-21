/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_system.h"
#include "esp_flash.h"
#include "geek_shell.h"

Shell shell;
char shellBuffer[512];

#define     SHELL_UART      UART_NUM_0
    // signed short (*read)(char *, unsigned short);               /**< shell读函数 */
    // signed short (*write)(char *, unsigned short);              /**< shell写函数 */

/**
 * @brief 用户shell写
 * 
 * @param data 数据
 */
signed short userShellWrite(char* data, unsigned short len)
{
    uart_write_bytes(SHELL_UART, (const char *)data, len);
    return 0;
}
/**
 * @brief 用户shell读
 * 
 * @param data 数据
 * @return char 状态
 */
signed short userShellRead(char *data, unsigned short len){
    return (uart_read_bytes(SHELL_UART, (uint8_t *)data, len, portMAX_DELAY) == 1)
        ? 1 : -1;
}

/**
 * @brief 用户shell初始化
 * 
 */
void userShellInit(void)
{
    uart_config_t uartConfig = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(SHELL_UART, &uartConfig);
    uart_driver_install(SHELL_UART, 256 * 2, 0, 0, NULL, 0);
    shell.write = userShellWrite;
    shell.read = userShellRead;
    shellInit(&shell, shellBuffer, 512);
}


void app_main(void)
{
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), WiFi%s%s, ",
            CONFIG_IDF_TARGET,
            chip_info.cores,
            (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
            (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");

    userShellInit();

    xTaskCreate(shellTask, "shell", 2048, &shell, 12, NULL);

    for (int i = 1000; i >= 0; i--) {
        //printf("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    printf("Restarting now.\n");
    fflush(stdout);
    esp_restart();
}
