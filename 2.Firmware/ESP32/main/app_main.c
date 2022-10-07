/******************** (C) COPYRIGHT 2020 GEEKIMU *******************************
* File Name          : geekimu_main.c
* Current Version    : V1.0  & ESP32
* Author             : zhanli 719901725@qq.com & JustFeng.
* Date of Issued     : 2021.7.20 zhanli : Create
* Comments           : GEEKIMUv3主程序
********************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "HAL/HAL.h"

// ESP网络相关
#include "esp_log.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
// ESP 串口配置相关
#include "freertos/queue.h"
#include "geek_shell_api.h"

// ADC 配置相关
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"

#include "lvgl.h"
#include "lvgl_helpers.h"
#include "System/GeekOS.h"
#include "WIFINetwork.h"
#include "mpu9250.h"

/**********************
 *  LVGL Support.
 **********************/
#define LV_TICK_PERIOD_MS 1
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);

#define UDP_PORT            9000
#define TAG                 "main"

/**-----------------------------------------------------------------------
* Function    : app_main
* Description : GEEKIMU 程序主入口
* Author      : zhanli&719901725@qq.com
* Date        : 2021/7/20 zhanli
*---------------------------------------------------------------------**/
void app_main(void)
{
    /* HAL init config. */
    HAL_Init();
    /* Create LVGL GUI task. */
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*8, NULL, 0, NULL, 1);

    /* Run letter shell cmd. */
    vTaskDelay(1500 / portTICK_PERIOD_MS);
    /* Config serial zero for shell. */
	userShellInit(0);
    xTaskCreate(shellTask, "shell", 4096, getEsp32Shell(), 12, NULL);
    
    // Network_Init();
    // xTaskCreate(Network_UdpSendData, "NetworkSendData", 4096, NULL, 5, NULL);

    /* Forever loop. */
    while(1){
        /* Delay 10ms in order to relase cpu. */
        vTaskDelay(10 / portTICK_PERIOD_MS);
        /* Update HAL task. */
        HAL_Update(lv_tick_get());
    }
}

/* Will be called by the library to read the encoder */
void keypad_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data)
{
    Button_Info_t info;
    Button_GetInfo(&info);
    data->enc_diff = info.btnUp - info.btnDown;
    int16_t isPush = info.btnOK;
    data->state = isPush ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    // 初始化LCD显示驱动
    lvgl_driver_init();

    /* malloc display buffer */
    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    /* rgb color screen recomand use double buff */
    lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf2 != NULL);

    static lv_disp_draw_buf_t disp_buf;
    uint32_t size_in_px = DISP_BUF_SIZE;
    /* Initialize the working buffer depending on the selected display.
     * NOTE: buf2 == NULL when using monochrome displays. */
    lv_disp_draw_buf_init(&disp_buf, buf1, buf2, size_in_px);

    static lv_disp_drv_t disp_drv;         /*A variable to hold the drivers. Must be static or global.*/
    lv_disp_drv_init(&disp_drv);           /*Basic initialization*/
    disp_drv.draw_buf = &disp_buf;         /*Set an initialized buffer*/
    disp_drv.flush_cb = disp_driver_flush; /*Set a flush callback to draw to the display*/
    disp_drv.hor_res = LV_HOR_RES_MAX;     /*Set the horizontal resolution in pixels*/
    disp_drv.ver_res = LV_VER_RES_MAX;     /*Set the vertical resolution in pixels*/
    lv_disp_drv_register(&disp_drv);       /*Register the driver and save the created display objects*/

    static lv_indev_drv_t indev_drv;

    /*Register a encoder input device*/
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_ENCODER;
    indev_drv.read_cb = keypad_read;
    lv_indev_t* indev = lv_indev_drv_register(&indev_drv);

    lv_group_t* group = lv_group_create();
    if (group){
        lv_group_set_default(group);
        lv_indev_set_group(indev, group);
    }
    /* Create and start a periodic timer interrupt to call lv_tick_inc */
    const esp_timer_create_args_t periodic_timer_args = {
        .callback = &lv_tick_task,
        .name = "periodic_gui"
    };
    esp_timer_handle_t periodic_timer;
    ESP_ERROR_CHECK(esp_timer_create(&periodic_timer_args, &periodic_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(periodic_timer, LV_TICK_PERIOD_MS * 1000));

    // 启动GEEK GUI
    GeekOS_Init();

    while (1) {
        /* Delay 1 tick (assumes FreeRTOS tick is 10ms */
        vTaskDelay(pdMS_TO_TICKS(10));
        /* Try to take the semaphore, call lvgl related function on success */
        if (pdTRUE == xSemaphoreTake(xGuiSemaphore, portMAX_DELAY)) {
            lv_task_handler();
            xSemaphoreGive(xGuiSemaphore);
       }
    }
    /* A task should NEVER return */
    free(buf1);
    free(buf2);
    vTaskDelete(NULL);
}

static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
