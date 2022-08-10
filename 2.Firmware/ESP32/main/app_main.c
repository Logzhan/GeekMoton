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
#include "freertos/event_groups.h"
// ESP存储相关
#include "esp_system.h"
#include "esp_vfs.h"
#include "ff.h"
#include "esp_vfs_fat.h"
#include "esp_spi_flash.h"
// ESP网络相关
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include <sys/param.h>
#include "esp_netif.h"
#include "lwip/sockets.h"
#include <lwip/netdb.h>
#include "mpu9250.h"
// ESP 串口配置相关
#include "freertos/queue.h"
#include "geek_shell_api.h"

// ADC 配置相关
#include "sdcard.h"
#include "battery.h"
#include "esp_freertos_hooks.h"
#include "freertos/semphr.h"

#include "driver/gpio.h"

/* Littlevgl specific */
#ifdef LV_LVGL_H_INCLUDE_SIMPLE
#include "lvgl.h"
#else
#include "lvgl/lvgl.h"
#endif


#include "Button.h"
#include "lvgl_helpers.h"
#include "System/GeekOS.h"

// WIFI账号和密码配置
#define CFG_DEV_INDEX       1
#define CFG_WIFI_SSID      "logzhan"          // 配置默认连接的WIFI的SSID
#define CFG_WIFI_PASS      "19931203"         // 配置默认连接的WIFI的密码
#define CFG_MAXIMUM_RETRY   1000              // 配置最大重新连接次数

// UDP广播配置
#define CONFIG_EXAMPLE_IPV4
#define UDP_PORT            9000
#define UDP_PORT_RECV       9001
#define MULTICAST_LOOPBACK  CONFIG_EXAMPLE_LOOPBACK
#define MULTICAST_TTL       255
#define MULTICAST_IPV4_ADDR CONFIG_EXAMPLE_MULTICAST_IPV4_ADDR
#define LISTEN_ALL_IF       EXAMPLE_MULTICAST_LISTEN_ALL_IF


static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;
// Mount path for the partition
const char *base_path = "/spiflash";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG        = "GEEKIMU";
static int        s_retry_num = 0;
static int        dev_idx     = 1;

#define LV_TICK_PERIOD_MS 1

/**********************
 *  STATIC PROTOTYPES
 **********************/
static void lv_tick_task(void *arg);
static void guiTask(void *pvParameter);


float yaw,roll,pitch;

wifi_config_t wifi_config = {
    .sta = {
        .ssid = CFG_WIFI_SSID,
        .password = CFG_WIFI_PASS,
        .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        .pmf_cfg = {
            .capable = true,
            .required = false
        },
    },
};

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < CFG_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            //ESP_LOGI(TAG, "retry to connect to the AP");
            esp32WriteShellEndLine("retry to connect to the AP\r\n");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        //ESP_LOGI(TAG,"connect to the AP fail");
        esp32WriteShellEndLine("connect to the AP fail\r\n");
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}


static void create_multicast_ipv4_socket(void *pvParameters)
{
    //struct sockaddr_in saddr = { 0 };
    struct sockaddr_in raddr = { 0 };
    
    int sock = -1; int sock_in = -1;


    sock_in = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);

    raddr.sin_family = PF_INET;
    raddr.sin_port = htons(UDP_PORT_RECV);
    raddr.sin_addr.s_addr = htonl(IPADDR_ANY);

    char rx_buffer[128];

    int err = bind(sock_in, (struct sockaddr *)&raddr, sizeof(struct sockaddr_in));

    if(err < 0){
        printf("bind socket err\n");
    }

    struct sockaddr_storage source_addr; 
    socklen_t socklen = sizeof(source_addr);

    while (1) {
        int len = recvfrom(sock_in, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
        // Error occurred during receiving
        if (len < 0) {
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
        }
        else {
            ESP_LOGI(TAG, "%s", rx_buffer);
            memset(rx_buffer, 0 , sizeof(rx_buffer));
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    close(sock);
}

static void udp_send_data(void *pvParameters)
{
    struct sockaddr_in saddr = { 0 };

    int sock = -1; 
    int err = 0;

    sock = socket(PF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Failed to create socket. Error %d", errno);
        //return -1;
    }

    saddr.sin_family = PF_INET;
    saddr.sin_port = htons(UDP_PORT);
    saddr.sin_addr.s_addr = htonl(IPADDR_BROADCAST);
    
	char line[128] = " ";
    while (1) {
		sprintf(line, "%f %f %f\n", yaw, roll, pitch);

        err = sendto(sock, line, 256, 0, (struct sockaddr *)&saddr, sizeof(struct sockaddr_in));
    // freeaddrinfo(res);
        if (err < 0) {
            ESP_LOGE(TAG, "IPV4 sendto failed. errno: %d", errno);
            //break;
        }   
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    // All set, socket is configured for sending and receiving
    // return sock;

//err:
    close(sock);
   // return 0;
}

void wifi_init_sta()
{

    s_wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 CFG_WIFI_SSID, CFG_WIFI_PASS);
        xTaskCreate(create_multicast_ipv4_socket, "udp_client", 4096, NULL, 5, NULL);
        xTaskCreate(udp_send_data, "udp_send", 4096, NULL, 5, NULL);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 CFG_WIFI_SSID, CFG_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

int load_esp_cfg(){
    FILE* f = fopen("/spiflash/config.bin", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return -1;
    }
    char line[128];
    int c = 0;
    char idx[10] = {0};
    char ssid[32] = {0};
    char pass[64] = {0};

    const int len = 14;

    while (fgets(line, sizeof(line), f)){
        ESP_LOGI(TAG, "Read from file: %s", line);
        if(c == 0){
            for(int i = len; line[i]!='\n' && line[i] != '\0'; i++){
                idx[i - len] = line[i];
            }
            dev_idx = atoi(idx);
            ESP_LOGI(TAG, "dev idx: %d", dev_idx);
        }
        if(c == 1){
            for(int i = len; line[i]!='\n' && line[i] != '\0'; i++){
                ssid[i - len] = line[i];
            }
            strcpy((char*)(wifi_config.sta.ssid), ssid);
            ESP_LOGI(TAG, "wifi_config.sta.ssid: %s", wifi_config.sta.ssid);
        }
        if(c == 2){
            for(int i = len; line[i]!='\n' && line[i] != '\0'; i++){
                pass[i - len] = line[i];
            }
            //wifi_config
            strcpy((char*)(wifi_config.sta.password), pass);
            ESP_LOGI(TAG, "wifi_config.sta.password: %s",wifi_config.sta.password);
        }
        c++;
    }
    fclose(f);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), load_esp_cfg, load_esp_cfg, load esp cfg);

void init_fatfs(){

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	ESP_LOGI(TAG, "Mounting FAT filesystem");
    // To mount device we need name of device partition, define base_path
    // and allow format partition in case if it is new one and was not formated before
    const esp_vfs_fat_mount_config_t mount_config = {
            .max_files = 4,
            .format_if_mount_failed = true,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };
    // 挂载spi flash为FAT文件系统
    esp_err_t err = esp_vfs_fat_spiflash_mount(base_path, "storage", 
                                               &mount_config, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }

    // Unmount FATFS
    // ESP_LOGI(TAG, "Unmounting FAT filesystem");
    // ESP_ERROR_CHECK( esp_vfs_fat_spiflash_unmount(base_path, s_wl_handle));
}

void init_default_cfg(){
    if(fopen("/spiflash/config.bin", "r") != NULL){
        ESP_LOGI(TAG, "already has config file");
        load_esp_cfg();
        return;
    }

    ESP_LOGI(TAG, "config file no exsit, load default config");

    FILE *f = fopen("/spiflash/config.bin", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "CFG_DEV_INDEX:%d\n", CFG_DEV_INDEX);
    ESP_LOGI(TAG, "CFG_DEV_INDEX:%d\n", CFG_DEV_INDEX);
    fprintf(f, "CFG_WIFI_SSID:%s\n", CFG_WIFI_SSID);
    ESP_LOGI(TAG, "CFG_WIFI_SSID:%s\n", CFG_WIFI_SSID);
    fprintf(f, "CFG_WIFI_PASS:%s\n", CFG_WIFI_PASS);
    ESP_LOGI(TAG, "CFG_WIFI_PASS:%s\n", CFG_WIFI_PASS);
    fclose(f);
}


int read_esp_cfg(){
    FILE* f = fopen("/spiflash/config.bin", "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return -1;
    }
    char line[128];
    while (fgets(line, sizeof(line), f)){
        ESP_LOGI(TAG, "Read from file: %s", line);
    }
    fclose(f);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), read_esp_cfg, read_esp_cfg, read esp cfg);


int write_esp_cfg(int idx, char* ssid, char* psw)
{
    FILE *f = fopen("/spiflash/config.bin", "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return -1;
    }
    fprintf(f, "CFG_DEV_INDEX:%d\n", idx);
    fprintf(f, "CFG_WIFI_SSID:%s\n", ssid);
    fprintf(f, "CFG_WIFI_PASS:%s\n", psw);

    ESP_LOGI(TAG, "dev_index = %d\n", idx);
    ESP_LOGI(TAG, "wifi_ssid = %s\n", ssid);
    ESP_LOGI(TAG, "wifi_pass = %s\n", psw);

    fclose(f);
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(3), write_esp_cfg, write_esp_cfg, write esp cfg);

void key_task(){
    int KEY1 = 25;
    int KEY2 = 32;
    int KEY3 = 33;

    gpio_pad_select_gpio(KEY1);
	gpio_set_direction(KEY1, GPIO_MODE_INPUT);
    gpio_set_pull_mode(KEY1, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(KEY2);
	gpio_set_direction(KEY2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(KEY2, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio(KEY3);
	gpio_set_direction(KEY3, GPIO_MODE_INPUT);
    gpio_set_pull_mode(KEY3, GPIO_PULLUP_ONLY);
    int key_press_flg = 0;
    int c = 50;
    while(1){
        if(key_press_flg > 0){
            key_press_flg--;
        }
        int level = 1;
        level = gpio_get_level(KEY2);
        if(level == 0 && key_press_flg == 0){
            printf("you press key2\n");
            key_press_flg = c;
        }
        level = gpio_get_level(KEY3);
        if(level == 0 && key_press_flg == 0){
            printf("you press key3\n");
            key_press_flg = c;
        }
        level = gpio_get_level(KEY1);
        if(level == 0 && key_press_flg == 0){
            printf("you press key1\n");
            //enter_func_page();
            key_press_flg = c;
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/**------------------------2----------------------------------------------
* Function    : app_main
* Description : GEEKIMU 程序主入口
* Author      : zhanli&719901725@qq.com
* Date        : 2021/7/20 zhanli
*---------------------------------------------------------------------**/
void app_main(void)
{
	//配置信息存储
	//init_fatfs();
    // 初始化默认配置
    // init_default_cfg();
    // init_sdcard();
	// // 初始化LED显示
    // ESP_LOGI(TAG, "Init led gpio.\n");
    // //init_led_gpio();
	// // 启动wifi连接配置
    // ESP_LOGI(TAG, "Config wifi sta.\n");
    //xTaskCreate(wifi_init_sta, "wifi task", 4096, NULL, 12, NULL);

    button_init();
    xTaskCreate(Button_Update, "button_task", 2048 * 4, NULL, 12, NULL);

    //xTaskCreate(key_task, "key_task", 2048, NULL, 12, NULL);

    xTaskCreate(battery_sample_task, "adc_task", 4096, NULL, 12, NULL);
    // // 初始化MPU9250任务
    // ESP_LOGI(TAG, "Init mpu9250.\n");
    // init_mpu9250();
    
    // // 配置串口0作为命令行输入输出
    // vTaskDelay(100 / portTICK_PERIOD_MS);
	// userShellInit(0);
    // // 启动命令行
    // xTaskCreate(shellTask, "shell", 4096, getEsp32Shell(), 12, NULL);
    // 启动图形GUI
    xTaskCreatePinnedToCore(guiTask, "gui", 4096*8, NULL, 0, NULL, 1);
	// MPU9250串口循环发送
    while(1){
        //GetMPU9250Data_Euler(&yaw,&roll,&pitch);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
void keypad_init(void)
{
    /*Your code comes here*/
}

/* Will be called by the library to read the encoder */
void keypad_read(lv_indev_drv_t* indev_drv, lv_indev_data_t* data)
{
    int ok = 0;
    int up = 0;
    int dn = 0;

    getKeyPadState(&ok, &up, &dn);

    data->enc_diff = up - dn;

    if(data->enc_diff != 0){
        printf("%d\n",data->enc_diff);
    }

    int16_t isPush = ok;

    data->state = isPush ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;


}

int reboot(){
    esp_restart();
    return 0;
}

SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), reboot, reboot, reset system);

/* Creates a semaphore to handle concurrent call to lvgl stuff
 * If you wish to call *any* lvgl function from other threads/tasks
 * you should lock on the very same semaphore! */
SemaphoreHandle_t xGuiSemaphore;

//#define DISP_BUF_SIZE  (240*135)

static void guiTask(void *pvParameter) {

    (void) pvParameter;
    xGuiSemaphore = xSemaphoreCreateMutex();

    lv_init();
    // 初始化LCD显示驱动
    lvgl_driver_init();
    keypad_init();

    lv_color_t* buf1 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    assert(buf1 != NULL);
    // 对于非单色显示器需要使用双BUFF
    // lv_color_t* buf2 = heap_caps_malloc(DISP_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA);
    // assert(buf2 != NULL);

    lv_color_t* buf2 = NULL;

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
    if (!lv_group_get_default())
    {
        group = lv_group_create();
        if (group){
            lv_group_set_default(group);
        }
    }
    lv_indev_set_group(indev, group);


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
#ifndef CONFIG_LV_TFT_DISPLAY_MONOCHROME
    free(buf2);
#endif
    vTaskDelete(NULL);
}

static void lv_tick_task(void *arg) {
    (void) arg;
    lv_tick_inc(LV_TICK_PERIOD_MS);
}
