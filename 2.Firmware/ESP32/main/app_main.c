/******************** (C) COPYRIGHT 2020 GEEKIMU *******************************
* File Name          : geekimu_main.c
* Current Version    : V1.0  & ESP32
* Author             : zhanli 719901725@qq.com & JustFeng.
* Date of Issued     : 2021.7.20 zhanli : Create
* Comments           : GEEKIMUv3主程序
********************************************************************************/
#include <stdio.h>
#include <string.h>
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
// 外设驱动相关
#include "led.h"
#include "mpu9250.h"
// ESP 串口配置相关
#include "freertos/queue.h"
#include "geek_shell_api.h"

// ADC 配置相关
#include "sdcard.h"
#include "adc.h"
#include "lcd.h"
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

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling



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



    while(1){
        int level = 1;
        level = gpio_get_level(KEY2);
        if(level == 0){
            printf("you press key2\n");
        }
        level = gpio_get_level(KEY3);
        if(level == 0){
            printf("you press key3\n");
        }
        level = gpio_get_level(KEY1);
        if(level == 0){
            printf("you press key1\n");
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
	// 配置串口0作为命令行输入输出
	userShellInit(0);
    xTaskCreate(shellTask, "shell", 4096, getEsp32Shell(), 12, NULL);
	//配置信息存储
	//init_fatfs();
    // 初始化默认配置
    //init_default_cfg();

    init_sdcard();
	// 初始化LED显示
    ESP_LOGI(TAG, "Init led gpio.\n");
    //init_led_gpio();
	// 启动wifi连接配置
    ESP_LOGI(TAG, "Config wifi sta.\n");
    //xTaskCreate(wifi_init_sta, "wifi task", 4096, NULL, 12, NULL);

	// 初始化MPU9250任务
    ESP_LOGI(TAG, "Init mpu9250.\n");
    // 初始化MPU9250
    
	// 创建udp任务
    ESP_LOGI(TAG, "Create udp task.\n");

    xTaskCreate(lcd_test, "lcd", 4096, NULL, 12, NULL);

    xTaskCreate(key_task, "key_task", 4096, NULL, 12, NULL);

    xTaskCreate(adc_sample_task, "adc_task", 4096, NULL, 12, NULL);

    init_mpu9250();
    //lcd_test();

	static int i = 0;
	
	// MPU9250串口循环发送
    while(1){
        i++;
        if(i > 20){
            //ESP_LOGI(TAG, "%f %f %f\n", yaw, roll, pitch);
            i = 0;
	    }
        GetMPU9250Data_Euler(&yaw,&roll,&pitch);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

int list_files(char *path)
{
	FF_DIR dir; //定义目录对象
	static FILINFO fno; //定义静态文件信息结构对象
	FRESULT res = f_opendir(&dir,path); //打开目录，返回状态 和 目录对象的指针
	if(res == FR_OK) //打开成功
	{
		for(;;) {
			res = f_readdir(&dir, &fno); //读取目录，返回状态 和 文件信息的指针
			if(res != FR_OK || fno.fname[0] == 0){
                break; //若打开失败 或 到结尾，则退出
            } 
			if(fno.fattrib & AM_DIR){
				printf("dir :%s\n",fno.fname);
			}else{
				printf("file:%s\n",fno.fname); //是文件
			}
		}
	}else{
		printf("open %s fail\n", path); //打开失败
	}
	f_closedir(&dir); //关闭目录
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(1), ls, list_files, list files and dir);


int remove_file(char* path)
{
    if(path == NULL)return -1;
    
    FRESULT res = f_unlink(path);
    if(res == FR_OK){
        printf("delete %s sucess!\n", path); //打开失败
    }else{
        printf("delete %s fail!\n", path); //打开失败
    }
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(1), rm, remove_file, delete file and dir);

int reboot(){
    esp_restart();
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), reboot, reboot, reset system);