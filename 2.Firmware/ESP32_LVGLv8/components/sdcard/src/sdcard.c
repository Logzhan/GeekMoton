/******************** (C) COPYRIGHT 2020 GEEKIMU *******************************
* File Name          : sdcard.c
* Current Version    : V1.0  & ESP32-IDF 4.3.0
* Author             : zhanli 719901725@qq.com
* Date of Issued     : 2021.10.21 zhanli : Create
* Comments           : ESP32芯片SDCARD模块驱动
********************************************************************************/
#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"
#include "driver/sdmmc_host.h"
#include "sdcard.h"
#include "pic.h"
#include "geek_shell_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ff.h"

static const char *TAG = "example";

#define MOUNT_POINT "/sdcard"

#define SPI_DMA_CHAN    2
#define PIN_NUM_MISO    2
#define PIN_NUM_MOSI    15
#define PIN_NUM_CLK     14               // 
#define PIN_NUM_CS      13               // SDCARD的片选CS端口


esp_vfs_fat_sdmmc_mount_config_t mount_config = {
#ifdef CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED
    .format_if_mount_failed = true,
#else
    .format_if_mount_failed = false,
#endif // EXAMPLE_FORMAT_IF_MOUNT_FAILED
    .max_files = 5,
    .allocation_unit_size = 16 * 1024
};
sdmmc_card_t* card;

sdmmc_host_t host = SDSPI_HOST_DEFAULT();
spi_bus_config_t bus_cfg = {
    .mosi_io_num = PIN_NUM_MOSI,
    .miso_io_num = PIN_NUM_MISO,
    .sclk_io_num = PIN_NUM_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
};

sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();

/**----------------------------------------------------------------------
* Function    : init_sdcard
* Description : 初始化sdcard模块并加载fatfs文件系统,采用SPI接口
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/21 zhanli
*---------------------------------------------------------------------**/
void init_sdcard(void)
{
    esp_err_t ret;
    // Options for mounting the filesystem.
    // If format_if_mount_failed is set to true, SD card will be partitioned and
    // formatted in case when mounting fails.

    const char mount_point[] = MOUNT_POINT;

    // 初始化PSI总线
    ret = spi_bus_initialize(host.slot, &bus_cfg, SPI_DMA_CHAN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return;
    }

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;

    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                "If you want the card to be formatted, set the EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return;
    }

    // 打印SD卡相关的信息
    sdmmc_card_print_info(stdout, card);

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");
    FILE* f = fopen(MOUNT_POINT"/hello.txt", "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    fprintf(f, "Hello %s!\n", card->cid.name);
    fclose(f);
    ESP_LOGI(TAG, "File written");

    // Check if destination file exists before renaming
    struct stat st;
    if (stat(MOUNT_POINT"/foo.txt", &st) == 0) {
        // Delete it if it exists
        unlink(MOUNT_POINT"/foo.txt");
    }

    // Rename original file
    ESP_LOGI(TAG, "Renaming file");
    if (rename(MOUNT_POINT"/hello.txt", MOUNT_POINT"/foo.txt") != 0) {
        ESP_LOGE(TAG, "Rename failed");
        return;
    }

    // Open renamed file for reading
    ESP_LOGI(TAG, "Reading file");
    f = fopen(MOUNT_POINT"/foo.txt", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for reading");
        return;
    }
    char line[64];
    fgets(line, sizeof(line), f);
    fclose(f);
    // strip newline
    char* pos = strchr(line, '\n');
    if (pos) {
        *pos = '\0';
    }
    ESP_LOGI(TAG, "Read from file: '%s'", line);

    // All done, unmount partition and disable SDMMC or SPI peripheral
    //esp_vfs_fat_sdcard_unmount(mount_point, card);
    //spi_bus_free(host.slot);
}

int decode(){
    printf("void hex2file() {\n \
                ofstream ouFile(data_out.bin, ios::out | ios::binary);\n \
                for (int i = 0; i < ArrayLength(image); i++) {\n \
                    char s = image[i] - 127;\n \
                    ouFile.write(&s, 1);\n \
                }\n \
                    ouFile.close();\n \
            }\n");
    printf("int image[] = {\n");
    int count = 0;
    for(long long i = 0; i <  (sizeof(image) / sizeof(*image)); i++){
        count++;
		if (count == 10) {
			printf("0x%02x,\n", image[i]);
			count = 0;
            vTaskDelay(10 / portTICK_PERIOD_MS);
		}else {
			printf("0x%02x,", image[i]);
		}
    }
    printf("}\n");

    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(0), decode, decode, decode data);

/**----------------------------------------------------------------------
* Function    : list_files
* Description : 列出指定路径的下的所有文件信息
                例子: ls "0:/" (如果SD卡的默认挂载点是0)
* Author      : zhanli&719901725@qq.com
* Date        : 2022/01/04 zhanli
*---------------------------------------------------------------------**/
int list_files(char *path)
{
    // 定义目录对象
	FF_DIR dir; 
    // 定义静态文件信息结构对象
	static FILINFO fno; 
    // 打开目录，返回状态和目录对象的指针
	FRESULT res = f_opendir(&dir,path); 
    // 打开成功
	if(res == FR_OK){
		for(;;) {
			res = f_readdir(&dir, &fno); 
            // 若打开失败或到结尾，则退出
			if(res != FR_OK || fno.fname[0] == 0){
                break;
            } 
			if(fno.fattrib & AM_DIR){
				printf("dir :%s\n",fno.fname);
			}else{
				printf("file:%s\n",fno.fname); 
			}
		}
	}else{
        // 打开失败
		printf("open %s fail\n", path); 
	}
    // 关闭目录
	f_closedir(&dir); 
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(1), ls, list_files, list files and dir);


/**----------------------------------------------------------------------
* Function    : list_files
* Description : 列出指定路径的下的所有文件信息
                例子: ls "0:/" (如果SD卡的默认挂载点是0)
* Author      : zhanli&719901725@qq.com
* Date        : 2022/01/04 zhanli
*---------------------------------------------------------------------**/
int remove_file(char* path)
{
    if(path == NULL)return -1;
    
    FRESULT res = f_unlink(path);
    if(res == FR_OK){
        // 删除成功
        printf("delete %s sucess!\n", path); 
    }else{
        // 删除失败
        printf("delete %s fail!\n", path); //打开失败
    }
    return 0;
}
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0)|SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC)|
                 SHELL_CMD_PARAM_NUM(1), rm, remove_file, delete file and dir);
