#ifndef __SPI_FLASH_STORAGE_H
#define __SPI_FLASH_STORAGE_H

#include "esp_err.h"
esp_err_t init_flash_storage(void);
esp_err_t save_param(void);

#endif