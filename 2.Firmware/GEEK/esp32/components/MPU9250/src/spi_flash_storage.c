
#include "MPU9250.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_log.h"
#define STORAGE_NAMESPACE "mpudata"

static const char *TAG = "storage";
esp_err_t init_flash_storage(){
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated and needs to be erased
        // Retry nvs_flash_init
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK( err );

    nvs_handle_t my_handle;

    // esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Read run time blob
    // size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    // obtain required memory space to store blob being read from NVS
    err = nvs_get_blob(my_handle, "plug_param", &plug_param, (size_t*)sizeof(struct plug_saved_param));
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) return err;

    // Close
    nvs_close(my_handle);

    ESP_LOGI(TAG, "plug_param.Gyro_Cali_flag:%x\n",plug_param.Gyro_Cali_flag);
	// no used SPI Flash
	if (plug_param.status == 0xff) {
		plug_param.status = 1;
	}
    return ESP_OK;
}

esp_err_t save_param(void)
{
    nvs_handle_t my_handle;
    esp_err_t err;

    // Open
    err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK) return err;

    // Write value including previously saved blob if available
    err = nvs_set_blob(my_handle, "plug_param", &plug_param, sizeof(struct plug_saved_param));
    if (err != ESP_OK) return err;

    // Commit
    err = nvs_commit(my_handle);
    if (err != ESP_OK) return err;
    // Close
    nvs_close(my_handle);
    ESP_LOGI(TAG, "plug_param saved");
    return ESP_OK;
}
