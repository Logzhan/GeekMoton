set(power_srcs "src/battery.c" "src/battery_hal.c")

idf_component_register(SRCS "${power_srcs}"
                       INCLUDE_DIRS "include"
					   REQUIRES esp_adc_cal esp32shell)
