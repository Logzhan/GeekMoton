set(adc_srcs "src/adc.c")

idf_component_register(SRCS "${adc_srcs}"
                       INCLUDE_DIRS "include"
					   REQUIRES esp_adc_cal esp32shell)
