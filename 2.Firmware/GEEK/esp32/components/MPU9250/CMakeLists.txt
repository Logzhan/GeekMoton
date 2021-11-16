set(component_srcs 
            "src/mpu9250_i2c.c"
            "src/ahrs.c"
            "src/calibrate.c"
            "src/spi_flash_storage.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver" "nvs_flash"
                       REQUIRES "")
