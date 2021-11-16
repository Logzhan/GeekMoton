set(component_srcs 
					"src/ssd1306.c"
					"src/ssd1306_i2c.c"
					"src/ssd1306_spi.c")

idf_component_register(SRCS "${component_srcs}"
                       INCLUDE_DIRS "include"
                       PRIV_INCLUDE_DIRS ""
                       PRIV_REQUIRES "driver"
                       REQUIRES "")
