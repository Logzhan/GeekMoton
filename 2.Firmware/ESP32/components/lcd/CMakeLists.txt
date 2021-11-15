set(lcd_srcs "src/lcd.c" "src/lcd_driver.c")

idf_component_register(SRCS "${lcd_srcs}"
                       INCLUDE_DIRS "include")
