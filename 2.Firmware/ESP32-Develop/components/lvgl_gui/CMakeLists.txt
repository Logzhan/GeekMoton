if(ESP_PLATFORM)

file(GLOB_RECURSE SOURCES Pages/*.cpp 
						  Resource/*.c 
						  System/*.cpp 
						  System/PageManager/*.cpp 
						  System/lv_ext/*.cpp
						  System/lv_ext/*.c 
						  Resource/*.cpp 
						  Resource/front/*.c)

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS .
                       REQUIRES lvgl power)

else()
    message(FATAL_ERROR "LVGL LV examples: ESP_PLATFORM is not defined. Try reinstalling ESP-IDF.")
endif()
