if(ESP_PLATFORM)

file(GLOB_RECURSE SOURCES HAL/*.cpp)

idf_component_register(SRCS ${SOURCES}
                       INCLUDE_DIRS .
                       REQUIRES lvgl lvgl_gui)

else()
    message(FATAL_ERROR "LVGL LV examples: ESP_PLATFORM is not defined. Try reinstalling ESP-IDF.")
endif()

