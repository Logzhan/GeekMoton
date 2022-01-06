set(button_srcs "src/Button.cpp" "src/ButtonEvent.cpp")

idf_component_register(SRCS "${button_srcs}"
                       INCLUDE_DIRS "include"
					   REQUIRES lvgl_gui)
