#
# Component Makefile
#

CFLAGS += -DLV_LVGL_H_INCLUDE_SIMPLE

COMPONENT_SRCDIRS := page           \
    assets/img      \
    assets/front    \

COMPONENT_ADD_INCLUDEDIRS := $(COMPONENT_SRCDIRS) .
