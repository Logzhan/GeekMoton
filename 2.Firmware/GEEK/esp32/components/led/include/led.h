
#pragma once
#ifndef __LED_H
#define __LED_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdio.h>

    void blink_led(uint8_t s_led_state);
    void configure_led(void);

#ifdef __cplusplus
}
#endif
#endif