/**
 * @file lv_geek_gui.h
 *
 */

#ifndef LV_GEEK_GUI_H
#define LV_GEEK_GUI_H

#if defined(_WIN32)
#include "../lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/*********************
 *      INCLUDES
 *********************/

/*********************
 *      DEFINES
 *********************/

/**********************
 *      TYPEDEFS
 **********************/

/**********************
 * GLOBAL PROTOTYPES
 **********************/
void geek_gui_init(void);

void enter_func_page(void);

/**********************
 *      MACROS
 **********************/

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*LV_GEEK_GUI_H*/
