#ifndef __RESOURCE_POOL
#define __RESOURCE_POOL

#if defined(_WIN32)
#include "../lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif

namespace ResourcePool
{

void Init();
lv_font_t* GetFont(const char* name);
const void* GetImage(const char* name);

}

#endif
