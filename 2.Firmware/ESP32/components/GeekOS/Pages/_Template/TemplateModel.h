#ifndef __TEMPLATE_MODEL_H
#define __TEMPLATE_MODEL_H

#if defined(_WIN32)
#include "../lvgl/lvgl.h"
#else
#include "lvgl.h"
#endif

namespace Page
{

class TemplateModel
{
public:
    uint32_t TickSave;
    uint32_t GetData();
private:

};

}

#endif
