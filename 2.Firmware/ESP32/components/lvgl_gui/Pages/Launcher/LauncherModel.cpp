#include "LauncherModel.h"

using namespace Page;

uint32_t LauncherModel::GetData()
{
    return lv_tick_get();
}
