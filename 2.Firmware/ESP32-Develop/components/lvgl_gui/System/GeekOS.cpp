#include "GeekOS.h"
#include "stdio.h"
#include "Pages/AppFactory.h"
#include "System/PageManager/PageManager.h"
#include "Resource/ResourcePool.h"

static AppFactory factory;
static PageManager manager(&factory);

#define ACCOUNT_SEND_CMD(ACT, CMD)\
do{\
    DataProc::ACT##_Info_t info;\
    memset(&info, 0, sizeof(info));\
    info.cmd = DataProc::CMD;\
    DataProc::Center()->AccountMain.Notify(#ACT, &info, sizeof(info));\
}while(0)


void GeekOS_Init() {

    //  lvgl initial config and set backgroud color to black
    lv_obj_t* scr = lv_scr_act();
    lv_obj_remove_style_all(scr);
    lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    lv_disp_set_bg_color(lv_disp_get_default(), lv_color_black());
    
    ResourcePool::Init();

    manager.Install("Template", "Pages/_Template");
    manager.Install("Launcher", "Pages/Launcher");
    manager.Install("Startup", "Pages/Startup");
    //manager.Install("Dialplate", "Pages/Dialplate");
    //manager.Install("SystemInfos", "Pages/SystemInfos");

    manager.SetGlobalLoadAnimType(PageManager::LOAD_ANIM_OVER_TOP, 500);

    manager.Push("Pages/Startup");
}

void GeekOS_Uninit(){

}

void PageSwitchByName(const char* name){
    lv_mem_monitor_t monitor;
    lv_mem_monitor(&monitor);
    printf("used: %6d (%3d %%), frag: %3d %%, biggest free: %6d\n",
        (int)monitor.total_size - monitor.free_size,
        monitor.used_pct,
        monitor.frag_pct,
        (int)monitor.free_biggest_size);
    manager.Push(name);
}

void ExitCurrentPages(){
    lv_mem_monitor_t monitor;
    lv_mem_monitor(&monitor);
    printf("used: %6d (%3d %%), frag: %3d %%, biggest free: %6d\n",
        (int)monitor.total_size - monitor.free_size,
        monitor.used_pct,
        monitor.frag_pct,
        (int)monitor.free_biggest_size);
    manager.Pop();
}
