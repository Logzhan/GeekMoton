#ifndef __SYSTEM_INFOS_PRESENTER_H
#define __SYSTEM_INFOS_PRESENTER_H

#include "SystemInfosView.h"
#include "SystemInfosModel.h"

namespace Page
{

class SystemInfos : public PageBase
{
public:

public:
    typedef struct
    {
        uint16_t time;
        lv_color_t color;
    } Param_t;

    SystemInfos();
    virtual ~SystemInfos();

    virtual void onCustomAttrConfig();
    virtual void onViewLoad();
    virtual void onViewDidLoad();
    virtual void onViewWillAppear();
    virtual void onViewDidAppear();
    virtual void onViewWillDisappear();
    virtual void onViewDidDisappear();
    virtual void onViewDidUnload();

private:
    void Update();
    void AttachEvent(lv_obj_t* obj);
    static void onTimerUpdate(lv_timer_t* timer);
    static void onEvent(lv_event_t* event);

private:
    SystemInfosView View;
    SystemInfosModel Model;
    lv_timer_t* timer;
};

}

#endif
