#include <stdio.h>
#include <conio.h>
#include "HalWin32_Button.h"
#include "Event/ButtonEvent.h"
#include "HalWin32_Button.h"
#include "System/GeekOS.h"

const uint8_t KEY1 = 25;
const uint8_t KEY2 = 32;
const uint8_t KEY3 = 33;

/*实例化按键对象*/
ButtonEvent btOK;              //选择键
ButtonEvent btUP;              //上键
ButtonEvent btDOWN;            //下键

static ButtonEvent btPOWER;    //电源键，不共享对象

/**
  * @brief  按键事件回调处理
  * @param  btn:按键对象地址
  * @param  event:事件类型
  * @retval 无
  */
static void HalWin32Button_EventHandler(ButtonEvent* btn, int event)
{
    // /*自动关机时间更新*/
    // Power_HandleTimeUpdate();
    if(event == ButtonEvent::EVENT_ButtonClick){
        if(btn == &btOK){
            printf("BtOk click\n");
            PageSwitchByName("Pages/SystemInfos");
        }
        if(btn == &btUP){
            PageSwitchByName("Pages/_Template");
            printf("BtUp click\n");
        }
        if(btn == &btDOWN){
            printf("BtDown click\n");
            ExitCurrentPages();
        }
    }
    if(event == ButtonEvent::EVENT_ButtonLongPressed){
        if(btn == &btOK){
            printf("BtOk LongPress\n");
            //ExitCurrentPages();
        }
    }
}

/**
  * @brief  电源按键事件回调处理
  * @param  btn:按键对象地址
  * @param  event:事件类型
  * @retval 无
  */
static void HalWin32ButtonPower_EventHandler(ButtonEvent* btn, int event){
    if(btn == &btPOWER){
        /*长按关机*/
        if(event == ButtonEvent::EVENT_ButtonLongPressed)
        {
            //Power_Shutdown();
        }
    }
}

/**
  * @brief  按键初始化
  * @param  无
  * @retval 无
  */
void HalWin32Button_Init()
{
    printf("Init Win32 Button\n");

    /*关联事件*/
    btUP.EventAttach(HalWin32Button_EventHandler);
    btOK.EventAttach(HalWin32Button_EventHandler);
    btDOWN.EventAttach(HalWin32Button_EventHandler);
    btPOWER.EventAttach(HalWin32ButtonPower_EventHandler);
}

/**
  * @brief  按键监控更新
  * @param  无
  * @retval 无
  */
void HalWin32Button_Update()
{
    char ch = 0;
    if (_kbhit()) {
        ch = _getch();
    }
    btOK.EventMonitor((ch == 'q'));
    btUP.EventMonitor((ch == 'w'));
    btDOWN.EventMonitor((ch == 'e'));
}
