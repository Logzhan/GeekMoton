#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ButtonEvent.h"
#include "Button.h"
#include "System/GeekOS.h"
const uint8_t KEY1 = 25;
const uint8_t KEY2 = 32;
const uint8_t KEY3 = 33;

/*实例化按键对象*/
ButtonEvent btOK;              //选择键
ButtonEvent btUP;              //上键
ButtonEvent btDOWN;            //下键

static ButtonEvent btPOWER;    //电源键，不共享对象
static int iBtnOK, iBtnUp, iBtnDown;

void getKeyPadState(int* ok, int* up, int* down){
    *ok = iBtnOK;
    *up = iBtnUp;
    *down = iBtnDown;
}

/**
  * @brief  按键事件回调处理
  * @param  btn:按键对象地址
  * @param  event:事件类型
  * @retval 无
  */
static void Button_EventHandler(ButtonEvent* btn, int event)
{
    // /*自动关机时间更新*/
    // Power_HandleTimeUpdate();
    if(event == ButtonEvent::EVENT_ButtonClick){
        if(btn == &btOK){
            printf("BtOk click\n");
            //PageSwitchByName("Pages/SystemInfos");
        }else if(btn == &btUP){
            //PageSwitchByName("Pages/_Template");
            printf("BtUp click\n");
        }else if(btn == &btDOWN){
            printf("BtDown click\n");

            //ExitCurrentPages();
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
static void ButtonPower_EventHandler(ButtonEvent* btn, int event){
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
void button_init()
{
    gpio_pad_select_gpio(KEY1);
    gpio_set_direction((gpio_num_t)KEY1, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)KEY1, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio((gpio_num_t)KEY2);
    gpio_set_direction((gpio_num_t)KEY2, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)KEY2, GPIO_PULLUP_ONLY);

    gpio_pad_select_gpio((gpio_num_t)KEY3);
    gpio_set_direction((gpio_num_t)KEY3, GPIO_MODE_INPUT);
    gpio_set_pull_mode((gpio_num_t)KEY3, GPIO_PULLUP_ONLY);

    printf("Init Button\n");

    /*关联事件*/
    btUP.EventAttach(Button_EventHandler);
    btOK.EventAttach(Button_EventHandler);
    btDOWN.EventAttach(Button_EventHandler);
    btPOWER.EventAttach(ButtonPower_EventHandler);
}

/**
  * @brief  按键监控更新
  * @param  无
  * @retval 无
  */
void Button_Update()
{
    while(1){
        iBtnOK = (gpio_get_level((gpio_num_t)KEY1) == 1);
        iBtnUp = (gpio_get_level((gpio_num_t)KEY2) == 1);
        iBtnDown = (gpio_get_level((gpio_num_t)KEY3) == 1);
        btOK.EventMonitor(iBtnOK);
        btUP.EventMonitor(iBtnUp);
        btDOWN.EventMonitor(iBtnDown);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
