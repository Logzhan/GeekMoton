#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ButtonEvent.h"
#include "Button.h"
#include "page/lv_geek_gui.h"

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
static void Button_EventHandler(ButtonEvent* btn, int event)
{
    // /*自动关机时间更新*/
    // Power_HandleTimeUpdate();
    if(event == ButtonEvent::EVENT_ButtonClick){
        if(btn == &btOK){
            printf("BtOk click\n");
            enter_func_page();
        }
        if(btn == &btUP){
            printf("BtUp click\n");
        }
        if(btn == &btDOWN){
            printf("BtDown click\n");
        }
    }
    // /*传递至页面事件*/
    // page.PageEventTransmit(btn, event);
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
        btOK.EventMonitor((gpio_get_level((gpio_num_t)KEY1) == 1));
        btUP.EventMonitor((gpio_get_level((gpio_num_t)KEY2) == 1));
        btDOWN.EventMonitor((gpio_get_level((gpio_num_t)KEY3) == 1));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
