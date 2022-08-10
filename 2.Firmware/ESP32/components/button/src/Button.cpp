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

/*ʵ������������*/
ButtonEvent btOK;              //ѡ���
ButtonEvent btUP;              //�ϼ�
ButtonEvent btDOWN;            //�¼�

static ButtonEvent btPOWER;    //��Դ�������������
static int iBtnOK, iBtnUp, iBtnDown;

void getKeyPadState(int* ok, int* up, int* down){
    *ok = iBtnOK;
    *up = iBtnUp;
    *down = iBtnDown;
}

/**
  * @brief  �����¼��ص�����
  * @param  btn:���������ַ
  * @param  event:�¼�����
  * @retval ��
  */
static void Button_EventHandler(ButtonEvent* btn, int event)
{
    // /*�Զ��ػ�ʱ�����*/
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
  * @brief  ��Դ�����¼��ص�����
  * @param  btn:���������ַ
  * @param  event:�¼�����
  * @retval ��
  */
static void ButtonPower_EventHandler(ButtonEvent* btn, int event){
    if(btn == &btPOWER){
        /*�����ػ�*/
        if(event == ButtonEvent::EVENT_ButtonLongPressed)
        {
            //Power_Shutdown();
        }
    }
}

/**
  * @brief  ������ʼ��
  * @param  ��
  * @retval ��
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

    /*�����¼�*/
    btUP.EventAttach(Button_EventHandler);
    btOK.EventAttach(Button_EventHandler);
    btDOWN.EventAttach(Button_EventHandler);
    btPOWER.EventAttach(ButtonPower_EventHandler);
}

/**
  * @brief  ������ظ���
  * @param  ��
  * @retval ��
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
