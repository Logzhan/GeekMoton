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

/*ʵ������������*/
ButtonEvent btOK;              //ѡ���
ButtonEvent btUP;              //�ϼ�
ButtonEvent btDOWN;            //�¼�

static ButtonEvent btPOWER;    //��Դ�������������

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
            enter_func_page();
        }
        if(btn == &btUP){
            printf("BtUp click\n");
        }
        if(btn == &btDOWN){
            printf("BtDown click\n");
        }
    }
    // /*������ҳ���¼�*/
    // page.PageEventTransmit(btn, event);
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
        btOK.EventMonitor((gpio_get_level((gpio_num_t)KEY1) == 1));
        btUP.EventMonitor((gpio_get_level((gpio_num_t)KEY2) == 1));
        btDOWN.EventMonitor((gpio_get_level((gpio_num_t)KEY3) == 1));
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
