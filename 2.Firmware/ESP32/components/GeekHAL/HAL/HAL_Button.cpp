#include "HAL.h"
#include "ButtonEvent.h"
#include "driver/gpio.h"

/* Construct Button Object. */
ButtonEvent btOK;              // Button OK
ButtonEvent btUP;              // Button Up
ButtonEvent btDOWN;            // Button Down
ButtonEvent btPOWER;           // Power Button, No share Object.
Button_Info_t Button_Info;

/**
  * @brief  Button callback handle event.
  * @param  btn: Button object adress.
  * @param  event: Event type.
  * @retval None
  */
static void Button_EventHandler(ButtonEvent* btn, int event)
{
    if(event == ButtonEvent::EVENT_ButtonClick){
        if(btn == &btOK){
            Button_Info.btnOK = 1;
        }else if(btn == &btUP){
            Button_Info.btnUp = 1;
        }else if(btn == &btDOWN){
            Button_Info.btnDown = 1;
        }
    }
    if(event == ButtonEvent::EVENT_ButtonLongPressed){
        if(btn == &btOK){
            Button_Info.btnPower = 1;
        }
    }
}

/**
  * @brief  Button power event handler.
  * @param  btn:Button object adress.
  * @param  event:Event type.
  * @retval None
  */
static void ButtonPower_EventHandler(ButtonEvent* btn, int event){
    if(btn == &btPOWER){
        /* Long press shut down. */
        if(event == ButtonEvent::EVENT_ButtonLongPressed)
        {
            //Power_Shutdown();
        }
    }
}

void Button_GPIO_Config(gpio_num_t gpio){
    gpio_pad_select_gpio(gpio);
    gpio_set_direction(gpio, GPIO_MODE_INPUT);
    gpio_set_pull_mode(gpio, GPIO_PULLUP_ONLY);
}

/**
  * @brief  Button initialize
  * @param  None
  * @retval None
  */
void Button_Init()
{
    btOK.PinNum = CONFIG_BUTTON_KEY_OK_PIN;
    btUP.PinNum = CONFIG_BUTTON_KEY_UP_PIN;
    btDOWN.PinNum = CONFIG_BUTTON_KEY_DOWN_PIN;

    /* Button GPIO configration. */
    Button_GPIO_Config((gpio_num_t)btOK.PinNum);
    Button_GPIO_Config((gpio_num_t)btUP.PinNum);
    Button_GPIO_Config((gpio_num_t)btDOWN.PinNum);

    /* Add button event. */
    btUP.EventAttach(Button_EventHandler);
    btOK.EventAttach(Button_EventHandler);
    btDOWN.EventAttach(Button_EventHandler);
    btPOWER.EventAttach(ButtonPower_EventHandler);
}

void Button_Update(){
    btOK.EventMonitor((gpio_get_level((gpio_num_t)btOK.PinNum) == 0));
    btUP.EventMonitor((gpio_get_level((gpio_num_t)btUP.PinNum) == 0));
    btDOWN.EventMonitor((gpio_get_level((gpio_num_t)btDOWN.PinNum) == 0));
}

void Button_GetInfo(Button_Info_t* info){
    /* Set the button info. */
    info->btnOK   = Button_Info.btnOK;
    info->btnUp   = Button_Info.btnUp;
    info->btnDown = Button_Info.btnDown;
    info->btnPower= Button_Info.btnPower;
    /* Reset Button event. */
    Button_Info.btnOK    = 0;
    Button_Info.btnUp    = 0;
    Button_Info.btnDown  = 0;
    Button_Info.btnPower = 0;
}