#include <stdio.h>
#include "HAL.h"
#include "System/Version.h"
#ifdef _WIN32
#include <windows.h>
#include <time.h>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#endif

#if CONFIG_SENSOR_ENABLE

static void HAL_Sensor_Init()
{

#if CONFIG_SENSOR_IMU_ENABLE

#endif

#if CONFIG_SENSOR_MAG_ENABLE

#endif
}

#endif

static void HAL_TimerInterrputUpdate()
{
    //HAL::Power_Update();
    //HAL::Encoder_Update();
    //HAL::Audio_Update();
}

void HAL_Init()
{

    //FaultHandle_Init();

    //Memory_DumpInfo();
    //Backlight_Init();
    //Encoder_Init();
    //Clock_Init();
    //Buzz_init();
    //GPS_Init();

#if CONFIG_SENSOR_ENABLE
    //HAL_Sensor_Init();
#endif

    //Audio_Init();
    SD_Init();
    Power_Init();
    Button_Init();
    //Display_Init();

    //taskManager.Register(Power_EventMonitor, 100);
    //taskManager.Register(GPS_Update, 200);
    //taskManager.Register(SD_Update, 500);
    //taskManager.Register(Memory_DumpInfo, 1000);

    //Timer_SetInterrupt(CONFIG_HAL_UPDATE_TIM, 10 * 1000, HAL_TimerInterrputUpdate);
    //Timer_SetEnable(CONFIG_HAL_UPDATE_TIM, true);
}

void HAL_Update()
{
    static uint64_t tick = 0;
    while(1){
        if((tick % POWER_UPDATE_INTERNAL == 0)){
            Power_Update();
        }
        Button_Update();
#ifdef _WIN32
        Sleep(10);
#else
        vTaskDelay(10 / portTICK_PERIOD_MS);
#endif
        tick++;
    }
}
