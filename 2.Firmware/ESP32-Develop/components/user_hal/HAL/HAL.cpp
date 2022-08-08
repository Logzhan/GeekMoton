#include "HAL.h"
#include "System/Version.h"


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

void HAL::HAL_Init()
{

    //FaultHandle_Init();

    //Memory_DumpInfo();

    //Power_Init();
    //Backlight_Init();
    //Encoder_Init();
    //Clock_Init();
    //Buzz_init();
    //GPS_Init();

#if CONFIG_SENSOR_ENABLE
    //HAL_Sensor_Init();
#endif

    //Audio_Init();
    //SD_Init();

    //Display_Init();

    //taskManager.Register(Power_EventMonitor, 100);
    //taskManager.Register(GPS_Update, 200);
    //taskManager.Register(SD_Update, 500);
    //taskManager.Register(Memory_DumpInfo, 1000);

    //Timer_SetInterrupt(CONFIG_HAL_UPDATE_TIM, 10 * 1000, HAL_TimerInterrputUpdate);
    //Timer_SetEnable(CONFIG_HAL_UPDATE_TIM, true);
}

void HAL::HAL_Update()
{
    //taskManager.Running(millis());
}
