#include <stdio.h>
#include "HAL.h"
#include "System/Version.h"
#include "MillisTaskManager/MillisTaskManager.h"
#ifdef _WIN32
#include <windows.h>
#include <time.h>
#define millis        clock
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#define millis        xTaskGetTickCount
#endif

static MillisTaskManager taskManager;

#if CONFIG_SENSOR_ENABLE

static void HAL_Sensor_Init()
{
#if CONFIG_SENSOR_IMU_ENABLE
    IMU_Init();
#endif

#if CONFIG_SENSOR_MAG_ENABLE
    MAG_Init();
#endif
}

#endif

static void HAL_TimerInterrputUpdate()
{

}

void HAL_Init()
{
#if CONFIG_SENSOR_ENABLE
    HAL_Sensor_Init();
#endif

    SD_Init();
    Power_Init();
    Button_Init();

    taskManager.Register(Power_Update, 1000);
    taskManager.Register(Button_Update, 10);
    taskManager.Register(IMU_Update, 100);
    taskManager.Register(MAG_Update, 100);
}

void HAL_Update()
{
    taskManager.Running(millis());
}
