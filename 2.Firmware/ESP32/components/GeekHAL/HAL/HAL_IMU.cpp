#include "HAL.h"
#include <stdlib.h>
#include "Pedometer.h"
#include "mpu9250.h"
#include "Common/DataProc/DataProc.h"
#include <stdio.h>

static CommitFunc_t CommitFunc;
static void* UserData;

void IMU_SetCommitCallback(CommitFunc_t func, void* userData)
{
    CommitFunc = func;
    UserData = userData;
}

uint8_t IMU_Init()
{
    MPU9250_Init();
    Pedomter_Init();
    return 0;
}

void IMU_Update()
{
    float temp;

    IMU_Info_t imu;

    MPU9250_GetData(&imu.acc[0], &imu.acc[1], &imu.acc[2],
					&imu.gyr[0], &imu.gyr[1], &imu.gyr[2], &temp);

    
    imu.ax = (int16_t)(imu.acc[0] * 2000);
    imu.ay = (int16_t)(imu.acc[1] * 2000);
    imu.az = (int16_t)(imu.acc[2] * 2000);

    imu.gx = rand() % 1000 - 500;
    imu.gy = rand() % 1000 - 500;
    imu.gz = rand() % 1000 - 500;

    imu.steps = Pedometer_Update(imu.ax, imu.ay, imu.az);

    if (CommitFunc)
    {
        CommitFunc(&imu, UserData);
    }
}

