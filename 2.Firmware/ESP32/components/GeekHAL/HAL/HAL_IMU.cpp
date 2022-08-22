#include "HAL.h"
#include <stdlib.h>
#include "Common/DataProc/DataProc.h"
#include <stdio.h>

static CommitFunc_t CommitFunc;
static void* UserData;

void IMU_SetCommitCallback(CommitFunc_t func, void* userData)
{
    CommitFunc = func;
    UserData = userData;
}

bool IMU_Init()
{
    return true;
}

void IMU_Update()
{
    static int16_t steps;
    steps++;
    if (steps > 9999)
    {
        steps = 0;
    }

    IMU_Info_t imu;
    imu.steps = steps;

    imu.ax = rand() % 1000 - 500;
    imu.ay = rand() % 1000 - 500;
    imu.az = rand() % 1000 - 500;
    imu.gx = rand() % 1000 - 500;
    imu.gy = rand() % 1000 - 500;
    imu.gz = rand() % 1000 - 500;

    if (CommitFunc)
    {
        CommitFunc(&imu, UserData);
    }
}

