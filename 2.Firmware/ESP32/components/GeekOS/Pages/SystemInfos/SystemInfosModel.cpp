#include "SystemInfosModel.h"
#include "System/HAL/HAL.h"
#include <stdio.h>

using namespace Page;

void SystemInfosModel::Init()
{
    account = new Account("SystemInfosModel", DataProc::Center(), 0, this);

    //account->Subscribe("SportStatus");
    //account->Subscribe("GPS");
    //account->Subscribe("MAG");
    //account->Subscribe("IMU");
    //account->Subscribe("Clock");
    //account->Subscribe("Power");
    //account->Subscribe("Storage");
    //account->Subscribe("StatusBar");
}

void SystemInfosModel::Deinit()
{
    if (account)
    {
        delete account;
        account = nullptr;
    }
}

void SystemInfosModel::GetSportInfo(
    float* trip,
    char* time, uint32_t len,
    float* maxSpd
)
{
    SportStatus_Info_t sport = { 0 };
    account->Pull("SportStatus", &sport, sizeof(sport));
    *trip = sport.totalDistance / 1000;
    DataProc::MakeTimeString(sport.totalTime, time, len);
    *maxSpd = sport.speedMaxKph;
}

void SystemInfosModel::GetGPSInfo(
    float* lat,
    float* lng,
    float* alt,
    char* utc, uint32_t len,
    float* course,
    float* speed
)
{
    GPS_Info_t gps = { 0 };
    account->Pull("GPS", &gps, sizeof(gps));
    *lat = (float)gps.latitude;
    *lng = (float)gps.longitude;
    *alt = gps.altitude;
    snprintf(
        utc, len,
        "%d-%d-%d\n%02d:%02d:%02d",
        gps.clock.year,
        gps.clock.month,
        gps.clock.day,
        gps.clock.hour,
        gps.clock.minute,
        gps.clock.second
    );
    *course = gps.course;
    *speed = gps.speed;
}

void SystemInfosModel::GetMAGInfo(
    float* dir,
    int* x,
    int* y,
    int* z
)
{
    MAG_Info_t mag = { 0 };

    account->Pull("MAG", &mag, sizeof(mag));

    *dir = 0;
    *x = mag.x;
    *y = mag.y;
    *z = mag.z;
}

void SystemInfosModel::GetIMUInfo(
    int* step,
    char* info, uint32_t len
)
{
    IMU_Info_t imu = { 0 };

    account->Pull("IMU", &imu, sizeof(imu));
    *step = imu.steps;
    snprintf(
        info,
        len,
        "%d\n%d\n%d\n%d\n%d\n%d",
        imu.ax,
        imu.ay,
        imu.az,
        imu.gx,
        imu.gy,
        imu.gz
    );
}

void SystemInfosModel::GetRTCInfo(
    char* dateTime, uint32_t len
)
{
    Clock_Info_t clock = { 0 };
    account->Pull("Clock", &clock, sizeof(clock));
    snprintf(
        dateTime,
        len,
        "%d-%d-%d\n%02d:%02d:%02d",
        clock.year,
        clock.month,
        clock.day,
        clock.hour,
        clock.minute,
        clock.second
    );
}

void SystemInfosModel::GetBatteryInfo(
    char* info,
    uint32_t len
)
{
    Power_Info_t power = { 0 };
    Power_GetInfo(&power);
    snprintf(
		info, len,
		"\n%d%% \n%0.2f V\n%s",
		power.usage,
        (power.voltage / 1000.0f),
        "DISCHARGE"
	);
}

void SystemInfosModel::GetStorageInfo(
    bool* detect,
    const char** type,
	const char** name,
    char* usage, uint32_t len
)
{
	*detect = SD_GetReady();
    *type = SD_GetTypeName();
	*name = SD_GetCardName();
	snprintf(
		usage, len,
		"%0.1f GB",
		SD_GetCardSizeMB() / 1024.0f
	);
}

void SystemInfosModel::SetStatusBarStyle(DataProc::StatusBar_Style_t style)
{
    DataProc::StatusBar_Info_t info;
    DATA_PROC_INIT_STRUCT(info);

    info.cmd = DataProc::STATUS_BAR_CMD_SET_STYLE;
    info.param.style = style;

    account->Notify("StatusBar", &info, sizeof(info));
}
