#include "battery_hal.h"
#include "battery.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static BatteryInfo batterInfo; 

// 电池百分比和电压曲线
int   precent[] = {100,   90,   80,  70,    60,   50,   40,   30,   20,  15,   10,   5,   0};
float voltage[] = {4.2, 4.08, 4.00, 3.93, 3.87, 3.82, 3.79, 3.77, 3.73, 3.7, 3.68, 3.5, 3.0};

static void battery_cal_capacity(float vol);

/**----------------------------------------------------------------------
* Function    : battery_sample_task
* Description : 电池信息的采样任务
* Author      : zhanli&719901725@qq.com
* Date        : 2021/10/26 zhanli
*---------------------------------------------------------------------**/
void battery_sample_task()
{
    // 电池硬件抽象层初始化
    battery_hal_init();

    while (1) {
        // 获取电池电压
        batterInfo.voltage = battery_hal_get_voltage();
        // 计算电池百分比
        battery_cal_capacity(batterInfo.voltage);
        // 延时周期
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
/**----------------------------------------------------------------------
* Function    : battery_cal_capacity
* Description : 一个小算法，通过电池电压计算当前电池容量
* Author      : zhanli&719901725@qq.com
* Date        : 2021/12/20 zhanli
*---------------------------------------------------------------------**/
static void battery_cal_capacity(float vol){
    int i = 0;
    int LEN = 13;
    for(i = 0; i < LEN; i++){
        if(vol >= voltage[i]){
            break;
        }
    }
    int offset = 0;
    // 对于非关键点的电压数据进行线性拟合
    if(i > 0 && i < LEN){
        float dv = vol - voltage[i];
        offset = (dv / (voltage[i-1] - voltage[i]))*(precent[i-1] - precent[i]);
    }
    batterInfo.capacity = precent[i] + offset;
}

int battery_get_capacity(){
    return batterInfo.capacity;
}

float battery_get_voltage(){
    return batterInfo.voltage;
}