/*----------------------------------------------------------------------------/
/ LED include file               (C)ChaN, 2019
/----------------------------------------------------------------------------*/
#ifndef DEF_BATTERY_H_
#define DEF_BATTERY_H_
/*---------------------------------------------------------------------------*/

typedef struct _BatteryInfo{
    float voltage;                 // 电池电压
    float capacity;                // 电池电量
}BatteryInfo;

void  battery_sample_task();
int   battery_get_capacity(void);
float battery_get_voltage(void);

#endif /* LED */
