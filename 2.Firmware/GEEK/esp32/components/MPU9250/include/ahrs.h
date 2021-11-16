#ifndef _AHRS_H_
#define _AHRS_H_
#include "esp_timer.h"


float invSqrt(float x); 

void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,char useMag); 

extern float q0,q1,q2,q3;
// #define	REG_READ(_r)			(*(volatile	uint32	*)(_r))
#define	WDEV_NOW()				esp_timer_get_time()
#endif
