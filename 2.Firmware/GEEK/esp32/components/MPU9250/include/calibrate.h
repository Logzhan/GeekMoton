/************************************************************************************

Filename    :   calibrate.h
Content     :   Sensor calibration store, fetch, and apply
Created     :
Authors     :   Nirav Patel

Copyright   :   Copyright 2013 Oculus VR, Inc. All Rights reserved.

Use of this software is subject to the terms of the Oculus license
agreement provided at the time of installation or download, or which
otherwise accompanies this software in either electronic or hard copy form.

*************************************************************************************/

#ifndef _CALIBRATE_H_
#define _CALIBRATE_H_




void Init_Gyro_Offset(void);

void Clean_Gyro_Offset(void);
	
void Store_Gyro_Offset(void);

void Compute_Gyro_Offset(void);
	
void calibrate_init(void);


#endif /* _CALIBRATE_H_ */
