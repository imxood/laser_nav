/*
*  3iRoboticsLIDAR System
*  Driver Interface
*
*  Copyright 2017 3iRobotics
*  All rights reserved.
*
*	Author: 3iRobotics, Data:2017-04-06
*
*/

#ifndef __RSLIDAR_H__
#define __RSLIDAR_H__
#include "rslidar_driver.h"

using namespace rs::standalone::rslidar;

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif


#ifdef __cplusplus
extern "C"{
#endif


bool checkRSLIDARHealthInfo(RSlidarDriver * drv);
bool startScanTest(RSlidarDriver * drv);
bool stopScanTest(RSlidarDriver * drv);
bool resetRSlidar(RSlidarDriver * drv);


#ifdef __cplusplus
}
#endif



#endif

