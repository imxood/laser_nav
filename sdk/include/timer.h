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


#ifndef TIMER_H
#define TIMER_H

#include "rstypes.h"

// TODO: the highest timer interface should be clock_gettime
namespace rs{ namespace arch{

_u64 rs_getus();
_u32 rs_getms();

}}

#define getms() rs::arch::rs_getms()
#endif
