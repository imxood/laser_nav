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


#include <stdio.h>
#include <stdlib.h>

#include "rstypes.h"
#include "rslidar_protocol.h"
#include "rslidar.h"


using namespace rs::standalone::rslidar;


bool checkRSLIDARHealthInfo(RSlidarDriver * drv)
{
	u_result     op_result;
	RSLIDAR_RESPONSE_HEALTH_INFO_T healthInfo;

	op_result = drv->getHealth(healthInfo);
	if (IS_OK(op_result)) {
		printf("3iRoboticsLidar health status: %d\n", healthInfo.deviceHealthInfo);
		if (!(0x02 == healthInfo.deviceHealthInfo || 0x00 == healthInfo.deviceHealthInfo))
		{
			return false;
		}
	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
		return false;
	}
	return true;
}

bool startScanTest(RSlidarDriver * drv)
{
	u_result     op_result;

	op_result = drv->startScan();
	if (IS_OK(op_result)) { 
		printf("3iRoboticsLidar start scan ok!\n");
	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar start scan: %x\n", op_result);
		return false;
	}
	return true;
}

bool stopScanTest(RSlidarDriver * drv)
{
	u_result     op_result;

	op_result = drv->stop();
	if (IS_OK(op_result)) { 
		printf("3iRoboticsLidar stop scan ok!\n");
	}
	else {
		fprintf(stderr, "Error, cannot retrieve the lidar stop scan: %x\n", op_result);
		return false;
	}
	return true;
}

bool resetRSlidar(RSlidarDriver * drv)
{
	u_result     op_result;

	op_result = drv->reset();
	if (IS_OK(op_result)) {
		printf("3iRoboticsLidar reset ok!\n");
	}
	else {
		fprintf(stderr, "Error, cannot reset the lidar : %x\n", op_result);
		return false;
	}
	return true;
}

