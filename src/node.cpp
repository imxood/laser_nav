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

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "rstypes.h"
#include "rslidar_protocol.h"
#include "rslidar_driver.h"
#include "rslidar.h"

#define DEG2RAD(x) ((x)*M_PI/180.)
#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace std;
using namespace rs::standalone::rslidar;

void publish_scan(ros::Publisher *pub,
                  RSLIDAR_SIGNAL_DISTANCE_UNIT_T *nodes,
                  size_t node_count, ros::Time start,
                  double scan_time,
                  float angle_min, float angle_max,
                  std::string frame_id)
{
    sensor_msgs::LaserScan scan_msg;

    scan_msg.header.stamp = start;
    scan_msg.header.frame_id = frame_id;
    scan_msg.angle_min = angle_min;
    scan_msg.angle_max = angle_max;
    scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / (360.0f - 1.0f);

    scan_msg.scan_time = scan_time;
    scan_msg.time_increment = scan_time / (double)(node_count-1);
    scan_msg.range_min = 0.15;
    scan_msg.range_max = 5.0;

    scan_msg.ranges.resize(360, std::numeric_limits<float>::infinity());

    //Unpack data
    for (size_t i = 0; i < node_count; i++)
    {
        size_t current_angle = floor(nodes[i].angle / 100.0f + 0.5f);
        if(current_angle > 360.0)
        {
            printf("Lidar angle is out of range %d\n", (int)current_angle);
            continue;
        }
        float read_value = (float) nodes[i].distanceValue/4.0f/1000.0f;
        if (read_value < scan_msg.range_min || read_value > scan_msg.range_max)
            scan_msg.ranges[360- 1- current_angle] = std::numeric_limits<float>::infinity();
        else
            scan_msg.ranges[360 -1- current_angle] = read_value;   
	}
    pub->publish(scan_msg);
}

int main(int argc, char * argv[])
{
    // read ros param
    ros::init(argc, argv, "iiiRoboticsLidar_node");

    std::string serial_port;
    int serial_baudrate = 115200;
    std::string frame_id;

    ros::NodeHandle nh;
    ros::Publisher scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 1000);
    ros::NodeHandle nh_private("~");
    nh_private.param<std::string>("serial_port", serial_port, "/dev/ttyUSB0");
    nh_private.param<int>("serial_baudrate", serial_baudrate, 115200);
    nh_private.param<std::string>("frame_id", frame_id, "laser");

    // Init lidar driver
    // create the driver instance
    u_result     op_result;
    RSlidarDriver * drv = RSlidarDriver::CreateDriver(RSlidarDriver::DRIVER_TYPE_SERIALPORT);

    if (!drv)
    {
        printf("Create Driver fail, exit\n");
        return -2;
    }

    // make connection...
    if (IS_FAIL(drv->connect(serial_port.c_str(), (_u32)serial_baudrate)))
    {
        printf("Error, cannot bind to the specified serial port %s.\n", serial_port.c_str());
        RSlidarDriver::DisposeDriver(drv);
        return -1;
    }

    printf("3iRoboticsLidar connected\n");
	//start scan
	if (!startScanTest(drv))
    {
		printf("Error, Cannot start scan");
	}

    ros::Time start_scan_time;
    ros::Time end_scan_time;
    double scan_duration;

    while (ros::ok())
    {
        RSLIDAR_SIGNAL_DISTANCE_UNIT_T nodes[360*2];
        size_t   count = _countof(nodes);

        start_scan_time = ros::Time::now();

        // read lidar data
        op_result = drv->grabScanData(nodes, count);
        end_scan_time = ros::Time::now();
        scan_duration = (end_scan_time - start_scan_time).toSec() * 1e-3;
       
		 _u8 eflag = 0;
         drv->getErrorInfo(&eflag);

        if (op_result == RESULT_OK)
        {
            float angle_min = DEG2RAD(0.0f);
            float angle_max = DEG2RAD(359.0f);
            if (IS_OK(op_result) && (eflag == 100) && count > 50)
            {
                //if successful, publish lidar scan
                int start_node = 0, end_node = 359;
                publish_scan(&scan_pub, &nodes[start_node], count,
                         start_scan_time, scan_duration,
                         angle_min, angle_max,
                         frame_id);
            }
            else if(( eflag > 0 )&&( eflag != 100 ))
            {
                printf("Lidar Error code%x\n",eflag);
            }
			else
			{
				printf("grabScanData error: %x\n", op_result);
			}
        }


        ros::spinOnce();
    }

    RSlidarDriver::DisposeDriver(drv);
    return 0;
}
