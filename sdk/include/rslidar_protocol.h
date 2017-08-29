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



#ifndef RSLIDAR_PROTOCOL_H
#define RSLIDAR_PROTOCOL_H
// RS-Lidar Input Packets

#if defined(_WIN32)
#pragma pack(1)
#endif

//----------------------------------------
// Commands
//-----------------------------------------
#define COMM_HEAD_FLAGE				0xAA	//Frame header

#define COMM_FRAME_TYPE_ATTR		0x20	//attribute frame
#define COMM_FRAME_TYPE_RSP_ATTR	0x21	//attribute frame 

#define COMM_FRAME_TYPE_CMD			0x40	//command farme
#define COMM_FRAME_TYPE_RSP_CMD		0x41	//command farme

#define COMM_FRAME_TYPE_MESSAGE		0x61	//message frame

// Parameter lengths
//-----------------------------------------
#define CMMD_SCAN_REQ_RSP_LENS			(0U)
#define CMMD_MOTOR_CONTROL_REQ_lENS		(1U)
#define CMMD_SET_MOTOR_RPM_REQ_LENS		(2U)
#define CMMD_SET_MEASURE_UNIT_REQ_LENS	(1U)
#define CMMD_RESET_REQ_LENS				(0U)
#define CMMD_DEV_RSP_LENS				(1U)

#define ATTR_DEV_INFO_PARAM_REQ_LENS	(0U)
#define ATTR_DEV_INFO_PARAM_RSP_LENS	(39U)
#define ATTR_DEV_INFO_HEALTH_RSP_LENS	(1U)
#define ATTR_DEV_INFO_MOTOR_RSP_LENS	(2U)

// Commmand response info
//-----------------------------------------
#define CMMD_FRAME_RSP_ERROR			0xC1
#define CMMD_FRAME_RSP_SUCCESS			0x00
#define CMMD_FRAME_RSP_CMD_ERROR		0x01
#define CMMD_FRAME_RSP_PARAM_LEN_ERROR	0x02
#define CMMD_FRAME_RSP_PARAM_ERROR		0x03
#define CMMD_FRAME_RSP_CRC_ERROR		0x04

#define ATTR_FRAME_RSP_ERROR			0xA1
#define ATTR_FRAME_RSP_CMD_ERROR		0x01
#define ATTR_FRAME_RSP_CRC_ERROR		0x02

// Motor start or stop control
//-----------------------------------------
#define CMMD_FRAME_PARAM_START_MOTOR	0x01
#define CMMD_FRAME_PARAM_STOP_MOTOR		0x00

// MACRO
//-----------------------------------------
#define MAX_SAMPLE_NUMBERS_PER_NOTE		128
#define NUMBER_OF_TEETH					16
#define NOTE_BUFFER_PING				0x00
#define NOTE_BUFFER_PONG				0x01

typedef enum _cmd_code
{
	CMD_STOP = 0x01,						
	CMD_START_SCAN,							
	CMD_INIT_DLIS2K,						
	CMD_WRITE_CAIL_DATA,					
	CMD_WRITE_CAIL_DATA_TOF,				
	CMD_LASER_CTRL,							
	CMD_DLIS2K_SAMPLE_DATA_8BIT,			
	CMD_DLIS2K_PIXEL_POS,					
	CMD_LENS_FOCUS_MODE,					
	CMD_PIXEL_POS_CAIL,						     
	CMD_MOTOR_WORK_CTRL,					      
	CMD_MOTOR_DUTY_SET,						
	CMD_MOTOR_RPM_SET,						         	
	CMD_DEBUG_MESSAGE_EN,					      
	CMD_EARE_CAIL_DATA,						        
	CMD_REGAIN_DEFAUT_SET,					  
	CMD_DEVICE_ADDR_SET,						
	CMD_SAMPLE_RATE_SET,				
	CMD_DISTANCE_OFFSET,					
	CMD_HIGH_VOLT_ADJUST,					     
	CMD_MEAS_PRINTF_EN,						
	CMD_HIGH_VOLT_RATE,						  
	CMD_DISTANCE_RANGE,						
	CMD_MEAS_UNIT,							           
	CMD_CAIL_MEAS,							
	CMD_WIRELESS_POWER_CTRL,				
	CMD_AUTO_MEAS,							
	CMD_WRITE_FLASH,						
	CMD_WRITE_DEVICE_INFO,					
	CMD_SYSTEM_RST,							
}CMD_CODE;


typedef enum _attr_code
{

	ATTR_READ_DEVICE_INFO = 0x53,     
	ATTR_READ_DLIS2K_REG,					    
	ATTR_READ_ONCE_MEAS,					      
	ATTR_READ_CAIL_DATA,					      
	ATTR_READ_MOTOR_DUTY,					
	ATTR_READ_MOTOR_RPM,				
	ATTR_READ_DEVICE_ADDR,					      
	ATTR_READ_SAMPLE_RATE,					
	ATTR_READ_DISTANCE_OFFSET,				
	ATTR_READ_HIGH_VOLT,				          
	ATTR_READ_MEAS_KEYE_MESSAGE,		        
	ATTR_READ_HIGHT_VOLT_RATE,				         
	ATTR_READ_MEAS_RANGE,					      
	ATTR_READ_MEAS_UNIT,					    
	ATTR_READ_MEAS_MODE,				 
	ATTR_READ_FLASH_DATA,				 
	ATTR_READ_DEVICE_HEALTH,			 
}ATTR_CODE;


typedef enum _message_code
{

	MESSAGE_DEVICE_ERROR = 0xA4,		
	MESSAGE_DLIS2K_SAMPLE_DATA_8BIT,	
	MESSAGE_DLIS2K_PIXEL_POS,			
	MESSAGE_DLIS2K_PIXEL_POS_DIS,		
	MESSAGE_TOF_DISTANCE,				
	MESSAGE_LIDAR_DISTANCE,				
}MESSAGE_CODE;

// Commonds
//-----------------------------------------

typedef enum _comm_error_code
{
	executeSuccess = 0,
	cmdError,
	parmaLen_Error,
	parma_Error,
	crc16_Error,
	SIMPLE_CAIL_Error,
}COMM_ERROR_CODE;


typedef struct _comm_frame_t
{
	uint8_t		frameStart;
	uint16_t	frameLen;
	uint8_t		addr;
	uint8_t		frameType;
	uint8_t		cmd;
	uint16_t	paramLen;
	uint8_t		paramBuf[0];
} __attribute__((packed)) COMM_FRAME_T;

typedef struct _comm_frame_head_t
{
	uint8_t		frameStart;
	uint16_t	frameLen;
	uint8_t		addr;
} __attribute__((packed)) COMM_FRAME_HEAD_T;

//sdk applicantion interface--------------
//----------------------------------------
#define START_MOTOR		true
#define STOP_MOTOR		false
typedef struct _rslidar_response_devive_info_t {
	_u8		productName[4];
	_u8		productDate[4];
	_u8		serialNum[8];
	_u8		softwareVersion[11];
	_u8		hardwareVersion[3];
	_u8		manufacturerInfo[3];
	_u8		gearNum[3];
	_u8		measureRange[3];
} __attribute__((packed)) RSLIDAR_RESPONSE_DEVICE_INFO_T;

typedef struct _rslidar_response_health_info_t {
	_u8		deviceHealthInfo;
} __attribute__((packed)) RSLIDAR_RESPONSE_HEALTH_INFO_T;

typedef struct _rslidar_response_motor_info_t {
	_u16		motorSpeed;
} __attribute__((packed)) RSLIDAR_RESPONSE_MOTOR_INFO_T;

typedef struct _rslidar_response_meature_unit_t {
	_u16		meatureUint;
} __attribute__((packed)) RSLIDAR_RESPONSE_MEATURE_INIT_T;

typedef struct _rslidar_signal_distance_unit_t {
	// _u8			signalValue;
	_u16		angle;
	_u16		distanceValue;
} __attribute__((packed)) RSLIDAR_SIGNAL_DISTANCE_UNIT_T;


#if defined(_WIN32)
#pragma pack()
#endif


#endif
