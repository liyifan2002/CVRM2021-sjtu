#pragma once

#include "RMPort.h"
//start 字节
#define    VISION_SOF         (0xA5)
//end字节,协议固定为0xA5
#define    VISION_TOF         (0xA6)
#ifdef __GNUC__
#define PACK(__Declaration__) __Declaration__ __attribute__((__packed__))
#endif
#ifdef _MSC_VER
#define PACK(__Declaration__) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif

/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/
/**    ----------------------------------------------------------------------------------------------------
FIELD  |  A5  |  CmdID  |  yaw  |  pitch  | distance  |  shoot  |  find  |  timestamp |  A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1  |    1    |   4   |    4    |     4     |    1    |    1   |     8     |   1  |
       ----------------------------------------------------------------------------------------------------
**/
/**---------------------------------------SEND DATA PROTOCOL--------------------------------------------**/

//TODO 通信协议优化
PACK(struct RobotSendData {
         uint8_t start = VISION_SOF;
         uint8_t cmdID = 0;    // 双板通信
         float yaw_angle = 0;        // 单位：度
         float pitch_angle = 0;      // 单位：度
         float distance = 0;       // 计算公式 //(int)(distance * 10)
         uint8_t shoot = 0;
         uint8_t find = 0;
         uint64_t timestamp = 0;
         //uint8_t sum = 0;
         uint8_t end = VISION_TOF;
     });
/**---------------------------------------RECEIVE DATA PROTOCOL-----------------------------------------------------------------------------**/
/**    -----------------------------------------------------------------------------------------------------------------------------------------
FIELD  |  head  |  cmdID  |  yawAngle  |  pitchAngle  | yawSpeed  |  pitchSpeed  |  shoot_mode  |  direction | blank |  blank |  blank |   A6  |
       ----------------------------------------------------------------------------------------------------
BYTE   |   1    |    1    |     4      |      4      |     4     |      4       |       4       |      8     |   1   |    1   |   1    |   1   |
------------------------------------------------------------------------------------------------------------------------------------------------
**/
PACK(struct RobotReceiveData {
         uint8_t cmdID = 0;
         float yaw_angle = 0;        // 单位：度
         float pitch_angle = 0;      // 单位：度
         float yaw_speed = 0;        // 单位：弧度/s
         float pitch_speed = 0;      // 单位：弧度/s
         uint32_t shoot_mode = 0;
         uint64_t direction = 0;
         uint8_t sum = 0;
         uint8_t blank = 0;
         uint8_t blank2 = 0;
     });

bool robot_io_usb(const string &robot_usb_hid = "");