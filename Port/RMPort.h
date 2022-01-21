#pragma once

#include "RMPort.h"

#ifdef __GNUC__
#define PACK( __Declaration__ ) __Declaration__ __attribute__((__packed__))
#endif
#ifdef _MSC_VER
#define PACK(__Declaration__) __pragma( pack(push, 1) ) __Declaration__ __pragma( pack(pop))
#endif
PACK(struct RobotCmd {
         uint8_t start = (unsigned) 's';
         uint8_t target_id = 255;    // 双板通信
         float pitch_angle = 0;      // 单位：度
         float yaw_angle = 0;        // 单位：度
         float pitch_speed = 0;      // 单位：弧度/s
         float yaw_speed = 0;        // 单位：弧度/s
         uint8_t distance = 0;       // 计算公式 (int)(distance * 10)
         uint8_t shoot_mode = 0;
         uint8_t lrc = 0;
         uint8_t end = (unsigned) 'e';
     });

bool robot_io_usb(const string &robot_usb_hid = "");