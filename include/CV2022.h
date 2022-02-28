#pragma once
#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>
#include "umt/umt.hpp"

#define PREFIX_DIR_CONFIG "../data/config/"

using namespace std;
using namespace cv;

//RoboMaster Game Define
enum RobotColor{
    RED,BLUE
};

enum RobotType{
    HERO=1,
    ENGINEER = 2,
    INFANTRY3 = 3,
    INFANTRY4 = 4,
    INFANTRY5 = 5,
    UAV = 6,
    SENTRY = 7
};

enum EnergyMode{
    UNIFORM,//匀速小符
    COSINE//三角函数大符
};
