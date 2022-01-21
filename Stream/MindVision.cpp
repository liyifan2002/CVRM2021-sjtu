// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/1/13.
//
#include "video.h"


#define MV_ASSERT_WARNING(expr, info, ...) do{        \
    if((expr) == false){                              \
         LOG(WARNING)<< "[MindVision]" #expr info "\n"; \
    }                                                 \
}while(0)

#define MV_ASSERT_ERROR(expr, info, ...) do{          \
    if((expr) == false){                              \
        LOG(ERROR)<< "[MindVision]" #expr info "\n";   \
        return false;                                 \
    }                                                 \
}while(0)

#define MV_CHECK_API_WARNING(expr, info, ...) do{                              \
    auto status = (expr);                                                      \
    if(status != CAMERA_STATUS_SUCCESS){                                       \
        LOG(WARNING)<< "[MindVision]" #expr info " return"<<status;             \
    }                                                                          \
}while(0)

#define MV_CHECK_API_ERROR(expr, info, ...)   do{                              \
    auto status = (expr);                                                      \
    if(status != CAMERA_STATUS_SUCCESS){                                       \
        LOG(ERROR)<< "[MindVision]" #expr info " return"<<status;               \
        return false;                                                          \
    }                                                                          \
}while(0)


MVCam::MVCam(const char *camera_name, const char *camera_cfg)
        : camera_name(camera_name), camera_cfg(camera_cfg), handle(-1) {
    MV_CHECK_API_WARNING(CameraSdkInit(1), "");
}


bool MVCam::openCamera() {
    if (isOpen()) {
        if (!closeCamera()) return false;
    }
    tSdkCameraDevInfo infos[2];
    int dev_num = 2;
    MV_CHECK_API_ERROR(CameraEnumerateDevice(infos, &dev_num), "");
    MV_ASSERT_ERROR(dev_num > 0, "no device found!");

    MV_ASSERT_WARNING(!camera_name.empty(), "camera name is not specified. no name check!");
    int camIndex = 0;
    for (auto &info: infos) {
        printf("Camera[%d] Info :\n", camIndex++);
        printf("    key           = [%s]\n", info.acLinkName);
        printf("    vendor name   = [%s]\n", info.acFriendlyName);
        printf("    model         = [%s]\n", info.acProductName);
        printf("    serial number = [%s]\n", info.acSn);
        MV_CHECK_API_ERROR(CameraInit(&info, -1, -1, &handle), "");
        break;

    }
    MV_ASSERT_ERROR(handle >= 0, "camera '{}' not found!", camera_name);

    MV_CHECK_API_WARNING(CameraReadParameterFromFile(handle, (char *) camera_cfg.data()),
                         "config file read error!");

    return true;
}

bool MVCam::close() {
    MV_ASSERT_WARNING(handle >= 0, "camera has already closed.");
    if (handle < 0) return true;
    MV_CHECK_API_ERROR(CameraUnInit(handle), "");
    handle = -1;
    return true;
}


bool MVCam::getFrame(cv::Mat &img) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkFrameHead head;
    BYTE *buffer;
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 100), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MVCam::read(cv::Mat &img, double &timestamp_ms) const {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkFrameHead head;
    BYTE *buffer;
    MV_CHECK_API_ERROR(CameraGetImageBuffer(handle, &head, &buffer, 100), "");
    img = cv::Mat(head.iHeight, head.iWidth, CV_8UC3);
    MV_CHECK_API_ERROR(CameraImageProcess(handle, buffer, img.data, &head), "");
    timestamp_ms = head.uiTimeStamp / 10.;
    MV_CHECK_API_ERROR(CameraReleaseImageBuffer(handle, buffer), "");
    return true;
}

bool MVCam::setExposureTime(double us) {
    MV_CHECK_API_ERROR(CameraSetAeState(handle, false), "");
    MV_CHECK_API_ERROR(CameraSetExposureTime(handle, us), "");
    return true;
}

bool MVCam::setTrigMode(RMCamera::ETrigType trigType) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraSetTriggerMode(handle, trigType), "");
    return false;
}

bool MVCam::isOpen() const {
    return handle >= 0;
}

bool MVCam::getExposureTime(double &us) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraGetExposureTime(handle, &us), "");
    return true;
}

bool MVCam::setFrameRate(double rate = 210) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkCameraCapbility camCap;
    CameraGetCapability(handle, &camCap);
    for (int i = 0; i < camCap.iFrameSpeedDesc; i++) {
        LOG(INFO) << camCap.pFrameSpeedDesc[i].iIndex << ":" << camCap.pFrameSpeedDesc[i].acDescription;
    }//todo 测试帧率配置

    MV_CHECK_API_ERROR(CameraSetFrameRate(handle, (int) rate), "");
    return true;
}

bool MVCam::setResolution(int height = 720, int width = 1280) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    tSdkImageResolution imageResolution;
    imageResolution.iWidth = width;
    imageResolution.iHeight = height;
    MV_CHECK_API_ERROR(CameraSetImageResolution(handle, &imageResolution), "");
    return true;
}

bool MVCam::setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio) {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraSetUserClrTempGain(handle, dRedBalanceRatio, dGreenBalanceRatio, dBlueBalanceRatio), "");
    return true;
}


bool MVCam::videoCheck() {
    return true;
}

bool MVCam::startGrabbing() {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraPlay(handle), "can not start grabbing");
    return true;
}

bool MVCam::stopGrabbing() {
    MV_ASSERT_ERROR(isOpen(), "camera not open.");
    MV_CHECK_API_ERROR(CameraPause(handle), "can not stop grabbing");
    return true;
}
