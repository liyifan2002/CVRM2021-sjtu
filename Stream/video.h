#ifndef VIDEO_H
#define VIDEO_H

#include <cmath>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <glog/logging.h>
#include "camera/DAHUA/IMVApi.h"
#include "camera/MVSDK/CameraApi.h"

using namespace std;
using namespace cv;

class RMCamera {//base class of camera interface
public:
    enum ETrigType // 触发模式
    {
        trigContinous = 0, //连续拉流
        trigSoftware = 1,  //软件触发
        trigLine = 2,      //外部触发
    };

    RMCamera(const string &cfg_name, const string &cfg_file = "../data/config/camera.yaml") {
        FileStorage fs(cfg_file, FileStorage::READ | FileStorage::FORMAT_YAML);
        FileNode node = fs[cfg_name];
        if (node.empty()) {
            LOG(ERROR) << "camera config name [" << cfg_name << "]not exist";
            return;
        }
        node["uniqueName"] >> camConfig.camUniqName;
        node["exposureTime"] >> camConfig.exposureTime;
        node["frameRate"] >> camConfig.frameRate;
        node["imHeight"] >> camConfig.imHeight;
        node["imWidth"] >> camConfig.imWidth;
        node["imGain"] >> camConfig.imGain;
        node["imGamma"] >> camConfig.imGamma;
        node["rGain"] >> camConfig.rGain;
        node["gGain"] >> camConfig.gGain;
        node["bGain"] >> camConfig.bGain;
        node["intrinsicMatrix"] >> camConfig.intrinsicMatrix;
        node["distortionCoefficient"] >> camConfig.distCoeffs;

        //(camConfig.exposureTime);
        fs.release();
    }

    bool saveConfig(const string &cfg_name, const string &cfg_file = "../data/config/camera.yaml") {
        FileStorage fs(cfg_file, FileStorage::APPEND | FileStorage::FORMAT_YAML);
        if (!fs.isOpened())return false;
        fs << cfg_name << "{"
           << "exposureTime" << camConfig.exposureTime
           << "frameRate" << camConfig.frameRate
           << "imHeight" << camConfig.imHeight
           << "imWidth" << camConfig.imWidth
           << "imGain" << camConfig.imGain
           << "imGamma" << camConfig.imGamma
           << "rGain" << camConfig.rGain
           << "gGain" << camConfig.gGain
           << "bGain" << camConfig.bGain
           << "intrinsicMatrix" << camConfig.intrinsicMatrix
           << "distortionCoefficient" << camConfig.distCoeffs
           << "}";
        fs.release();
        return true;
    }

    virtual bool videoCheck() = 0;                                      //搜索相机
    virtual bool openCamera() = 0;                                      //初始化相机
    virtual bool close() = 0;                                     //断开相机
    virtual bool startGrabbing() = 0;                                   //开始拉流
    virtual bool stopGrabbing() = 0;                                    //断开拉流
    virtual bool getFrame(Mat &img) = 0;                                //获取一帧图片
    virtual bool setExposureTime(double us) = 0;//设置曝光
    virtual bool setTrigMode(ETrigType trigType) = 0; //设置触发模式，一般为软触发
    virtual bool getExposureTime(double &us) = 0;

    virtual bool setFrameRate(double rate = 210) = 0;

    virtual bool setResolution(int height = 720, int width = 1280) = 0;

    virtual bool setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio) = 0;

    struct {
        string camUniqName;//相机名
        int exposureTime = 10000;//曝光时间us
        int frameRate = 120;//帧率
        int imHeight = 1280, imWidth = 1024;//图像长宽
        int imGain = 1;//图像增益
        int imGamma = 1;//图像伽马
        int rGain = 1, gGain = 1, bGain = 1;//RGB增益
        Mat intrinsicMatrix;//内参矩阵
        Mat distCoeffs;//畸变系数
    } camConfig;
};

class MVCam : public RMCamera {
public:
    explicit MVCam(const char *camera_name = "", const char *camera_cfg = "");

    ~MVCam() {
        MVCam::close();
    }

    bool videoCheck() override;                                          //搜索相机
    bool openCamera() override;                                          //初始化相机
    bool close() override;

    [[nodiscard]] bool isOpen() const;

    bool startGrabbing() override;                                   //开始拉流
    bool stopGrabbing() override;                                      //断开拉流
    bool getFrame(cv::Mat &img) override;

    bool read(cv::Mat &img, double &timestamp_ms) const;

    bool getExposureTime(double &us) override;

    bool setExposureTime(double us) override;

    bool setTrigMode(ETrigType trigType) override;

    bool setFrameRate(double rate) override;                    //设置帧率
    bool setResolution(int height, int width) override; //设置分辨率
    bool setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio) override;


private:
    const std::string camera_name;
    const std::string camera_cfg;
    CameraHandle handle;
};

/***
 * 大华相机的驱动
 */
class DHCam : public RMCamera {
public:
    DHCam()=default;
    explicit DHCam(const string &cfg_name, const string &cfg_file = "../data/config/camera.yaml"):RMCamera(cfg_name,cfg_file){};

    bool videoCheck() override;                                          //搜索相机
    bool openCamera() override;                                          //初始化相机
    bool close() override;                                          //断开相机
    bool startGrabbing() override;                                   //开始拉流
    bool stopGrabbing() override;                                      //断开拉流
    bool getFrame(Mat &img) override;                                  //获取一帧图片
    bool setExposureTime(double us) override;                            //设置曝光
    bool getExposureTime(double &us) override;

    bool setTrigMode(ETrigType trigType = trigSoftware) override; //设置触发模式，一般为软触发
    bool setFrameRate(double rate = 210) override;                    //设置帧率
    bool setResolution(int height = 720, int width = 1280); //设置分辨率
    bool setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio);

    bool setGain(double adj);                            //设置增益

    bool executeSoftTrig();                                      //执行一次软触发

    bool setROI(int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight);

    bool loadSetting(int mode);


    ~DHCam() {
        DHCam::stopGrabbing(); //断流
        DHCam::close();    //析构断开与相机的链接
    }

private:
    IMV_HANDLE m_devHandle = nullptr;
};

#endif
