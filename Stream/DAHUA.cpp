#include "video.h"

bool DHCam::loadSetting(int mode) {

    if (mode == 0) {
        if (!IMV_SetEnumFeatureSymbol(m_devHandle, "UserSetSelector", "UserSet1")) {
            cout << "set UserSetSelector failed!" << endl;
        }
    } else if (mode == 1) {
        if (!IMV_SetEnumFeatureSymbol(m_devHandle, "UserSetSelector", "UserSet2")) {
            cout << "set UserSetSelector failed!" << endl;
        }
    }

    if (!IMV_ExecuteCommandFeature(m_devHandle, "UserSetLoad")) {
        cout << "set UserSetLoad failed!" << endl;
    }
    return true;
}

bool DHCam::videoCheck() {
    unsigned int cameraIndex = 0;
    // 发现设备
    // discover camera
    int ret = IMV_OK;
    IMV_DeviceList deviceInfoList;
    ret = IMV_EnumDevices(&deviceInfoList, interfaceTypeAll);
    if (IMV_OK != ret) {
        printf("Enumeration devices failed! ErrorCode[%d]\n", ret);
        return false;
    }
    IMV_DeviceInfo *pDevInfo = nullptr;

    // 打印相机基本信息（key, 制造商信息, 型号, 序列号）
    for (cameraIndex = 0; cameraIndex < deviceInfoList.nDevNum; cameraIndex++) {
        pDevInfo = &deviceInfoList.pDevInfo[cameraIndex];
        printf("Camera[%d] Info :\n", cameraIndex);
        printf("    key           = [%s]\n", pDevInfo->cameraKey);
        printf("    vendor name   = [%s]\n", pDevInfo->vendorName);
        printf("    model         = [%s]\n", pDevInfo->modelName);
        printf("    serial number = [%s]\n", pDevInfo->serialNumber);
    }

    if (deviceInfoList.nDevNum < 1) {
        printf("no camera\n");
        return false;
    } else {
        cameraIndex = 0;
        //默认设置列表中的第一个相机为当前相机，其他操作比如打开、关闭、修改曝光都是针对这个相机。
        ret = IMV_CreateHandle(&m_devHandle, modeByIndex, (void *) &cameraIndex);
        if (IMV_OK != ret) {
            printf("Create devHandle failed! ErrorCode[%d]\n", ret);
            return false;
        }
    }
    return true;
}

/***
 * 打开相机
 * @return 打开成功？
 */
bool DHCam::openCamera() {
    int ret = IMV_Open(m_devHandle);
    if (IMV_OK != ret) {
        printf("open camera failed! ErrorCode[%d]\n", ret);
        return false;
    }
    setExposureTime(camConfig.exposureTime);
    return true;
}

/***
 * 关闭相机
 * @return: 关闭成功？
 */
bool DHCam::close() {
    int ret = IMV_OK;

    if (!m_devHandle) {
        printf("close camera fail. No camera.\n");
        return false;
    }

    if (!IMV_IsOpen(m_devHandle)) {
        printf("camera is already close.\n");
        return false;
    }

    ret = IMV_Close(m_devHandle);
    if (IMV_OK != ret) {
        printf("close camera failed! ErrorCode[%d]\n", ret);
        return false;
    }

    ret = IMV_DestroyHandle(m_devHandle);
    if (IMV_OK != ret) {
        printf("destroy devHandle failed! ErrorCode[%d]\n", ret);
        return false;
    }

    m_devHandle = nullptr;
    return true;
}

/***
 * 开始拉流
 * @return 开始成功？
 */
bool DHCam::startGrabbing() {
    int ret = IMV_StartGrabbing(m_devHandle);
    if (IMV_OK != ret) {
        printf("Start grabbing failed! ErrorCode[%d]\n", ret);
        return false;
    }
    return true;
}

/***
 * 断开拉流
 */
bool DHCam::stopGrabbing() {
    int ret = IMV_OK;
    if (!IMV_IsGrabbing(m_devHandle)) {
        printf("camera is already stop grabbing.\n");
        return false;
    }

    ret = IMV_StopGrabbing(m_devHandle);
    if (IMV_OK != ret) {
        printf("Stop grabbing failed! ErrorCode[%d]\n", ret);
        return false;
    }
    return true;
}

/***
 * 设置触发模式
 * @param trigType
 */
bool DHCam::setTrigMode(ETrigType trigType) {
    int ret = IMV_OK;

    if (trigContinous == trigType) {
        // 设置触发模式
        // set trigger mode
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "Off");
        if (IMV_OK != ret) {
            printf("set TriggerMode value = Off fail, ErrorCode[%d]\n", ret);
            return false;
        }
    } else if (trigSoftware == trigType) {
        // 设置触发器
        // set trigger
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
        if (IMV_OK != ret) {
            printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
            return false;
        }

        // 设置触发模式
        // set trigger mode
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
        if (IMV_OK != ret) {
            printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
            return false;
        }

        // 设置触发源为软触发
        // set triggerSource as software trigger
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Software");
        if (IMV_OK != ret) {
            printf("set TriggerSource value = Software fail, ErrorCode[%d]\n", ret);
            return false;
        }
    } else if (trigLine == trigType) {
        // 设置触发器
        // set trigger
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSelector", "FrameStart");
        if (IMV_OK != ret) {
            printf("set TriggerSelector value = FrameStart fail, ErrorCode[%d]\n", ret);
            return false;
        }

        // 设置触发模式
        // set trigger mode
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerMode", "On");
        if (IMV_OK != ret) {
            printf("set TriggerMode value = On fail, ErrorCode[%d]\n", ret);
            return false;
        }

        // 设置触发源为Line1触发
        // set triggerSource as Line1 trigger
        ret = IMV_SetEnumFeatureSymbol(m_devHandle, "TriggerSource", "Line1");
        if (IMV_OK != ret) {
            printf("set TriggerSource value = Line1 fail, ErrorCode[%d]\n", ret);
            return false;
        }
    }

    return true;
}

bool DHCam::executeSoftTrig() {
    int ret = IMV_OK;

    ret = IMV_ExecuteCommandFeature(m_devHandle, "TriggerSoftware");
    if (IMV_OK != ret) {
        printf("executeSoftTrig fail, ErrorCode[%d]\n", ret);
        return false;
    }

    printf("executeSoftTrig success.\n");
    return true;
}

bool DHCam::getFrame(Mat &img) {
    IMV_Frame frame;
    int ret = IMV_GetFrame(m_devHandle, &frame, 500);
    if (IMV_OK != ret) {
        printf("Get frame failed! ErrorCode[%d]\n", ret);
        return false;
    }
    IMV_PixelConvertParam stPixelConvertParam;
//    unsigned char *pDstBuf = nullptr;
    unsigned int nDstBufSize =  sizeof(unsigned char) * frame.frameInfo.width * frame.frameInfo.height * 3;
//    pDstBuf = (unsigned char *) malloc(nDstBufSize);
//    if (nullptr == pDstBuf) {
//        printf("malloc pDstBuf failed!\n");
//        return false;
//    }
//    memset(&stPixelConvertParam, 0, sizeof(stPixelConvertParam));
    img = Mat((int)frame.frameInfo.height,(int)frame.frameInfo.width, CV_8UC3);
    stPixelConvertParam.nWidth = frame.frameInfo.width;
    stPixelConvertParam.nHeight = frame.frameInfo.height;
    stPixelConvertParam.ePixelFormat = frame.frameInfo.pixelFormat;
    stPixelConvertParam.pSrcData = frame.pData;
    stPixelConvertParam.nSrcDataLen = frame.frameInfo.size;
    stPixelConvertParam.nPaddingX = frame.frameInfo.paddingX;
    stPixelConvertParam.nPaddingY = frame.frameInfo.paddingY;
    stPixelConvertParam.eBayerDemosaic = demosaicNearestNeighbor;
    stPixelConvertParam.eDstPixelFormat = gvspPixelBGR8;
    stPixelConvertParam.pDstBuf = img.data;
    stPixelConvertParam.nDstBufSize = nDstBufSize;
    ret = IMV_PixelConvert(m_devHandle, &stPixelConvertParam);
    if (IMV_OK != ret) {
        printf("image convert failed! ErrorCode[%d]\n", ret);
        return false;
    }
    IMV_ReleaseFrame(m_devHandle, &frame);
    return true;
}
/***
 * 设置白平衡增益
 * @param dRedBalanceRatio
 * @param dGreenBalanceRatio
 * @param dBlueBalanceRatio
 * @return
 */
bool DHCam::setBalanceRatio(double dRedBalanceRatio, double dGreenBalanceRatio, double dBlueBalanceRatio) {
    bool bRet;

    /* 关闭自动白平衡 */
    IMV_FeatureIsReadable(m_devHandle,"");
    if (!IMV_FeatureIsReadable(m_devHandle, "BalanceWhiteAuto")) {
        printf("balanceRatio not support.\n");
        return false;
    }
    bRet = IMV_SetEnumFeatureSymbol(m_devHandle,"BalanceWhiteAuto","Off");
    if (IMV_OK != bRet) {
        printf("set balanceWhiteAuto Off fail.\n");
        return false;
    }

    bRet = IMV_SetEnumFeatureSymbol(m_devHandle,"BalanceRatioSelector","Red");
    if (false == bRet) {
        printf("set red balanceRatioSelector fail.\n");
        return false;
    }
    bRet = IMV_SetDoubleFeatureValue(m_devHandle,"BalanceRatio",dRedBalanceRatio);
    if (false == bRet) {
        printf("set red balanceRatio fail.\n");
        return false;
    }

    bRet = IMV_SetEnumFeatureSymbol(m_devHandle,"BalanceRatioSelector","Green");
    if (false == bRet) {
        printf("set green balanceRatioSelector fail.\n");
        return false;
    }
    bRet = IMV_SetDoubleFeatureValue(m_devHandle,"BalanceRatio",dGreenBalanceRatio);
    if (false == bRet) {
        printf("set green balanceRatio fail.\n");
        return false;
    }

    bRet = IMV_SetEnumFeatureSymbol(m_devHandle,"BalanceRatioSelector","Blue");
    if (false == bRet) {
        printf("set blue balanceRatioSelector fail.\n");
        return false;
    }
    bRet = IMV_SetDoubleFeatureValue(m_devHandle,"BalanceRatio",dBlueBalanceRatio);
    if (false == bRet) {
        printf("set blue balanceRatio fail.\n");
        return false;
    }
    return true;
}

bool DHCam::setExposureTime(double us) {
    int ret = IMV_OK;

    ret = IMV_SetDoubleFeatureValue(m_devHandle, "ExposureTime", us);
    if (IMV_OK != ret) {
        printf("set ExposureTime value = %0.2f fail, ErrorCode[%d]\n", us, ret);
        return false;
    }
    camConfig.exposureTime = us;
    return true;

}

bool DHCam::setGain(double dGainRaw) {
    int ret = IMV_OK;

    ret = IMV_SetDoubleFeatureValue(m_devHandle, "GainRaw", dGainRaw);
    if (IMV_OK != ret) {
        printf("set GainRaw value = %0.2f fail, ErrorCode[%d]\n", dGainRaw, ret);
        return false;
    }
    camConfig.imGain = int(dGainRaw*1000);
    return true;
}

bool DHCam::setResolution(int height, int width) {
    int ret;
    ret = IMV_SetIntFeatureValue(m_devHandle, "Width", width);
    if (IMV_OK != ret) {
        printf("set width fail.\n");
        return false;
    }

    ret = IMV_SetIntFeatureValue(m_devHandle, "Height", height);
    if (IMV_OK != ret) {
        printf("set height fail.\n");
        return false;
    }
    return true;
}

bool DHCam::setROI(int64_t nX, int64_t nY, int64_t nWidth, int64_t nHeight) {
    int ret;
    ret = IMV_SetIntFeatureValue(m_devHandle, "Width", nWidth);
    if (IMV_OK != ret) {
        printf("set width fail.\n");
        return false;
    }

    ret = IMV_SetIntFeatureValue(m_devHandle, "Height", nHeight);
    if (IMV_OK != ret) {
        printf("set Height fail.\n");
        return false;;
    }

    ret = IMV_SetIntFeatureValue(m_devHandle, "OffsetX", nX);
    if (IMV_OK != ret) {
        printf("set OffsetX fail.\n");
        return false;;
    }
    ret = IMV_SetIntFeatureValue(m_devHandle, "OffsetY", nY);
    if (IMV_OK != ret) {
        printf("set OffsetY fail.\n");
        return false;;
    }
    return true;
}

bool DHCam::setFrameRate(double rate) {
    int ret;

    ret = IMV_SetBoolFeatureValue(m_devHandle, "AcquisitionFrameRateEnable", true);
    if (IMV_OK != ret) {
        printf("set acquisitionFrameRateEnable fail.\n");
        return false;
    }

    ret = IMV_SetDoubleFeatureValue(m_devHandle, "AcquisitionFrameRate", rate);
    if (IMV_OK != ret) {
        printf("set acquisitionFrameRate fail.\n");
        return false;
    }
    return true;
}

bool DHCam::getExposureTime(double &us) {
    //TODO
    return false;
}


