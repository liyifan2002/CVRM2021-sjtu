#include "CV2022.h"
#include "Stream.h"
#include "utils/timers.h"
#include "utils/calibration.h"


DHCam *vid;

/***
 * 从相机中读图
 * @param required_stop
 * @param camera_is_ok
 * @return
 */
bool camera_grab_loop(bool &required_stop, bool &camera_is_ok) {
    umt::Publisher<Mat> data_pub("camera_data");
    vid = new DHCam("DH01");
    LOG(INFO) << "START GRAB";
    if (!vid->videoCheck()) {
        LOG(WARNING) << "NO CAMERA";
        return false;
    }
    vid->openCamera();
    vid->startGrabbing();
    vid->setFrameRate(120);
    Mat ret;
    Timers t;
    camera_is_ok = true;
    while (!required_stop) {
        t.reset();
        vid->getFrame(ret);
        data_pub.push(ret);
    }
    return true;
}

/***
 * 从文件中读入视频
 * @param required_stop
 * @param camera_is_ok
 * @param vid_path
 * @return
 */
bool video_file_loop(bool &required_stop, bool &camera_is_ok, const string &vid_path) {//相机读图
    umt::Publisher<Mat> data_pub("camera_data");
    VideoCapture cap(vid_path);
    LOG(INFO) << "START VideoFileStream";
    if (!cap.isOpened())//如果视频不能正常打开则返回
    {
        LOG(FATAL) << "FILE " << vid_path << " OPEN FAILED";
    }
    camera_is_ok = true;
    Mat frame;
    double frameRate = cap.get(CAP_PROP_FPS);
    while (!required_stop) {
        cap >> frame;
        if (frame.empty()) { //如果某帧为空则退出循环
            required_stop = true;
            break;
        }
        data_pub.push(frame);
        this_thread::sleep_for(std::chrono::milliseconds (20));//每帧延时
    }
    cap.release();//释放资源
    return true;
}
/***
 * imshow 图像显示
 * @param required_stop
 * @param camera_is_ok
 */
void debug_view_loop(bool &required_stop, bool &camera_is_ok) {

    umt::Subscriber<Mat> data_sub("camera_data");
    Timers t;
    myCalibration::init();
    //              data_sub.pop();
//    cv::namedWindow("RM console");
//    cv::createTrackbar("exp", "RM console", nullptr, 1e5, ([](int v, void *u) -> void {
//        vid->setExposureTime(v);
//    }));
//    cv::createTrackbar("gain", "RM console", nullptr, 3e4, ([](int v, void *u) -> void {
//        vid->setGain(double(v) / 1000);
//    }));
//    cv::setTrackbarPos("exp", "RM console", vid->camConfig.exposureTime);
//    cv::setTrackbarPos("gain", "RM console", vid->camConfig.imGain);
    Mat console_panel(300, 300, CV_8UC1);
    //myCalibration::init();
    LOG(INFO) << "START VIEW";
    try {
        while (!required_stop) {
            t.reset();
            Mat ret = data_sub.pop();
            //eng.EnergyTask(ret, false, 20);
            char key = (char) cv::waitKey(1);

            console_panel *= 0;
            if (key == 'c')myCalibration::inputIMG(ret);
            if (key == 's')myCalibration::calibrate();
            if (key == 'w')vid->saveConfig("DH01");
            double fps = 1000 / t.rec();
            cv::putText(console_panel, to_string(fps).append("fps"), Point(20, 30), FONT_HERSHEY_SIMPLEX, 0.5,
                        Scalar(255));
            //cv::selectROI(ret);
            imshow("RM console", console_panel);
            cv::imshow("out", ret);
            if (key == 27) required_stop = true;
        }
    } catch (Exception &e) {
        LOG(FATAL) << e.what();
    }
}

/***
 * 比赛视频录制函数，强制关机会出现未更新文件头的情况，数据不会丢失
 * @param storage_location
 */
void video_record_loop(const std::string &storage_location) {
    char now[64];
    std::time_t tt;
    struct tm *ttime;
    tt = time(nullptr);
    ttime = localtime(&tt);
    strftime(now, 64, "%Y-%m-%d_%H_%M_%S", ttime);  // 以时间为名字
    std::string now_string(now);
    std::string path(std::string(storage_location + now_string).append(".avi"));
    LOG(INFO) << "Start recording";
    try {
        //注意这里的尺寸，尺寸不对会造成图像写入失败
        auto writer = cv::VideoWriter(path, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 20.0, cv::Size(1024, 820));
        if (!writer.isOpened()) {
            LOG(ERROR) << "can't open file";
            return;
        }
//        system(std::string("sudo chmod 777 ").append(path).c_str());
        umt::Subscriber<cv::Mat> data_sub("camera_data");
        while (true) {
            try {
                const auto im = data_sub.pop();
                writer.write(im);
            }
            catch (umt::MessageError &e) {
                LOG(WARNING) << "[camera_data]'" << e.what();
                std::this_thread::sleep_for(500ms);
            }
        }
    } catch (...) {
        LOG(FATAL) << "Record FAILED";
    }
}