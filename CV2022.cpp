#include "CV2022.h"
#include <gflags/gflags.h>
#include "Stream/Stream.h"
#include "Port/RMPort.h"
//#include "utils/oscilloscope.h"

#include "../Energy/Energy.h"
static bool required_stop = false, camera_is_ok = false;

DEFINE_bool(debug, true, "debug mode");
DEFINE_bool(record, false, "record frame");
DEFINE_string(robot, "sentry, melee, infantry, hero",
              "robot type");
DEFINE_string(source, "",
              "image source");
DEFINE_string(serial, "",
              "serial name");

void autoaim_loop() {
    umt::Subscriber<Mat> data_sub("camera_data");
    float timestamp=0;
    EnergyDetector eng(BLUE,UNIFORM);
    try {
        while (!required_stop) {
            Mat ret = data_sub.pop();
            timestamp+=1000/61.89;
            eng.detect(ret, timestamp);
            eng.drawOutline();
        }
    } catch (Exception &e) {
        LOG(FATAL) << e.what();
    }

}

int main(int argc, char *argv[]) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    google::InitGoogleLogging(argv[0]);
    FLAGS_alsologtostderr = true;
    FLAGS_colorlogtostderr = true;  //设置输出文本
    FLAGS_log_dir = "./log/";
    //draw();
    //开启拉流线程
    if (FLAGS_source.empty()) {//是否指定source
        std::thread cameraGrab(camera_grab_loop, std::ref(required_stop), std::ref(camera_is_ok));
        cameraGrab.detach();
    } else {
        std::thread videoFile(video_file_loop, std::ref(required_stop), std::ref(camera_is_ok), FLAGS_source);
        videoFile.detach();
    }
    if (FLAGS_record) {
        std::thread videoRecoder(video_record_loop, "../data/");
        videoRecoder.detach();
    }
    std::thread autoAim(autoaim_loop);
    autoAim.detach();
    std::thread viewShow(debug_view_loop, std::ref(required_stop), std::ref(camera_is_ok));
    viewShow.join();//等待debug窗口退出
    //videoCap.join();
    //robot_io_usb("USB\\VID_1A86&PID_7523&REV_0254");
    google::ShutdownGoogleLogging();
    return 0;
}

