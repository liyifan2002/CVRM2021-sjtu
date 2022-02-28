
#include "CV2022.h"
#include <serial/include/serial/serial.h>
#include "RMPort.h"

RobotReceiveData robotstatus;
using namespace serial;

/*
 * 封包发送
 */
void robot_cmd_loop(Serial &robot_port, const bool &required_stop, bool &is_ok) {
    umt::Subscriber<RobotSendData> robot_cmd_sub("robot_cmd", 0);
    while (!required_stop) {
        try {
            auto robot_cmd = robot_cmd_sub.pop();
//            for(auto *ptr=(uint8_t*)&robot_cmd; ptr < &robot_cmd.sum; ptr++){
//                robot_cmd.sum += *ptr;
//            }//校验和计算
            try {
                robot_port.write((uint8_t *) &robot_cmd, sizeof(robot_cmd));
            } catch (SerialException &e) {
                LOG(ERROR) << e.what();
                is_ok = false;
                break;
            }
        } catch (umt::MessageError &e) {
            std::this_thread::sleep_for(100ms);
            LOG(ERROR) << "robot_cmd_loop pop error";
        }
    }
}

bool robot_io_usb(const string &robot_usb_hid) {
    /*
     * 使用 ttl 模块发送数据
     */
    Serial robot_port;
    LOG(INFO) << "finding serial port...";
    // 第一次运行程序是可以打印所有可用串口
    for (const auto &port_info: list_ports()) {
        LOG(INFO) << port_info.port << "|" << port_info.hardware_id;
        //TODO 可以改成读取配置文件
        if (port_info.hardware_id == robot_usb_hid) {
            robot_port.setPort(port_info.port);
            robot_port.setBaudrate(115200);  // 设置波特率
            auto timeout = Timeout::simpleTimeout(Timeout::max());
            robot_port.setTimeout(timeout);
            break;
        }
    }

    robot_port.open();
    if (!robot_port.isOpen()) {
        LOG(ERROR) << "robot serial init fail!\n";
        return false;
    }

    LOG(INFO) << "find serial port!" << std::endl;
    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    std::thread robot_cmd_thread(robot_cmd_loop, std::ref(robot_port),
                                 std::ref(robot_cmd_required_stop), std::ref(robot_cmd_is_ok));
    // 双通信协议
    // 读数据
    while (robot_cmd_is_ok) {
        try {
            uint8_t start;

            do robot_port.read(&start, 1);  // 读入起始位
            while (unsigned(start) != VISION_SOF);
            RobotReceiveData temp;
            robot_port.read((uint8_t *) &temp, sizeof(RobotReceiveData));//读入数据
            uint8_t lrc = 0;
//            for (auto *ptr = ((uint8_t *) &(hcurr.program_mode)); ptr < ((uint8_t *) &(hcurr.lrc)); ptr++)
//                lrc += *ptr; //checksum //TODO:sizeof
            uint8_t end = 0;
            robot_port.read(&end, 1);
            if (end == VISION_SOF)
                memcpy((uint8_t *) &robotstatus, (uint8_t *) &temp, sizeof(RobotReceiveData));
            else
                while (unsigned(end) != VISION_SOF) robot_port.read(&end, 1);
        } catch (SerialException &e) {
            LOG(ERROR) << "(robot_io_usb)" << e.what();
            break;
        }
    }
    robot_cmd_thread.join();
    robot_cmd_required_stop = true;
    return false;
}