
#include "CV2022.h"
#include <Drivers/serial/serial.h>
#include "RMPort.h"
using namespace serial;
/**
 * @brief RM视觉串口通信类
*/
void robot_cmd_loop(Serial &robot_port, const bool &required_stop, bool &is_ok) {
    /*
     * 将收到的数据统一发送出去
     */
    umt::Subscriber<RobotCmd> robot_cmd_sub("robot_cmd", 0);
    while (!required_stop) {
        try {
            auto robot_cmd = robot_cmd_sub.pop();
//            for(auto *ptr=(uint8_t*)&robot_cmd.priority; ptr < &robot_cmd.lrc; ptr++){
//                robot_cmd.lrc += *ptr;
//            }
            try {
                robot_port.write((uint8_t *) &robot_cmd, sizeof(robot_cmd));
            } catch (SerialException &e) {
                LOG(ERROR)<<e.what();
                is_ok = false;
                break;
            }
        } catch (umt::MessageError &e) {
            std::this_thread::sleep_for(100ms);
            LOG(ERROR) << "robot_cmd_loop pop error" ;
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
    for (const auto &port_info : list_ports()) {
        LOG(INFO) << port_info.port << "|" << port_info.hardware_id;
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
        LOG(ERROR)<<"robot serial init fail!\n";
        return false;
    }

    LOG(INFO) << "find serial port!" << std::endl;
    bool robot_cmd_required_stop = false;
    bool robot_cmd_is_ok = true;
    std::thread robot_cmd_thread(robot_cmd_loop, std::ref(robot_port),
                                 std::ref(robot_cmd_required_stop), std::ref(robot_cmd_is_ok));
    // 双通信协议
//    while (robot_cmd_is_ok) {
//        try {
//            uint8_t start;
//            robot_port.read(&start, 1);  // 读入起始位
//            while (unsigned(start) != unsigned('s') && unsigned(start) != unsigned('h')) robot_port.read(&start, 1);
//            if (unsigned(start) == unsigned('s')) {  // 短包
//                robot_port.read((uint8_t *) &scurr, sizeof(ShortRobotStatus));
//                uint8_t lrc = 0;
//                for(auto *ptr = ((uint8_t*)&(scurr.enemy_color)); ptr < ((uint8_t*)&(scurr.lrc)); ptr++) lrc += *ptr;  //TODO:sizeof
//                uint8_t end = 0;
//                robot_port.read(&end, 1);
//                if (lrc == scurr.lrc && end == unsigned('e')) memcpy((uint8_t *)robot_status_short.get(),(uint8_t *)&scurr,sizeof(scurr));
//                else while (unsigned(end) != unsigned('e')) robot_port.read(&end,1);
//            }
//            else if (unsigned(start) == unsigned('h')) {  // 长包
//                robot_port.read((uint8_t *) &hcurr, sizeof(LongRobotStatus));
//                uint8_t lrc = 0;
//                for(auto *ptr = ((uint8_t*)&(hcurr.program_mode)); ptr < ((uint8_t*)&(hcurr.lrc)); ptr++) lrc += *ptr;  //TODO:sizeof
//                uint8_t end = 0;
//                robot_port.read(&end, 1);
//                if (lrc == hcurr.lrc && end == unsigned('e')) memcpy((uint8_t *)robot_status_long.get(),(uint8_t *)&hcurr,sizeof(hcurr));
//                else while (unsigned(end) != unsigned('e')) robot_port.read(&end,1);
//            }
//        } catch (SerialException &e) {
//            LOG(ERROR)<<"(robot_io_usb)"<<e.what();
//            break;
//        }
//    }
    robot_cmd_thread.join();
    robot_cmd_required_stop = true;
    return false;
}