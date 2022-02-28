// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/2/23.
//

#include "Energy.h"
#include "utils/timers.h"
#include <fstream>

Timers time_energy;


bool geometryFilter(const std::vector<cv::Point> &contour,
                    rangef *contour_area,
                    rangef *area_ratio,
                    rangef *rect_length,
                    rangef *rect_width,
                    rangef *hw_ratio) {
    double cur_contour_area = contourArea(contour);
    if (!IN_RANGE(cur_contour_area, contour_area))return false;

    cv::RotatedRect cur_rect = minAreaRect(contour);
    cv::Size2f cur_size = cur_rect.size;
    float length, width;
    if (cur_size.height > cur_size.width)
        length = cur_size.height, width = cur_size.width;
    else length = cur_size.width, width = cur_size.height;

    float length_width_ratio = length / width;
    if (!IN_RANGE(length, rect_length) || !IN_RANGE(width, rect_width))return false;
    if (!IN_RANGE(length_width_ratio, hw_ratio))return false;
    if (!IN_RANGE(cur_contour_area / cur_size.area(), area_ratio))return false;

    return true;
}

/**
* @brief EnergyDetector::clearAll
* @param null
* @return null
* @remark 在每帧任务开始前清空容器
*/
void EnergyDetector::clearAll() {
    valid_armors.clear();
    valid_fan_strip.clear();
//    valid_fan_strip.clear();
//    target_blades.clear();
//    centerRs.clear();
//    armor_centers.clear();
}

/**
 * @brief EnergyDetector::detect
 * @param src 摄像头读入的图片
 * @param timestamp 读图时的时间戳
 * @remark 能量机关任务执行接口
 */
void EnergyDetector::detect(const Mat &src, const float timestamp) {
    static int misscount;
    clearAll();//先清空容器
    roi = binary.clone();
    time_energy.reset();
    preprocess(src); //预处理 通道运算 提取出指定颜色的数据 并过滤白光
    //binary = preprocess(img);
    //roi = binary;
    //findROI(binary,roi); //设定roi
    //imshow("roi",roi);
    //Ticks(time_energy, "preprocess");
    if (detectArmor() && detectFlowStripFan() && getTargetPoint()) {
        //getPts(target_armor); //获得的装甲板4点
        //Ticks(time_energy, "getPts");
        detectCircleCenter(); //识别旋转圆心
        calOmega(timestamp);
        //Ticks(time_energy, "detectCircleCenter");
        //calOmega(timestamp); //计算当前的角速度 cur_omega 为当前三阶差分计算的角速度 av_omega.back() 为 4 次角速度平滑均值
        //Ticks(time_energy, "calOmega");
        //waveClass.displayWave(av_omega.back(), 1.305);
//        if (judgeRotation(src, timestamp)) {
//            //大小符模式 开始预测
//            if (mode == 2) {
//                getPredictPoint(roi, timestamp); //变速预测
//            } else if (mode == 1) {
//                getPredictPointSmall(roi);
//            }
//            Ticks(time_energy, "getPredictPoint");
//            //roiPoint2src();
//            detect_flag = true;
//            misscount = 0;
//        }
    } else {
        misscount++;
        predict_point = Point2f(0, 0);
        if (misscount > 5) { //连续5帧丢目标
            misscount = 0;
            detect_flag = false;
        }
    }
//    if (showEnergy) {
//        imshow("binary", roi);
//        circle(outline, Point(IMGWIDTH / 2, IMGHEIGHT / 2), 2, Scalar(255, 255, 255), 3); //画出图像中心
//        imshow("outline", outline);
//        waitKey(1);
//    }
}

/**
 * @brief EnergyDetector::preprocess
 * @param Mat& src
 * @return Mat& binary
 * @remark 图像预处理，完成二值化，提取
 */
void EnergyDetector::preprocess(const Mat &src) {
    Mat blue_binary, red_binary;
    //cvtColor(src, dst, COLOR_BGR2GRAY);
    Mat single, blue_c, red_c;
    vector<Mat> channels;

    split(src, channels);
    blue_c = channels.at(0);
    red_c = channels.at(2);
    findContours(binary, contours, RETR_LIST, CHAIN_APPROX_NONE);
    findContours(binary, contours_external, RETR_EXTERNAL, CHAIN_APPROX_NONE);

//    if (blueTarget) {
//        single = channels.at(0);
//    } else {
//        single = channels.at(2);
//    }

    //threshold(single, binary, 90, 255, THRESH_BINARY);
    threshold(blue_c, blue_binary, 90, 255, THRESH_BINARY);
    threshold(red_c, red_binary, 90, 255, THRESH_BINARY);

    binary = energy_color == BLUE ? blue_binary - red_binary : red_binary - blue_binary; //滤掉白光
    //Ticks(time_energy, "threshold");

//TODO 用相机时卷积核用3*3，视频测试亮度低用5*5
    Mat element_dilate_1 = getStructuringElement(MORPH_RECT, Size(5, 5));
    dilate(binary, binary, element_dilate_1);
    morphologyEx(binary, binary, MORPH_CLOSE, element_dilate_1);
    threshold(binary, binary, 0, 255, THRESH_BINARY);
    GaussianBlur(binary, binary, Size(3, 3), 0);//???
    imshow("bin", binary);
}

/**
 * @brief EnergyDetector::detectArmor
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的装甲
 */
bool EnergyDetector::detectArmor() {
    //armor dilate
    //寻找所有装甲
    std::vector<vector<Point> > armor_contours(contours);

    for (auto &i: contours_external)//去除外轮廓
    {
        auto external_contour_size = i.size();
        for (auto &j: armor_contours) {
            auto all_size = j.size();
            if (external_contour_size == all_size) {
                swap(j, armor_contours[armor_contours.size() - 1]);
                armor_contours.pop_back();//清除掉流动条
                break;
            }
        }
    }
    //对装甲板进行筛选
    for (auto &armor_contour: armor_contours) {
        if (!geometryFilter(armor_contour,
                            &energy_param.armor_contour_area,
                            nullptr,
                            &energy_param.armor_contour_length,
                            &energy_param.armor_contour_width,
                            &energy_param.armor_contour_hw_ratio
        ))
            continue;
        RotatedRect flow_rect = minAreaRect(armor_contour);
        //todo 这里计算了两次minAreaRect
        valid_armors.emplace_back(flow_rect);
        armor_centers.emplace_back(flow_rect.center);//回传所有装甲板center到armor_center容器中
    }
    //Ticks(time_energy, "detectArmor");
    if (valid_armors.empty()) {
        LOG(WARNING) << "CAN'T DETECT ARMOR";
        return false;
    }
    return true;

}

/**
 * @brief EnergyDetector::detectFlowStripFan
 * @param Mat& src
 * @return null
 * @remark 检测所有可能的流动条所在的装甲板
 */
bool EnergyDetector::detectFlowStripFan() {

    for (auto &flow_strip_fan_contour: contours_external) {
        if (!geometryFilter(flow_strip_fan_contour,
                            &energy_param.flow_strip_fan_contour_area,
                            &energy_param.flow_strip_fan_contour_area_ratio,
                            &energy_param.flow_strip_fan_contour_length,
                            &energy_param.flow_strip_fan_contour_width,
                            &energy_param.flow_strip_fan_contour_hw_ratio
        ))
            continue;

        //cout << "flow Area : " << contourArea(flow_strip_fan_contour) << endl;

        RotatedRect flow_rect = cv::minAreaRect(flow_strip_fan_contour);
        //cout << "flow width : "<< flow_rect.size.width << "\tflow height : " << flow_rect.size.height << endl;

        Point2f flow_pts[4];
        flow_rect.points(flow_pts);

        valid_fan_strip.emplace_back(flow_rect);
    }
    //Ticks(time_energy, "detectFlowFan");
    if (valid_fan_strip.empty()) {
        LOG(WARNING) << "CAN'T DETECT FLOW";
        return false;
    }
    return true;
}
//Point2f EnergyDetector::calR() {
//    int center_blade_edge_index;
//    int max_length = 0;
//    Point2f blade_points[4];
//    valid_fan_strip[target_blade.flow_strip_fan_index].points(blade_points);
//
//    for (int i = 0; i < 4; i++) {
//        float cur_getTargetPoint_length = getDistance((blade_points[i] + blade_points[(i + 1) % 4]) / 2.0,
//                                                        target_armor.center);
//        if (cur_getTargetPoint_length > max_length) {
//            max_length = cur_getTargetPoint_length;
//            center_blade_edge_index = i;
//        }
//    }
//
//    Point2f blade_bounding_mid =
//            (blade_points[center_blade_edge_index] + blade_points[(center_blade_edge_index + 1) % 4]) /
//            2; //扇叶靠近圆心的边的中点
//
//    Point2f armor_bounding_mid;
//    Point2f armor_mid_vec;
//    Point2f armor2center_vec;
//    for (int j = 0; j < 4; j++) {
//        armor_bounding_mid = (pts[j] + pts[(j + 1) % 4]) / 2;
//        armor_mid_vec = armor_bounding_mid - target_point;
//        float d = (blade_bounding_mid - target_point).dot(armor_mid_vec);
//        if (d > 500) {
//            armor2center_vec = armor_mid_vec;
//            break;
//        }
//    }
//    Point2f center_point = target_point + K * armor2center_vec;
//    return center_point;
//}
/**
 * @brief
 * @remark 检测提取风车中心点
 */
bool EnergyDetector::detectCircleCenter() {
    Mat CannyEdge;
    vector<vector<Point> > circle_contours;
    vector<Vec3f> circle_point;

    //Point2f cal_center = calR(); //反解的圆心位置用于判断检测圆心的可信度
//    Canny(binary, CannyEdge, 50, 150);
//    findContours(CannyEdge, circle_contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

    for (auto &circle_contour: contours_external) {
        if (!geometryFilter(circle_contour,
                            &energy_param.centerR_contour_area,
                            nullptr,
                            nullptr,
                            nullptr,
                            &energy_param.centerR_contour_hw_ratio
        ))
            continue;

        Rect rect = boundingRect(circle_contour);
        int x = rect.x + rect.width / 2,
                y = rect.y + rect.height / 2;
        //if (getDistance(cal_center, Point(x, y)) < 50)
        {
            circle_center_point = Point(x, y);
            //circle(outline, circle_center_point, 3, Scalar(0, 255, 0), 2, 8, 0);
            return true;
        }


    }
    LOG(WARNING) << "CAN'T DETECT CenterR";
    return false;
}

//找出矩形的四个顶点
void EnergyDetector::getPts(RotatedRect armor) {
    Point2f rectPoints[4];//定义矩形的4个顶点
    armor.points(rectPoints); //计算矩形的4个顶点

    //judge long side
    if (sqrt(pow((rectPoints[0].x - rectPoints[1].x), 2) + pow((rectPoints[0].y - rectPoints[1].y), 2))
        > sqrt(pow((rectPoints[2].x - rectPoints[1].x), 2) + pow((rectPoints[2].y - rectPoints[1].y), 2))) {
        //pts[0]-pts[1] is long side
        pts[0] = rectPoints[0];
        pts[1] = rectPoints[1];
        pts[2] = rectPoints[2];
        pts[3] = rectPoints[3];
    } else {
        //pts[1]-pts[2] is long side
        pts[0] = rectPoints[1];
        pts[1] = rectPoints[2];
        pts[2] = rectPoints[3];
        pts[3] = rectPoints[0];
    }
}

/**
 * @brief 计算当前差角及角速度
 * */
void EnergyDetector::calOmega(float timestamp) {
    static fstream fs("omega.csv", ios::in | ios::out | ios::trunc);
    Point2f p = circle_center_point - target_point;    //指向圆心的向量
    p.y /= 0.96;
    float cur_theta = atan2(p.y, p.x) * DEG;     //当前角度
    float dist_ct = getDistance(circle_center_point, target_point);
    time_series.push_back(timestamp / 1000); //记录下当前的时间戳
    theta_series.push_back(cur_theta);

    if (theta_series.size() <= 3)return;

    float delta_theta = (cur_theta - *(theta_series.end() - 3));  //相隔3个数相减  size()-1-3
    float delta_time = (timestamp / 1000 - *(time_series.end() - 3));
    delta_theta > 0 ? clockwise_cnt++ : clockwise_cnt--;
    delta_theta = abs(delta_theta);
    if (delta_theta > 300) //解决 180 -180 跳变问题
        delta_theta = 360 - delta_theta;
    float cur_omega = delta_theta / delta_time / DEG; //转为弧度制,算3帧的角速度
//    //cout << "--- current spd : " << cur_omega << endl;
    if (cur_omega > 2.5)//角速度限幅
    {
        cout << cur_omega << endl;
        return;
    }
    if (omega_series.size() > 200)//todo
        estimateParam(omega_series, time_series, 0);
    omega_series.push_back(cur_omega); //将当前的 cur_omega 存放在 omega 数组中
    smooth_time_series.push_back(timestamp + 0.5 * delta_time);

    fs << timestamp + 0.5 * delta_time << "," << cur_omega << "," << dist_ct << endl;
}


void EnergyDetector::loadConfig() {
    energy_param.armor_contour_area = {400, 1400};
    energy_param.armor_contour_length = {25, 55};
    energy_param.armor_contour_width = {10, 40};
    energy_param.armor_contour_hw_ratio = {1, 3};

    energy_param.flow_strip_fan_contour_area = {2000, 4400};
    energy_param.flow_strip_fan_contour_length = {90, 160};
    energy_param.flow_strip_fan_contour_width = {45, 70};
    energy_param.flow_strip_fan_contour_hw_ratio = {1.2, 2.8};
    energy_param.flow_strip_fan_contour_area_ratio = {0.20, 0.55};

    energy_param.centerR_contour_area = {150, 1000};
    energy_param.centerR_contour_hw_ratio = {0.8, 1.2};

    energy_param.target_intersection_contour_area_min = 20;

}

EnergyDetector::EnergyDetector(RobotColor color, EnergyMode mode) : energy_color(color), energy_mode(mode) {
    loadConfig();
}

void EnergyDetector::changeMode(RobotColor color, EnergyMode mode) {
    energy_color = color;
    energy_mode = mode;
}

void EnergyDetector::drawOutline() {
    static Mat outline = Mat(binary.size(), CV_8UC3, Scalar(0, 0, 0)); //轮廓成员
//    for (auto armor_rect: valid_armors) {
//        Point2f armor_pts[4];
//        armor_rect.points(armor_pts);
//        for (int i = 0; i < 4; i++)
//            line(outline, armor_pts[i], armor_pts[(i + 1) % (4)],
//                 Scalar(255, 255, 255), 2, LINE_8);
//
//    }
//    for (auto strip_rect: valid_fan_strip) {
//        Point2f flow_pts[4];
//        strip_rect.points(flow_pts);
//        for (int i = 0; i < 4; i++)
//            line(outline, flow_pts[i], flow_pts[(i + 1) % (4)],
//                 Scalar(255, 255, 0), 2, LINE_8);
//
//    }
    circle(outline, circle_center_point, 3, Scalar(0, 255, 0), 2, 8, 0);
    circle(outline, target_point, 2, Scalar(0, 255, 0), 3);
    stringstream fmt;
    if (!omega_series.empty())
        fmt << "theta:" << theta_series.back() << " omega:" << omega_series.back();
    putText(outline, fmt.str(), Point(20, 20), FONT_HERSHEY_SIMPLEX, 1, Scalar(255, 255, 255));
    imshow("outline", outline);
    waitKey(1);
}


void EnergyDetector::estimateParam(vector<float> omega_, vector<float> t_, int times) {
    double a_ = 0.89, w_ = 1.94, phi_ = 0; //初始值
    Problem problem;

    for (int i = 0; i < omega_.size(); i++) {
        ceres::CostFunction *cost_func =
                new ceres::AutoDiffCostFunction<SinResidual, 1, 1, 1, 1>(
                        new SinResidual(t_[i], omega_[i])); //确定拟合问题是横坐标问题，需要初始化第一个坐标为 0
        problem.AddResidualBlock(cost_func,
                                 nullptr,
                                 &a_, &w_, &phi_);
    }
    Solver::Options options;
    Solver::Summary summary;
    options.max_num_iterations = 25;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    LOG(WARNING) << "Initial A: " << 0.0 << " W: " << 0.0 << "\n";
    problem.SetParameterLowerBound(&a_, 0, 0.780);
    problem.SetParameterUpperBound(&a_, 0, 1.045);
    problem.SetParameterLowerBound(&w_, 0, 1.884);
    problem.SetParameterUpperBound(&w_, 0, 2.000);
    problem.SetParameterLowerBound(&phi_, 0, -CV_PI);
    problem.SetParameterUpperBound(&phi_, 0, CV_PI);
    Solve(options, &problem, &summary);
    LOG(INFO) << summary.BriefReport() << "\n";

    LOG(WARNING) << "Final   a: " << a_ << " w: " << w_ << " phi: " << phi_ << "\n";
    waitKey(0);

}

bool EnergyDetector::getTargetPoint() {
    //todo find armor in best_fan 目标armor
    vector<RotatedRect> target_blades;
    for (auto &fan_strip: valid_fan_strip) {
        //为target_armors打分
        for (auto &armor: valid_armors) {
            std::vector<cv::Point2f> intersection;
            if (rotatedRectangleIntersection(armor, fan_strip, intersection) == 0) //计算两个旋转矩形的交集面积
                continue;
            double cur_contour_area = contourArea(intersection);
            if (cur_contour_area > energy_param.target_intersection_contour_area_min) {
                target_blades.emplace_back(armor);
            }
        }
    }
    if (!target_blades.empty()) {
        if (target_blades.size() == 1) {
            target_armor = RotatedRect(target_blades.front());
        } else {
            //为target_armors打分
            //todo 用container
            float max_grade = 0, grade;
            auto max_armor = target_blades.begin();

            for (auto candidate = target_blades.begin(); candidate != target_blades.end(); candidate++) {
                grade = getDistance((*candidate).center,
                                    last_target_armor.center);      //距离远的为新的待打击装甲
                if (grade > max_grade) {
                    max_armor = candidate;
                    max_grade = grade;
                }
            }
            target_armor = RotatedRect(*max_armor);
        }
        last_target_armor = target_armor;        //update
    } else {
        return false;
    }
    target_point = target_armor.center;
    target_armor_centers.push_back(target_point);
    return true;
}