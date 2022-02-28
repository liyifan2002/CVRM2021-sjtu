// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/2/23.
//

#ifndef CV2022_ENERGY_H
#define CV2022_ENERGY_H

#include "CV2022.h"
#include "utils/gemotryFliter.h"
#include <ceres/ceres.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;
struct EnergyParam {
    int RED_GRAY_THRESH;//敌方红色时的阈值
    int BLUE_GRAY_THRESH;//敌方蓝色时的阈值

    rangef armor_contour_area;//装甲板的相关筛选参数
    rangef armor_contour_length;
    rangef armor_contour_width;
    rangef armor_contour_hw_ratio;

    rangef flow_strip_fan_contour_area;//流动条所在扇叶的相关筛选参数
    rangef flow_strip_fan_contour_length;
    rangef flow_strip_fan_contour_width;
    rangef flow_strip_fan_contour_hw_ratio;
    rangef flow_strip_fan_contour_area_ratio;

    rangef centerR_contour_area;//中心R的相关参数筛选
    rangef centerR_contour_hw_ratio;

    float target_intersection_contour_area_min;

};

typedef struct {
    int radius;
    float angle;
    double time_stamp;
} polarLocal;

struct SinResidual{
    SinResidual(double t, double omega): omega_(omega), t_(t) {}

    template<class T>
//    bool operator()(const T* const a, const T* const phi, T* residual) const{
//        residual[0] = omega_ - (a[0] * sin(1.884 * t_ + phi[0]) + 2.09 - a[0]); // spd = a*sin(wt) + b
//        return true;
//    }
    bool operator()(const T* const a,const T* const w, const T* const phi, T* residual) const{
        residual[0] = omega_ - (a[0] * sin(w[0] * t_ + phi[0]) + 2.09 - a[0]); // spd = a*sin(w*t) + b
        return true;
    }
private:
    const double omega_;
    const double t_;
};
class EnergyDetector {
public:
    EnergyDetector(RobotColor color, EnergyMode mode);//构造函数
    void detect(const Mat &src, float timestamp);

    void changeMode(RobotColor color, EnergyMode mode);//改变状态
    void loadConfig();

    void drawOutline();

    bool detect_flag = false;
    vector<Point2f> pts;
    vector<Point2f> predict_pts;
    vector<Point2f> target_armor_centers;//get R
    vector<RotatedRect> valid_armors;//合法的裝甲板
    vector<RotatedRect> valid_fan_strip;//可能的流动扇叶
    RotatedRect target_armor, last_target_armor;//目标装甲板
    vector<Point2f> armor_centers;//用来做最小二乘拟合
    vector<float> time_series, smooth_time_series,
        delta_theta_series, theta_series, omega_series;
    Point2f circle_center_point, target_point, predict_point;
private:
    const float R = 168;
    const float DEG = 180.0 / CV_PI;
    const float K = 10; //半径倍数
    int clockwise_cnt = 0;
    EnergyParam energy_param{};
    RobotColor energy_color;
    EnergyMode energy_mode;
    Mat binary, roi;
    std::vector<vector<Point> > contours;
    std::vector<vector<Point> > contours_external;
    Point2f last_circle_center_point;//上一次风车圆心坐标
    Point2f last_target_point;//上一次目标装甲板坐标
    void preprocess(const Mat &src);

    void clearAll();

    bool detectArmor();

    bool detectFlowStripFan();

    bool detectCircleCenter();

    bool getTargetPoint();

    void getPts(RotatedRect armor);

    Point2f calR();

    void calOmega(float deltaT);

    void estimateParam(vector<float> omega_, vector<float> t_, int times);
};


#endif //CV2022_ENERGY_H
