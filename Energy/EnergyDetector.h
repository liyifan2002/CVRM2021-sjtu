#ifndef ENERGY_H
#define ENERGY_H

#include <iostream>
#include <cmath>
#include <chrono>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <ceres/ceres.h>
//#include <utils/oscilloscope.h>

using namespace std;
using namespace cv;
using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solve;
using ceres::Solver;

#define SHOOT_TIME 1
#define FPS 60
#define OMEGA 1.884
#define MAX_ANGLE 75
#ifdef DAHUA
#define IMGWIDTH 1024
#define IMGHEIGHT 820
#endif
#ifdef MIND
#define IMGWIDTH 1280
#define IMGHEIGHT 1024
#endif

//风车参数
struct WindmillParamFlow {
    int RED_GRAY_THRESH;//敌方红色时的阈值
    int BLUE_GRAY_THRESH;//敌方蓝色时的阈值
    float armor_contour_area_max;//装甲板的相关筛选参数
    float armor_contour_area_min;
    float armor_contour_length_max;
    float armor_contour_length_min;
    float armor_contour_width_max;
    float armor_contour_width_min;
    float armor_contour_hw_ratio_max;
    float armor_contour_hw_ratio_min;

    float flow_strip_fan_contour_area_max;//流动条所在扇叶的相关筛选参数
    float flow_strip_fan_contour_area_min;
    float flow_strip_fan_contour_length_max;
    float flow_strip_fan_contour_length_min;
    float flow_strip_fan_contour_width_max;
    float flow_strip_fan_contour_width_min;
    float flow_strip_fan_contour_hw_ratio_max;
    float flow_strip_fan_contour_hw_ratio_min;
    float flow_strip_fan_contour_area_ratio_max;
    float flow_strip_fan_contour_area_ratio_min;

    float Strip_Fan_Distance_max;//流动条到装甲板距离参数
    float Strip_Fan_Distance_min;

    float flow_strip_contour_area_max;//流动条相关参数筛选
    float flow_strip_contour_area_min;
    float flow_strip_contour_length_max;
    float flow_strip_contour_length_min;
    float flow_strip_contour_width_max;
    float flow_strip_contour_width_min;
    float flow_strip_contour_hw_ratio_max;
    float flow_strip_contour_hw_ratio_min;
    float flow_strip_contour_area_ratio_min;
    float flow_strip_contour_intersection_area_min;

    long target_intersection_contour_area_min;

    float twin_point_max;

    float Center_R_Control_area_max;//中心R的相关参数筛选
    float Center_R_Control_area_min;
    float Center_R_Control_length_max;
    float Center_R_Control_length_min;
    float Center_R_Control_width_max;
    float Center_R_Control_width_min;
    float Center_R_Control_radio_max;
    float Center_R_Control_radio_min;
    float Center_R_Control_area_radio_min;
    float Center_R_Control_area_intersection_area_min;

    float flow_area_max;//扇叶相关参数筛选
    float flow_area_min;
    float flow_length_max;
    float flow_length_min;
    float flow_width_max;
    float flow_width_min;
    float flow_aim_max;
    float flow_aim_min;
    float flow_area_ratio_min;
};

typedef struct {
    int radius;
    float angle;
    double time_stamp;
} polarLocal;

typedef struct Blade_{
    int armor_index;
    int flow_strip_fan_index;
    Blade_(int i_, int j_)
    {
        flow_strip_fan_index = i_;
        armor_index = j_;
    }

    Blade_()
    {
        armor_index = 0;
        flow_strip_fan_index = 0;
    }
}Blade;

class EnergyDetector_{
public:
    explicit EnergyDetector();//构造函数
    ~EnergyDetector();//析构函数
    void EnergyTask(const Mat &src, int8_t mode, const float deltaT);//接口
    void init();
    vector<Point2f> pts;
    vector<Point2f> predict_pts;
    cv::Point2f target_point;//目标装甲板中心坐标
    cv::Point2f predict_point;//预测的击打点坐标
    cv::Point2f circle_center_point;//风车圆心坐标
    std::vector<cv::Point2f> target_armor_centers;//get R

    bool detect_flag = false;
    float cur_phi = 0;
    float cur_omega = 0;
    float predict_rad = 0;

private:
    const float R = 168;
    const float PI = 3.14;
    const float K = 10; //半径倍数

    polarLocal polar_t;
    bool show_armors; //是否显示所有装甲
    bool show_target_armor; //是否显示目标装甲
    bool show_strip_fan;//是否显示有流动条的扇叶
    bool show_center_R;//是否显示中心点R
    bool show_target_point;//是否显示目标点
    bool show_predict_point;//是否显示预测点
    bool BIG_MODE = true;//是否为大符模式
    bool inter_flag = false;//是否contour有交集
    bool start_flag = false;//是否开始预测
    cv::Point2f last_circle_center_point;//上一次风车圆心坐标
    cv::Point2f last_target_point;//上一次目标装甲板坐标

    WindmillParamFlow _flow;

    void clearAll();//清空所有容器vector
    void initEnergy();//能量机关初始化
    void initEnergyPartParam();//能量机关参数初始化
    static Mat preprocess(const Mat& src);

    bool detectArmor(Mat &src);
    bool detectFlowStripFan(Mat &src);
    bool detectR(Mat &src, Mat &show);
    bool getTargetPoint(Mat &src);

    Point2f calR();
    bool detectCircleCenter(Mat &src);

    void findROI(Mat &src, Mat &dst);
    void roiPoint2src();
    Point2f roi_sp; //roi的左上角起始点

    bool isValidArmorContour(const vector<cv::Point>& armor_contour) const;//装甲板矩形尺寸要求
    bool isValidCenterRContour(const vector<cv::Point>& center_R_contour);//风车中心选区尺寸要求
    bool isValidFlowStripFanContour(cv::Mat& src, const vector<cv::Point>& flow_strip_fan_contour) const;//流动条扇叶矩形尺寸要求

    static double pointDistance(const cv::Point& point_1, const cv::Point& point_2);//计算两点距离
    static double magnitude(const cv::Point& p);
    polarLocal toPolar(Point cart, double time_stamp);
    Point2f toCartesian(polarLocal pol);
    Point rotate(cv::Point target_point) const;
    float calPreAngle(float start_time,float end_time);
    //void getPredictPoint(Mat src);
    void getPts(RotatedRect armor);


    //Point2f calPredict(float theta) const;
    Point2f calPredict(Point2f p, Point2f center, float theta) const;

    Mat roi;

/*** *** *** *** *** ***/

/*** new predict ***/
    float spd_int(float t);
    float spdInt(float t);
    float spd_phi(float omega, int flag);
    float spdPhi(float omega, int flag);
    void calOmega(float deltaT);
    float startT = 0;
    vector<float> delta_theta;
    vector<float> theta;
    vector<float> omega;
    vector<float> av_omega;
    vector<float> x_list;
    vector<float> predict_arr;
    vector<float> filter_omega;
    float sum_time;
    float init_time;
    //float predict_arr[6];
    int predict_cnt = 0;
    int angle_length = 4;
    int omega_length = 6;

    int flag = 0;
    int last_flag = 0;
    void getPredictPointSmall(const Mat& src);
    void getPredictPoint(const Mat& src,float deltaT);
    void getPredictRect(float theta, vector<Point2f> pts);
    //Oscilloscope osc= Oscilloscope ("asd",{"omega","theta"});

    float min_omega = 5 , max_omega = 0;
    float min_t, max_t;
/*** *** *** *** *** ***/

    /***/
    bool judgeRotation(const Mat &src, const float deltaT);
    void estimateParam(vector<float>omega_, vector<float>t_, int times);
    vector<float> time_series; //记录每次的时间
    vector<float> omega_series; //记录omega的值
    Problem problem;
    int cnt_t = 0, cnt_i = 0;
    double a_ = 0.780, w_ = 1.884, phi_ = 2.09 - 0.78; //参数初值
    /***/

    std::vector<Blade> target_blades;//可能的目标装甲板
    std::vector<cv::RotatedRect> armors;//图像中所有可能装甲板
    std::vector<cv::RotatedRect> valid_fan_strip;//可能的流动扇叶
    std::vector<cv::RotatedRect> centerRs;//可能的中心
    std::vector<cv::Point2f> armor_centers;//用来做最小二乘拟合
    std::vector<RotatedRect> valid_armors;//合法的裝甲板

    cv::Rect center_r_area;
    cv::RotatedRect centerR;//风车中心字母R
    cv::RotatedRect pre_centerR;//风车中心字母R

    Blade target_blade;//目标扇叶
    cv::RotatedRect target_armor;//目标装甲板
    cv::RotatedRect last_target_armor;//上一次目标装甲板

    int energy_rotation_direction = 1;//风车旋转方向
    int clockwise_rotation_init_cnt;//装甲板顺时针旋转次数
    int anticlockwise_rotation_init_cnt;//装甲板逆时针旋转次数
    bool energy_rotation_init;//若仍在判断风车旋转方向，则为true
    void initRotation();//对能量机关旋转方向进行初始化
    void updateLastValues();//更新上一次的各种值

    //预测提前角
    float predict_rad_norm;//预测提前角的绝对值

    int misscount = 0;

    float target_polar_angle;//待击打装甲板的极坐标角度
    float last_target_polar_angle_judge_rotation;//上一帧待击打装甲板的极坐标角度（用于判断旋向）

    list<polarLocal> history_target_armor_polar;
    polarLocal predict_polar;
    polarLocal target_polar;
    //todo predict task

    int64 last_frame_time;
    int64 frame_time;


    void em();
};
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
#endif //ENERGY_H

