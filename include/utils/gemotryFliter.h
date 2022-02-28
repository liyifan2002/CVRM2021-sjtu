// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/2/23.
// 根据轮廓的几何特征进行过滤
//

#ifndef CV2022_GEMOTRYFLITER_H
#define CV2022_GEMOTRYFLITER_H

#include <opencv2/opencv.hpp>
#include <vector>

#define IN_RANGE(value, range)   (range==nullptr || (value <= range->max && value >= range->min))

template<typename T>
struct range {
    T min;
    T max;
};
typedef struct range<float> rangef;
typedef struct {
    rangef contour_area;
    rangef area_ratio;
    rangef rect_length;
    rangef rect_width;
    rangef hw_ratio;
} geometryFilterParam;

//两点间距离公式
inline float getDistance(cv::Point2f pointO, cv::Point2f pointA) {
    return sqrtf(powf((pointO.x - pointA.x), 2) + powf((pointO.y - pointA.y), 2));
}


bool geometryFilter(const std::vector<cv::Point> &contour,
                    rangef *contour_area,
                    rangef *area_ratio,
                    rangef *rect_length,
                    rangef *rect_width,
                    rangef *hw_ratio);

#endif //CV2022_GEMOTRYFLITER_H
