// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/1/18.
//

#ifndef CV2022_CALIBRATION_H
#define CV2022_CALIBRATION_H

#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

using namespace std;
using namespace cv;

class myCalibration {
public:
    static vector<vector<Point3f> > point_grid_pos;
    static vector<vector<Point2f> > point_pix_pos;
    static vector<Point3f> point_grid_pos_buf;
    static Mat intrinsicMatrix;
    static Mat distCoeffs;
    static vector<Mat> rvec, tvec;
    static Size board_size;
    static Size square_size;
    static Size img_size;
    static int cnt;

    static void init();

    static void inputIMG(const Mat &img);

    static bool calibrate();

    static bool saveCalibParam();

    static bool readCalibParam();
};

Mat myCalibration::intrinsicMatrix(3, 3, CV_32FC1, Scalar::all(0)),
        myCalibration::distCoeffs(1, 5, CV_32FC1, Scalar::all(0));
Size myCalibration::board_size(5, 8),
        myCalibration::square_size(20, 20),
        myCalibration::img_size;
vector<Mat> myCalibration::rvec, myCalibration::tvec;
vector<vector<Point3f> > myCalibration::point_grid_pos;
vector<vector<Point2f> > myCalibration::point_pix_pos;
vector<Point3f> myCalibration::point_grid_pos_buf;
int myCalibration::cnt = 0;

void myCalibration::init() {
    for (int i = 0; i < board_size.height; i++) {
        for (int j = 0; j < board_size.width; j++) {
            Point3f pt;
            pt.y = static_cast<float>(i * square_size.height);
            pt.x = static_cast<float>(j * square_size.width);
            pt.z = 0;
            point_grid_pos_buf.push_back(pt);
        }
    }
}

void myCalibration::inputIMG(const Mat &img) {
    vector<Point2f> corners;
    Mat gray_img;
    img_size = Size(img.rows, img.cols);
    if (findChessboardCorners(img, board_size, corners)) {
        cvtColor(img, gray_img, COLOR_BGR2GRAY);
        find4QuadCornerSubpix(gray_img, corners, board_size);
        imwrite("../data/calibrate/" + to_string(cnt++) + ".jpg", img);
        drawChessboardCorners(img, board_size, corners, true);
        cv::imshow("calibration", img);
        point_grid_pos.push_back(point_grid_pos_buf);
        point_pix_pos.push_back(corners);
    }
}

bool myCalibration::calibrate() {
    double meanReprojErr = calibrateCamera(point_grid_pos, point_pix_pos, img_size, intrinsicMatrix, distCoeffs, rvec,
                                           tvec);
    cout << intrinsicMatrix << endl << distCoeffs << endl;
    saveCalibParam();
    return true;
}

bool myCalibration::saveCalibParam() {
    cv::FileStorage fs("/data/config/camera.yaml",
                       cv::FileStorage::Mode::WRITE | cv::FileStorage::FORMAT_YAML);
    if (fs.isOpened()) {
        fs << "DAHUA" << "{" << "IntrinsicMatrix" << intrinsicMatrix
           << "DistortionCoefficient" << distCoeffs
           << "}";
        return true;
    }
    return false;

}

bool myCalibration::readCalibParam() {
    return false;
}


#endif //CV2022_CALIBRATION_H
