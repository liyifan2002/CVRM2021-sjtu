// Copyright (c) 2022, HELIOS, Southwest Jiaotong University, All Rights Reserved

//
// Created by liyif on 2022/1/15.
//

#ifndef CV2022_OSCILLOSCOPE_H
#define CV2022_OSCILLOSCOPE_H

#include <string>
#include <vector>
//#include "matplotlib-cpp/matplotlibcpp.h"

namespace plt = matplotlibcpp;

class Oscilloscope {
    std::string plot_name;
    std::vector<std::string> channel_name;
    std::vector<std::vector<double>> value;
public:
    Oscilloscope(std::string plot_name, std::vector<std::string> channel_name) : plot_name(plot_name),
                                                                                 channel_name(channel_name) {
        value.push_back({});//横轴
        for (std::string n: channel_name)
            value.push_back({});
    }

    void addPoint(double x, const std::vector<double> y) {
        value[0].push_back(x);
        for (int i = 0; i < y.size(); i++)
            value[i + 1].push_back(y[i]);
    }

    void showPlt() {
        plt::clf();
        // Plot line from given x and y data. Color is selected automatically.
        // Plot a line whose name will show up as "log(x)" in the legend.
        if (value[0].size() % 10 == 0) {
            for (int i = 0; i < channel_name.size(); i++)
                plt::named_plot(channel_name[i], value[0], value[i + 1]);
            // Add graph title
            plt::title(plot_name);
            // Enable legend.
            plt::legend();
            plt::pause(0.01);
        }

    }
};


#endif //CV2022_OSCILLOSCOPE_H
