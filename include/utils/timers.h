//
// Created by Ivan Lee on 2021/12/4.
//

#pragma once
#include <chrono>

#ifdef _DEBUG
#define Ticks(tim, stageName) LOG(INFO) << stageName << ":" << tim.rec()<<"ms"
#else
#define Ticks(tim, stageName)
#endif

class Timers{
    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;
    std::chrono::duration<float, std::ratio<1, 1000>> val{};		//float精度的以毫秒为单位的时间间隔
public:
    Timers() {
        start = std::chrono::high_resolution_clock::now();
    }
    void reset(){
        start = std::chrono::high_resolution_clock::now();
    }
    double rec(){
        end = std::chrono::high_resolution_clock::now();
        val = end - start;
        reset();
        return val.count();
    }
};