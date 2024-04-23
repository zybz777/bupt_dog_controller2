//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_TIMER_HPP
#define BUPT_DOG_CONTROLLER2_TIMER_HPP

#include <chrono>
#include <string>
#include <iostream>
#include <armadillo>

// 时间戳  微秒级
inline long long getSystemTime() {
    using namespace std::chrono;
    // 获取当前时间点
    auto now = high_resolution_clock::now();
    // 将时间点转换为微秒级时间戳
    auto micros = time_point_cast<microseconds>(now).time_since_epoch();
    return micros.count();
}

// 时间戳  秒级
inline double getTimeSecond() {
    double time = (double) getSystemTime() * 1e-6;
    return time;
}

inline std::string getTimeStamp() {
    return std::to_string(getTimeSecond());
}

// 等待函数，微秒级，从startTime开始等待waitTime微秒
inline void absoluteWait(long long start_time, long long wait_time) {
    if (getSystemTime() - start_time > wait_time) {
        std::cout << "[WARNING] The waitTime=" << wait_time << " of function absoluteWait is not enough!" << std::endl
                  << "The program has already cost " << getSystemTime() - start_time << "us." << std::endl;
    }
    while (getSystemTime() - start_time < wait_time) {
        usleep(50);
    }
}

#endif //BUPT_DOG_CONTROLLER2_TIMER_HPP
