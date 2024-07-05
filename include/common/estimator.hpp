//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_ESTIMATOR_HPP
#define BUPT_DOG_CONTROLLER2_ESTIMATOR_HPP

#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
#include <cstdio>
#include <lcm/lcm-cpp.hpp>
#include "doglcm/EstimatorData_t.hpp"

#include "yaml-cpp/yaml.h"
#include "gait/gait.hpp"
#include "utils/math_tools.hpp"
#include "robot.hpp"

#define X_NUM 18
#define U_NUM 3
#define Y_NUM 28

class Estimator {
public:
    explicit Estimator(int ms, const std::shared_ptr<Gait> &gait, const std::shared_ptr<Robot> &robot);

#ifdef USE_ES_THREAD
    void begin();
#endif

    void step();

    Vec3 getPosition() { return _xhat.segment<3>(0); }

    Vec3 getLpPosition() { return Vec3(_px_filter->getValue(), _py_filter->getValue(), _pz_filter->getValue()); }

    Vec3 getVelocity() { return _xhat.segment<3>(3); }

    Vec3 getLpVelocity() { return Vec3(_vx_filter->getValue(), _vy_filter->getValue(), _vz_filter->getValue()); }

    Vec3 getFootPos_inWorld(int leg_id) { return _xhat.segment<3>(6 + 3 * leg_id); }

private:
    void init();

    void publishEsData();

    /*QR噪声标定与读取*/
    void calibrateQR();

    static void readQR(Mat3 &Cu, Eigen::Matrix<double, Y_NUM, Y_NUM> &R_init);

    std::shared_ptr<Gait> _gait;
    std::shared_ptr<Robot> _robot;

    // measure Q R
    std::shared_ptr<AvgCov> _R_check, _u_check;
    // Linear System
    Vec3 _u; // 输入变量 acc(3)-gravity(3)
    Vec18 _xhat; // 状态变量 pos(3) | vel(3) | feetPos(3x4)
    Eigen::Matrix<double, Y_NUM, 1> _y; // 观测变量 feetPos(3x4)-pos(3x4) | vel(3x4) | feetH(4)
    Eigen::Matrix<double, Y_NUM, 1> _yhat;
    Eigen::Matrix<double, X_NUM, X_NUM> _A; // 状态转移矩阵
    Eigen::Matrix<double, X_NUM, U_NUM> _B; // 输入矩阵
    Eigen::Matrix<double, Y_NUM, X_NUM> _C; // 输出矩阵
    // Covariance Matrix
    Eigen::Matrix<double, X_NUM, X_NUM> _P; // 后验协方差
    Eigen::Matrix<double, X_NUM, X_NUM> _Ppriori; // 先验协方差
    Eigen::Matrix<double, X_NUM, X_NUM> _Q; // 过程噪声方差
    Eigen::Matrix<double, Y_NUM, Y_NUM> _R; // 测量噪声方差
    Eigen::Matrix<double, X_NUM, X_NUM> _Q_init; // 过程噪声方差初始值
    Eigen::Matrix<double, Y_NUM, Y_NUM> _R_init; // 测量噪声方差初始值
    Vec18 _Q_diag; // 调整过程噪声方差
    Mat3 _Cu; // 输入变量的协方差矩阵
    // Output Measurement
    Vec12 _feetPos2Body_inWorld; // 世界坐标系下足端相对质心的位置
    Vec12 _feetVel2Body_inWorld; // 世界坐标系下足端相对质心的速度
    Vec4 _feetH_inWorld = Vec4::Zero(); // 世界坐标系下足端高度,默认为0;
    // LU分解简化求解卡尔曼增益
    Eigen::Matrix<double, Y_NUM, Y_NUM> _S; // S = C*P*C.T + R
    Eigen::PartialPivLU<Eigen::Matrix<double, Y_NUM, Y_NUM> > _Slu; // S.lu()
    Eigen::Matrix<double, Y_NUM, 1> _Sy; // Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, Y_NUM, X_NUM> _Sc; // Sc = S.inv() * C
    Eigen::Matrix<double, Y_NUM, Y_NUM> _SR; // SR = S.inv() * R
    Eigen::Matrix<double, Y_NUM, X_NUM> _STC; // STC = (S.transpose()).inv() * C
    Eigen::Matrix<double, X_NUM, X_NUM> _IKC;

    Vec3 _omega = Vec3::Zero();
    Vec3 _acc = Vec3::Zero();
    Vec3 _g; // IKC = I - KC
    double _trust = 0.0;
    double _dt;
    double _large_variance = 100;
    // Low pass filter
    std::shared_ptr<LPFilter> _vx_filter, _vy_filter, _vz_filter;
    std::shared_ptr<LPFilter> _px_filter, _py_filter, _pz_filter;
    // lcm
    lcm::LCM _lcm;
    std::string _es_data_topic_name;
    doglcm::EstimatorData_t _es_data{};

    int _ms;
#ifdef USE_ES_THREAD
    std::thread _thread;

    [[noreturn]] void run(int ms);
#endif
};


#endif //BUPT_DOG_CONTROLLER2_ESTIMATOR_HPP
