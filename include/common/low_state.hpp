//
// Created by zyb on 24-4-22.
//

#ifndef BUPT_DOG_CONTROLLER2_LOW_STATE_HPP
#define BUPT_DOG_CONTROLLER2_LOW_STATE_HPP

#include <thread>
#include <chrono>
#include <iostream>
#include <functional>
// lcm
#include <lcm/lcm-cpp.hpp>
#include "doglcm/ImuData_t.hpp"
#include "doglcm/LegData_t.hpp"
#include "doglcm/UserCmd_t.hpp"
// utils
#include "utils/math_types.hpp"
#include "utils/math_tools.hpp"

class LowState {
public:
    LowState() {
        // lcm
        _lcm.subscribe("imu", &LowState::handleImuMsg, this);
        for (int i = 0; i < 4; ++i) {
            _lcm.subscribe("leg" + std::to_string(i) + "/leg_data", &LowState::handleLegMsg, this);
        }
        _lcm.subscribe("user_cmd", &LowState::handleUserCmdMsg, this);

        _user_cmd = std::make_shared<doglcm::UserCmd_t>();
        _user_cmd->gait_type = 0x00;
        for (int i = 0; i < 3; ++i) {
            _user_cmd->cmd_angular_velocity[i] = 0.0;
            _user_cmd->cmd_linear_velocity[i] = 0.0;
        }
        _recv_thread = std::thread([this] { run(); });
        std::cout << "[LowState] Init Success!" << std::endl;
    }

    void begin() { _recv_thread.join(); }

    // user cmd
    const std::shared_ptr<doglcm::UserCmd_t> &getUserCmd() { return _user_cmd; }

    // joint
    const Vec12 &getQ() { return _q; }

    const Vec12 &getDq() { return _dq; }

    const Vec12 &getTau() { return _tau; }

    // imu
    const Vec3 &getRpy() { return _rpy; }

    const RotMat &getRotMat() { return _rot_mat; }

    const Vec3 &getAngularVelocity() { return _angular_velocity; }

    const Vec3 &getLinearAccelerometer() { return _linear_accelerometer; }

    const Vec3 &getLinearAccelerometer_inWorld() { return _linear_accelerometer_in_world; }

    const Quat &getQuaternion() { return _quat; }

private:
    std::thread _recv_thread;
    // lcm
    lcm::LCM _lcm;
    doglcm::LegData_t _legs[4]{};
    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    // motor
    Vec12 _q = Vec12::Zero();
    Vec12 _dq = Vec12::Zero();
    Vec12 _tau = Vec12::Zero();
    // imu
    Vec3 _rpy = Vec3::Zero();
    RotMat _rot_mat = RotMat::Identity();
    Vec3 _angular_velocity = Vec3::Zero();
    Vec3 _linear_accelerometer = Vec3::Zero();
    Vec3 _linear_accelerometer_in_world = Vec3::Zero();
    Quat _quat = Quat::Zero(); // x y z w

    [[noreturn]] void run() {
        while (true) {
            _lcm.handle();
        }
    }

    void handleImuMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const doglcm::ImuData_t *msg) {
        // rpy
        static double last_yaw = 0.0;
        double delta_yaw = msg->rpy[2] - last_yaw;
        last_yaw = msg->rpy[2];
        if (delta_yaw > M_PI)
            delta_yaw -= 2.0 * M_PI;
        else if (delta_yaw < -M_PI)
            delta_yaw += 2.0 * M_PI;
        _rpy << msg->rpy[0], msg->rpy[1], _rpy[2] + delta_yaw;
//        std::cout << delta_yaw << " " << msg->rpy[2] << " " << _rpy[2] << std::endl;
        // R
        _rot_mat = rotMatRzyx(_rpy);
        // angular_velocity
        memcpy(_angular_velocity.data(), msg->angular_velocity, sizeof(_angular_velocity));
        // linear_accelerometer
        memcpy(_linear_accelerometer.data(), msg->linear_accelerometer, sizeof(_linear_accelerometer));
        _linear_accelerometer_in_world = _rot_mat * _linear_accelerometer;
        // quat x y z w
        Eigen::Quaterniond quaternion = Eigen::AngleAxisd(msg->rpy[2], Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(msg->rpy[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(msg->rpy[0], Eigen::Vector3d::UnitX());
        _quat << quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w();
    }

    void handleLegMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const doglcm::LegData_t *msg) {
        memcpy(&_legs[msg->leg_id], msg, sizeof(_legs[msg->leg_id]));
        for (int i = 0; i < 3; ++i) {
            _q[3 * msg->leg_id + i] = msg->joint_data[i].Pos;
            _dq[3 * msg->leg_id + i] = msg->joint_data[i].W;
            _tau[3 * msg->leg_id + i] = msg->joint_data[i].T;
        }
    }

    void handleUserCmdMsg(const lcm::ReceiveBuffer *rbuf, const std::string &chan, const doglcm::UserCmd_t *msg) {
        memcpy(_user_cmd.get(), msg, sizeof(_user_cmd));
    }
};


#endif //BUPT_DOG_CONTROLLER2_LOW_STATE_HPP
