//
// Created by zyb on 24-4-22.
//

#ifndef BUPT_DOG_CONTROLLER2_LOW_STATE_HPP
#define BUPT_DOG_CONTROLLER2_LOW_STATE_HPP

#include <iostream>
#include <thread>
// lcm
#include "doglcm/ImuData_t.hpp"
#include "doglcm/LegData_t.hpp"
#include "doglcm/UserCmd_t.hpp"
#include <lcm/lcm-cpp.hpp>
// utils
#include "safety_param.hpp"
#include "utils/math_tools.hpp"
#include "utils/math_types.hpp"
#include "utils/timer.hpp"

class LowState {
    //brief 包含低级状态信息：关节信息、IMU信息、用户输入指令
public:
    LowState() {
        // lcm
        _lcm.subscribe("imu", &LowState::handleImuMsg, this);
        for (int i = 0; i < LEG_NUM; ++i) {
            _lcm.subscribe("leg" + std::to_string(i) + "/leg_data", &LowState::handleLegMsg, this);
        }
        _lcm.subscribe("user_cmd", &LowState::handleUserCmdMsg, this);

        _user_cmd = std::make_shared<doglcm::UserCmd_t>();
        _user_cmd->gait_type = 0x00;
        for (int i = 0; i < ONE_LEG_DOF_NUM; ++i) {
            _user_cmd->cmd_angular_velocity[i] = 0.0;
            _user_cmd->cmd_linear_velocity[i] = 0.0;
        }
        for (auto &filter: _rpy_filter) {
            filter = std::make_shared<LPFilter>(0.005, 5);
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
    const Vec3 &getRpy() { return _rpy_filtered; }

    const RotMat &getRotMat() { return _rot_mat; }

    const Vec3 &getEulerAngularVelocity() { return _euler_angular_velocity; }

    const Vec3 &getAngularVelocity() { return _angular_velocity; }

    const Vec3 &getAngularVelocity_inWorld() { return _angular_velocity_in_world; }

    const Vec3 &getLinearAccelerometer() { return _linear_accelerometer; }

    const Vec3 &getLinearAccelerometer_inWorld() { return _linear_accelerometer_in_world; }

    const Quat &getQuaternion() { return _quat; }

    // gps
    const Vec3 &getGpsVel() { return _gps_vel; }

private:
    std::thread _recv_thread;
    // lcm
    lcm::LCM _lcm;
    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    // motor
    Vec12 _q = Vec12::Zero();
    Vec12 _dq = Vec12::Zero();
    Vec12 _tau = Vec12::Zero();
    // imu
    Vec3 _rpy = Vec3::Zero();
    Vec3 _rpy_filtered = Vec3::Zero();
    RotMat _rot_mat = RotMat::Identity();
    Vec3 _euler_angular_velocity = Vec3::Zero();
    Vec3 _angular_velocity = Vec3::Zero();
    Vec3 _angular_velocity_in_world = Vec3::Zero();
    Vec3 _linear_accelerometer = Vec3::Zero();
    Vec3 _linear_accelerometer_in_world = Vec3::Zero();
    Quat _quat = Quat::Zero(); // x y z w
    // gps
    Vec3 _gps_vel = Vec3::Zero();
    //LP filter
    std::shared_ptr<LPFilter> _rpy_filter[3];

    [[noreturn]] void run() {
        while (true) {
            _lcm.handle();
        }
    }

    void handleImuMsg(const lcm::ReceiveBuffer *, const std::string &, const doglcm::ImuData_t *msg) {
        // rpy
        static double last_yaw = 0.0;
        double delta_yaw = msg->rpy[2] - last_yaw;
        last_yaw = msg->rpy[2];
        if (delta_yaw > M_PI)
            delta_yaw -= 2.0 * M_PI;
        else if (delta_yaw < -M_PI)
            delta_yaw += 2.0 * M_PI;
        _rpy << msg->rpy[0], msg->rpy[1], _rpy[2] + delta_yaw;
        for (int i = 0; i < 3; ++i) {
            _rpy_filter[i]->addValue(_rpy[i]);
        }
        _rpy_filtered << _rpy_filter[0]->getValue(), _rpy_filter[1]->getValue(), _rpy_filter[2]->getValue();
        // R
        _rot_mat = rotMatRzyx(getRpy());
        // angular_velocity
        memcpy(_angular_velocity.data(), msg->angular_velocity, sizeof(_angular_velocity));
        _angular_velocity_in_world = _rot_mat * _angular_velocity;
        _euler_angular_velocity = rotMatEluerVel2BodyOmega(getRpy()).inverse() * _angular_velocity;
        // linear_accelerometer
        memcpy(_linear_accelerometer.data(), msg->linear_accelerometer, sizeof(_linear_accelerometer));
        _linear_accelerometer_in_world = _rot_mat * _linear_accelerometer;
        // quat x y z w
        Eigen::Quaterniond quaternion = Eigen::AngleAxisd(getRpy()[2], Eigen::Vector3d::UnitZ()) *
                                        Eigen::AngleAxisd(getRpy()[1], Eigen::Vector3d::UnitY()) *
                                        Eigen::AngleAxisd(getRpy()[0], Eigen::Vector3d::UnitX());
        _quat << quaternion.x(), quaternion.y(), quaternion.z(), quaternion.w();
        // gps_vel
#ifdef USE_SIM
        memcpy(_gps_vel.data(), msg->gps_vel, sizeof(_gps_vel));
#endif
    }

    void handleLegMsg(const lcm::ReceiveBuffer *, const std::string &, const doglcm::LegData_t *msg) {
        for (int i = 0; i < ONE_LEG_DOF_NUM; ++i) {
            _q[3 * msg->leg_id + i] = msg->joint_data[i].Pos;
            _dq[3 * msg->leg_id + i] = msg->joint_data[i].W;
            _tau[3 * msg->leg_id + i] = msg->joint_data[i].T;
        }
    }

    void handleUserCmdMsg(const lcm::ReceiveBuffer *, const std::string &, const doglcm::UserCmd_t *msg) {
        memcpy(_user_cmd.get(), msg, sizeof(*_user_cmd));
        _user_cmd->cmd_linear_velocity[0] = clip(msg->cmd_linear_velocity[0], Vec2(BASE_MAX_CMD_VX, BASE_MIN_CMD_VX));
        _user_cmd->cmd_linear_velocity[1] = clip(msg->cmd_linear_velocity[1], Vec2(BASE_MAX_CMD_VY, -BASE_MAX_CMD_VY));
        _user_cmd->cmd_angular_velocity[2] = clip(msg->cmd_angular_velocity[2],
                                                  Vec2(BASE_MAX_CMD_DYAW, -BASE_MAX_CMD_DYAW));
    }
};

#endif //BUPT_DOG_CONTROLLER2_LOW_STATE_HPP
