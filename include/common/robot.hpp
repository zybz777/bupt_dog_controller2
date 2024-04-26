//
// Created by zyb on 24-4-22.
//

#ifndef BUPT_DOG_CONTROLLER2_ROBOT_HPP
#define BUPT_DOG_CONTROLLER2_ROBOT_HPP

#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/dynamics.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <utility>
// common
#include "low_state.hpp"
// lcm
#include "lcm/lcm-cpp.hpp"
#include "doglcm/LegData_t.hpp"

using namespace pinocchio;
using namespace std;
const std::string foot_link[4] = {"FL_FOOT", "FR_FOOT", "HL_FOOT", "HR_FOOT"};

class Robot {
public:
    Robot(const std::shared_ptr<LowState> &low_state, int ms);

//    void begin();
    void step();

    const shared_ptr<LowState> &getLowState() { return _low_state; };

    // 关节数据
    const Vec12 &getQ() { return _low_state->getQ(); }

    const Vec12 &getDq() { return _low_state->getDq(); }

    const Vec12 &getTau() { return _low_state->getTau(); }

    // 浮动基下的关节数据
    const VecX &getFloatBaseQ() { return _q; }

    const VecX &getFloatBaseDq() { return _dq; }

    // imu
    const Vec3 &getRpy() { return _low_state->getRpy(); }

    const RotMat &getRotMat() { return _low_state->getRotMat(); }

    const Vec3 &getAngularVelocity() { return _low_state->getAngularVelocity(); }

    const Vec3 &getLinearAccelerometer() { return _low_state->getLinearAccelerometer(); }

    const Vec3 &getLinearAccelerometer_inWorld() { return _low_state->getLinearAccelerometer_inWorld(); }

    // 足端运动学数据
    Vec3 getFootPosition_inBody(int leg_id) { return _foot_pos_inBody.col(leg_id); }

    Vec3 getFootVelocity_inBody(int leg_id) { return _foot_vel_inBody.col(leg_id); }

    const Vec34 &getFootPositions_inBody() { return _foot_pos_inBody; }

    const Vec34 &getFootVelocities_inBody() { return _foot_vel_inBody; }

    Mat3 getFootJaco_inBody(int leg_id) { return _foot_jaco_inBody[leg_id]; }

    // 动力学数据
    const VecX &getNoLinearTorque() { return _nle; }

    Vec12 getLegNoLinearTorque() { return _nle.segment<12>(6); }

    const MatX &getMassMat() { return _M; }

// WBC雅可比矩阵
    const MatX &getJ_BodyOrientation() { return _J_Body_Orientation; }

    const MatX &getJ_BodyPosition() { return _J_Body_Position; }

    const MatX &getJ_FeetPosition() { return _J_Foot_Position; }

    const MatX &getDJ_BodyOrientation() { return _dJ_Body_Orientation; }

    const MatX &getDJ_BodyPosition() { return _dJ_Body_Position; }

    const MatX &getDJ_FeetPosition() { return _dJ_Foot_Position; }

    // 状态估计数据
    const Vec3 &getComPosition_inWorld() { return _com_pos_inWorld; }

    const Vec3 &getComVelocity_inWorld() { return _com_vel_inWorld; }

    const Vec3 &getComVelocity_inBody() { return _com_vel_inBody; }

    const Vec3 &getComLpVelocity_inWorld() { return _com_lp_vel_inWorld; }

    const Vec3 &getComLpVelocity_inBody() { return _com_lp_vel_inBody; }

    void setComPosition_inWorld(Vec3 pos) {
        _com_pos_inWorld = std::move(pos);
        _q.segment<3>(0) << _com_pos_inWorld;
    }

    void setComVelocity_inWorld(Vec3 vel) {
        _com_vel_inWorld = std::move(vel);
        _com_vel_inBody = getRotMat().transpose() * _com_vel_inWorld;
    }

    void setComLpVelocity_inWorld(Vec3 lp_vel) {
        _com_lp_vel_inWorld = std::move(lp_vel);
        _com_lp_vel_inBody = getRotMat().transpose() * _com_lp_vel_inWorld;
        _dq.segment<3>(0) << _com_lp_vel_inBody;
    }

private:
    void init();

//    [[noreturn]] void run(int ms);

    void forwardKinematics();

    void inverseDynamics();

    std::shared_ptr<LowState> _low_state;
    int _ms;
//    std::thread _robot_thread;
    // pinocchio
    std::unique_ptr<pinocchio::Model> _robot_model;
    std::unique_ptr<pinocchio::Data> _robot_data;
    // 关节参数
    VecX _q;  //  关节角度
    VecX _dq; //  关节速度
    // 运动学足端数据
    Vec34 _foot_pos_inWorld;   // 世界坐标系下足端位置
    Vec34 _foot_pos_inBody;    // 质心坐标系下足端位置
    Vec34 _foot_vel_inBody;    // 质心坐标系下足端速度
    Mat3 _foot_jaco_inBody[4]; // 质心坐标系下足端雅可比
    // 动力学参数
    VecX _nle;              // 非线性力矩 重力+科势力
    MatX _M;                // 广义质量矩阵
    MatX _J_contact;        // 足端雅可比矩阵 12x18
    Vec12 _friction_torque; // 摩擦力矩
    // WBC 雅可比矩阵
    MatX _J_Body_Orientation;
    MatX _J_Body_Position;
    MatX _J_Foot_Position;
    MatX _dJ_Body_Orientation;
    MatX _dJ_Body_Position;
    MatX _dJ_Foot_Position;
    // 状态估计数据
    Vec3 _com_pos_inWorld; // 状态估计质心位置
    Vec3 _com_vel_inWorld; // 状态估计质心速度
    Vec3 _com_vel_inBody;
    Vec3 _com_lp_vel_inWorld; // 状态估计+低通滤波速度
    Vec3 _com_lp_vel_inBody;
    // lcm
    lcm::LCM _lcm;
    // leg data
    std::string _foot_data_topic_name[4];
    doglcm::LegData_t _foot_data[4]{};
    std::unique_ptr<LPFilter> _foot_vel_inBody_filter[12];
    std::unique_ptr<LPFilter> _foot_pos_inBody_filter[12];
};


#endif //BUPT_DOG_CONTROLLER2_ROBOT_HPP
