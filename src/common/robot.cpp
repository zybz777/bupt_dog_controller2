//
// Created by zyb on 24-4-22.
//

#include "common/robot.hpp"

Robot::Robot(const std::shared_ptr<LowState> &low_state, int ms) {
    _low_state = low_state;
    _ms = ms;
    init();
//    _robot_thread = std::thread([this] { run(_ms); });
    std::cout << "[Robot] Init Success!" << std::endl;
}

//void Robot::begin() {
//    while (!_robot_thread.joinable()) {}
//    _robot_thread.join();
//}

void Robot::init() {
    // pinocchio
    string urdf_path = CONFIG_PATH;
    urdf_path += "robot.urdf";
    _robot_model = std::make_unique<pinocchio::Model>();
    pinocchio::urdf::buildModel(urdf_path, *_robot_model);
    _robot_data = std::make_unique<pinocchio::Data>(*_robot_model);
    // 关节参数初始化
    _q = VecX::Zero(_robot_model->nq);
    _dq = VecX::Zero(_robot_model->nv);
    // 运动学参数初始化
    _foot_pos_inWorld.setZero();
    _foot_pos_inBody.setZero();
    _foot_vel_inBody.setZero();
    // 动力学参数初始化
    _M = MatX::Zero(_robot_model->nv, _robot_model->nv);
    _nle = VecX::Zero(_robot_model->nv);
    _J_contact = MatX::Zero(12, _robot_model->nv);
    _friction_torque.setZero();
    // WBC
    _J_Body_Orientation = MatX::Zero(3, _robot_model->nv);
    // _J_Body_Orientation.block<3, 3>(0, 3) << _I3;
    _J_Body_Position = MatX::Zero(3, _robot_model->nv);
    _J_Foot_Position = MatX::Zero(12, _robot_model->nv);

    _dJ_Body_Orientation = MatX::Zero(3, _robot_model->nv);
    // _dJ_Body_Orientation.block<3, 3>(0, 3) << _I3;
    _dJ_Body_Position = MatX::Zero(3, _robot_model->nv);
    _dJ_Foot_Position = MatX::Zero(12, _robot_model->nv);
    // 状态估计数据
    _com_pos_inWorld.setZero();
    _com_vel_inWorld.setZero();
    _com_vel_inBody.setZero();
    _com_lp_vel_inWorld.setZero();
    _com_lp_vel_inBody.setZero();
    // lcm
    for (int i = 0; i < LEG_NUM; ++i) {
        _foot_data_topic_name[i] = "leg" + std::to_string(i) + "/foot_data";
    }
    // filter
    for (int i = 0; i < DOF_NUM; ++i) {
        _foot_vel_inBody_filter[i] = std::make_unique<LPFilter>((double) _ms / 1000.0, 20);
        _foot_pos_inBody_filter[i] = std::make_unique<LPFilter>((double) _ms / 1000.0, 20);
    }
}

//[[noreturn]] void Robot::run(int ms) {
//    std::chrono::microseconds period(ms * 1000);
//    std::chrono::microseconds zero_us(0);
//    auto start_time = std::chrono::high_resolution_clock::now();
//    auto end_time = std::chrono::high_resolution_clock::now();
//    auto execute_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//    while (true) {
//        start_time = std::chrono::high_resolution_clock::now();
//        step();
//        end_time = std::chrono::high_resolution_clock::now();
//        execute_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
//        auto sleep_time = period - execute_time;
//        if (sleep_time > zero_us) {
//            std::this_thread::sleep_for(sleep_time);
//        }
//    }
//}

void Robot::step() {
    _q.segment<16>(3) << _low_state->getQuaternion(), _low_state->getQ();
    _dq.segment<15>(3) << _low_state->getAngularVelocity(), _low_state->getDq();
//    _q << Vec3::Zero(), _low_state->getQuaternion(), _low_state->getQ();
//    _dq << Vec3::Zero(), _low_state->getAngularVelocity(), _low_state->getDq();
    forwardKinematics();
    inverseDynamics();
}

void Robot::forwardKinematics() {
    const RotMat &R = _low_state->getRotMat();
    pinocchio::framesForwardKinematics(*_robot_model, *_robot_data, _q);
    pinocchio::computeJointJacobiansTimeVariation(*_robot_model, *_robot_data, _q, _dq);
    Vec3 com_pos_inWorld = _robot_data->oMf[_robot_model->getFrameId("body")].translation();
    pinocchio::Data::Matrix6x J(6, _robot_model->nv);
    pinocchio::Data::Matrix6x dJ(6, _robot_model->nv);
    for (int i = 0; i < LEG_NUM; ++i) {
        // 世界坐标系下足端位置
        _foot_pos_inWorld.col(i) << _robot_data->oMf[_robot_model->getFrameId(foot_link[i])].translation();
        // 质心坐标系下足端位置
        Vec3 foot_pos_inBody = R.transpose() * (_foot_pos_inWorld.col(i) - com_pos_inWorld);
        _foot_pos_inBody_filter[3 * i + 0]->addValue(foot_pos_inBody[0]);
        _foot_pos_inBody_filter[3 * i + 1]->addValue(foot_pos_inBody[1]);
        _foot_pos_inBody_filter[3 * i + 2]->addValue(foot_pos_inBody[2]);
        _foot_pos_inBody.col(i) << _foot_pos_inBody_filter[3 * i + 0]->getValue(),
                _foot_pos_inBody_filter[3 * i + 1]->getValue(),
                _foot_pos_inBody_filter[3 * i + 2]->getValue();
        // 足端雅可比矩阵
        J.setZero();
        pinocchio::computeFrameJacobian(*_robot_model, *_robot_data, _q, _robot_model->getFrameId(foot_link[i]),
                                        pinocchio::LOCAL_WORLD_ALIGNED, J);
        _J_Foot_Position.block<3, 18>(3 * i, 0) << J.block<3, 18>(0, 0);     // pin该雅可比矩阵为世界系足端速度=J * (质心系 线速度 角速度 关节速度)
        _foot_jaco_inBody[i] << R.transpose() * J.block<3, 3>(0, 6 + 3 * i); // 将雅可比矩阵由世界系转为质心系
        // 质心坐标系下足端速度 =J*质心系关节速度
        Vec3 foot_vel_inBody = _foot_jaco_inBody[i] * _dq.segment<3>(6 + 3 * i);
        // _foot_vel_inBody.col(i) << _foot_jaco_inBody[i] * _dq.segment<3>(6 + 3 * i);
        _foot_vel_inBody_filter[3 * i + 0]->addValue(foot_vel_inBody[0]);
        _foot_vel_inBody_filter[3 * i + 1]->addValue(foot_vel_inBody[1]);
        _foot_vel_inBody_filter[3 * i + 2]->addValue(foot_vel_inBody[2]);
        _foot_vel_inBody.col(i) << _foot_vel_inBody_filter[3 * i + 0]->getValue(),
                _foot_vel_inBody_filter[3 * i + 1]->getValue(),
                _foot_vel_inBody_filter[3 * i + 2]->getValue();
        // 足端雅可比矩阵导数
        dJ.setZero();
        pinocchio::getFrameJacobianTimeVariation(*_robot_model, *_robot_data, _robot_model->getFrameId(foot_link[i]),
                                                 pinocchio::LOCAL_WORLD_ALIGNED, dJ);
        _dJ_Foot_Position.block<3, 18>(3 * i, 0) << dJ.block<3, 18>(0, 0);
        // lcm
        _foot_data[i].joint_data[0].Pos = (float) _foot_pos_inBody.col(i)[0];
        _foot_data[i].joint_data[1].Pos = (float) _foot_pos_inBody.col(i)[1];
        _foot_data[i].joint_data[2].Pos = (float) _foot_pos_inBody.col(i)[2];
        _foot_data[i].joint_data[0].W = (float) _foot_vel_inBody.col(i)[0];
        _foot_data[i].joint_data[1].W = (float) _foot_vel_inBody.col(i)[1];
        _foot_data[i].joint_data[2].W = (float) _foot_vel_inBody.col(i)[2];
        _lcm.publish(_foot_data_topic_name[i], &_foot_data[i]);
    }
    /*得到质心线速度的雅可比矩阵*/
    J.setZero();
    pinocchio::computeFrameJacobian(*_robot_model, *_robot_data, _q, _robot_model->getFrameId("body"),
                                    pinocchio::LOCAL_WORLD_ALIGNED, J);
    _J_Body_Position << J.block<3, 18>(0, 0); // 世界系速度 = J * 质心系速度
    J.setZero();
    pinocchio::computeFrameJacobian(*_robot_model, *_robot_data, _q, _robot_model->getFrameId("body"), pinocchio::LOCAL,
                                    J);
    _J_Body_Orientation << J.block<3, 18>(3, 0); // 本体角速度坐标系不变化
    /*质心线速度雅可比矩阵的导数*/
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(*_robot_model, *_robot_data, _robot_model->getFrameId("body"),
                                             pinocchio::LOCAL_WORLD_ALIGNED, dJ);
    _dJ_Body_Position << dJ.block<3, 18>(0, 0);
    dJ.setZero();
    pinocchio::getFrameJacobianTimeVariation(*_robot_model, *_robot_data, _robot_model->getFrameId("body"),
                                             pinocchio::LOCAL, dJ);
    _dJ_Body_Orientation << dJ.block<3, 18>(3, 0);
}

void Robot::inverseDynamics() {
    pinocchio::nonLinearEffects(*_robot_model, *_robot_data, _q, _dq);
    pinocchio::crba(*_robot_model, *_robot_data, _q);
    _nle << _robot_data->nle;
    _M << _robot_data->M;
}

