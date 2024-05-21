//
// Created by zyb on 24-5-20.
//

#include "control/rl/rl_controller.hpp"

RLController::RLController(const std::shared_ptr<LowState> &low_state, const std::shared_ptr<Estimator> &estimator,
                           const std::shared_ptr<Gait> &gait) {
    //data
    _low_state = low_state;
    _estimator = estimator;
    _gait = gait;
    _gravity_vec << 0.0, 0.0, -1.0;
    _base_lin_vel = torch::zeros({3}, torch::kFloat);
    _base_ang_vel = torch::zeros({3}, torch::kFloat);
    _projected_gravity = torch::zeros({3}, torch::kFloat);
    _cmd_vel = torch::zeros({3}, torch::kFloat);
    _default_pos = torch::zeros({12}, torch::kFloat);
    for (int i = 0; i < 12; ++i) {
        _default_pos[i] = _target_pos[i];
    }
    _dof_pos = _default_pos.clone();
    _dof_vel = torch::zeros({12}, torch::kFloat);
    _actions = torch::zeros({12}, torch::kFloat);
    _clock_inputs = torch::zeros({4}, torch::kFloat);
    _cmd_gait = torch::zeros({5}, torch::kFloat);
    _cmd_gait[ID_FOOT_HEIGHT] = 0.05; // 足端高度
    _cmd_gait[ID_PHASES] = 0.5; // 相位控制
    _cmd_gait[ID_OFFSETS] = 0.0;
    _cmd_gait[ID_BOUNDS] = 0.0;
    _cmd_gait[ID_STANCE_RATIO] = 0.5; // 站立比例[0~1]
    // scale
    _lin_vel_scale = 2.0;
    _ang_vel_scale = 0.25;
    _cmd_vel_scale = torch::tensor({2.0, 2.0, 0.25}, torch::kFloat);
    _dof_pos_scale = 1.0;
    _dof_vel_scale = 0.05;
    _action_scale = 0.25; // use in rl_tau
    // output cmd
    _rl_tau = Vec12::Zero();
    for (int i = 0; i < 12; ++i) {
        _rl_target_pos[i] = _target_pos[i];
    }
    _rl_target_vel = Vec12::Zero();
    _Kp = 20.0;
    _Kd = 1.0;
    // torch
    std::string pt_path = CONFIG_PATH;
    pt_path += "policy_1.pt";
    _module = std::make_shared<Module>(torch::jit::load(pt_path));
    _module->eval();
    _module->to(at::kCPU);
    // thread
    _ms = 20;
    _rl_thread = std::thread([this] { run(_ms); });
    std::cout << "[RLController] Init Successful!" << std::endl;
}

void RLController::begin() {
    _rl_thread.join();
}

void RLController::run(int ms) {
    std::cout << "[RLController] Task Run!" << std::endl;
    // assignTask2Cpu(2);
    std::chrono::microseconds period(ms * 1000);
    auto start_time = std::chrono::high_resolution_clock::now();

    while (true) {
        step();
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}

void RLController::step() {
    computeObservations();
}

void RLController::computeObservations() {
    // 质心数据
    Vec3 world_vel = _low_state->getGpsVel();
    Vec3 body_vel = _low_state->getRotMat().transpose() * world_vel;
    _base_lin_vel[0] = body_vel[0];
    _base_lin_vel[1] = body_vel[1];
    _base_lin_vel[2] = body_vel[2];
    _base_ang_vel[0] = _low_state->getAngularVelocity()[0];
    _base_ang_vel[1] = _low_state->getAngularVelocity()[1];
    _base_ang_vel[2] = _low_state->getAngularVelocity()[2];
    Vec3 projected_gravity = _low_state->getRotMat().transpose() * _gravity_vec;
    _projected_gravity[0] = projected_gravity[0];
    _projected_gravity[1] = projected_gravity[1];
    _projected_gravity[2] = projected_gravity[2];
    _cmd_vel[0] = _low_state->getUserCmd()->cmd_linear_velocity[0];
    _cmd_vel[1] = _low_state->getUserCmd()->cmd_linear_velocity[1];
    _cmd_vel[2] = _low_state->getUserCmd()->cmd_angular_velocity[2];
    // 关节数据
    for (int i = 0; i < 12; ++i) {
        _dof_pos[i] = _low_state->getQ()[i];
        _dof_vel[i] = _low_state->getDq()[i];
    }
    // 时钟更新 先不考虑周期 默认为1
    for (int leg_id = 0; leg_id < LEG_NUM; ++leg_id) {
        if (_gait->getContact(leg_id) == CONTACT) {
            _clock_inputs[leg_id] = sin(M_PI * _gait->getPhase(leg_id));
        } else {
            _clock_inputs[leg_id] = -sin(M_PI * _gait->getPhase(leg_id));
        }
    }
    // 步态更新
    _cmd_gait[ID_FOOT_HEIGHT] = 0.05; // 足端高度
    _cmd_gait[ID_PHASES] = 0.5; // 相位控制
    _cmd_gait[ID_OFFSETS] = 0.0;
    _cmd_gait[ID_BOUNDS] = 0.0;
    _cmd_gait[ID_STANCE_RATIO] = 0.5; // 站立比例[0~1]
    // 组合观测量
    torch::Tensor obs_buf = torch::cat({
                                               _base_ang_vel * _ang_vel_scale,
                                               _projected_gravity,
                                               torch::mul(_cmd_vel, _cmd_vel_scale),
                                               (_dof_pos - _default_pos) * _dof_pos_scale,
                                               _dof_vel * _dof_vel_scale,
                                               _actions,
                                               _clock_inputs,
                                               _cmd_gait}, -1);
    at::Tensor output = _module->forward({obs_buf}).toTensor();
    _actions = torch::clip(output, -100, 100);
    for (int i = 0; i < 12; ++i) {
        _rl_tau[i] = _Kp * _action_scale * _actions[i].item<double>();
    }
//    std::cout << getTimeStamp() << std::endl;
}
