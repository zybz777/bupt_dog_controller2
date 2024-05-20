//
// Created by zyb on 24-5-20.
//

#include "control/rl/rl_controller.hpp"

RLController::RLController(const std::shared_ptr<LowState> &low_state) {
    //data
    _low_state = low_state;
    _gravity_vec << 0.0, 0.0, -1.0;
    _base_lin_vel = torch::zeros({3}, torch::kDouble);
    _base_ang_vel = torch::zeros({3}, torch::kDouble);
    _projected_gravity = torch::zeros({3}, torch::kDouble);
    _cmd_vel = torch::zeros({3}, torch::kDouble);
    _default_pos = torch::from_blob(const_cast<double *>(_target_pos), {12},
                                    torch::kDouble).clone();
    _dof_pos = _default_pos.clone();
    _dof_vel = torch::zeros({12}, torch::kDouble);
    _actions = torch::zeros({12}, torch::kDouble);
    _clock_inputs = torch::zeros({4}, torch::kDouble);
    _cmd_gait = torch::zeros({5}, torch::kDouble);
    _cmd_gait[ID_FOOT_HEIGHT] = 0.05; // 足端高度
    _cmd_gait[ID_PHASES] = 0.5; // 相位控制
    _cmd_gait[ID_OFFSETS] = 0.0;
    _cmd_gait[ID_BOUNDS] = 0.0;
    _cmd_gait[ID_STANCE_RATIO] = 0.5; // 站立比例[0~1]
    // scale
    _lin_vel_scale = 2.0;
    _ang_vel_scale = 0.25;
    _cmd_vel_scale = torch::tensor({2.0, 2.0, 0.25}, torch::kDouble);
    _dof_pos_scale = 1.0;
    _dof_vel_scale = 0.05;
    // torch
    std::string pt_path = CONFIG_PATH;
    pt_path += "robot.pt";
    _module = std::make_shared<Module>(torch::jit::load(pt_path));
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
    _base_lin_vel = torch::zeros({3}, torch::kDouble);
    _base_ang_vel = torch::from_blob(const_cast<double *>(_low_state->getAngularVelocity().data()), {3},
                                     torch::kDouble).clone();
    Vec3 projected_gravity = _low_state->getRotMat() * _gravity_vec;
    _projected_gravity = torch::from_blob(projected_gravity.data(), {3}, torch::kDouble).clone();
    _cmd_vel[0] = _low_state->getUserCmd()->cmd_linear_velocity[0];
    _cmd_vel[1] = _low_state->getUserCmd()->cmd_linear_velocity[1];
    _cmd_vel[2] = _low_state->getUserCmd()->cmd_angular_velocity[2];
    // 关节数据
    _dof_pos = torch::from_blob(const_cast<double *>(_low_state->getQ().data()), {12}, torch::kDouble).clone();
    _dof_vel = torch::from_blob(const_cast<double *>(_low_state->getDq().data()), {12}, torch::kDouble).clone();
    // 上次action
    // 时钟更新
    // 步态更新
    torch::Tensor obs_buf = torch::cat({
                                               _base_lin_vel * _lin_vel_scale,
                                               _base_ang_vel * _ang_vel_scale,
                                               _projected_gravity,
                                               torch::mul(_cmd_vel, _cmd_vel_scale),
                                               (_dof_pos - _default_pos) * _dof_pos_scale,
                                               _dof_vel * _dof_vel_scale,
                                               _actions,
                                               _clock_inputs,
                                               _cmd_gait}, -1);
    std::cout << obs_buf << std::endl;
}
