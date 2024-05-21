//
// Created by zyb on 24-5-20.
//

#ifndef BUPT_DOG_CONTROLLER2_RL_CONTROLLER_HPP
#define BUPT_DOG_CONTROLLER2_RL_CONTROLLER_HPP

#include <torch/script.h>
#include "common/low_state.hpp"
#include "common/estimator.hpp"
#include "gait/gait.hpp"

using torch::jit::script::Module;
#define ID_FOOT_HEIGHT 0
#define ID_PHASES 1
#define ID_OFFSETS 2
#define ID_BOUNDS 3
#define ID_STANCE_RATIO 4

class RLController {
public:
    RLController(const std::shared_ptr<LowState> &low_state, const std::shared_ptr<Estimator> &estimator,
                 const std::shared_ptr<Gait> &gait);

    void begin();

    const Vec12 &getLegCmdQ() { return _rl_target_pos; }

    const Vec12 &getLegCmdDq() { return _rl_target_vel; }

    const Vec12 &getLegCmdTau() { return _rl_tau; }

    const double &getLegCmdKp() { return _Kp; }

    const double &getLegCmdKd() { return _Kd; }

private:
    [[noreturn]]  void run(int ms);

    void step();

    void computeObservations();

    // data
    const double _target_pos[12] = {0.0, 0.81521, -1.57079, 0.0, 0.81521, -1.57079,
                                    0.0, 0.81521, -1.57079, 0.0, 0.81521, -1.57079};
    Vec3 _gravity_vec;
    std::shared_ptr<LowState> _low_state;
    std::shared_ptr<Estimator> _estimator;
    std::shared_ptr<Gait> _gait;
    torch::Tensor _base_lin_vel;
    torch::Tensor _base_ang_vel;
    torch::Tensor _projected_gravity;
    torch::Tensor _cmd_vel; // x y w
    torch::Tensor _dof_pos;
    torch::Tensor _default_pos;
    torch::Tensor _dof_vel;
    torch::Tensor _actions;
    torch::Tensor _clock_inputs;
    torch::Tensor _cmd_gait;
    // scale
    double _lin_vel_scale;
    double _ang_vel_scale;
    torch::Tensor _cmd_vel_scale;
    double _dof_pos_scale;
    double _dof_vel_scale;
    double _action_scale;
    // output cmd
    Vec12 _rl_tau;
    Vec12 _rl_target_pos;
    Vec12 _rl_target_vel;
    double _Kp = 20.0;
    double _Kd = 0.5;
    // thread
    int _ms;
    std::thread _rl_thread;
    // torch
    std::shared_ptr<Module> _module;
};


#endif //BUPT_DOG_CONTROLLER2_RL_CONTROLLER_HPP
