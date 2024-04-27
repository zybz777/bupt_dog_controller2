//
// Created by zyb on 24-4-26.
//

#include <iostream>
#include "control/mpc/mrt_generator.hpp"
#include "control/mpc/mpc_param.hpp"

MrtGenerator::MrtGenerator() {
    _dt = (double) 1.0 / MPC_FREQUENCY;
    _X.setZero();
    _X_traj = std::vector<VecX>(HORIZON);
    for (int i = 0; i < HORIZON; ++i) {
        _X_traj[i] = Vec12::Zero();
    }
    std::cout << "[MrtGenerator] Init Success!" << std::endl;
}

void MrtGenerator::resetXtraj(const Vec12 &X) {
    for (size_t i = 0; i < _X_traj.size(); ++i) {
        // 角度
        _X_traj[i][0] = 0.0;
        _X_traj[i][1] = 0.0;
        _X_traj[i][2] = X[2];
        // 位置
        _X_traj[i][3] = X[3];
        _X_traj[i][4] = X[4];
        _X_traj[i][5] = X[5];
        // 角速度
        _X_traj[i][6] = 0.0;
        _X_traj[i][7] = 0.0;
        _X_traj[i][8] = 0.0;
        // 线速度
        _X_traj[i][9] = 0.0;
        _X_traj[i][10] = 0.0;
        _X_traj[i][11] = 0.0;
    }
}

void MrtGenerator::step(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                        const std::shared_ptr<Estimator> &estimator) {
    _X << robot->getRpy(),
            estimator->getPosition(),
//            robot->getAngularVelocity_inWorld(),
            rotMatW(robot->getRpy()) * robot->getAngularVelocity(),
            estimator->getLpVelocity();
    auto user_cmd = robot->getLowState()->getUserCmd();
    switch (gait->getGaitType()) {
        case GaitType::FIXEDSTAND:
            resetXtraj(_X);
            break;
        case GaitType::TROTTING: {

            Vec12 last_X_traj = _X_traj[1];
            for (int i = 0; i < HORIZON; ++i) {
                Vec3 cmd_vel(user_cmd->cmd_linear_velocity[0], user_cmd->cmd_linear_velocity[1],
                             user_cmd->cmd_linear_velocity[2]);
                cmd_vel = robot->getRotMat() * cmd_vel;
                // 角度 位置 角速度 速度
                // _X_traj[i][0] = 0.0;
                // _X_traj[i][1] = 0.0;
                _X_traj[i][2] = last_X_traj[2] + user_cmd->cmd_angular_velocity[2] * i * _dt;

                _X_traj[i][3] = last_X_traj[3] + cmd_vel[0] * i * _dt;
                _X_traj[i][4] = last_X_traj[4] + cmd_vel[1] * i * _dt;
                _X_traj[i][5] = last_X_traj[5];

                _X_traj[i].segment<3>(6) << rotMatW(robot->getRpy()) * Vec3(0, 0, user_cmd->cmd_angular_velocity[2]);
                // _X_traj[i][6] = 0.0;
                // _X_traj[i][7] = 0.0;
                // _X_traj[i][8] = _user_cmd->cmd_angular_velocity[2];

                _X_traj[i][9] = cmd_vel[0];
                _X_traj[i][10] = cmd_vel[1];
                // _X_traj[i][11] = 0.0;
            }
        }
            break;
        case GaitType::FREESTAND: {
            Vec12 last_X_traj = _X_traj[1];
            for (int i = 0; i < HORIZON; ++i) {
                // 角度 位置 角速度 速度
                // _X_traj[i][0] = 0.0;
                // _X_traj[i][1] = 0.0;
                // yaw 角
                _X_traj[i][2] = last_X_traj[2] + user_cmd->cmd_angular_velocity[2] * i * _dt;
//                _X_traj[i][2] = clip(_X_traj[i][2], Vec2(-0.25, 0.25));

                _X_traj[i].segment<3>(6) << rotMatW(robot->getRpy()) * Vec3(0, 0, user_cmd->cmd_angular_velocity[2]);
                if (std::abs(_X_traj[i][2]) >= 0.245) {
                    _X_traj[i].segment<3>(6).setZero();
                }
                // H 高度
                Vec3 cmd_vel(user_cmd->cmd_linear_velocity[0], user_cmd->cmd_linear_velocity[1],
                             user_cmd->cmd_linear_velocity[2]);
                cmd_vel = robot->getRotMat() * cmd_vel;
                _X_traj[i][5] = last_X_traj[5] + cmd_vel[2] * i * _dt;
                _X_traj[i][5] = clip(_X_traj[i][5], Vec2(0.25, 0.32));
                if (_X_traj[i][5] < 0.245 || _X_traj[i][5] > 0.315) {
                    _X_traj[i][11] = 0.0;
                } else {
                    _X_traj[i][11] = cmd_vel[2];
                }

                // _X_traj[i][6] = 0.0;
                // _X_traj[i][7] = 0.0;
                // _X_traj[i][8] = _user_cmd->cmd_angular_velocity[2];
            }
        }
            break;
        default:
            break;
    }
}

