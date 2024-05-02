//
// Created by zyb on 24-4-24.
//

#include "control/vmc/vmc_controller.hpp"

VmcController::VmcController() {
    _vmc_data = std::make_shared<VmcData>();
    _vmc_cmd = std::make_shared<VmcCmd>();
    _vmc_data->std_foot_pos << 0.240142, 0.240142, -0.240142, -0.240142,
            0.1551, -0.1551, 0.1551, -0.1551,
            -0.313384, -0.313384, -0.313384, -0.313384;
    _vmc_data->start_foot_pos = _vmc_data->std_foot_pos;
    _vmc_data->end_foot_pos = _vmc_data->std_foot_pos;
    // theta0
    for (int i = 0; i < 4; ++i) {
        _theta0[i] = atan2(_vmc_data->std_foot_pos(1, i), _vmc_data->std_foot_pos(0, i));
    }
    _r = sqrt(_vmc_data->std_foot_pos(0, 0) * _vmc_data->std_foot_pos(0, 0) +
              _vmc_data->std_foot_pos(1, 0) * _vmc_data->std_foot_pos(1, 0));
    std::cout << "[VmcController] Init Success!" << std::endl;
}

void VmcController::step(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                         const std::shared_ptr<Estimator> &estimator,
                         const std::shared_ptr<doglcm::UserCmd_t> &user_cmd) {
    // 数据同步
    _vmc_data->curr_foot_pos = robot->getFootPositions_inBody();
    _vmc_data->curr_foot_vel = robot->getFootVelocities_inBody();
    for (int i = 0; i < 4; ++i) {
        _vmc_data->curr_foot_pos_in_world.col(i) << estimator->getFootPos_inWorld(i);
        _vmc_data->curr_foot_vel_in_world.col(i) << estimator->getLpVelocity() +
                                                    robot->getRotMat() * (_vmc_data->curr_foot_vel.col(i) +
                                                                          skew(robot->getAngularVelocity()) *
                                                                          _vmc_data->curr_foot_pos.col(i));
        _vmc_data->std_foot_pos_in_world.col(i) << estimator->getPosition() +
                                                   robot->getRotMat() * _vmc_data->std_foot_pos.col(i);
    }
    // 记录足端起点 终点
    updateStartFeetPos_inWorld(gait, estimator);
    updateEndFeetPos_inWorld(robot, gait, estimator, user_cmd);
    // 计算轨迹
    double h = 0.05;
    for (int i = 0; i < 4; ++i) {
        SwingLegPolynomialCurve_inWorld(i,
                                        gait->getContact(i),
                                        gait->getPhase(i),
                                        gait->getTswing(),
                                        h);
    }
}

void VmcController::updateStartFeetPos_inWorld(const shared_ptr<Gait> &gait, const shared_ptr<Estimator> &estimator) {
    for (int i = 0; i < 4; ++i) {
        if (gait->getGaitType() != GaitType::TROTTING) { // 初始化一下start_foot_pos_in_world
            _vmc_data->start_foot_pos_in_world.col(i) = _vmc_data->curr_foot_pos_in_world.col(i);
        }
        if (gait->getContact(i) == CONTACT && gait->getPhase(i) > 0.98) {  // 记录摆动项起点位置
            _vmc_data->start_foot_pos_in_world.col(i) = _vmc_data->curr_foot_pos_in_world.col(i);
        }
    }
}

void VmcController::updateEndFeetPos_inWorld(const shared_ptr<Robot> &robot, const shared_ptr<Gait> &gait,
                                             const shared_ptr<Estimator> &estimator,
                                             const shared_ptr<doglcm::UserCmd_t> &user_cmd) {
    switch (gait->getGaitType()) {
        case GaitType::TROTTING: {
            double kx = 0.005;
            double ky = 0.005;
            double kw = 0.005;
            Vec3 cmd_vel_in_world(user_cmd->cmd_linear_velocity[0], user_cmd->cmd_linear_velocity[1],
                                  user_cmd->cmd_linear_velocity[2]);
            Vec3 cmd_omega_in_world(user_cmd->cmd_angular_velocity[0], user_cmd->cmd_angular_velocity[1],
                                    user_cmd->cmd_angular_velocity[2]);
            cmd_vel_in_world = robot->getRotMat() * cmd_vel_in_world;
            cmd_omega_in_world = rotMatW(robot->getRpy()) * cmd_omega_in_world;
            Vec3 omega_in_world = rotMatW(robot->getRpy()) * robot->getAngularVelocity();
            double k = 0.05;
            Vec3 v = k * estimator->getLpVelocity() + (1 - k) * cmd_vel_in_world;
            double w = omega_in_world[2];
            for (int i = 0; i < LEG_NUM; ++i) {
                double theta_f = robot->getRpy()[2] + _theta0[i] + w * (1 - gait->getPhase(i)) * gait->getTswing() +
                                 0.5 * w * gait->getTstance() + kw * (w - cmd_omega_in_world[2]);
                // x y
                _vmc_data->end_foot_pos_in_world(0, i) = estimator->getPosition()[0] + _r * cos(theta_f)
                                                         + v[0] * (1 - gait->getPhase(i)) *
                                                           gait->getTswing()
                                                         + 0.5 * v[0] * gait->getTstance()
                                                         + kx * (estimator->getLpVelocity()[0] - cmd_vel_in_world[0]);

                _vmc_data->end_foot_pos_in_world(1, i) = estimator->getPosition()[1] + _r * sin(theta_f)
                                                         + v[1] * (1 - gait->getPhase(i)) *
                                                           gait->getTswing()
                                                         + 0.5 * v[1] * gait->getTstance()
                                                         + ky * (estimator->getLpVelocity()[1] - cmd_vel_in_world[1]);
            }
        }
            break;
        default:
            _vmc_data->end_foot_pos_in_world = _vmc_data->std_foot_pos_in_world;
            break;
    }
}

void VmcController::SwingLegPolynomialCurve_inWorld(int leg_id, int contact, double phase, double swing_T, double h) {
    double &x = _vmc_cmd->cmd_foot_pos_in_world(0, leg_id);
    double &y = _vmc_cmd->cmd_foot_pos_in_world(1, leg_id);
    double &z = _vmc_cmd->cmd_foot_pos_in_world(2, leg_id);
    double &vx = _vmc_cmd->cmd_foot_vel_in_world(0, leg_id);
    double &vy = _vmc_cmd->cmd_foot_vel_in_world(1, leg_id);
    double &vz = _vmc_cmd->cmd_foot_vel_in_world(2, leg_id);
    double &ax = _vmc_cmd->cmd_foot_acc_in_world(0, leg_id);
    double &ay = _vmc_cmd->cmd_foot_acc_in_world(1, leg_id);
    double &az = _vmc_cmd->cmd_foot_acc_in_world(2, leg_id);

    double &x0 = _vmc_data->start_foot_pos_in_world(0, leg_id);
    double &y0 = _vmc_data->start_foot_pos_in_world(1, leg_id);
    double &z0 = _vmc_data->start_foot_pos_in_world(2, leg_id);
    double &x1 = _vmc_data->end_foot_pos_in_world(0, leg_id);
    double &y1 = _vmc_data->end_foot_pos_in_world(1, leg_id);
    double &z1 = _vmc_data->end_foot_pos_in_world(2, leg_id);

    if (contact == 1) {
        x = x1;
        y = y1;
        z = z1;
        vx = 0.0;
        vy = 0.0;
        vz = 0.0;
        ax = 0.0;
        ay = 0.0;
        az = 0.0;
        return;
    }

    double t = phase * swing_T;
    double H = z0 + h;
    double dx = x0 - x1, dy = y0 - y1;
    double t2 = pow(t, 2), t3 = pow(t, 3), t4 = pow(t, 4), t5 = pow(t, 5), t6 = pow(t, 6), t7 = pow(t, 7);
    double T1_3 = pow(swing_T, 3), T1_4 = pow(swing_T, 4), T1_5 = pow(swing_T, 5), T1_6 = pow(swing_T, 6), T1_7 = pow(
            swing_T, 7), T1_8 = pow(swing_T, 8);

    double th1 = 361 * z0 - 512 * H + 151 * z1;
    double th2 = 411 * z0 - 640 * H + 229 * z1;
    double th3 = 99 * z0 - 128 * H + 29 * z1;
    double th4 = 69 * z0 - 128 * H + 59 * z1;
    double th5 = 713 * z0 - 1216 * H + 503 * z1;
    x = x0 - 10 * t3 * dx / T1_3 + 15 * t4 * dx / T1_4 - 6 * t5 * dx / T1_5;
    y = y0 - 10 * t3 * dy / T1_3 + 15 * t4 * dy / T1_4 - 6 * t5 * dy / T1_5;
    z = z0 - 2 * t3 * th3 / T1_3 -
        24 * t7 * th4 / T1_7 +
        3 * t4 * th1 / T1_4 -
        6 * t5 * th2 / T1_5 +
        4 * t6 * th5 / T1_6 +
        384 * pow(t, 8) * (z0 - 2 * H + z1) / T1_8;

    vx = 60 * t3 * dx / T1_4 - 30 * t2 * dx / T1_3 - 30 * t4 * dx / T1_5;
    vy = 60 * t3 * dy / T1_4 - 30 * t2 * dy / T1_3 - 30 * t4 * dy / T1_5;
    vz = 12 * t3 * th1 / T1_4 -
         168 * t6 * th4 / T1_7 -
         6 * t2 * th3 / T1_3 -
         30 * t4 * th2 / T1_5 +
         24 * t5 * th5 / T1_6 +
         3072 * t7 * (z0 - 2 * H + z1) / T1_8;

    ax = 180 * t2 * dx / T1_4 - 60 * t * dx / T1_3 - 120 * t3 * dx / T1_5;
    ay = 180 * t2 * dy / T1_4 - 60 * t * dy / T1_3 - 120 * t3 * dy / T1_5;
    az = 36 * t2 * th1 / T1_4 -
         1008 * t5 * th4 / T1_7 -
         12 * t * th3 / T1_3 -
         120 * t3 * th2 / T1_5 +
         120 * t4 * th5 / T1_6 +
         21504 * t6 * (z0 - 2 * H + z1) / T1_8;
}

