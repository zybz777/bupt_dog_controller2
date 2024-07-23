//
// Created by zyb on 24-4-24.
//

#include "control/vmc/vmc_controller.hpp"

VmcController::VmcController(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                             const std::shared_ptr<Estimator> &estimator) {
    _robot = robot;
    _gait = gait;
    _estimator = estimator;
    _user_cmd = robot->getLowState()->getUserCmd();
    _vmc_data = std::make_shared<VmcData>();
    _vmc_cmd = std::make_shared<VmcCmd>();
    _vmc_data->std_foot_pos = _robot->getRobotStdFootPos_inBody();
    _vmc_data->start_foot_pos = _vmc_data->std_foot_pos;
    _vmc_data->end_foot_pos = _vmc_data->std_foot_pos;
    _h_vec << 0.05, 0.05, 0.05, 0.05;
    _k = 1.0;
    // theta0
    for (int i = 0; i < 4; ++i) {
        _theta0[i] = atan2(_vmc_data->std_foot_pos(1, i), _vmc_data->std_foot_pos(0, i));
    }
    _r = sqrt(_vmc_data->std_foot_pos(0, 0) * _vmc_data->std_foot_pos(0, 0) +
              _vmc_data->std_foot_pos(1, 0) * _vmc_data->std_foot_pos(1, 0));
    std::cout << "[VmcController] Init Success!" << std::endl;
}

void VmcController::step() {
    // 数据同步
    _vmc_data->curr_foot_pos = _robot->getFootPositions_inBody();
    _vmc_data->curr_foot_vel = _robot->getFootVelocities_inBody();
    for (int i = 0; i < 4; ++i) {
        _vmc_data->curr_foot_pos_in_world.col(i) << _robot->getFootPosition_inWorld(i);
        _vmc_data->curr_foot_vel_in_world.col(i) << _estimator->getLpVelocity() +
                _robot->getRotMat() * (_vmc_data->curr_foot_vel.col(i) +
                                       skew(_robot->getAngularVelocity()) *
                                       _vmc_data->curr_foot_pos.col(i));
        _vmc_data->std_foot_pos_in_world.col(i) << _estimator->getLpPosition() +
                _robot->getRotMat() * _vmc_data->std_foot_pos.col(i);
    }
    // 记录足端起点 终点
    updateStartFeetPos_inWorld(_gait, _estimator);
    updateEndFeetPos_inWorld(_robot, _gait, _estimator, _user_cmd);
    // 计算轨迹
    for (int i = 0; i < 4; ++i) {
        SwingLegPolynomialCurve_inWorld(i,
                                        _gait->getContact(i),
                                        _gait->getPhase(i),
                                        _gait->getTswing(),
                                        _h_vec[i]);
    }
}

void VmcController::updateStartFeetPos_inWorld(const std::shared_ptr<Gait> &gait,
                                               const std::shared_ptr<Estimator> &estimator) {
    for (int i = 0; i < 4; ++i) {
        if (gait->getContact(i) == CONTACT) {
            _vmc_data->start_foot_pos_in_world.col(i) = _vmc_data->curr_foot_pos_in_world.col(i);
        }
    }
}

void VmcController::updateEndFeetPos_inWorld(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                                             const std::shared_ptr<Estimator> &estimator,
                                             const std::shared_ptr<doglcm::UserCmd_t> &user_cmd) {
    switch (gait->getGaitType()) {
        case GaitType::TROTTING: {
            for (int i = 0; i < LEG_NUM; ++i) {
                calcEndFootPos_inWorld(i, robot, gait, estimator);
            }
            // 地形适应摆动腿落足点高度
            static double x_distance = _vmc_data->std_foot_pos.col(0)[0] - _vmc_data->std_foot_pos.col(3)[0];
            double dh = tan(estimator->getFakePitch()) * x_distance;
            if (estimator->getFakePitch() >= 0.0) {
                _vmc_data->end_foot_pos_in_world(2, 0) = 0.0;
                _vmc_data->end_foot_pos_in_world(2, 1) = 0.0;
                _vmc_data->end_foot_pos_in_world(2, 2) = dh;
                _vmc_data->end_foot_pos_in_world(2, 3) = dh;
            } else {
                _vmc_data->end_foot_pos_in_world(2, 0) = -dh;
                _vmc_data->end_foot_pos_in_world(2, 1) = -dh;
                _vmc_data->end_foot_pos_in_world(2, 2) = 0.0;
                _vmc_data->end_foot_pos_in_world(2, 3) = 0.0;
            }
        }
        break;
        default:
            _vmc_data->end_foot_pos_in_world = _vmc_data->std_foot_pos_in_world;
            _vmc_data->end_foot_pos_in_world(2, 0) = 0;
            _vmc_data->end_foot_pos_in_world(2, 1) = 0;
            _vmc_data->end_foot_pos_in_world(2, 2) = 0;
            _vmc_data->end_foot_pos_in_world(2, 3) = 0;
            break;
    }
}

void VmcController::calcEndFootPos_inWorld(int leg_id, const std::shared_ptr<Robot> &robot,
                                           const std::shared_ptr<Gait> &gait,
                                           const std::shared_ptr<Estimator> &estimator) {
    constexpr double kx = 0.005, ky = 0.005, kw = 0.005;
    const std::shared_ptr<doglcm::UserCmd_t> &user_cmd = robot->getLowState()->getUserCmd();
    Vec3 cmd_vel_in_world = robot->getRotMat() * Vec3(user_cmd->cmd_linear_velocity[0],
                                                      user_cmd->cmd_linear_velocity[1],
                                                      user_cmd->cmd_linear_velocity[2]);
    Vec3 cmd_omega_in_world = robot->getRotMat() * Vec3(user_cmd->cmd_angular_velocity[0],
                                                        user_cmd->cmd_angular_velocity[1],
                                                        user_cmd->cmd_angular_velocity[2]);
    Vec3 omega_in_world = robot->getAngularVelocity_inWorld();
    Vec3 v = _k * estimator->getLpVelocity() + (1 - _k) * cmd_vel_in_world;
    double w = omega_in_world[2];
    double theta_f = robot->getRpy()[2] + _theta0[leg_id] + w * (1 - gait->getPhase(leg_id)) * gait->getTswing() +
                     0.5 * w * gait->getTstance() + kw * (w - cmd_omega_in_world[2]);
    _vmc_data->end_foot_pos_in_world(0, leg_id) =
            estimator->getLpPosition()[0] + _r * cos(theta_f) + v[0] * (1 - gait->getPhase(leg_id)) * gait->
            getTswing() + 0.5 * v[0] * gait->getTstance() + kx * (
                estimator->getLpVelocity()[0] - cmd_vel_in_world[0]);
    _vmc_data->end_foot_pos_in_world(1, leg_id) =
            estimator->getLpPosition()[1] + _r * sin(theta_f) + v[1] * (1 - gait->getPhase(leg_id)) * gait->
            getTswing() + 0.5 * v[1] * gait->getTstance() + ky * (
                estimator->getLpVelocity()[1] - cmd_vel_in_world[1]);
    _vmc_data->end_foot_pos_in_world(2, leg_id) = 0.0;
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
    double T1_3 = pow(swing_T, 3), T1_4 = pow(swing_T, 4), T1_5 = pow(swing_T, 5), T1_6 = pow(swing_T, 6), T1_7 =
            pow(swing_T, 7), T1_8 = pow(swing_T, 8);

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

void VmcController::updateStdFootPos(double lx, double ly, double lz) {
    _vmc_data->std_foot_pos = _robot->getRobotStdFootPos_inBody();
    _vmc_data->std_foot_pos.col(0)[1] += ly;
    _vmc_data->std_foot_pos.col(1)[1] += -ly;
    _vmc_data->std_foot_pos.col(2)[1] += ly;
    _vmc_data->std_foot_pos.col(3)[1] += -ly;
    // theta0
    for (int i = 0; i < 4; ++i) {
        _theta0[i] = atan2(_vmc_data->std_foot_pos(1, i), _vmc_data->std_foot_pos(0, i));
    }
    _r = sqrt(_vmc_data->std_foot_pos(0, 0) * _vmc_data->std_foot_pos(0, 0) +
              _vmc_data->std_foot_pos(1, 0) * _vmc_data->std_foot_pos(1, 0));
}
