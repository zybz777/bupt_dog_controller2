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
    std::cout << "[VmcController] Init Success!" << std::endl;
}

void VmcController::step(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                         const std::shared_ptr<doglcm::UserCmd_t> &user_cmd) {
    // 数据同步
    _vmc_data->curr_foot_pos = robot->getFootPositions_inBody();
    _vmc_data->curr_foot_vel = robot->getFootVelocities_inBody();
    // 记录足端起点 终点
    updateStartFootPos(gait);
    updateEndFootPos(robot, gait, user_cmd);
    // 计算轨迹
    double h = 0.05;
    for (int i = 0; i < 4; ++i) {
        SwingLegPolynomialCurve(i,
                                gait->getContact(i),
                                gait->getPhase(i),
                                gait->getTswing(),
                                h);
    }
}

void VmcController::updateStartFootPos(const std::shared_ptr<Gait> &gait) {
    for (int i = 0; i < 4; ++i) {
        if (gait->getPhase(i) > 0.98) {  // 摆动项或支撑项快结束时记录支撑相起点
            _vmc_data->start_foot_pos.col(i) = _vmc_data->curr_foot_pos.col(i);
        }
    }
}

void VmcController::updateEndFootPos(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                                     const std::shared_ptr<doglcm::UserCmd_t> &user_cmd) {
    switch (gait->getGaitType()) {
        case GaitType::TROTTING: {
            RotMat R = rotMatRy(-robot->getRpy()[1]) * rotMatRx(-robot->getRpy()[0]);
            double swing_T;
            double incre_linear_x, incre_linear_y, incre_angular_x, incre_angular_y;
            double x1, y1, th1, r, v_w, v_w_x, v_w_y;
            for (int i = 0; i < 4; ++i) {
                if (gait->getContact(i) == CONTACT) // 支撑腿末端轨迹规划
                {
                    _vmc_data->end_foot_pos(0, i) = _vmc_data->start_foot_pos(0, i) -
                                                    user_cmd->cmd_linear_velocity[0] * gait->getPhase(i) *
                                                    gait->getTstance();
                    _vmc_data->end_foot_pos(1, i) = _vmc_data->start_foot_pos(1, i) -
                                                    user_cmd->cmd_linear_velocity[1] * gait->getPhase(i) *
                                                    gait->getTstance();
                    continue;
                }
                // 摆动腿最终期望位置计算
                _vmc_data->rot_start_foot_pos.col(i) = R * _vmc_data->std_foot_pos.col(i);
                swing_T = gait->getTswing();
                // 线速度
                incre_linear_x = user_cmd->cmd_linear_velocity[0] * swing_T * 0.5;
                incre_linear_y = user_cmd->cmd_linear_velocity[1] * swing_T * 0.5;
                // 角速度
                x1 = _vmc_data->std_foot_pos(0, i);
                y1 = _vmc_data->std_foot_pos(1, i);
                th1 = atan2(x1, y1);
                r = sqrt(pow(x1, 2) + pow(y1, 2));
                v_w = user_cmd->cmd_angular_velocity[2] * r; // 质心角速度所产生的足端线速度
                v_w_x = v_w * cos(th1);
                v_w_y = v_w * sin(th1);
                incre_angular_x = v_w_x * swing_T * 0.5;
                incre_angular_y = v_w_y * swing_T * 0.5;
                // 赋值
                _vmc_data->end_foot_pos(0, i) = _vmc_data->std_foot_pos(0, i) + incre_linear_x + incre_angular_x;
                _vmc_data->end_foot_pos(1, i) = _vmc_data->std_foot_pos(1, i) + incre_linear_y + incre_angular_y;
            }
        }
            break;
        default:
            _vmc_data->end_foot_pos = _vmc_data->std_foot_pos;
            break;
    }
}

void VmcController::SwingLegPolynomialCurve(int leg_id, int contact, double phase, double swing_T, double h) {
    double &x = _vmc_cmd->cmd_foot_pos(0, leg_id);
    double &y = _vmc_cmd->cmd_foot_pos(1, leg_id);
    double &z = _vmc_cmd->cmd_foot_pos(2, leg_id);
    double &vx = _vmc_cmd->cmd_foot_vel(0, leg_id);
    double &vy = _vmc_cmd->cmd_foot_vel(1, leg_id);
    double &vz = _vmc_cmd->cmd_foot_vel(2, leg_id);
    double &ax = _vmc_cmd->cmd_foot_acc(0, leg_id);
    double &ay = _vmc_cmd->cmd_foot_acc(1, leg_id);
    double &az = _vmc_cmd->cmd_foot_acc(2, leg_id);

    double &x0 = _vmc_data->start_foot_pos(0, leg_id);
    double &y0 = _vmc_data->start_foot_pos(1, leg_id);
    double &z0 = _vmc_data->start_foot_pos(2, leg_id);
    double &x1 = _vmc_data->end_foot_pos(0, leg_id);
    double &y1 = _vmc_data->end_foot_pos(1, leg_id);
    double &z1 = _vmc_data->end_foot_pos(2, leg_id);

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

