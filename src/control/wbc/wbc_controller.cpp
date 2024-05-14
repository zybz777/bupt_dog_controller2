//
// Created by zyb on 24-4-27.
//

#include "control/wbc/wbc_controller.hpp"

WbcController::WbcController(int ms, const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                             const std::shared_ptr<Estimator> &estimator,
                             const std::shared_ptr<MpcController> &mpc, const std::shared_ptr<VmcController> &vmc) {
    _dt = double(ms / 1000.0);
    _robot = robot;
    _estimator = estimator;
    _gait = gait;
    _mpc = mpc;
    _vmc = vmc;
    _user_cmd = _robot->getLowState()->getUserCmd();
    _mrt = _mpc->getMrtGenerator();
    // task
    _task_list.push_back(&_task_contact_foot);
    _task_list.push_back(&_task_swing_foot);
    _task_list.push_back(&_task_body_orientation);
    _task_list.push_back(&_task_body_pos);
    // 关节指令
    _cmd_q.setZero();
    _cmd_dq.setZero();
    _cmd_ddq.setZero();
    _cmd_tau.setZero();
    // 机器人数据
    _rot_mat.setIdentity();
    _com_pos_inWorld.setZero();
    _com_vel_inWorld.setZero();
    _com_rpy.setZero();
    _com_omega_inBody.setZero();
    _feet_positions_inBody.setZero();
    _f_mpc.setZero();
    // qp solver
    _optimizer = std::make_shared<WbcOptimizer>(_robot, _gait);
    std::cout << "[WBC] Init Successful!" << std::endl;
}

void WbcController::step() {
    updateData();
    updateTask();
    solve();
}

void WbcController::updateData() {
    _rot_mat = _robot->getRotMat();
    _com_pos_inWorld = _estimator->getPosition();
    _com_vel_inWorld = _estimator->getLpVelocity();
    _com_rpy = _robot->getRpy();
    _com_omega_inBody = _robot->getAngularVelocity();
    _feet_positions_inBody = _robot->getFootPositions_inBody();
    _f_mpc = _mpc->getMpcOutput();
}

void WbcController::updateTask() {
    updateContactFootTask(_task_contact_foot);
    updateBodyOrientationTask(_task_body_orientation);
    updateBodyPosTask(_task_body_pos);
    updateSwingFootTask(_task_swing_foot);
}

void WbcController::updateBodyPosTask(WbcTask_BodyPos &task) {
    /*  工作空间： 世界系质心位置 世界系质心速度 世界系质心加速度
         关节空间q： 世界系质心位置 欧拉角 关节电机角度
         关节空间dq: 质心系质心速度 质心系角速度 关节电机角速度
     */
    // 更新雅可比矩阵
    task.updateTaskJacobi(_robot->getJ_BodyPosition(), _robot->getDJ_BodyPosition());
    // 更新位置任务
    Vec3 target_pos = _mrt->getXtraj()[0].segment<3>(3);
    Vec3 curr_pos = _com_pos_inWorld;
    target_pos.segment<2>(0) = curr_pos.segment<2>(0);
    // 更新速度任务
    Vec3 target_vel = _mrt->getXtraj()[0].segment<3>(9);
    Vec3 curr_vel = _com_vel_inWorld;
    // 更新加速度任务
    Vec3 target_acc = Vec3::Zero();
    task.updateTask(target_pos, target_vel, target_acc, curr_pos, curr_vel);
}

void WbcController::updateBodyOrientationTask(WbcTask_BodyOrientation &task) {
    /*  工作空间： 欧拉角 质心系角速度 质心系角加速度
       关节空间q： 世界系质心位置 欧拉角 关节电机角度
       关节空间dq: 质心系质心速度 质心系角速度 关节电机角速度
   */
    // 更新雅可比矩阵
    task.updateTaskJacobi(_robot->getJ_BodyOrientation(), _robot->getDJ_BodyOrientation());
    // 更新位置任务
    Vec3 target_rpy = _mrt->getXtraj()[0].segment<3>(0);
    Vec3 curr_rpy = _robot->getRpy();
    Vec3 task_err = rotMatW(curr_rpy) * (target_rpy - curr_rpy);
    if (fabs(curr_rpy[2]) > M_PI) {
        int n = int(fabs(curr_rpy[2]) / M_PI);
        if (n > 0) {
            task_err[2] = task_err[2] / double(n);
        }
    }
    // 更新速度任务
    Vec3 target_vel = _mrt->getXtraj()[0].segment<3>(6);
    Vec3 curr_vel = _robot->getAngularVelocity_inWorld();
    // 更新加速度任务
    Vec3 target_acc = Vec3::Zero();
    task.updateTask(task_err, target_vel, target_acc, curr_vel);
}

void WbcController::updateContactFootTask(WbcTask_FootPos &task) {
    // 更新雅可比矩阵
    auto J = _robot->getJ_FeetPosition();
    auto dJ = _robot->getDJ_FeetPosition();
    for (int i = 0; i < LEG_NUM; ++i) {
        if (_gait->getContact(i) == SWING) { // 过滤摆动项
            J.block<3, 18>(0 + 3 * i, 0).setZero();
            dJ.block<3, 18>(0 + 3 * i, 0).setZero();
        }
    }
    task.updateTaskJacobi(J, dJ);
    // 更新位置速度加速度任务
    Vec12 target_pos, target_vel = Vec12::Zero(), target_acc = Vec12::Zero(), curr_pos, curr_vel;
    curr_vel = _robot->getJ_FeetPosition() * _robot->getFloatBaseDq();
    for (int i = 0; i < LEG_NUM; ++i) {
        curr_pos.segment<3>(3 * i) << _estimator->getFootPos_inWorld(i);
        target_pos.segment<3>(3 * i) << curr_pos.segment<3>(3 * i);
    }
    task.updateTask(target_pos, target_vel, target_acc, curr_pos, curr_vel);
}

void WbcController::updateSwingFootTask(WbcTask_FootPos &task) {
    /*  工作空间： 世界系足端位置 世界系足端速度 世界系足端加速度
       关节空间q： 世界系质心位置 欧拉角 关节电机角度
       关节空间dq: 质心系质心速度 质心系角速度 关节电机角速度
   */
    // 更新雅可比矩阵
    // 更新雅可比矩阵
    auto J = _robot->getJ_FeetPosition();
    auto dJ = _robot->getDJ_FeetPosition();
    for (int i = 0; i < LEG_NUM; ++i) {
        if (_gait->getContact(i) == CONTACT) { // 过滤支撑项
            J.block<3, 18>(0 + 3 * i, 0).setZero();
            dJ.block<3, 18>(0 + 3 * i, 0).setZero();
        }
    }
    task.updateTaskJacobi(J, dJ);
    // 更新位置速度加速度任务
    Vec12 target_pos, curr_pos;
    Vec12 target_vel, curr_vel;
    Vec12 target_acc;
    curr_vel = _robot->getJ_FeetPosition() * _robot->getFloatBaseDq();
    for (int i = 0; i < 4; ++i) {
        curr_pos.segment<3>(3 * i) << _estimator->getFootPos_inWorld(i);
        if (_gait->getContact(i) == SWING) // VMC
        {
            target_pos.segment<3>(3 * i) << _vmc->getCmdFootPos_inWorld(i);
            target_vel.segment<3>(3 * i) << _vmc->getCmdFootVel_inWorld(i);
            target_acc.segment<3>(3 * i) << _vmc->getCmdFootAcc_inWorld(i);
        } else // 支撑腿不在此处控制，下列无效，固定为0
        {
            target_pos.segment<3>(3 * i).setZero();
            target_vel.segment<3>(3 * i).setZero();
            target_acc.segment<3>(3 * i).setZero();
        }
    }
    // std::cout << _gait->getPhase().transpose() << std::endl;
    task.updateTask(target_pos, target_vel, target_acc, curr_pos, curr_vel);
}

void WbcController::solve() {
    MatX pinv_J0 = pinv(_task_list[0]->getTask_J());
    // MatX pinv_MJ0 = dynamicPinv(_task_list[0]->getTask_J(), _robot->getMassMatInv());
    /*0号任务 支撑腿触地不动*/
    MatX N = I18 - pinv_J0 * _task_list[0]->getTask_J();
    // 位置0号任务
    VecX cmd_delta_q = VecX::Zero(18);
    // 速度0号任务
    VecX cmd_dq = VecX::Zero(18);
    //  加速度0号任务
    VecX cmd_ddq = pinv_J0 * (-_task_list[0]->getTask_J() * _robot->getFloatBaseDq() -
                              _task_list[0]->getTask_dJ() * _robot->getFloatBaseDq());
    for (int i = 0; i < LEG_NUM; ++i) {
        if (_gait->getContact(i) == SWING) {
            cmd_ddq.segment<3>(6 + 3 * i).setZero();
        }
    }
    /*N号任务计算*/
    MatX J_pre, pinv_J_pre, pinv_MJ_pre, MJ_pre;
    for (int i = 1; i < _task_list.size(); ++i) {
        J_pre = _task_list[i]->getTask_J() * N;
        pinv_J_pre = pinv(J_pre);
        // pinv_MJ_pre = dynamicPinv(J_pre, _robot->getMassMatInv());
        // 位置任务
        cmd_delta_q += pinv_J_pre * (_task_list[i]->getTask_e() - _task_list[i]->getTask_J() * cmd_delta_q);
        // 速度任务
        cmd_dq += pinv_J_pre * (_task_list[i]->getTask_dx() - _task_list[i]->getTask_J() * cmd_dq);
        // 加速度任务
        cmd_ddq += pinv_J_pre *
                   (_task_list[i]->getTask_ddx() - _task_list[i]->getTask_dJ() * _robot->getFloatBaseDq() -
                    _task_list[i]->getTask_J() * cmd_ddq);
        N *= I18 - pinv_J_pre * J_pre;
    }
    _cmd_q << _com_pos_inWorld, _com_rpy, _robot->getQ();
    _cmd_q += cmd_delta_q; // 位置任务解算的关节角
    _cmd_dq = cmd_dq;
    _cmd_ddq = cmd_ddq;
    _cmd_tau = _optimizer->calcCmdTau(_cmd_ddq, _f_mpc);
}

