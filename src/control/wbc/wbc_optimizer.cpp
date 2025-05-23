//
// Created by zyb on 24-4-27.
//

#include "control/wbc/wbc_optimizer.hpp"
#include "control/mpc/mpc_param.hpp"

WbcOptimizer::WbcOptimizer(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,const std::shared_ptr<Estimator> &estimator) {
    _robot = robot;
    _gait = gait;
    _estimator = estimator;
    _qp_solver = std::make_shared<DenseQpSolver>(_nv, _ne, _ng);
    _cmd_tau = VecX::Zero(18);
    _last_contact_force.setZero();
    _last_float_ddq.setZero();
    /* 二次型矩阵 */
    _H = MatX::Zero(_nv, _nv);
    // _Q1.setOnes(); // 质心加速度调整权重 1  lin ang
    // _Q1 = 1 * _Q1;
    _Q1 << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
    // _Q2.setOnes(); // 足端MPC力权重 0.1
    // _Q2 = 0.01 * _Q2;
    _Q2 << 0.01, 0.01, 0.001, 0.01, 0.01, 0.001, 0.01, 0.01, 0.001, 0.01, 0.01, 0.001;
    // _Q3.setOnes(); // 两相邻时刻优化后足端力突变减小
    // _Q3 = 0.001 * _Q3;
    _Q3 << 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7;
    _Q4 << 1e-5, 1e-5, 1e-7, 1e-5, 1e-5, 1e-5;
    _H.diagonal()
            << _Q1 + _Q4,
            _Q2 + _Q3;
    _g = VecX::Zero(_nv);
    _g.setZero();
    /* 等式约束 */
    _A = MatX::Zero(_ne, _nv);
    _b = VecX::Zero(_ne);
    _Sf = MatX::Zero(6, 18);
    for (int i = 0; i < 6; ++i) {
        _Sf(i, i) = 1;
    }
    /* 不等式约束 */
    _mu = mu;
    _f_min = f_min;
    _f_max = f_max;
    _lg = VecX::Zero(_ng);
    _ug = VecX::Zero(_ng);
    _lg_mask = VecX::Ones(_ng);
    _ug_mask = VecX::Ones(_ng);
    _C = MatX::Zero(_ng, _nv);
    MatX C_leg = MatX::Zero(5, 3); // 一条腿
    C_leg << 1, 0, -_mu,
            1, 0, _mu,
            0, 1, -_mu,
            0, 1, _mu,
            0, 0, 1;
    for (int i = 0; i < 4; ++i) {
        _C.block<5, 3>(0 + 5 * i, 6 + 3 * i) << C_leg;
    }
    /* set up */
    _qp_solver->DenseQpSetMat_H(_H);
    _qp_solver->DenseQpSetVec_g(_g);
    _qp_solver->DenseQpSetMat_C(_C);
    _qp_solver->DenseQpSetVec_lg_mask(_lg_mask);
    _qp_solver->DenseQpSetVec_ug_mask(_ug_mask);
    // output
    _mpc_topic_name = "wbc_mpc_output";
}

const VecX &WbcOptimizer::calcCmdTau(Vec18 cmd_ddq, Vec12 f_mpc) {
    Vec12 df = f_mpc - _last_contact_force;
    Vec6 delta_ddq = cmd_ddq.segment<6>(0) - _last_float_ddq;
    updateDenseQpEqualityConstraints(cmd_ddq, f_mpc);
    updateDenseQpInequalityConstraints(f_mpc);
    _g.segment<6>(0) << _Q4.asDiagonal() * delta_ddq;
    _g.segment<12>(6) << _Q3.asDiagonal() * df;
    _qp_solver->DenseQpSetVec_g(_g);
    _qp_solver->DenseQpSolve();
    cmd_ddq.segment<6>(0) += _qp_solver->getOutput().segment<6>(0);
    for (int i = 0; i < LEG_NUM; ++i) {
        if (_estimator->getContact(i) == SWING) {
            f_mpc.segment<3>(3 * i).setZero();
        } else {
            f_mpc.segment<3>(3 * i) += _qp_solver->getOutput().segment<3>(6 + 3 * i);
        }
    }
    _last_contact_force = f_mpc;
    _last_float_ddq = cmd_ddq.segment<6>(0);
    for (int i = 0; i < 12; ++i) {
        _mpc_output.force[i] = f_mpc[i];
    }
    _lcm.publish(_mpc_topic_name, &_mpc_output);
    _cmd_tau = -_robot->getJ_FeetPosition().transpose() * f_mpc +
               _robot->getNoLinearTorque() +
               _robot->getMassMat() * cmd_ddq; // WBC优化后的力矩
    return _cmd_tau;
}

void WbcOptimizer::updateDenseQpEqualityConstraints(Vec18 cmd_ddq, Vec12 f_mpc) {
    const MatX &M = _robot->getMassMat();
    const Vec18 &nle = _robot->getNoLinearTorque();
    MatX J = _robot->getJ_FeetPosition();
    for (int i = 0; i < LEG_NUM; ++i) {
        if (_estimator->getContact(i) == SWING) {
            // 过滤摆动项
            J.block<3, 18>(0 + 3 * i, 0).setZero();
        }
    }
    static MatX extend_M = MatX::Zero(18, 18);
    extend_M.block<18, 6>(0, 0) << M.block<18, 6>(0, 0);
    static MatX extend_J_T = MatX::Zero(18, 18);
    extend_J_T.block<18, 12>(0, 6) << J.transpose();
    _A = _Sf * (extend_M - extend_J_T);
    _b = -_Sf * (M * cmd_ddq + nle - J.transpose() * f_mpc);
    _qp_solver->DenseQpSetMat_A(_A);
    _qp_solver->DenseQpSetVec_b(_b);
}

void WbcOptimizer::updateDenseQpInequalityConstraints(Vec12 f_mpc) {
    const static double L = 1e6;
    double fx, fy, fz;
    for (int i = 0; i < LEG_NUM; ++i) {
        fx = f_mpc[0 + 3 * i];
        fy = f_mpc[1 + 3 * i];
        fz = f_mpc[2 + 3 * i];
        _lg.segment<5>(5 * i) << -L, -_mu * fz - fx, -L, -_mu * fz - fy, _f_min - fz;
        _ug.segment<5>(5 * i) << _mu * fz - fx, L, _mu * fz - fy, L, _f_max - fz;
    }
    _qp_solver->DenseQpSetVec_lg(_lg);
    _qp_solver->DenseQpSetVec_ug(_ug);
}
