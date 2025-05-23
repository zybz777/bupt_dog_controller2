//
// Created by zyb on 24-4-26.
//

#include "control/mpc/mpc_controller.hpp"
#include "control/mpc/mpc_param.hpp"
#include "utils/real_time.hpp"

MpcController::MpcController(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                             const std::shared_ptr<Estimator> &estimator) {
    _robot = robot;
    _gait = gait;
    _estimator = estimator;
    _mrt = std::make_shared<MrtGenerator>();
    _dt = (double) 1.0 / MPC_FREQUENCY;
    _ms = int(_dt * 1000.0);
    init();
#ifdef USE_MPC1
    _mpc_thread = std::thread([this] { run(_ms); });
#endif
    std::cout << "[MpcController] Init Successful!" << std::endl;
}

void MpcController::begin() {
#ifdef USE_MPC1
    _mpc_thread.join();
#endif
}

void MpcController::init() {
    /*机器人物理属性*/
    _M = _robot->getRobotMass();
    _mu = mu;
    _f_min = f_min;
    _f_max = f_max;
    _I_body << _robot->getRobotInertial();
    _body_com = _robot->getRobotStdCom();
    /*mpc input*/
    _X.setZero();
    /*mpc output*/
    _mpc_f.setZero();
    /*mpc 权重*/
#ifdef USE_SIM
    // x-xref L x-xref
    _L_diag << 5.0, 5.0, 5.0, // 角度
            0.0, 0.0, 5.0,
            0.01, 0.01, 0.01, // 角速度
            0.5, 0.5, 0.1; // simulink weight
    // uKu
    _K_diag << 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6;
    // u-ulast M u-ulast
    _M_diag << 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7;
    // x-xlast N x-xlast
    _N_diag << 1e-3, 1e-3, 1e-4,
            0, 0, 1e-4,
            1e-7, 1e-7, 1e-7,
            1e-5, 1e-5, 1e-7;
    // _M_diag.setZero();
    // _N_diag.setZero();
#else
    // x-xref L x-xref
    _L_diag << 5.0, 5.0, 5.0, // 角度
            0.0, 0.0, 5.0,
            0.01, 0.01, 0.01, // 角速度
            0.5, 0.5, 0.1; // simulink weight
    // uKu
    _K_diag << 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6, 1.0e-4, 1.0e-4, 5.0e-6;
    // u-ulast M u-ulast
    _M_diag << 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7, 1e-4, 1e-4, 5e-7;
    // x-xlast N x-xlast
    _N_diag << 1e-3, 1e-3, 1e-4,
            0, 0, 1e-4,
            1e-7, 1e-7, 1e-7,
            1e-5, 1e-5, 1e-7;
    // _M_diag.setZero();
    // _N_diag.setZero();
#endif
    /*矩阵*/
    initMat();
    /*solver*/
    initSolver();
    // lcm
    _mpc_topic_name = "mpc_output";
    for (double &force: _mpc_output.force) {
        force = 0.0;
    }
}

void MpcController::initMat() {
    // A B g
    _A_dt.setIdentity();
    _A_dt.block<3, 3>(3, 9) << _I3 * _dt;
    _B_dt.setZero();
    for (int i = 0; i < 4; ++i) {
        _B_dt.block<3, 3>(9, 3 * i) << _I3 / _M * _dt;
    }
    _g_dt.setZero();
    _g_dt[11] = -9.81 * _dt;
    /*约束条件：摩擦锥 lg <= D u <= ug*/
    _D = std::vector<MatX>(HORIZON);
    _D_min = std::vector<VecX>(HORIZON);
    _D_max = std::vector<VecX>(HORIZON);
    for (int i = 0; i < HORIZON; ++i) {
        _D[i] = MatX::Zero(20, 12);
        _D[i].setZero();
        for (int leg_id = 0; leg_id < 4; ++leg_id) {
            _D[i].block<5, 3>(5 * leg_id, 3 * leg_id) << 1., 0., 0.,
                    -1., 0., 0.,
                    0., 1., 0.,
                    0., -1., 0.,
                    0., 0., 1.;
        }
        _D_min[i] = VecX::Zero(20);
        _D_max[i] = VecX::Zero(20);
    }
    /*旋转矩阵*/
    _inv_Rw.setIdentity();
}

void MpcController::initSolver() {
    _solver = std::make_shared<MpcSolver>(HORIZON);
    // 状态空间初始化
    _solver->setupStateMat_A(_A_dt);
    _solver->setupStateMat_B(_B_dt);
    _solver->setupStateVec_b(_g_dt);
    // 损失函数初始化
    Mat12 Q, R;
    Q.setZero();
    Q += _L_diag.asDiagonal();
    Q += _N_diag.asDiagonal();
    R.setZero();
    R += _K_diag.asDiagonal();
    R += _M_diag.asDiagonal();
    // std::cout << R << std::endl;
    Vec12 q = -Q * Vec12::Zero(); // q = -Q * x_ref
    _solver->setupLossMat_Q(Q);
    _solver->setupLossMat_R(R);
    _solver->setupLossVec_q(q);
    // 约束条件初始化
    _solver->setupConstraintMat_C(MatX::Zero(20, 12));
    _solver->setupConstraintMat_D(_D[0]);
    _solver->setupConstraintVec_lg(_D_min[0]);
    _solver->setupConstraintVec_ug(_D_max[0]);
    // 求解器初始化
    _solver->setupQpSolver();
}

[[noreturn]] void MpcController::run(int ms) {
    std::cout << "[MPC] Task Run!" << std::endl;
    // assignTask2Cpu(2);
    std::chrono::microseconds period(ms * 1000);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        step();
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}

void MpcController::step() {
    _mrt->step(_robot, _gait, _estimator);
    updateMat();
    updateConstraint();
    solve();
    publishMpcOutput();
}

void MpcController::updateMat() {
    const RotMat &R = _robot->getRotMat();
    _inv_Rw = invRotMatW(_robot->getRpy());
    _I_world = R * _I_body * R.transpose();
    Mat3 I_world_inv = _I_world.inverse();
    // A
    _A_dt.block<3, 3>(0, 6) << _inv_Rw * _dt;
    // B
    for (int i = 0; i < 4; ++i) {
        Vec3 r = R * (_robot->getFootPosition_inBody(i) - _body_com);
        _B_dt.block<3, 3>(6, i * 3) << I_world_inv * skew(r) * _dt;
    }
}

void MpcController::updateConstraint() {
    const std::vector<Vec4_i8> &gait_list = _gait->getGaitList();
    for (int leg_id = 0; leg_id < 4; ++leg_id) {
        if (_estimator->getContact(leg_id) == CONTACT) {
            _D[0].block<4, 1>(5 * leg_id + 0, 3 * leg_id + 2) << -_mu, -_mu, -_mu, -_mu;
            _D_min[0].segment<5>(5 * leg_id) << -inf, -inf, -inf, -inf, _f_min;
            _D_max[0](5 * leg_id + 4) = _f_max;
        } else {
            _D[0].block<4, 1>(5 * leg_id + 0, 3 * leg_id + 2) << 0, 0, 0, 0;
            _D_min[0].segment<5>(5 * leg_id) << 0., 0., 0., 0., 0.;
            _D_max[0](5 * leg_id + 4) = 0.;
        }
    }
    for (int i = 1; i < HORIZON; ++i) {
        for (int leg_id = 0; leg_id < 4; ++leg_id) {
            if (gait_list[i](leg_id) == CONTACT) {
                _D[i].block<4, 1>(5 * leg_id + 0, 3 * leg_id + 2) << -_mu, -_mu, -_mu, -_mu;
                _D_min[i].segment<5>(5 * leg_id) << -inf, -inf, -inf, -inf, _f_min;
                _D_max[i](5 * leg_id + 4) = _f_max;
            } else {
                _D[i].block<4, 1>(5 * leg_id + 0, 3 * leg_id + 2) << 0, 0, 0, 0;
                _D_min[i].segment<5>(5 * leg_id) << 0., 0., 0., 0., 0.;
                _D_max[i](5 * leg_id + 4) = 0.;
            }
        }
    }
}

void MpcController::solve() {
    // 更新状态空间表达时
    _solver->updateStateMat_A(_A_dt);
    _solver->updateStateMat_B(_B_dt);
    // 更新约束矩阵
    _solver->updateConstraintMat_D(_D);
    _solver->updateConstraintVec_lg(_D_min);
    _solver->updateConstraintVec_ug(_D_max);
    // 更新损失函数
    _solver->updateLossVec_q(_mrt->getXtraj(), _N_diag.asDiagonal());
    _solver->updateLossVec_r(_M_diag.asDiagonal());
    // solver
    _X << _robot->getRpy(),
            _estimator->getLpPosition(),
            _robot->getAngularVelocity_inWorld(),
            _estimator->getLpVelocity();
    _mpc_f = _solver->solve(_X);
}

void MpcController::publishMpcOutput() {
    for (int i = 0; i < 12; ++i) {
        _mpc_output.force[i] = _mpc_f[i];
    }
    _lcm.publish(_mpc_topic_name, &_mpc_output);
}
