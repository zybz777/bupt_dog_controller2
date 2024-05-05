//
// Created by zyb on 24-4-26.
//

#include "control/mpc/mpc_controller.hpp"
#include "control/mpc/mpc_param.hpp"


MpcController::MpcController(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                             const std::shared_ptr<Estimator> &estimator) {
    _robot = robot;
    _gait = gait;
    _estimator = estimator;
    _mrt = std::make_shared<MrtGenerator>();
    _dt = (double) 1.0 / MPC_FREQUENCY;
    _ms = int(_dt * 1000.0);
    init();
    _mpc_thread = std::thread([this] { run(_ms); });
    std::cout << "[MpcController] Init Successful!" << std::endl;
}

void MpcController::begin() {
    _mpc_thread.join();
}

void MpcController::init() {
    /*机器人物理属性*/
    _M = M;
    _I_body << I_diag[0], 0, 0,
            0, I_diag[1], 0,
            0, 0, I_diag[2];
    _body_com << Body_Com[0], Body_Com[1], Body_Com[2];
    /*mpc input*/
    _X.setZero();
    /*mpc output*/
    _mpc_f.setZero();
    _mpc_f_last.setZero();
    /*mpc 权重*/
#ifdef USE_SIM
    _L_diag << 0.8, 0.8, 1.0, // 角度
            0.8, 0.8, 1.2,
            0.1, 0.1, 0.001, // 角速度
            1.0, 1.0, 1.0; // simulink weight
    _K = 2.0e-6;       // 1e-6
    _S = 0;
#else
    //    _L_diag << 0.6, 0.6, 1.0,
    //            0.5, 0.5, 1.5,
    //            0.6, 0.6, 1.0,
    //            0.5, 0.5, 1.0; // simulink weight
    //    _K = 5.0e-5;       // 1e-6
    //    _S = 0.0;
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
    _R.setIdentity();
    _inv_Rw.setIdentity();
}

void MpcController::initSolver() {
    _solver = std::make_shared<MpcSolver>(HORIZON);
    // 状态空间初始化
    _solver->setupStateMat_A(_A_dt);
    _solver->setupStateMat_B(_B_dt);
    _solver->setupStateVec_b(_g_dt);
    // 损失函数初始化
    Mat12 Q, R, S;
    Q.setZero();
    Q.diagonal() << _L_diag;
    S.setIdentity();
    S << _S * S;
    Vec12 r = -S * _mpc_f_last; // r = -S * u_last
    R.setIdentity();
    R << _K * R;
    R += S;
    Vec12 q = -Q * Vec12::Zero(); // q = -Q * x_ref
    _solver->setupLossMat_Q(Q);
    _solver->setupLossMat_R(R);
    _solver->setupLossVec_q(q);
    _solver->setupLossVec_r(r);
    // 约束条件初始化
    _solver->setupConstraintMat_C(MatX::Zero(20, 12));
    _solver->setupConstraintMat_D(_D[0]);
    _solver->setupConstraintVec_lg(_D_min[0]);
    _solver->setupConstraintVec_ug(_D_max[0]);
    // 求解器初始化
    _solver->setupQpSolver();
}

[[noreturn]] void MpcController::run(int ms) {
    std::chrono::microseconds period(ms * 1000);
    std::chrono::microseconds zero_us(0);
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
    _R = _robot->getRotMat();
    _inv_Rw = invRotMatW(_robot->getRpy());
    _I_world = _R * _I_body * _R.transpose();
    Mat3 I_world_inv = _I_world.inverse();
    // A
    _A_dt.block<3, 3>(0, 6) << _inv_Rw * _dt;
    // B
    Vec3 r;
    for (int i = 0; i < 4; ++i) {
        r = _R * (_robot->getFootPosition_inBody(i) - _body_com);
        _B_dt.block<3, 3>(6, i * 3) << I_world_inv * skew(r) * _dt;
    }
}

void MpcController::updateConstraint() {
    std::vector<Vec4_i8> gait_list = _gait->getGaitList();
    for (int i = 0; i < HORIZON; ++i) {
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
    _solver->updateLossVec_q(_mrt->getXtraj());
    Mat12 S;
    S.setIdentity();
    S << _S * S;
    _solver->updateLossVec_r(_mpc_f, S);
    // solver
    _X << _robot->getRpy(),
            _estimator->getPosition(),
            rotMatW(_robot->getRpy()) * _robot->getAngularVelocity(),
            _estimator->getLpVelocity();
    _mpc_f_last = _mpc_f;
    _mpc_f = _solver->solve(_X);
}

void MpcController::publishMpcOutput() {
    for (int i = 0; i < 12; ++i) {
        _mpc_output.force[i] = _mpc_f[i];
    }
    _lcm.publish(_mpc_topic_name, &_mpc_output);
}
