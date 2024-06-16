#include "control/mpc/mpc_controller2.hpp"
#include "control/mpc/mpc_param.hpp"
#include "utils/math_types.hpp"

MpcController2::MpcController2(const std::shared_ptr<Robot>& robot, const std::shared_ptr<Gait>& gait,
                               const std::shared_ptr<Estimator>& estimator) {
    _robot = robot;
    _gait = gait;
    _estimator = estimator;
    _mrt = std::make_shared<MrtGenerator>();
    _dt = (double)1.0 / MPC_FREQUENCY;
    _ms = int(_dt * 1000.0);
    _force_mapper = std::make_shared<CentroidalForceMapper>(_robot, _gait);
    init();
#ifdef USE_MPC2
    _mpc_thread = std::thread([this] { run(_ms); });
    _force_mapper_thread = std::thread([this] { run_force_mapper(1); });
#endif
    std::cout << "[MpcController2] Init Successful!" << std::endl;
}

void MpcController2::begin() {
#ifdef USE_MPC2
    _mpc_thread.join();
    _force_mapper_thread.join();
#endif
}

void MpcController2::init() {
    /*机器人物理属性*/
    _M = _robot->getRobotMass();
    _I_body << _robot->getRobotInertial();
    _body_com = _robot->getRobotStdCom();
    /*mpc input*/
    _X.setZero();
    /*mpc output*/
    _mpc_f.setZero();
    /*mpc 权重*/
    _L_diag << 10.0, 10.0, 10.0, // 角度
        0.0, 0.0, 10.0,
        0.01, 0.01, 1.0, // 角速度
        5.0, 5.0, 0.01; // simulink weight
    _K_diag << 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6, 1.0e-6;
    /*矩阵*/
    initMat();
    /*solver*/
    initSolver();
    // lcm
    _mpc_topic_name = "mpc_output2";
    for (double& force : _mpc_output.force) {
        force = 0.0;
    }
}

void MpcController2::initMat() {
    // A B g
    _A_dt.setIdentity();
    _A_dt.block<3, 3>(3, 9) << _I3 * _dt;
    _B_dt = MatX::Zero(12, 6);
    _B_dt.block<3, 3>(9, 0) << _I3 / _M * _dt;
    _g_dt.setZero();
    _g_dt[11] = -9.81 * _dt;
    /*约束条件：摩擦锥 lg <= D u <= ug*/
    _D = std::vector<MatX>(HORIZON);
    _D_min = std::vector<VecX>(HORIZON);
    _D_max = std::vector<VecX>(HORIZON);
    for (int i = 0; i < HORIZON; ++i) {
        _D[i] = MatX::Identity(6, 6);
        _D_min[i] = VecX::Zero(6);
        _D_max[i] = VecX::Zero(6);
        _D_min[i] << -inf, -inf, -inf, -inf, -inf, -inf;
        _D_max[i] << inf, inf, inf, inf, inf, inf;
    }
    /*旋转矩阵*/
    _R.setIdentity();
    _inv_Rw.setIdentity();
}

void MpcController2::initSolver() {
    _solver = std::make_shared<MpcSolver>(HORIZON);
    // 状态空间初始化
    _solver->setupStateMat_A(_A_dt);
    _solver->setupStateMat_B(_B_dt);
    _solver->setupStateVec_b(_g_dt);
    // 损失函数初始化
    Mat12 Q;
    MatX R = MatX::Zero(6, 6);
    Q.setZero();
    Q.diagonal() << _L_diag;
    R.setIdentity();
    R.diagonal() << _K_diag;
    Vec12 q = -Q * Vec12::Zero(); // q = -Q * x_ref
    _solver->setupLossMat_Q(Q);
    _solver->setupLossMat_R(R);
    _solver->setupLossVec_q(q);
    // 约束条件初始化
    _solver->setupConstraintMat_C(MatX::Zero(6, 12));
    _solver->setupConstraintMat_D(_D[0]);
    _solver->setupConstraintVec_lg(_D_min[0]);
    _solver->setupConstraintVec_ug(_D_max[0]);
    // 求解器初始化
    _solver->setupQpSolver();
}

[[noreturn]] void MpcController2::run(int ms) {
    std::cout << "[MPC] Task Run!" << std::endl;
    // assignTask2Cpu(2);
    std::chrono::microseconds period(ms * 1000);
    std::chrono::microseconds zero_us(0);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        step();
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}

void MpcController2::step() {
    _mrt->step(_robot, _gait, _estimator);
    updateMat();
    solve();
    publishMpcOutput();
}

void MpcController2::updateMat() {
    _R = _robot->getRotMat();
    _inv_Rw = invRotMatW(_robot->getRpy());
    _I_world = _R * _I_body * _R.transpose();
    // A
    _A_dt.block<3, 3>(0, 6) << _inv_Rw * _dt;
    // B
    _B_dt.block<3, 3>(6, 3) << _I_world.inverse() * _dt;
}

void MpcController2::solve() {
    // 更新状态空间表达时
    _solver->updateStateMat_A(_A_dt);
    _solver->updateStateMat_B(_B_dt);
    // 更新约束矩阵
    _solver->updateConstraintMat_D(_D);
    _solver->updateConstraintVec_lg(_D_min);
    _solver->updateConstraintVec_ug(_D_max);
    // 更新损失函数
    _solver->updateLossVec_q(_mrt->getXtraj());
    // solver
    _X << _robot->getRpy(),
        _estimator->getPosition(),
        _robot->getAngularVelocity_inWorld(),
        _estimator->getLpVelocity();
    _mpc_f = _solver->solve(_X);
    // std::cout << "mpc2 f " << _mpc_f.transpose() << std::endl;
}

void MpcController2::publishMpcOutput() {
    for (int i = 0; i < 12; ++i) {
        _mpc_output.force[i] = getContactForce()[i];
    }
    _lcm.publish(_mpc_topic_name, &_mpc_output);
}

void MpcController2::run_force_mapper(int ms) {
    std::cout << "[MPC Mapper] Task Run!" << std::endl;
    // assignTask2Cpu(2);
    std::chrono::microseconds period(ms * 1000);
    std::chrono::microseconds zero_us(0);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        _force_mapper->solve(_mpc_f);
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}