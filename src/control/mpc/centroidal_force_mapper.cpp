#include "control/mpc/centroidal_force_mapper.hpp"

CentroidalForceMapper::CentroidalForceMapper(const std::shared_ptr<Robot>& robot, const std::shared_ptr<Gait>& gait) {
    _robot = robot;
    _gait = gait;
    _nv = 12;
    _ne = 6;
    _ng = 20;
    _qp_solver = std::make_shared<DenseQpSolver>(_nv, _ne, _ng);
    _H = MatX::Identity(_nv, _nv);
    _g = VecX::Zero(_nv);
    _A = MatX::Zero(_ne, _nv);
    _b = VecX::Zero(_ne);
    _lg = VecX::Zero(_ng);
    _ug = VecX::Zero(_ng);
    _lg_mask = VecX::Ones(_ng);
    _ug_mask = VecX::Ones(_ng);
    _C = MatX::Zero(_ng, _nv);
    for (int i = 0; i < 4; ++i) {
        _C.block<5, 3>(5 * i, 3 * i) << 1, 0, 0,
            -1, 0, 0,
            0, 1, 0,
            0, -1, 0,
            0, 0, 1;
    }
    _qp_solver->DenseQpSetMat_H(_H);
    _qp_solver->DenseQpSetVec_g(_g);
    _qp_solver->DenseQpSetMat_A(_A);
    _qp_solver->DenseQpSetVec_b(_b);
    _qp_solver->DenseQpSetMat_C(_C);
    _qp_solver->DenseQpSetVec_lg_mask(_lg_mask);
    _qp_solver->DenseQpSetVec_ug_mask(_ug_mask);
    // loss param
    Vec6 s;
    Vec12 w, u;
    s << 20, 20, 50, 450, 450, 450;
    _S = s.asDiagonal();
    w << 10, 10, 4, 10, 10, 4, 10, 10, 4, 10, 10, 4;
    _W = w.asDiagonal();
    _alpha = 1e-3;
    _beta = 0.1;
    u << 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3;
    u.setZero(); // temp
    _U = u.asDiagonal();
    // output
    _contact_force.setZero();
    std::cout << "[CentroidalForceMapper] Init Successful!" << std::endl;
}

void CentroidalForceMapper::solve(const Vec6& F) {
    Vec3 r;
    for (int i = 0; i < LEG_NUM; ++i) {
        _A.block<3, 3>(0, 3 * i) << _I3;
        r = _robot->getRotMat() * (_robot->getFootPosition_inBody(i) - _robot->getRobotStdCom());
        _A.block<3, 3>(3, 3 * i) << skew(r);
        if (_gait->getContact(i) == CONTACT) {
            _lg.segment<5>(5 * i) << -inf, -inf, -inf, -inf, f_min;
            _ug.segment<5>(5 * i) << 0, 0, 0, 0, f_max;
            _C.block<4, 1>(5 * i, 2 + 3 * i) << -mu, -mu, -mu, -mu;
        } else {
            _C.block<4, 1>(5 * i, 2 + 3 * i).setZero();
            _lg.segment<5>(5 * i).setZero();
            _ug.segment<5>(5 * i).setZero();
        }
    }
    _b = F;
    _H = _A.transpose() * _S * _A + _alpha * _W + _U;
    _g = -_A.transpose() * _S * _b - _U * Vec12::Zero(); // -A.TxSxb - belta_Uxf_prev

    _qp_solver->DenseQpSetMat_H(_H);
    _qp_solver->DenseQpSetVec_g(_g);
    _qp_solver->DenseQpSetMat_C(_C);
    _qp_solver->DenseQpSetVec_lg(_lg);
    _qp_solver->DenseQpSetVec_ug(_ug);
    _qp_solver->DenseQpSolve();
    _contact_force = _qp_solver->getOutput();
    std::cout << "contact_force " << _contact_force.transpose() << std::endl;
}