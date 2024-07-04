//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_MPC_SOLVER_HPP
#define BUPT_DOG_CONTROLLER2_MPC_SOLVER_HPP

#include "hpipm-cpp/hpipm-cpp.hpp"
#include "utils/math_types.hpp"

class MpcSolver {
public:
    MpcSolver(int h) {
        N_ = h;
        // qp 初始化
        qp_ = std::vector<hpipm::OcpQp>(N_ + 1);
    }

    // 初始化离散形式状态方程 x(k+1) = A x(k) + B u(k) + b
    void setupStateMat_A(const MatX &A) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].A = A;
        }
        X_NUM_ = (int) A.rows();
    }

    void setupStateMat_B(const MatX &B) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].B = B;
        }
        U_NUM_ = (int) B.cols();
        u_output_ = VecX::Zero(U_NUM_);
    }

    void setupStateVec_b(const VecX &b) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].b = b;
        }
    }

    // 初始化损失函数 Q R q； Q R 分别为x和u的权重矩阵 q=-Q * x_ref
    void setupLossMat_Q(const MatX &Q) {
        for (int i = 0; i < N_ + 1; ++i) {
            qp_[i].Q = Q;
        }
    }

    void setupLossMat_R(const MatX &R) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].R = R;
        }
    }

    void setupLossVec_q(const VecX &q) {
        MatX S(U_NUM_, X_NUM_);
        S.setZero();
        VecX r = VecX::Zero(U_NUM_);
        for (int i = 0; i < N_; ++i) {
            qp_[i].S = S;
            qp_[i].q = q;
            qp_[i].r = r;
        }
        qp_[N_].q = q;
    }

    void setupLossVec_r() {
        for (int i = 0; i < N_; ++i) {
            qp_[i].r = VecX::Zero(U_NUM_);
        }
    }

    // 约束条件 lg < C x + D u < ug
    void setupConstraintMat_C(const MatX &C) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].C = C;
        }
    }

    void setupConstraintMat_D(const MatX &D) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].D = D;
        }
    }

    void setupConstraintVec_lg(const VecX &lg) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].lg = lg;
        }
    }

    void setupConstraintVec_ug(const VecX &ug) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].ug = ug;
        }
    }

    // 初始化求解器
    void setupQpSolver() {
        // solver setting 初始化
        solver_settings_.mode = hpipm::HpipmMode::SpeedAbs;
        solver_settings_.iter_max = 200;
        solver_settings_.warm_start = 1;
        solver_settings_.alpha_min = 1e-12;
        solver_settings_.mu0 = 1e4;
        solver_settings_.tol_stat = 1e-04;
        solver_settings_.tol_eq = 1e-05;
        solver_settings_.tol_ineq = 1e-05;
        solver_settings_.tol_comp = 1e-05;
        solver_settings_.reg_prim = 1e-12;
        solver_settings_.pred_corr = 1;
        solver_settings_.ric_alg = 1;
        solver_settings_.split_step = 1;
        // solution 初始化
        solution_ = std::vector<hpipm::OcpQpSolution>(N_ + 1);
        for (int i = 0; i < N_; ++i) {
            solution_[i].x = VecX::Zero(X_NUM_);
            solution_[i].u.resize(U_NUM_);
        }
        solution_[N_].x = VecX::Zero(X_NUM_);
        // solver 求解器 初始化
        solver_ = new hpipm::OcpQpIpmSolver(qp_, solver_settings_);
    }

    // 更新离散形式状态方程
    void updateStateMat_A(const MatX &A) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].A = A;
        }
    }

    void updateStateMat_B(const MatX &B) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].B = B;
        }
    }

    void updateStateVec_b(const VecX &b) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].b = b;
        }
    }

    void updateStateMat_A(const MatX &A, int n) {
        qp_[n].A = A;
    }

    void updateStateMat_B(const MatX &B, int n) {
        qp_[n].B = B;
    }

    // 更新损失函数
    // void updateLossMat_Q(const MatX &Q);
    // void updateLossMat_R(const MatX &R);
    void updateLossVec_q(const std::vector<VecX> &X_ref) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].q = -qp_[i].Q * X_ref[i];
        }
    }

    void updateLossVec_q(const std::vector<VecX> &X_ref, const MatX &N) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].q = -qp_[i].Q * X_ref[i] - N * solution_[i].x;
        }
    }

    void updateLossVec_r(const MatX &S) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].r = -S * solution_[i].u;
        }
    }

    // 更新约束条件
    // void updateConstraintMat_C(const MatX &C);
    void updateConstraintMat_D(const std::vector<MatX> &D) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].D = D[i];
        }
    }

    void updateConstraintVec_lg(const std::vector<VecX> &lg) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].lg = lg[i];
        }
    }

    void updateConstraintVec_ug(const std::vector<VecX> &ug) {
        for (int i = 0; i < N_; ++i) {
            qp_[i].ug = ug[i];
        }
    }

    // 求解结果
    const VecX &solve(const VecX &x) {
        switch (solver_->solve(x, qp_, solution_)) {
            case hpipm::HpipmStatus::Success:
                u_output_ << solution_[0].u;
                break;
            case hpipm::HpipmStatus::MaxIterReached:
                std::cout << "[MPC SOLVER] MaxIterReached" << std::endl;
                break;
            case hpipm::HpipmStatus::MinStepLengthReached:
                std::cout << "[MPC SOLVER] MinStepLengthReached" << std::endl;
                break;
            case hpipm::HpipmStatus::NaNDetected:
                std::cout << "[MPC SOLVER] NaNDetected" << std::endl;
                break;
            case hpipm::HpipmStatus::UnknownFailure:
                std::cout << "[MPC SOLVER] UnknownFailure" << std::endl;
                break;
        }
        return u_output_;
    }

    const std::vector<hpipm::OcpQpSolution> &getSolution() {
        return solution_;
    }

private:
    int N_; // 预测步长
    /* hpipm solver */
    std::vector<hpipm::OcpQp> qp_;
    hpipm::OcpQpIpmSolverSettings solver_settings_;
    std::vector<hpipm::OcpQpSolution> solution_;
    hpipm::OcpQpIpmSolver *solver_;
    // 状态空间变量
    int X_NUM_, U_NUM_;
    // solve 结果
    VecX u_output_;
};

#endif //BUPT_DOG_CONTROLLER2_MPC_SOLVER_HPP
