//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_DENSE_QP_SOLVER_HPP
#define BUPT_DOG_CONTROLLER2_DENSE_QP_SOLVER_HPP

#include "utils/math_types.hpp"
#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_dense_qp_ipm.h>
#include <hpipm_d_dense_qp_dim.h>
#include <hpipm_d_dense_qp.h>
#include <hpipm_d_dense_qp_sol.h>
#include <hpipm_d_dense_qp_utils.h>
#include <hpipm_timing.h>
#include <iostream>

class DenseQpSolver {
public:
    DenseQpSolver(int nv, int ne, int ng) {
        nv_ = nv;
        ne_ = ne;
        ng_ = ng;
        /*output*/
        u_ = VecX::Zero(nv_);
        /***************
         * dim
         ***************/
        hpipm_size_t dim_size = d_dense_qp_dim_memsize();
        dim_mem_ = malloc(dim_size);
        d_dense_qp_dim_create(&dim_, dim_mem_);
        d_dense_qp_dim_set_all(nv_, ne_, 0, ng_, 0, 0, &dim_);
        /***************
         * qp
         ***************/
        initDenseQpData();
        /************************************************
         * dense qp sol
         ************************************************/
        hpipm_size_t qp_sol_size = d_dense_qp_sol_memsize(&dim_);
        qp_sol_mem_ = malloc(qp_sol_size);
        d_dense_qp_sol_create(&dim_, &qp_sol_, qp_sol_mem_);
        /************************************************
         * ipm arg
         ************************************************/
        hpipm_size_t ipm_arg_size = d_dense_qp_ipm_arg_memsize(&dim_);
        ipm_arg_mem_ = malloc(ipm_arg_size);
        d_dense_qp_ipm_arg_create(&dim_, &arg_, ipm_arg_mem_);
        d_dense_qp_ipm_arg_set_default(mode, &arg_);
        d_dense_qp_ipm_arg_set_mu0(&mu0, &arg_);
        d_dense_qp_ipm_arg_set_iter_max(&iter_max, &arg_);
        d_dense_qp_ipm_arg_set_alpha_min(&alpha_min, &arg_);
        d_dense_qp_ipm_arg_set_mu0(&mu0, &arg_);
        d_dense_qp_ipm_arg_set_tol_stat(&tol_stat, &arg_);
        d_dense_qp_ipm_arg_set_tol_eq(&tol_eq, &arg_);
        d_dense_qp_ipm_arg_set_tol_ineq(&tol_ineq, &arg_);
        d_dense_qp_ipm_arg_set_tol_comp(&tol_comp, &arg_);
        d_dense_qp_ipm_arg_set_reg_prim(&reg_prim, &arg_);
        d_dense_qp_ipm_arg_set_reg_dual(&reg_dual, &arg_);
        d_dense_qp_ipm_arg_set_warm_start(&warm_start, &arg_);
        d_dense_qp_ipm_arg_set_pred_corr(&pred_corr, &arg_);
        d_dense_qp_ipm_arg_set_split_step(&split_step, &arg_);
        /************************************************
         * ipm workspace
         ************************************************/
        hpipm_size_t ipm_size = d_dense_qp_ipm_ws_memsize(&dim_, &arg_);
        ipm_mem_ = malloc(ipm_size);
        d_dense_qp_ipm_ws_create(&dim_, &arg_, &workspace_, ipm_mem_);

        std::cout << "dense qp solver initialized" << std::endl;
    }

    void test() {
        MatX M = Mat3::Zero();
        M << 1, 2, 0,
                -8, 3, 2,
                0, 1, 1;
        // 二次型矩阵
        H_ = M.transpose() * M;
        g_ = (Vec3(3, 2, 3).transpose() * M).transpose();
        // 不等式约束
        C_ << 1, 2, 1,
                2, 0, 1,
                -1, 2, -1;
        lg_ << 3, 2, -2;
        ug_ << 3, 2, -2;
        lg_mask_.setZero();
        ug_mask_.setOnes();
        // 等式约束
        A_ << 1, 1, 1;
        b_ << 1.0;

        /************************************************
         * dense qp
         ************************************************/
        DenseQpSetMat_H(H_);
        DenseQpSetVec_g(g_);
        DenseQpSetMat_A(A_);
        DenseQpSetVec_b(b_);
        DenseQpSetMat_C(C_);
        DenseQpSetVec_lg(lg_);
        DenseQpSetVec_ug(ug_);
        DenseQpSetVec_lg_mask(lg_mask_);
        DenseQpSetVec_ug_mask(ug_mask_);

        DenseQpSolve();
    }

    /* 二次规划: 0.5 v.T H v + g.T v*/
    bool DenseQpSetMat_H(const MatX &H) {
        // check
        if (H.rows() != nv_ || H.cols() != nv_) {
            std::cerr << "H shape error" << std::endl;
            return false;
        }
        H_ = H;
        d_dense_qp_set_H(H_.data(), &qp_);
        return true;
    }

    bool DenseQpSetVec_g(const VecX &g) {
        // check
        if (g.rows() != nv_) {
            std::cerr << "g shape error" << std::endl;
            return false;
        }
        g_ = g;
        d_dense_qp_set_g(g_.data(), &qp_);
        return true;
    }

    /* 等式约束: Av = b */
    bool DenseQpSetMat_A(const MatX &A) {
        if (A.rows() != ne_ || A.cols() != nv_) {
            std::cerr << "A shape error" << std::endl;
            return false;
        }
        A_ = A;
        d_dense_qp_set_A(A_.data(), &qp_);
        return true;
    }

    bool DenseQpSetVec_b(const VecX &b) {
        if (b.rows() != ne_) {
            std::cerr << "b shape error" << std::endl;
            return false;
        }
        b_ = b;
        d_dense_qp_set_b(b_.data(), &qp_);
        return true;
    }

    /* 不等式约束: lg <= Cv <= ug */
    bool DenseQpSetMat_C(const MatX &C) {
        if (C.rows() != ng_ || C.cols() != nv_) {
            std::cerr << "C shape error" << std::endl;
            return false;
        }
        C_ = C;
        d_dense_qp_set_C(C_.data(), &qp_);
        return true;
    }

    bool DenseQpSetVec_lg(const VecX &lg) {
        if (lg.rows() != ng_) {
            std::cerr << "lg shape error" << std::endl;
            return false;
        }
        lg_ = lg;
        d_dense_qp_set_lg(lg_.data(), &qp_);
        return true;
    }

    bool DenseQpSetVec_ug(const VecX &ug) {
        if (ug.rows() != ng_) {
            std::cerr << "ug shape error" << std::endl;
            return false;
        }
        ug_ = ug;
        d_dense_qp_set_ug(ug_.data(), &qp_);
        return true;
    }

    /* mask 向量元素为1或0,1为该约束生效，0为不生效 */
    bool DenseQpSetVec_lg_mask(const VecX &lg_mask) {
        if (lg_mask.rows() != ng_) {
            std::cerr << "lg_mask shape error" << std::endl;
            return false;
        }
        lg_mask_ = lg_mask;
        d_dense_qp_set_lg_mask(lg_mask_.data(), &qp_);
        return true;
    }

    bool DenseQpSetVec_ug_mask(const VecX &ug_mask) {
        if (ug_mask.rows() != ng_) {
            std::cerr << "ug_mask shape error" << std::endl;
            return false;
        }
        ug_mask_ = ug_mask;
        d_dense_qp_set_ug_mask(ug_mask_.data(), &qp_);
        return true;
    }

    bool DenseQpSolve() {
        /************************************************
         * ipm solver
         ************************************************/
        int hpipm_status;
        // call solver
        d_dense_qp_ipm_solve(&qp_, &qp_sol_, &arg_, &workspace_);
        d_dense_qp_ipm_get_status(&workspace_, &hpipm_status);

        /************************************************
         * print solution info
         ************************************************/
        switch (hpipm_status) {
            case 0:
                break;
            case 1:
                std::cout << "[WBC SOLVER] MaxIterReached" << std::endl;
                return false;
            case 2:
                std::cout << "[WBC SOLVER] MinStepLengthReached" << std::endl;
                return false;
            case 3:
                std::cout << "[WBC SOLVER] NaNDetected" << std::endl;
                return false;
            case 4:
            default:
                std::cout << "[WBC SOLVER] UnknownFailure" << std::endl;
                return false;
        }
        /************************************************
         * extract and print solution
         ************************************************/
        d_dense_qp_sol_get_v(&qp_sol_, u_.data());
        return true;
    }

    const VecX &getOutput() { return u_; }

private:
    void initDenseQpData() {
        H_ = MatX::Zero(nv_, nv_);
        g_ = VecX::Zero(nv_);
        A_ = MatX::Zero(ne_, nv_);
        b_ = VecX::Zero(ne_);
        C_ = MatX::Zero(ng_, nv_);
        lg_ = VecX::Zero(ng_);
        ug_ = VecX::Zero(ng_);
        lg_mask_ = VecX::Ones(ng_);
        ug_mask_ = VecX::Ones(ng_);

        hpipm_size_t qp_size = d_dense_qp_memsize(&dim_);
        qp_mem_ = malloc(qp_size);
        d_dense_qp_create(&dim_, &qp_, qp_mem_);
    }

    /*output*/
    VecX u_;
    /***************
     * dim
     ***************/
    void *dim_mem_;
    struct d_dense_qp_dim dim_;

    int nv_;     // number of variables
    int ne_;     // number of equality constraints
    int nb_ = 0; // number of box constraints
    int ng_;     // number of general (inequality) constraints
    int nsb_ = 0;
    int nsg_ = 0;
    /***************
     * qp
     ***************/
    void *qp_mem_;
    struct d_dense_qp qp_;
    /*H*/
    MatX H_;
    /*g*/
    VecX g_;
    /*A*/
    MatX A_;
    /*b*/
    VecX b_;
    /*C*/
    MatX C_;
    /*lg*/
    VecX lg_;
    /*ug*/
    VecX ug_;
    /*lg_mask*/
    VecX lg_mask_;
    /*ug_mask*/
    VecX ug_mask_;
    /***************
     * arg
     ***************/
    /* mode */
    hpipm_mode mode = hpipm_mode::SPEED_ABS;
    /* iter_max */
    int iter_max = 100;
    /* alpha_min */
    double alpha_min = 1.000000000000000e-12;
    /* mu0 */
    double mu0 = 1.000000000000000e+04;
    /* tol_stat */
    double tol_stat = 1.000000000000000e-04;
    /* tol_eq */
    double tol_eq = 1.000000000000000e-05;
    /* tol_ineq */
    double tol_ineq = 1.000000000000000e-05;
    /* tol_comp */
    double tol_comp = 1.000000000000000e-05;
    /* reg_prim */
    double reg_prim = 1.000000000000000e-12;
    /* reg_dual */
    double reg_dual = 1.000000000000000e-15;
    /* warm_start */
    int warm_start = 1;
    /* pred_corr */
    int pred_corr = 1;
    /* split_step */
    int split_step = 1;
    /************************************************
     * dense qp sol
     ************************************************/
    void *qp_sol_mem_;
    struct d_dense_qp_sol qp_sol_;
    /************************************************
     * ipm arg
     ************************************************/
    void *ipm_arg_mem_;
    struct d_dense_qp_ipm_arg arg_;
    /************************************************
     * ipm workspace
     ************************************************/
    void *ipm_mem_;
    struct d_dense_qp_ipm_ws workspace_;
};


#endif //BUPT_DOG_CONTROLLER2_DENSE_QP_SOLVER_HPP
