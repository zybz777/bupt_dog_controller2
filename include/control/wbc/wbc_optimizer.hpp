//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP
#define BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP

#include <common/estimator.hpp>

#include "common/robot.hpp"
#include "dense_qp_solver.hpp"
#include "doglcm/MpcOutput_t.hpp"
#include "gait/gait.hpp"

class WbcOptimizer {
public:
    WbcOptimizer(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait, const std::shared_ptr<Estimator> &estimator);

    const VecX &calcCmdTau(Vec18 cmd_ddq, Vec12 f_mpc);

    const VecX &getCollisionTau() { return _tau; }

private:
    void updateDenseQpEqualityConstraints(Vec18 cmd_ddq, Vec12 f_mpc);

    void updateDenseQpInequalityConstraints(Vec12 f_mpc);

    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Gait> _gait;
    std::shared_ptr<Estimator> _estimator;
    /* qp solver */
    std::shared_ptr<DenseQpSolver> _qp_solver;
    int _nv = 18; // 变量数量 6 + 12
    int _ne = 6; // 等式约束
    int _ng = 20; // 不等式约束
    MatX _H;
    VecX _g;
    Vec6 _Q1;
    Vec12 _Q2;
    Vec12 _Q3;
    Vec6 _Q4;
    // 等式约束
    MatX _A;
    VecX _b;
    MatX _Sf;
    // 不等式约束
    double _mu;
    double _f_min, _f_max;
    MatX _C;
    VecX _lg;
    VecX _ug;
    VecX _lg_mask;
    VecX _ug_mask;
    // output
    VecX _cmd_tau;
    VecX _tau; // _cmd_tau+J.TxF 碰撞检测用
    Vec12 _last_contact_force;
    Vec6 _last_float_ddq;
    lcm::LCM _lcm;
    std::string _mpc_topic_name;
    doglcm::MpcOutput_t _mpc_output;
};

#endif //BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP
