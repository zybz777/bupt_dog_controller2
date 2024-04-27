//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP
#define BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP


#include "common/robot.hpp"
#include "gait/gait.hpp"
#include "dense_qp_solver.hpp"

class WbcOptimizer {
public:
    WbcOptimizer(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait);

    const VecX &calcCmdTau(Vec18 cmd_ddq, Vec12 f_mpc);

private:
    void updateDenseQpEqualityConstraints(Vec18 cmd_ddq, Vec12 f_mpc);

    void updateDenseQpInequalityConstraints(Vec12 f_mpc);

    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Gait> _gait;
    /* qp solver */
    std::shared_ptr<DenseQpSolver> _qp_solver;
    int _nv = 18; // 变量数量 6 + 12
    int _ne = 6;  // 等式约束
    int _ng = 20; // 不等式约束
    MatX _H;
    VecX _g;
    // 等式约束
    MatX _A;
    VecX _b;
    MatX _Sf;
    // 不等式约束
    double _mu = 0.4;
    double _f_min = 10.0, _f_max = 120.0;
    MatX _C;
    VecX _lg;
    VecX _ug;
    VecX _lg_mask;
    VecX _ug_mask;
    // output
    VecX _cmd_tau;
};


#endif //BUPT_DOG_CONTROLLER2_WBC_OPTIMIZER_HPP
