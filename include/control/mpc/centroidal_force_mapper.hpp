//
// Created by zyb on 24-4-26.
//
#ifndef BUPT_DOG_CONTROLLER2_CENTRODIAL_FROCE_MAPPER_HPP
#define BUPT_DOG_CONTROLLER2_CENTRODIAL_FROCE_MAPPER_HPP

#include "common/robot.hpp"
#include "control/wbc/dense_qp_solver.hpp"
#include "gait/gait.hpp"
#include "mpc_param.hpp"

class CentroidalForceMapper {

  public:
    CentroidalForceMapper(const std::shared_ptr<Robot>& robot, const std::shared_ptr<Gait>& gait);

    void solve(const Vec6& F);

    Vec12 getContactForce() { return _contact_force; };

  private:
    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Gait> _gait;
    std::shared_ptr<DenseQpSolver> _qp_solver;
    int _nv;
    int _ne;
    int _ng; // number of u  || equality constraints || inequality constraints
    MatX _H;
    VecX _g;
    MatX _A;
    VecX _b;
    VecX _lg;
    VecX _ug;
    MatX _C;
    VecX _lg_mask;
    VecX _ug_mask;
    // loss param
    MatX _S;
    MatX _W;
    MatX _U;
    double _alpha, _beta;
    // output
    Vec12 _contact_force;
};

#endif