//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_MPC_CONTROLLER_HPP
#define BUPT_DOG_CONTROLLER2_MPC_CONTROLLER_HPP

#include "common/estimator.hpp"
#include "common/robot.hpp"
#include "doglcm/MpcOutput_t.hpp"
#include "gait/gait.hpp"
#include "mpc_solver.hpp"
#include "mrt_generator.hpp"

class MpcController {
  public:
    MpcController(const std::shared_ptr<Robot>& robot, const std::shared_ptr<Gait>& gait,
                  const std::shared_ptr<Estimator>& estimator);

    void begin();

    const Vec12& getMpcOutput() { return _mpc_f; }

    Vec3 getMpcOutput(int leg_id) { return _mpc_f.segment<3>(3 * leg_id); }

    const std::shared_ptr<MrtGenerator>& getMrtGenerator() { return _mrt; }

  private:
    void init();

    void initMat();

    void initSolver();

    [[noreturn]] void run(int ms);

    void step();

    void updateMat();

    void updateConstraint();

    void solve();

    void publishMpcOutput();

    std::thread _mpc_thread;
    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Gait> _gait;
    std::shared_ptr<MrtGenerator> _mrt;
    std::shared_ptr<Estimator> _estimator;
    double _dt;
    int _ms;
    /*mpc solver*/
    std::shared_ptr<MpcSolver> _solver;
    /*机器人物理属性*/
    Mat3 _I_body, _I_world; // 惯性矩阵
    double _M;              // 质量
    double _f_min, _f_max;  // 摩擦力约束
    Vec3 _body_com;         // 质心位置
    double _mu;             // 摩擦系数
    /*mpc input*/
    Vec12 _X;
    /*mpc output*/
    Vec12 _mpc_f;
    /*mpc 权重*/
    Vec12 _L_diag; // 状态量x的权重
    Vec12 _K_diag; // 输入量u的权重
    Vec12 _M_diag; // u - ulast
    Vec12 _N_diag; // x - xlast
    /*离散矩阵*/
    Mat12 _A_dt;
    Mat12 _B_dt;
    Vec12 _g_dt;
    /*约束条件：摩擦锥 lg <= D u <= ug*/
    vector<MatX> _D;
    vector<VecX> _D_min, _D_max;
    /*旋转矩阵*/
    RotMat _inv_Rw;
    // lcm
    lcm::LCM _lcm;
    std::string _mpc_topic_name;
    doglcm::MpcOutput_t _mpc_output;
};

#endif // BUPT_DOG_CONTROLLER2_MPC_CONTROLLER_HPP