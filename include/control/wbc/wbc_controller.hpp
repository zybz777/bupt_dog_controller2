//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_CONTROLLER_HPP
#define BUPT_DOG_CONTROLLER2_WBC_CONTROLLER_HPP

#include "common/robot.hpp"
#include "control/mpc/mpc_controller.hpp"
#include "control/mpc/mpc_controller2.hpp"
#include "control/mpc/mpc_param.hpp"
#include "control/vmc/vmc_controller.hpp"
#include "control/wbc/dense_qp_solver.hpp"
#include "control/wbc/task/wbc_task.hpp"
#include "control/wbc/task/wbc_task_body_orientation.hpp"
#include "control/wbc/task/wbc_task_body_pos.hpp"
#include "control/wbc/task/wbc_task_foot_pos.hpp"
#include "wbc_optimizer.hpp"

class WbcController {
public:
    WbcController(int ms, const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                  const std::shared_ptr<Estimator> &estimator,
                  const std::shared_ptr<MpcController> &mpc,
                  const std::shared_ptr<MpcController2> &mpc2, const std::shared_ptr<VmcController> &vmc);
#ifdef USE_WBC_THREAD
    void begin();
#endif
    void step();

    Vec12 getLegCmdQ() { return _cmd_q.segment<12>(6); }

    Vec12 getLegCmdDq() { return _cmd_dq.segment<12>(6); }

    Vec12 getLegCmdTau() { return _cmd_tau.segment<12>(6); }

private:
    void updateData();

    void updateTask();

    void updateContactFootTask(WbcTask_FootPos &task);

    void updateBodyPosTask(WbcTask_BodyPos &task);

    void updateBodyOrientationTask(WbcTask_BodyOrientation &task);

    void updateSwingFootTask(WbcTask_FootPos &task);

    void solve();

    // common
    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Estimator> _estimator;
    std::shared_ptr<Gait> _gait;
    std::shared_ptr<MpcController> _mpc;
    std::shared_ptr<MpcController2> _mpc2;
    std::shared_ptr<VmcController> _vmc;
    std::shared_ptr<MrtGenerator> _mrt;
    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    double _dt;
    /* 任务空间 */
    std::vector<WbcTask *> _task_list;
    WbcTask_BodyPos _task_body_pos;
    WbcTask_BodyOrientation _task_body_orientation;
    WbcTask_FootPos _task_swing_foot;
    WbcTask_FootPos _task_contact_foot;
    /* 关节指令 */
    Vec18 _cmd_q; // 位置关节指令
    Vec18 _cmd_dq; // 速度关节指令
    Vec18 _cmd_ddq; // 加速度关节指令
    Vec18 _cmd_tau; // 力矩关节指令
    std::shared_ptr<LPFilter> _cmd_q_filter[18];
    std::shared_ptr<LPFilter> _cmd_dq_filter[18];
    std::shared_ptr<LPFilter> _cmd_ddq_filter[18];
    /* 机器人数据 */
    RotMat _rot_mat; // 旋转矩阵
    Vec3 _com_pos_inWorld;
    Vec3 _com_vel_inWorld;
    Vec3 _com_rpy;
    Vec3 _com_omega_inBody;
    Vec34 _feet_positions_inBody;
    Vec12 _f_mpc;
    // qp solver
    std::shared_ptr<WbcOptimizer> _optimizer;

    int _ms;
#ifdef USE_WBC_THREAD
    std::thread _wbc_thread;
    [[noreturn]] void run(int ms);
#endif
};

#endif //BUPT_DOG_CONTROLLER2_WBC_CONTROLLER_HPP
