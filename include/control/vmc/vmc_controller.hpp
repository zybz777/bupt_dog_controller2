//
// Created by zyb on 24-4-24.
//

#ifndef BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP
#define BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP

#include <memory>
#include "utils/math_types.hpp"
#include "common/robot.hpp"
#include "gait/gait.hpp"
#include "common/estimator.hpp"

typedef struct VmcData {
    // 当前足端状态
    Vec34 curr_foot_pos;
    Vec34 curr_foot_vel;
    // 足端轨迹起点、终点、标准点
    Vec34 start_foot_pos;
    Vec34 std_foot_pos;
    Vec34 end_foot_pos;
    Vec34 rot_start_foot_pos;
    // 世界系下数据
    Vec34 curr_foot_pos_in_world;
    Vec34 curr_foot_vel_in_world;
    Vec34 start_foot_pos_in_world;
    Vec34 std_foot_pos_in_world;
    Vec34 end_foot_pos_in_world;
} VmcData;

typedef struct VmcCmd {
    // 期望足端状态
    Vec34 cmd_foot_pos;
    Vec34 cmd_foot_vel;
    Vec34 cmd_foot_force;
    Vec34 cmd_foot_acc;
    // 世界系
    Vec34 cmd_foot_pos_in_world;
    Vec34 cmd_foot_vel_in_world;
    Vec34 cmd_foot_acc_in_world;
} VmcCmd;


class VmcController {
public:
    VmcController(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                  const std::shared_ptr<Estimator> &estimator);

    void step();

    void setHeight(double h) {
        if (h < 0.03 || h > 0.15) {
            return;
        }
        _h_vec << h, h, h, h;
    }

    void setHeight(Vec4 h) {
        for (int i = 0; i < 4; ++i) {
            if (h[i] < 0.03 || h[i] > 0.15) {
                return;
            }
        }
        _h_vec = h;
    }

    void setHeursticRatio(double k) {
        if (k < 0.0 || k > 1.0) {
            return;
        }
        _k = k;
    }

    void updateStdFootPos(double lx, double ly, double lz);

    Vec3 getCmdFootPos_inWorld(int leg_id) { return _vmc_cmd->cmd_foot_pos_in_world.col(leg_id); }

    Vec3 getCmdFootVel_inWorld(int leg_id) { return _vmc_cmd->cmd_foot_vel_in_world.col(leg_id); }

    Vec3 getCmdFootAcc_inWorld(int leg_id) { return _vmc_cmd->cmd_foot_acc_in_world.col(leg_id); }

private:
    void updateStartFeetPos_inWorld(const std::shared_ptr<Gait> &gait, const std::shared_ptr<Estimator> &estimator);

    void updateEndFeetPos_inWorld(const std::shared_ptr<Robot> &robot,
                                  const std::shared_ptr<Gait> &gait,
                                  const std::shared_ptr<Estimator> &estimator,
                                  const std::shared_ptr<doglcm::UserCmd_t> &user_cmd);

    void SwingLegPolynomialCurve_inWorld(int leg_id, int contact, double phase, double swing_T, double h = 0.05);

    void calcEndFootPos_inWorld(int leg_id, const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                                const std::shared_ptr<Estimator> &estimator);

    std::shared_ptr<VmcData> _vmc_data;
    std::shared_ptr<VmcCmd> _vmc_cmd;
    Vec4 _theta0;
    double _r;
    std::shared_ptr<Robot> _robot;
    std::shared_ptr<Gait> _gait;
    std::shared_ptr<Estimator> _estimator;
    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    Vec4 _h_vec;
    double _k; // heurstic 根据实际速度计算落足点位置比例 =1表示完全使用实际速度
};


#endif //BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP
