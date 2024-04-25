//
// Created by zyb on 24-4-24.
//

#ifndef BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP
#define BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP

#include <memory>
#include "utils/math_types.hpp"
#include "common/robot.hpp"
#include "gait/gait.hpp"

typedef struct VmcData {
    // 当前足端状态
    Vec34 curr_foot_pos;
    Vec34 curr_foot_vel;
    // 足端轨迹起点、终点、标准点
    Vec34 start_foot_pos;
    Vec34 std_foot_pos;
    Vec34 end_foot_pos;
    Vec34 rot_start_foot_pos;
} VmcData;
typedef struct VmcCmd {
    // 期望足端状态
    Vec34 cmd_foot_pos;
    Vec34 cmd_foot_vel;
    Vec34 cmd_foot_force;
    Vec34 cmd_foot_acc;
} VmcCmd;

class VmcController {
public:
    VmcController();

    void step(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
              const std::shared_ptr<doglcm::UserCmd_t> &user_cmd);

    const Vec34 &getCmdFootPos() { return _vmc_cmd->cmd_foot_pos; }

    const Vec34 &getCmdFootVel() { return _vmc_cmd->cmd_foot_vel; }

    const Vec34 &getCmdFootAcc() { return _vmc_cmd->cmd_foot_acc; }

    Vec3 getCmdFootPos(int leg_id) { return _vmc_cmd->cmd_foot_pos.col(leg_id); }

    Vec3 getCmdFootVel(int leg_id) { return _vmc_cmd->cmd_foot_vel.col(leg_id); }

    Vec3 getCmdFootAcc(int leg_id) { return _vmc_cmd->cmd_foot_acc.col(leg_id); }

private:
    void SwingLegPolynomialCurve(int leg_id, int contact, double phase, double swing_T, double h = 0.05);

    void updateStartFootPos(const std::shared_ptr<Gait> &gait);

    void updateEndFootPos(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait,
                          const std::shared_ptr<doglcm::UserCmd_t> &user_cmd);

    std::shared_ptr<VmcData> _vmc_data;
    std::shared_ptr<VmcCmd> _vmc_cmd;
};


#endif //BUPT_DOG_CONTROLLER2_VMC_CONTROLLER_HPP
