//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP
#define BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP


#include "wbc_task.hpp"

class WbcTask_FootPos : public WbcTask {
public:
    WbcTask_FootPos() {
        _task_e = Vec3::Zero();
        _task_dx = Vec3::Zero();
        _task_ddx = Vec3::Zero();
        _J = MatX::Zero(12, 18);
        _dJ = MatX::Zero(12, 18);
    }

    void updateTask(const VecX &target_pos,
                    const VecX &target_vel,
                    const VecX &target_acc,
                    const VecX &curr_pos,
                    const VecX &curr_vel) override {
        static double Kp = 100, Kd = 10;
        _task_e = target_pos - curr_pos;
        _task_dx = target_vel;
        _task_ddx = target_acc + Kp * _task_e + Kd * (target_vel - curr_vel);
    }

private:
};


#endif //BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP
