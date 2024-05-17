//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_TASK_BODY_ORIENTATION_HPP
#define BUPT_DOG_CONTROLLER2_WBC_TASK_BODY_ORIENTATION_HPP


#include "wbc_task.hpp"

class WbcTask_BodyOrientation : public WbcTask {
public:
    WbcTask_BodyOrientation() {
        _task_e = Vec3::Zero();
        _task_dx = Vec3::Zero();
        _task_ddx = Vec3::Zero();
        _J = MatX::Zero(3, 18);
        _dJ = MatX::Zero(3, 18);
    }

    void updateTask(const VecX &target_pos,
                    const VecX &target_vel,
                    const VecX &target_acc,
                    const VecX &curr_pos,
                    const VecX &curr_vel) override {
        static Vec3 Kp(50, 50, 50), Kd(3, 3, 3);
        _task_e << target_pos - curr_pos;
//        _task_e[2] *= 0.005;
        _task_dx << target_vel;
        _task_ddx << target_acc + Kp.asDiagonal() * _task_e + Kd.asDiagonal() * (target_vel - curr_vel);
    }

    void updateTask(const VecX &task_err,
                    const VecX &target_vel,
                    const VecX &target_acc,
                    const VecX &curr_vel) {
        static Vec3 Kp(30, 30, 30), Kd(5, 5, 5);
        _task_e << task_err;
//        _task_e[2] *= 0.005;
        _task_dx << target_vel;
        _task_ddx << target_acc + Kp.asDiagonal() * _task_e + Kd.asDiagonal() * (target_vel - curr_vel);
    }

private:
};


#endif //BUPT_DOG_CONTROLLER2_WBC_TASK_BODY_ORIENTATION_HPP
