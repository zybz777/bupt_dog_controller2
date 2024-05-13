/*
 * @Author       : Zybz
 * @Date         : 2024-05-05 19:51:11
 * @LastEditors  : Zybz
 * @LastEditTime : 2024-05-13 15:16:58
 * @FilePath     : /bupt_dog_controller2/include/control/wbc/task/wbc_task_foot_pos.hpp
 * @Description  : 
 * 
 * Copyright (c) 2024 by BUPT RobotTeam, All Rights Reserved. 
 */
//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP
#define BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP


#include "wbc_task.hpp"

class WbcTask_FootPos : public WbcTask {
public:
    WbcTask_FootPos() {
        _task_e = Vec12::Zero();
        _task_dx = Vec12::Zero();
        _task_ddx = Vec12::Zero();
        _J = MatX::Zero(12, 18);
        _dJ = MatX::Zero(12, 18);
    }

    void updateTask(const VecX &target_pos,
                    const VecX &target_vel,
                    const VecX &target_acc,
                    const VecX &curr_pos,
                    const VecX &curr_vel) override {
        static double Kp = 1, Kd = 1;
        _task_e << target_pos - curr_pos;
        _task_dx << target_vel;
        _task_ddx << target_acc + Kp * _task_e + Kd * (target_vel - curr_vel);
    }

private:
};


#endif //BUPT_DOG_CONTROLLER2_WBC_TASK_FOOT_POS_HPP
