//
// Created by zyb on 24-4-27.
//

#ifndef BUPT_DOG_CONTROLLER2_WBC_TASK_HPP
#define BUPT_DOG_CONTROLLER2_WBC_TASK_HPP


#include "utils/math_types.hpp"

class WbcTask {
public:
    WbcTask() = default;

    virtual void updateTask(const VecX &target_pos,
                            const VecX &target_vel,
                            const VecX &target_acc,
                            const VecX &curr_pos,
                            const VecX &curr_vel) = 0;

    void updateTaskJacobi(const MatX &J, const MatX &dJ) {
        _J = J;
        _dJ = dJ;
    }

    const VecX &getTask_e() { return _task_e; };

    const VecX &getTask_dx() { return _task_dx; };

    const VecX &getTask_ddx() { return _task_ddx; };

    const MatX &getTask_J() { return _J; };

    const MatX &getTask_dJ() { return _dJ; };

protected:
    /*任务空间*/
    VecX _task_e;
    VecX _task_dx;
    VecX _task_ddx;
    MatX _J;
    MatX _dJ;
};


#endif //BUPT_DOG_CONTROLLER2_WBC_TASK_HPP
