//
// Created by zyb on 24-4-24.
//

#ifndef BUPT_DOG_CONTROLLER2_STATE_TROT_HPP
#define BUPT_DOG_CONTROLLER2_STATE_TROT_HPP


#include "fsm_state.hpp"

class State_Trot : public FSMState {
public:
    explicit State_Trot(const std::shared_ptr<CtrlComponents> &ctrl_comp);

    void enter() override;

    void step() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    Vec3 _delta_q;
    Vec12 _cmd_q;
    Vec12 _cmd_dq;
    Vec12 _cmd_tau;
};


#endif //BUPT_DOG_CONTROLLER2_STATE_TROT_HPP
