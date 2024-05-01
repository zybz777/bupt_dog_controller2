//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_STATE_FREE_STAND_HPP
#define BUPT_DOG_CONTROLLER2_STATE_FREE_STAND_HPP


#include "fsm_state.hpp"

class State_FreeStand: public FSMState{
public:
    explicit State_FreeStand(const std::shared_ptr<CtrlComponents> &ctrl_comp);

    void enter() override;

    void step() override;

    void exit() override;

    FSMStateName checkChange() override;

private:
    Vec3 _delta_q;
    Vec12 _cmd_q;
    Vec12 _cmd_dq;
    Vec12 _cmd_tau;

    void swingGainStand();
    void zeroGainStand();
    void zeroGainMpcWbcStand();
    void balanceSoftTest();
};


#endif //BUPT_DOG_CONTROLLER2_STATE_FREE_STAND_HPP
