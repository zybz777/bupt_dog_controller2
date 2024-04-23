//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_STATE_PASSIVE_HPP
#define BUPT_DOG_CONTROLLER2_STATE_PASSIVE_HPP


#include "fsm_state.hpp"

class State_Passive : public FSMState {
public:
    State_Passive(std::shared_ptr<CtrlComponents> ctrl_comp);

    void enter() override;

    void step() override;

    void exit() override;

    FSMStateName checkChange() override;

};


#endif //BUPT_DOG_CONTROLLER2_STATE_PASSIVE_HPP
