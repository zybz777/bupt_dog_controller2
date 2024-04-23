//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_FSM_STATE_HPP
#define BUPT_DOG_CONTROLLER2_FSM_STATE_HPP


#include <utility>

#include "common/ctrl_components.hpp"
#include "fsm_enum.hpp"

class FSMState {
public:
    FSMState(std::shared_ptr<CtrlComponents> ctrl_comp,
             FSMStateName state_name,
             std::string state_name_string) {
        _ctrl_comp = std::move(ctrl_comp);
        _state_name = state_name;
        _state_name_string = std::move(state_name_string);
    }

    virtual void enter() = 0;

    virtual void step() = 0;

    virtual void exit() = 0;

    virtual FSMStateName checkChange() { return FSMStateName::INVALID; }

    FSMStateName _state_name;
    std::string _state_name_string;

protected:
    std::shared_ptr<CtrlComponents> _ctrl_comp;
    FSMStateName _next_state_name;
};


#endif //BUPT_DOG_CONTROLLER2_FSM_STATE_HPP
