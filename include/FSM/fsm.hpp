//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_FSM_HPP
#define BUPT_DOG_CONTROLLER2_FSM_HPP

#include "state_fixed_stand.hpp"
#include "state_fixed_down.hpp"
#include "state_passive.hpp"
#include "state_trot.hpp"
#include "state_free_stand.hpp"

struct FSMStateList {
    // FSMState *invalid;
    std::shared_ptr<State_Passive> passive;
    std::shared_ptr<State_FixedStand> fixed_stand;
    std::shared_ptr<State_FixedDown> fixed_down;
    std::shared_ptr<State_Trot> trotting;
    std::shared_ptr<State_FreeStand> free_stand;
//    State_FreeStand *free_stand;
//    State_Test *test;
};

class FSM {
public:
    FSM(const std::shared_ptr<CtrlComponents> &ctrl_comp, int ms);

    void begin();

    ~FSM() {
        _current_state = _state_list.passive;
        _current_state->enter();
        _current_state->step();
        std::cout << "[FSM] reset to Passive over!" << std::endl;
    }

private:
    // thread
    std::thread _fsm_thread;
    int _ms;
    // FSM
    std::shared_ptr<CtrlComponents> _ctrl_comp;
    std::shared_ptr<FSMState> _current_state;
    std::shared_ptr<FSMState> _next_state;
    FSMStateName _next_state_name = FSMStateName::PASSIVE;
    FSMStateList _state_list;
    FSMMode _mode;

    [[noreturn]] void run(int ms);

    void step();

    [[nodiscard]] std::shared_ptr<FSMState> getNextState(FSMStateName state_name) const;

    bool checkSafety();
};


#endif //BUPT_DOG_CONTROLLER2_FSM_HPP
