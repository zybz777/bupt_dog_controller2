//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_STATE_FIXED_STAND_HPP
#define BUPT_DOG_CONTROLLER2_STATE_FIXED_STAND_HPP

#include "fsm_state.hpp"

class State_FixedStand : public FSMState {
  public:
    explicit State_FixedStand(const std::shared_ptr<CtrlComponents>& ctrl_comp);

    void enter() override;

    void step() override;

    void exit() override;

    FSMStateName checkChange() override;

  private:
    const double _target_pos[12] = {JOINT0_STAND_POS, JOINT1_STAND_POS, JOINT2_STAND_POS,
                                    JOINT0_STAND_POS, JOINT1_STAND_POS, JOINT2_STAND_POS,
                                    JOINT0_STAND_POS, JOINT1_STAND_POS, JOINT2_STAND_POS,
                                    JOINT0_STAND_POS, JOINT1_STAND_POS, JOINT2_STAND_POS};
    Vec12 _start_pos;
    double _percent = 0;  //%
    double _duration = 2; // 单位 s ，完成站立的周期
    double _freq;         // 状态机执行频率
};

#endif //BUPT_DOG_CONTROLLER2_STATE_FIXED_STAND_HPP
