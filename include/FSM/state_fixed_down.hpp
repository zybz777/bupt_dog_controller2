//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_STATE_FIXED_DOWN_HPP
#define BUPT_DOG_CONTROLLER2_STATE_FIXED_DOWN_HPP

#include "fsm_state.hpp"

class State_FixedDown : public FSMState {
  public:
    explicit State_FixedDown(const std::shared_ptr<CtrlComponents>& ctrl_comp);

    void enter() override;

    void step() override;

    void exit() override;

    FSMStateName checkChange() override;

  private:
    const double _target_pos[12] = {JOINT0_DOWN_POS, JOINT1_DOWN_POS, JOINT2_DOWN_POS,
                                    -JOINT0_DOWN_POS, JOINT1_DOWN_POS, JOINT2_DOWN_POS,
                                    JOINT0_DOWN_POS, JOINT1_DOWN_POS, JOINT2_DOWN_POS,
                                    -JOINT0_DOWN_POS, JOINT1_DOWN_POS, JOINT2_DOWN_POS};
    Vec12 _start_pos;
    double _percent = 0;
    double _duration = 2; // 单位 s ，完成站立的周期
    double _freq;         // 状态机执行频率
};

#endif //BUPT_DOG_CONTROLLER2_STATE_FIXED_DOWN_HPP
