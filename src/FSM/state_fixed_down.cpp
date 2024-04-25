//
// Created by zyb on 24-4-23.
//

#include "FSM/state_fixed_down.hpp"

#include <utility>
#include "gait/enum_gait.hpp"

State_FixedDown::State_FixedDown(const std::shared_ptr<CtrlComponents> &ctrl_comp) : FSMState(ctrl_comp,
                                                                                              FSMStateName::FIXEDDOWN,
                                                                                              "fixed down") {
    _freq = 1000;
    _percent = 0;
    _duration = FIXEDDOWN_T;
}

void State_FixedDown::enter() {
// 记录关节起点角度
    _ctrl_comp->getLowCmd()->setQ(_ctrl_comp->getLowState()->getQ());
    _start_pos << _ctrl_comp->getLowState()->getQ();
    // 设置关节增益
    for (int i = 0; i < 4; ++i) {
        _ctrl_comp->getLowCmd()->setSimStanceGain(i); // 设置关节增益
        _ctrl_comp->getLowCmd()->setZeroDq(i);        // 关节速度为0
        _ctrl_comp->getLowCmd()->setZeroTau(i);       // 关节力矩为0
    }
    _percent = 0;
}

void State_FixedDown::step() {
    _percent += (double) 1.0 / (_duration * _freq);
    _percent = clip(_percent, Vec2(0.0, 1.0));
    Vec12 cmd_q = Vec12::Zero();
    for (int i = 0; i < 12; ++i) {
        cmd_q[i] = (1.0 - _percent) * _start_pos[i] + _percent * _target_pos[i];
    }
    _ctrl_comp->getLowCmd()->setQ(cmd_q);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FixedDown::exit() {
    _percent = 0;
}

FSMStateName State_FixedDown::checkChange() {
    switch (_ctrl_comp->getGait()->getGaitType()) {
        case GaitType::PASSIVE:
            return FSMStateName::PASSIVE;
        case GaitType::FIXEDSTAND:
            return FSMStateName::FIXEDSTAND;
        default:
            return FSMStateName::FIXEDDOWN;
    }
}
