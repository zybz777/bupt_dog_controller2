/*
 * @Author       : Zybz
 * @Date         : 2024-05-08 14:41:32
 * @LastEditors  : Zybz
 * @LastEditTime : 2024-05-09 16:39:56
 * @FilePath     : /bupt_dog_controller2/src/FSM/state_fixed_stand.cpp
 * @Description  :
 *
 * Copyright (c) 2024 by BUPT RobotTeam, All Rights Reserved.
 */
 //
 // Created by zyb on 24-4-23.
 //

#include "FSM/state_fixed_stand.hpp"

State_FixedStand::State_FixedStand(const std::shared_ptr<CtrlComponents>& ctrl_comp) : FSMState(ctrl_comp,
    FSMStateName::FIXEDSTAND,
    "fixed stand") {
    _freq = 1000;
    _percent = 0;
    _duration = FIXEDSTAND_T;
}

void State_FixedStand::enter() {
    // 记录关节起点角度
    _ctrl_comp->getLowCmd()->setQ(_ctrl_comp->getLowState()->getQ());
    _start_pos << _ctrl_comp->getLowState()->getQ();
    // 设置关节增益
    for (int i = 0; i < 4; ++i) {
#ifdef USE_SIM
        _ctrl_comp->getLowCmd()->setSimStanceGain(i); // 设置关节增益
#else
        _ctrl_comp->getLowCmd()->setRealStanceGain(i);
#endif
        _ctrl_comp->getLowCmd()->setZeroDq(i);  // 关节速度为0
        _ctrl_comp->getLowCmd()->setZeroTau(i); // 关节力矩为0
    }
    _percent = 0;
}

void State_FixedStand::step() {
    _percent += (double)1.0 / (_duration * _freq);
    _percent = clip(_percent, Vec2(0.0, 1.0));
    Vec12 cmd_q = Vec12::Zero();
    for (int i = 0; i < 12; ++i) {
        cmd_q[i] = (1.0 - _percent) * _start_pos[i] + _percent * _target_pos[i];
    }
    _ctrl_comp->getLowCmd()->setQ(cmd_q);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FixedStand::exit() {
    _percent = 0;
}

FSMStateName State_FixedStand::checkChange() {
    if (_ctrl_comp->getUserCmd()->test_mode == 1) {
        return FSMStateName::TEST;
    }
    switch (_ctrl_comp->getGait()->getGaitType()) {
    case GaitType::PASSIVE:
        return FSMStateName::PASSIVE;
    case GaitType::FIXEDDOWN:
        return FSMStateName::FIXEDDOWN;
    case GaitType::TROTTING:
        return FSMStateName::TROTTING;
    case GaitType::FREESTAND:
        return FSMStateName::FREESTAND;
    default:
        return FSMStateName::FIXEDSTAND;
    }
}