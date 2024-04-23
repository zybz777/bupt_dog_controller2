//
// Created by zyb on 24-4-23.
//

#include "FSM/state_passive.hpp"

#include <utility>

State_Passive::State_Passive(std::shared_ptr<CtrlComponents> ctrl_comp)
        : FSMState(std::move(ctrl_comp),
                   FSMStateName::PASSIVE,
                   "passive") {}

void State_Passive::enter() {
    _ctrl_comp->getLowCmd()->setPassiveGain();
    _ctrl_comp->getLowCmd()->setZeroDq();
    _ctrl_comp->getLowCmd()->setZeroTau();
}

void State_Passive::step() {
    _ctrl_comp->getLowCmd()->setQ(_ctrl_comp->getRobot()->getQ());
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_Passive::exit() {
}

FSMStateName State_Passive::checkChange() {
    switch (_ctrl_comp->getGait()->getGaitType()) {
        case GaitType::FIXEDSTAND:
            return FSMStateName::FIXEDSTAND;
        case GaitType::FIXEDDOWN:
            return FSMStateName::FIXEDDOWN;
        default:
            return FSMStateName::PASSIVE;
    }
}