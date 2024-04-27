//
// Created by zyb on 24-4-24.
//

#include "FSM/state_trot.hpp"

State_Trot::State_Trot(const shared_ptr<CtrlComponents> &ctrl_comp)
        : FSMState(ctrl_comp, FSMStateName::TROTTING,
                   "trotting") {}

void State_Trot::enter() {
    _cmd_q = _ctrl_comp->getLowState()->getQ();
    _cmd_dq.setZero();
    _cmd_tau.setZero();
    _delta_q.setZero();
}

void State_Trot::step() {
    swingGainMpcTrot();
//    swingGainMpcWbcTrot();
}

void State_Trot::exit() {

}

FSMStateName State_Trot::checkChange() {
    switch (_ctrl_comp->getGait()->getGaitType()) {
        case GaitType::PASSIVE:
            return FSMStateName::PASSIVE;
        case GaitType::FIXEDDOWN:
            return FSMStateName::FIXEDDOWN;
        case GaitType::FIXEDSTAND:
            return FSMStateName::FIXEDSTAND;
        case GaitType::FREESTAND:
            return FSMStateName::FREESTAND;
        default:
            return FSMStateName::TROTTING;
    }
}

void State_Trot::swingGainMpcTrot() {
    auto cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose()
                   * _ctrl_comp->getMpcController()->getMpcOutput()
                   + _ctrl_comp->getRobot()->getNoLinearTorque();
    for (int i = 0; i < 4; ++i) {
        _ctrl_comp->getLowCmd()->setSimSwingGain(i);
        if (_ctrl_comp->getGait()->getContact(i) == SWING) {
            _cmd_tau.segment<3>(3 * i) = _ctrl_comp->getRobot()->getLegNoLinearTorque().segment<3>(3 * i);

        } else {
            _cmd_tau.segment<3>(3 * i) = cmd_tau.segment<3>(6 + 3 * i);
        }
    }
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}


void State_Trot::swingGainMpcWbcTrot() {
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = _ctrl_comp->getWbcController()->getLegCmdTau();
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}
