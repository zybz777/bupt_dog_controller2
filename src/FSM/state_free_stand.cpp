//
// Created by zyb on 24-4-26.
//

#include "FSM/state_free_stand.hpp"

State_FreeStand::State_FreeStand(const shared_ptr<CtrlComponents> &ctrl_comp)
        : FSMState(ctrl_comp, FSMStateName::FREESTAND, "freestand") {}

void State_FreeStand::enter() {
    _cmd_q = _ctrl_comp->getLowState()->getQ();
    _cmd_dq.setZero();
    _cmd_tau.setZero();
    _delta_q.setZero();
}

void State_FreeStand::step() {
    ZeroGainStand();
//    SwingGainStand();
}

void State_FreeStand::exit() {

}

FSMStateName State_FreeStand::checkChange() {
    switch (_ctrl_comp->getGait()->getGaitType()) {
        case GaitType::PASSIVE:
            return FSMStateName::PASSIVE;
        case GaitType::FIXEDDOWN:
            return FSMStateName::FIXEDDOWN;
        case GaitType::FIXEDSTAND:
            return FSMStateName::FIXEDSTAND;
        case GaitType::TROTTING:
            return FSMStateName::TROTTING;
        default:
            return FSMStateName::FREESTAND;
    }
}

void State_FreeStand::SwingGainStand() {
    auto cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose()
                   * _ctrl_comp->getMpcController()->getMpcOutput()
                   + _ctrl_comp->getRobot()->getNoLinearTorque();
    _cmd_tau = cmd_tau.segment<12>(6);

    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
#ifdef USE_SIM
    _ctrl_comp->getLowCmd()->setSimSwingGain(0);
    _ctrl_comp->getLowCmd()->setSimSwingGain(1);
    _ctrl_comp->getLowCmd()->setSimSwingGain(2);
    _ctrl_comp->getLowCmd()->setSimSwingGain(3);
#else
    _ctrl_comp->getLowCmd()->setRealFreeStanceGain();
#endif
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FreeStand::ZeroGainStand() {
    auto cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose()
                   * _ctrl_comp->getMpcController()->getMpcOutput()
                   + _ctrl_comp->getRobot()->getNoLinearTorque();

    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = cmd_tau.segment<12>(6);
    _ctrl_comp->getLowCmd()->setZeroGain();
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}
