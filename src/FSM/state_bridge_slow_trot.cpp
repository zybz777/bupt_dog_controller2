//
// Created by zyb on 2024/7/9.
//

#include "FSM/state_bridge_slow_trot.hpp"

State_BridgeSlowTrot::State_BridgeSlowTrot(const shared_ptr<CtrlComponents> &ctrl_comp) :
        FSMState(ctrl_comp, FSMStateName::BRIDGESLOWTROTING,
                 "bridge slow trotting") {}

void State_BridgeSlowTrot::enter() {
    _cmd_q = _ctrl_comp->getLowState()->getQ();
    _cmd_dq.setZero();
    _cmd_tau.setZero();
    _delta_q.setZero();
    _ctrl_comp->getVmcController()->setHeight(0.05);
    _ctrl_comp->getVmcController()->setHeursticRatio(1.0);
    _ctrl_comp->getVmcController()->updateStdFootPos(0, 0.04, 0);
}

void State_BridgeSlowTrot::step() {
    swingGainMpcWbcTrot();
}

void State_BridgeSlowTrot::exit() {

}

FSMStateName State_BridgeSlowTrot::checkChange() {
    switch (_ctrl_comp->getGait()->getGaitType()) {
        case GaitType::PASSIVE:
            return FSMStateName::PASSIVE;
        case GaitType::FIXEDDOWN:
            return FSMStateName::FIXEDDOWN;
        case GaitType::FIXEDSTAND:
            return FSMStateName::FIXEDSTAND;
        case GaitType::FREESTAND:
            return FSMStateName::FREESTAND;
        case GaitType::TROTTING:
            return FSMStateName::TROTTING;
        case GaitType::BRIDGETROTING:
            return FSMStateName::BRIDGETROTING;
        default:
            return FSMStateName::BRIDGESLOWTROTING;
    }
}

void State_BridgeSlowTrot::swingGainMpcWbcTrot() {
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = _ctrl_comp->getWbcController()->getLegCmdTau();
    for (int i = 0; i < LEG_NUM; ++i) {
#ifdef USE_SIM
        if (_ctrl_comp->getGait()->getContact(i) == SWING) {
            _ctrl_comp->getLowCmd()->setSimSwingGain(i);
        } else {
            _ctrl_comp->getLowCmd()->setSimFreeStanceGain(i);
        }
#else
        if (_ctrl_comp->getGait()->getContact(i) == SWING) {
            _ctrl_comp->getLowCmd()->setRealSwingGain(i);
        } else {
            _ctrl_comp->getLowCmd()->setRealFreeStanceGain(i);
            // _cmd_tau[2 + 3 * i] = _cmd_tau[2 + 3 * i] / pow(sin(_ctrl_comp->getLowState()->getQ()[2 + 3 * i]), 2);
        }
        _cmd_tau[2 + 3 * i] = _cmd_tau[2 + 3 * i] / pow(sin(_ctrl_comp->getLowState()->getQ()[2 + 3 * i]), 2);
#endif
    }
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}
