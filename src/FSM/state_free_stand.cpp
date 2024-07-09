//
// Created by zyb on 24-4-26.
//

#include "FSM/state_free_stand.hpp"

State_FreeStand::State_FreeStand(const std::shared_ptr<CtrlComponents> &ctrl_comp) :
        FSMState(ctrl_comp, FSMStateName::FREESTAND, "freestand") {}

void State_FreeStand::enter() {
    _cmd_q = _ctrl_comp->getLowState()->getQ();
    _cmd_dq.setZero();
    _cmd_tau.setZero();
    _delta_q.setZero();
}

void State_FreeStand::step() {
    //    zeroGainMpcWbcStand();
    // zeroGainStand();
    //    swingGainStand();
    //    balanceSoftTest();
    swingGainMpcWbcStand();
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
        case GaitType::BRIDGETROTING:
            return FSMStateName::BRIDGETROTING;
        default:
            return FSMStateName::FREESTAND;
    }
}

void State_FreeStand::swingGainStand() {
    auto cmd_tau =
            -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose() * _ctrl_comp->getMpcController()->getMpcOutput() +
            _ctrl_comp->getRobot()->getNoLinearTorque();
    //    _cmd_tau = cmd_tau.segment<12>(6);
    _cmd_tau = _ctrl_comp->getRobot()->getLegNoLinearTorque();
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

void State_FreeStand::zeroGainStand() {
    Vec18 cmd_tau;
#ifdef USE_MPC1
    cmd_tau =
            -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose() * _ctrl_comp->getMpcController()->getMpcOutput() +
            _ctrl_comp->getRobot()->getNoLinearTorque();
#endif
#ifdef USE_MPC2
    cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose() * _ctrl_comp->getMpcController2()->getContactForce() + _ctrl_comp->getRobot()->getNoLinearTorque();
#endif
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = cmd_tau.segment<12>(6);
    _ctrl_comp->getLowCmd()->setZeroGain();
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FreeStand::zeroGainMpcWbcStand() {
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = _ctrl_comp->getWbcController()->getLegCmdTau();
    _ctrl_comp->getLowCmd()->setZeroGain();
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FreeStand::balanceSoftTest() {
    // 使用较小的PD增益，测试机器人原地旋转和高度的位置速度跟踪情况
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = _ctrl_comp->getRobot()->getLegNoLinearTorque();
#ifdef USE_SIM
    //    _ctrl_comp->getLowCmd()->setSimSwingGain(0);
    //    _ctrl_comp->getLowCmd()->setSimSwingGain(1);
    //    _ctrl_comp->getLowCmd()->setSimSwingGain(2);
    //    _ctrl_comp->getLowCmd()->setSimSwingGain(3);
    _ctrl_comp->getLowCmd()->setSimStanceGain(0);
    _ctrl_comp->getLowCmd()->setSimStanceGain(1);
    _ctrl_comp->getLowCmd()->setSimStanceGain(2);
    _ctrl_comp->getLowCmd()->setSimStanceGain(3);

#else
    _ctrl_comp->getLowCmd()->setRealFreeStanceGain();
#endif
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}

void State_FreeStand::swingGainMpcWbcStand() {
    _cmd_q = _ctrl_comp->getWbcController()->getLegCmdQ();
    _cmd_dq = _ctrl_comp->getWbcController()->getLegCmdDq();
    _cmd_tau = _ctrl_comp->getWbcController()->getLegCmdTau();
#ifdef USE_SIM
    _ctrl_comp->getLowCmd()->setSimFreeStanceGain(0);
    _ctrl_comp->getLowCmd()->setSimFreeStanceGain(1);
    _ctrl_comp->getLowCmd()->setSimFreeStanceGain(2);
    _ctrl_comp->getLowCmd()->setSimFreeStanceGain(3);
//    _ctrl_comp->getLowCmd()->setZeroGain();
#else
    _ctrl_comp->getLowCmd()->setRealFreeStanceGain();
    for (int i = 0; i < LEG_NUM; ++i) {
        _cmd_tau[2 + 3 * i] = _cmd_tau[2 + 3 * i] / pow(sin(_ctrl_comp->getLowState()->getQ()[2 + 3 * i]), 2);
    }
#endif
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
}
