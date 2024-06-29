/*
 * @Author       : Zybz
 * @Date         : 2024-04-24 14:35:03
 * @LastEditors  : Zybz
 * @LastEditTime : 2024-05-07 15:43:46
 * @FilePath     : /bupt_dog_controller2/src/FSM/state_trot.cpp
 * @Description  :
 *
 * Copyright (c) 2024 by BUPT RobotTeam, All Rights Reserved.
 */
//
// Created by zyb on 24-4-24.
//

#include "FSM/state_trot.hpp"

State_Trot::State_Trot(const std::shared_ptr<CtrlComponents>& ctrl_comp) :
    FSMState(ctrl_comp, FSMStateName::TROTTING,
             "trotting") {}

void State_Trot::enter() {
    _cmd_q = _ctrl_comp->getLowState()->getQ();
    _cmd_dq.setZero();
    _cmd_tau.setZero();
    _delta_q.setZero();
}

void State_Trot::step() {
    // swingGainMpcTrot();
    swingGainMpcWbcTrot();
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
    Vec18 cmd_tau;
#ifdef USE_MPC1
    cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose() * _ctrl_comp->getMpcController()->getMpcOutput() +
              _ctrl_comp->getRobot()->getNoLinearTorque();
#endif
#ifdef USE_MPC2
    cmd_tau = -_ctrl_comp->getRobot()->getJ_FeetPosition().transpose() * _ctrl_comp->getMpcController2()->getContactForce() +
              _ctrl_comp->getRobot()->getNoLinearTorque();
#endif
    for (int i = 0; i < 4; ++i) {
        _ctrl_comp->getLowCmd()->setSimSwingGain(i);
        if (_ctrl_comp->getGait()->getContact(i) == SWING) {
            _cmd_tau.segment<3>(3 * i) = _ctrl_comp->getRobot()->getLegNoLinearTorque().segment<3>(3 * i);
        } else {
            _ctrl_comp->getLowCmd()->setSimFreeStanceGain(i);
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

void State_Trot::RLTrot() {
}
