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
    Mat3 J_inv;
    Vec3 target_foot_vel, target_foot_pos, curr_foot_pos;
    Vec12 q = _ctrl_comp->getLowState()->getQ();
    Vec12 dq = _ctrl_comp->getLowState()->getDq();
    for (int i = 0; i < 4; ++i) {
        J_inv = _ctrl_comp->getRobot()->getFootJaco_inBody(i).inverse();
        target_foot_vel = _ctrl_comp->getVmcController()->getCmdFootVel(i);
        target_foot_pos = _ctrl_comp->getVmcController()->getCmdFootPos(i);
        curr_foot_pos = _ctrl_comp->getRobot()->getFootPosition_inBody(i);
        _cmd_q.segment<3>(3 * i) = q.segment<3>(3 * i) + J_inv * (target_foot_pos - curr_foot_pos);
        _cmd_dq.segment<3>(3 * i) = J_inv * target_foot_vel;
        _cmd_tau.segment<3>(3 * i) = _ctrl_comp->getRobot()->getLegNoLinearTorque().segment<3>(3 * i);
#ifdef USE_SIM
        _ctrl_comp->getLowCmd()->setSimSwingGain(i);
#else
        _ctrl_comp->getLowCmd()->setRealSwingGain(i);
#endif
    }
    _ctrl_comp->getLowCmd()->setQ(_cmd_q);
    _ctrl_comp->getLowCmd()->setDq(_cmd_dq);
    _ctrl_comp->getLowCmd()->setTau(_cmd_tau);
    _ctrl_comp->getLowCmd()->publishLegCmd();
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
