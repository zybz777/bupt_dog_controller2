//
// Created by zyb on 24-4-23.
//

#include "gait/gait.hpp"
#include "control/mpc/mpc_param.hpp"
#include "utils/timer.hpp"

#include <utility>

Gait::Gait(std::shared_ptr<doglcm::UserCmd_t> user_cmd) {
    _user_cmd = std::move(user_cmd);
    init();
    std::cout << "[Gait] Init Success!" << std::endl;
}

void Gait::init() {
    setGaitType(GaitType::PASSIVE, 0.6);
    _mpc_contact_list = std::vector<Vec4_i8>(HORIZON);
    for (auto &mpc_contact: _mpc_contact_list) {
        mpc_contact << CONTACT, CONTACT, CONTACT, CONTACT;
    }
    _mpc_dt = double(1.0 / MPC_FREQUENCY);
}

void Gait::step() {
    checkGaitChange();
    _pass_T = (double) (getSystemTime() - _start_T) * 1e-6;
    calcWave(_phase, _contact);
    calcMpcWave(_mpc_contact_list);
}

void Gait::checkGaitChange() {
    // 步态重叠，退出
    if (_user_cmd->gait_type == (int8_t) _gait_type) {
        return;
    }
    // std::cout << "user cmd " << (int)_user_cmd->gait_type << std::endl;
    // std::cout << "now gait " << (int)_gait_type << std::endl;
    // 未到一个步态周期结束，不切换，退出
    if (_gait_type == GaitType::FIXEDDOWN ||
        _gait_type == GaitType::FIXEDSTAND ||
        _gait_type == GaitType::PASSIVE ||
        _gait_type == GaitType::FREESTAND) {
        if (_pass_T < _period)
            return;
    } else if (fmod(_pass_T, _period) / _period < 0.95) {
        return;
    }
    // 切换步态
    _next_gait_type = (GaitType) _user_cmd->gait_type;
    switch (_next_gait_type) {
        case GaitType::FIXEDSTAND:
            setGaitType(GaitType::FIXEDSTAND, FIXEDSTAND_T);
            break;
        case GaitType::FIXEDDOWN:
            setGaitType(GaitType::FIXEDDOWN, FIXEDDOWN_T);
            break;
        case GaitType::TROTTING:
            setGaitType(GaitType::TROTTING, 0.6);
            break;
        case GaitType::FREESTAND:
            setGaitType(GaitType::FREESTAND, 0.6);
            break;
        case GaitType::PASSIVE:
            setGaitType(GaitType::PASSIVE, 0.6);
            break;
        default:
            setGaitType(GaitType::INVALID, 0.6);
            break;
    }
}

void Gait::setGaitType(GaitType gait_type, double T) {
    _gait_type = gait_type;
    _period = T;
    _start_T = getSystemTime();
    switch (_gait_type) {
        case GaitType::FIXEDSTAND:
        case GaitType::FIXEDDOWN:
            _wave = WaveStatus::STANCE_ALL;
            _st_ratio = 1.0;
            _bias << 0, 0, 0, 0;
            break;
        case GaitType::BRIDGETROTING:
        case GaitType::TROTTING:
            _wave = WaveStatus::WAVE_ALL;
            _st_ratio = 0.5;
            _bias << 0, 0.5, 0.5, 0;
            break;
        case GaitType::FREESTAND:
            _wave = WaveStatus::STANCE_ALL;
            _st_ratio = 1.0;
            _bias << 0, 0, 0, 0;
            break;
        case GaitType::PASSIVE:
            _wave = WaveStatus::SWING_ALL;
            _st_ratio = 0.0;
            _bias << 0, 0, 0, 0;
            break;
        default:
            _wave = WaveStatus::STANCE_ALL;
            _st_ratio = 1.0;
            _bias << 0, 0, 0, 0;
            break;
    }
}

void Gait::calcWave(Vec4 &phase, VecInt4 &contact) {
    switch (_wave) {
        case WaveStatus::WAVE_ALL:
            for (int i = 0; i < 4; ++i) {
                _normal_T[i] = fmod(_pass_T + _period - _period * _bias[i], _period) / _period;
                if (_normal_T[i] < _st_ratio) {
                    contact[i] = CONTACT;
                    phase[i] = _normal_T[i] / _st_ratio; // 支撑相相位
                } else {
                    contact[i] = SWING;
                    phase[i] = (_normal_T[i] - _st_ratio) / (1 - _st_ratio); // 摆动项相位
                }
            }
            break;
        case WaveStatus::STANCE_ALL:
            contact << CONTACT, CONTACT, CONTACT, CONTACT;
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        case WaveStatus::SWING_ALL:
            contact << SWING, SWING, SWING, SWING;
            phase << 0.5, 0.5, 0.5, 0.5;
            break;
        default:
            break;
    }
}

void Gait::calcMpcWave(std::vector<Vec4_i8> &mpc_contact_list) {
    switch (_wave) {
        case WaveStatus::WAVE_ALL: {
            double t = _pass_T;
            double normal_T;
            for (auto &mpc_contact: _mpc_contact_list) {
                for (int i = 0; i < 4; ++i) {
                    normal_T = fmod(t + _period - _period * _bias[i], _period) / _period;
                    if (normal_T < _st_ratio) {
                        mpc_contact[i] = CONTACT;
                    } else {
                        mpc_contact[i] = SWING;
                    }
                }
                t += _mpc_dt;
            }
        }
            break;
        case WaveStatus::STANCE_ALL:
            for (auto &mpc_contact: _mpc_contact_list) {
                mpc_contact << CONTACT, CONTACT, CONTACT, CONTACT;
            }
            break;
        case WaveStatus::SWING_ALL:
            for (auto &mpc_contact: _mpc_contact_list) {
                mpc_contact << SWING, SWING, SWING, SWING;
            }
            break;
        default:
            break;
    }
}
