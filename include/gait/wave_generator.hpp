//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_WAVE_GENERATOR_HPP
#define BUPT_DOG_CONTROLLER2_WAVE_GENERATOR_HPP

#include <iostream>
#include <utility>
#include "utils/math_types.hpp"
#include "enum_gait.hpp"
#include "utils/timer.hpp"
#include "control/mpc/mpc_param.hpp"

class WaveGenerator {
public:
    WaveGenerator(double period, double stance_phase_ratio, Vec4 bias) {
        _period = period;
        _st_ratio = stance_phase_ratio;
        _bias = std::move(bias);

        if ((_st_ratio <= 0) || (_st_ratio >= 1)) {
            std::cout << "[ERROR] The stancePhaseRatio of WaveGenerator should between (0, 1)" << std::endl;
            exit(-1);
        }

        for (int i = 0; i < _bias.rows(); ++i) {
            if ((_bias[i] < 0) || (_bias[i] > 1)) {
                std::cout << "[ERROR] The bias of WaveGenerator should between [0, 1]" << std::endl;
                exit(-1);
            }
        }

        _start_T = getSystemTime();
    }

    void step(WaveStatus status) {
        calcWave(_phase, _contact, status);
        // std::cout << "Phase: " << _phase.transpose() << std::endl;
        // std::cout << "Contact: " << _contact.transpose() << std::endl;
    }

    // get
    double getTstance() {
        return _period * _st_ratio;
    }

    double getTswing() {
        return _period * (1 - _st_ratio);
    }

    double getT() {
        return _period;
    }

    const Vec4 &getPhase() {
        return _phase;
    }

    double getPhase(int leg_id) {
        return _phase[leg_id];
    }

    const VecInt4 &getContact() {
        return _contact;
    }

    int getContact(int leg_id) {
        return _contact[leg_id];
    }

private:
    double _period;                  // 周期
    double _st_ratio;                // 站立 比例
    Vec4 _bias;                      // 站立偏移时间(相位)
    Vec4 _normal_T;                  // [0,1)
    Vec4 _phase;        // 当前足的相位，支撑摆动都是0-1
    VecInt4 _contact; // 1: contact 0: no contact
    VecInt4 _switch_status;          // 1: switching, 0: do not switch

    double _pass_T;     // unit: second
    long long _start_T; // unit: us

    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status) {
        switch (status) {
            case WaveStatus::WAVE_ALL:                                 // 生成的phase 每条腿无论支撑还是摆动都是[0-1]
                _pass_T = (double) (getSystemTime() - _start_T) * 1e-6; // unit: s
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
                // mpc
                for (int epoch = 0; epoch < HORIZON; ++epoch) {
                }
                break;
            case WaveStatus::SWING_ALL:
                contact.setZero();
                phase << 0.5, 0.5, 0.5, 0.5;
                break;
            case WaveStatus::STANCE_ALL:
                contact.setOnes();
                phase << 0.5, 0.5, 0.5, 0.5;
                break;
        }
    }
};

#endif //BUPT_DOG_CONTROLLER2_WAVE_GENERATOR_HPP
