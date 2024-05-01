//
// Created by zyb on 24-4-23.
//

#include "FSM/fsm.hpp"
#include "utils/timer.hpp"

FSM::FSM(const std::shared_ptr<CtrlComponents> &ctrl_comp, int ms) {
    _ms = ms;
    _ctrl_comp = ctrl_comp;
    _state_list.passive = std::make_shared<State_Passive>(_ctrl_comp);
    _state_list.fixed_stand = std::make_shared<State_FixedStand>(_ctrl_comp);
    _state_list.fixed_down = std::make_shared<State_FixedDown>(_ctrl_comp);
    _state_list.trotting = std::make_shared<State_Trot>(_ctrl_comp);
    _state_list.free_stand = std::make_shared<State_FreeStand>(_ctrl_comp);
    // init
    _current_state = _state_list.passive;
    _current_state->enter();
    _next_state = _current_state;
    _mode = FSMMode::NORMAL;

    _fsm_thread = std::thread([this] { run(_ms); });
    std::cout << "[FSM] Init Success!" << std::endl;
}

[[noreturn]] void FSM::run(int ms) {
    std::chrono::microseconds period(ms * 1000);
    std::chrono::microseconds zero_us(0);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        step();
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}

void FSM::step() {
    _ctrl_comp->step();
    switch (_mode) {
        case FSMMode::NORMAL:
            checkSafety();
            _current_state->step();
            _next_state_name = _current_state->checkChange();
            if (_next_state_name != _current_state->_state_name) { // 状态切换
                _mode = FSMMode::CHANGE;
                _next_state = getNextState(_next_state_name);
                std::cout << "Switched from [" << _current_state->_state_name_string
                          << "] to [" << _next_state->_state_name_string << "]" << std::endl;
            }
            break;
        case FSMMode::CHANGE:
            _current_state->exit();
            _current_state = _next_state;
            _current_state->enter();
            _mode = FSMMode::NORMAL;
            break;
        case FSMMode::ABNORMAL: // 异常状态处理
            // 异常状态令机器人进入阻尼状态
            // 目前不提供退出窗口，必须将程序重新启动
            if (_current_state->_state_name != FSMStateName::PASSIVE) {
                std::cout << "[ABNORMAL] Switched to [passive]" << std::endl;
                _current_state->exit();
                _current_state = _state_list.passive;
                _current_state->enter();
            }
            _current_state->step();
            break;
    }
}

std::shared_ptr<FSMState> FSM::getNextState(FSMStateName state_name) const {
    switch (state_name) {
        case FSMStateName::PASSIVE:
            return _state_list.passive;
        case FSMStateName::FIXEDSTAND:
            return _state_list.fixed_stand;
        case FSMStateName::FIXEDDOWN:
            return _state_list.fixed_down;
        case FSMStateName::TROTTING:
            return _state_list.trotting;
        case FSMStateName::FREESTAND:
            return _state_list.free_stand;
//        case FSMStateName::TEST:
//            return _state_list.test;
        default:
            return _state_list.passive;
    }
}

bool FSM::checkSafety() {
    // 仅在力矩控制状态下检查
    if (_current_state->_state_name != FSMStateName::FREESTAND &&
        _current_state->_state_name != FSMStateName::TROTTING) {
        return true;
    }
    // 检查质心角度，判断机器人是否失去平衡姿态
    double roll_abs = fabs(_ctrl_comp->getLowState()->getRpy()[0]);
    double pitch_abs = fabs(_ctrl_comp->getLowState()->getRpy()[1]);
    if (roll_abs > BASE_MAX_ROLL)
    {
        _mode = FSMMode::ABNORMAL;
        std::cout << "[ABNORMAL] Roll out of safe range" << std::endl;
        return false;
    } else if (pitch_abs > BASE_MAX_PITCH)
    {
        _mode = FSMMode::ABNORMAL;
        std::cout << "[ABNORMAL] Pitch out of safe range" << std::endl;
        return false;
    }
    // 检查电机角度是否超出限位
    static Vec3 q_max(0.8 * MOTOR0_MAX_POS, 0.8 * MOTOR1_MAX_POS, 0.8 * MOTOR2_MAX_POS);
    static Vec3 q_min(0.8 * MOTOR0_MIN_POS, 0.8 * MOTOR1_MIN_POS, 0.8 * MOTOR2_MIN_POS); // 关节限位最小值
    Vec12 q = _ctrl_comp->getLowState()->getQ();
    for (int i = 0; i < LEG_NUM; ++i) {
        // 位置检测
        if (fabs(q[0 + 3 * i]) > q_max[0]) {
            _mode = FSMMode::ABNORMAL;
            std::cout << "[ABNORMAL] q" << 0 + 3 * i << " " << q[2 + 3 * i] << " out of safe range" << std::endl;
            return false;
        } else if (q[1 + 3 * i] < q_min[1] || q[1 + 3 * i] > q_max[1]) {
            _mode = FSMMode::ABNORMAL;
            std::cout << "[ABNORMAL] q" << 1 + 3 * i << " " << q[2 + 3 * i] << " out of safe range" << std::endl;
            return false;
        } else if (q[2 + 3 * i] < q_min[2] || q[2 + 3 * i] > q_max[2]) {
            _mode = FSMMode::ABNORMAL;
            std::cout << "[ABNORMAL] q" << 2 + 3 * i << " " << q[2 + 3 * i] << " out of safe range" << std::endl;
            return false;
        }
    }
    // 速度检测
    Vec12 dq = _ctrl_comp->getLowState()->getDq();
    for (int i = 0; i < DOF_NUM; ++i) {
        if (fabs(dq[i]) > 0.8 * MOTOR_MAX_VEL) {
            _mode = FSMMode::ABNORMAL;
            std::cout << "[ABNORMAL] dq" << i << " " << dq[i] << " out of safe range" << std::endl;
            return false;
        }
    }
    // 力矩检测
    Vec12 tau = _ctrl_comp->getLowState()->getTau();
    for (int i = 0; i < DOF_NUM; ++i) {
        if (fabs(tau[i]) > 0.8 * MOTOR_MAX_TAU) {
            _mode = FSMMode::ABNORMAL;
            std::cout << "[ABNORMAL] tau" << i << " " << tau[i] << " out of safe range" << std::endl;
            return false;
        }
    }
    return true;
}

void FSM::begin() {
    _fsm_thread.join();
}
