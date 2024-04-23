//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_CTRL_COMPONENTS_HPP
#define BUPT_DOG_CONTROLLER2_CTRL_COMPONENTS_HPP

#include "doglcm/UserCmd_t.hpp"
#include "low_cmd.hpp"
#include "low_state.hpp"
#include "robot.hpp"
#include "gait/gait.hpp"

class CtrlComponents {
public:
    explicit CtrlComponents(int ms) {
        // common
        _low_cmd = std::make_shared<LowCmd>();
        _low_state = std::make_shared<LowState>();
        _robot = std::make_shared<Robot>(_low_state, ms);
        // gait
        _user_cmd = _low_state->getUserCmd();
        _gait = std::make_shared<Gait>(_user_cmd);
        std::cout << "[CtrlComponents] Init Success!" << std::endl;
    }

    void begin() {
        _low_state->begin();
        _robot->begin();
        std::cout << "[CtrlComponents] begin!" << std::endl;
    }

    // common
    shared_ptr<doglcm::UserCmd_t> getUserCmd() { return _user_cmd; }

    std::shared_ptr<LowCmd> getLowCmd() { return _low_cmd; }

    std::shared_ptr<LowState> getLowState() { return _low_state; }

    std::shared_ptr<Robot> getRobot() { return _robot; }

    // gait
    std::shared_ptr<Gait> getGait() { return _gait; }

    // control
//    VmcController *getVmcController();

//    MpcController *getMpcController();

//    WbcController *getWbcController();

private:
//    void gaitThreadRun(int ms);

    // common
    std::shared_ptr<LowCmd> _low_cmd; // 底层电机控制接口
    std::shared_ptr<LowState> _low_state; // 底层电机与IMU数据、手柄控制指令
    std::shared_ptr<Robot> _robot;// 机器人运动学与动力学接口，包含底层电机与IMU数据
    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    // gait
//    std::thread _gait_thread;
    std::shared_ptr<Gait> _gait;
    // control
//    Estimator *_es;
//    VmcController *_vmc;
//    WbcController *_wbc;
//    MpcController *_mpc;
//    MrtGenerator *_mrt;
};

#endif //BUPT_DOG_CONTROLLER2_CTRL_COMPONENTS_HPP
