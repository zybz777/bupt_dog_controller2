/*
 * @Author       : Zybz
 * @Date         : 2024-05-08 14:41:32
 * @LastEditors  : Zybz
 * @LastEditTime : 2024-05-09 16:37:31
 * @FilePath     : /bupt_dog_controller2/main.cpp
 * @Description  :
 *
 * Copyright (c) 2024 by BUPT RobotTeam, All Rights Reserved.
 */
#include "FSM/fsm.hpp"
#include "common/ctrl_components.hpp"
#include "utils/real_time.hpp"
#include <iostream>

void MACRO_PRINT() {
#ifdef USE_SIM
    std::cout << "#### SIM MODE ON ####" << std::endl;
#endif
#ifdef USE_REAL
    std::cout << "#### REAL MODE ON ####" << std::endl;
#endif
#ifdef USE_CALIBRATE
    std::cout << "#### CALIBRATE MODE ON ####" << std::endl;
#else
    std::cout << "#### CALIBRATE MODE OFF ####" << std::endl;
#endif
#ifdef USE_MPC1
    std::cout << "#### MPC MODE1 ON ####" << std::endl;
#endif
#ifdef USE_MPC2
    std::cout << "#### MPC MODE2 ON ####" << std::endl;
#endif
#ifdef USE_WBC_THREAD
    std::cout << "#### WBC THREAD ON ####" << std::endl;
#else
    std::cout << "#### WBC THREAD OFF ####" << std::endl;
#endif
#ifdef USE_PIN_THREAD
    std::cout << "#### PINOCCHIO THREAD ON ####" << std::endl;
#else
    std::cout << "#### PINOCCHIO THREAD OFF ####" << std::endl;
#endif
#ifdef USE_ES_THREAD
    std::cout << "#### ESTIMATOR THREAD ON ####" << std::endl;
#else
    std::cout << "#### ESTIMATOR THREAD OFF ####" << std::endl;
#endif
}

int main() {
    MACRO_PRINT();
    setPriority();
    int ms = CONTROL_DT_MS;
    auto ctrl_comp = std::make_shared<CtrlComponents>(ms);
    auto fsm = std::make_shared<FSM>(ctrl_comp, ms);
    ctrl_comp->begin();
    fsm->begin();
    return 0;
}
