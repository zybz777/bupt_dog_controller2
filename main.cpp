#include <iostream>
#include "common/ctrl_components.hpp"
#include "FSM/fsm.hpp"
#include "utils/real_time.hpp"

int main() {
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
    setPriority();
    int ms = 1;
    auto ctrl_comp = std::make_shared<CtrlComponents>(ms);
    auto fsm = std::make_shared<FSM>(ctrl_comp, ms);
    ctrl_comp->begin();
    fsm->begin();
    return 0;
}
