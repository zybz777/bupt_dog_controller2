//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_REAL_TIME_HPP
#define BUPT_DOG_CONTROLLER2_REAL_TIME_HPP

#include <armadillo>

inline void setPriority() {
    pid_t pid = getpid();
    sched_param param;
    param.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (sched_setscheduler(pid, SCHED_FIFO, &param) == -1) {
        std::cout << "Set ProcessScheduler Failed!" << std::endl;
    } else {
        std::cout << "Set ProcessScheduler Success!" << std::endl;
    }
}

#endif //BUPT_DOG_CONTROLLER2_REAL_TIME_HPP
