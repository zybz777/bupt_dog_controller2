//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_REAL_TIME_HPP
#define BUPT_DOG_CONTROLLER2_REAL_TIME_HPP

#include <sys/syscall.h>

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

// 获取线程tid
inline pid_t gettid(void)
{
    return syscall(SYS_gettid);
}

// 为线程绑定指定cpu核心 cpu_id: 0 ~ max_cpu-1
inline void assignTask2Cpu(int cpu_id)
{
    // 获取线程ID
    pid_t tid = gettid();
    // 创建一个CPU集合并将其初始化为空
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    // 将线程绑定到第一个CPU核心（CPU0）
    CPU_SET(cpu_id, &cpuset);
    // 使用sched_setaffinity分配CPU核心
    if (sched_setaffinity(tid, sizeof(cpuset), &cpuset) == -1)
    {
        std::cerr << "Failed to set thread affinity." << std::endl;
    }
    else
    {
        std::cout << "Thread is running on CPU" + std::to_string(cpu_id) << std::endl;
    }
}

#endif //BUPT_DOG_CONTROLLER2_REAL_TIME_HPP
