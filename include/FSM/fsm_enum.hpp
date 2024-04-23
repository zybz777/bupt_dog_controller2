//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_FSM_ENUM_HPP
#define BUPT_DOG_CONTROLLER2_FSM_ENUM_HPP
enum class FSMStateName {
    INVALID,
    PASSIVE,    // 阻尼状态
    FIXEDSTAND, // 固定站立
    FIXEDDOWN,  // 固定趴下
    FREESTAND,  // 自由站立
    TROTTING,   // 2+2
    TEST        // 测试
};

enum class FSMMode {
    NORMAL,   // 正常
    CHANGE,   // 切换
    ABNORMAL, // 异常
};
#endif //BUPT_DOG_CONTROLLER2_FSM_ENUM_HPP
