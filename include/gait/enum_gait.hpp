//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_ENUM_GAIT_HPP
#define BUPT_DOG_CONTROLLER2_ENUM_GAIT_HPP
enum class GaitType {
    INVALID = 0x00,
    FIXEDSTAND = 0x01,
    FIXEDDOWN = 0x02,
    TROTTING = 0x03,
    FREESTAND = 0x04,
    PASSIVE = 0x05,
    BRIDGETROTING = 0x06,
    BRIDGESLOWTROTING = 0x07
};

enum class WaveStatus {
    STANCE_ALL,
    SWING_ALL,
    WAVE_ALL
};

#define SWING 0
#define CONTACT 1

const double FIXEDSTAND_T = 3.0;
const double FIXEDDOWN_T = 3.0;
#endif //BUPT_DOG_CONTROLLER2_ENUM_GAIT_HPP
