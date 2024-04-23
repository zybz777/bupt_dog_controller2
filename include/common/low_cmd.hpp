//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_LOW_CMD_HPP
#define BUPT_DOG_CONTROLLER2_LOW_CMD_HPP

#include <utility>

#include "lcm/lcm-cpp.hpp"
#include "doglcm/LegCmd_t.hpp"
#include "utils/math_types.hpp"
#include "utils/math_tools.hpp"

class LowCmd {
public:
    LowCmd() {
        for (int leg_id = 0; leg_id < 4; ++leg_id) {
            topic_names[leg_id] = "leg" + std::to_string(leg_id) + "/leg_cmd";
            _leg_cmd[leg_id].leg_id = (int8_t) leg_id;
            for (int motor_id = 0; motor_id < 3; ++motor_id) {
                _leg_cmd[leg_id].joint_cmd[motor_id].joint_id = (int8_t) motor_id;
                _leg_cmd[leg_id].joint_cmd[motor_id].mode = 0x0A;
                _leg_cmd[leg_id].joint_cmd[motor_id].K_P = 0.0;
                _leg_cmd[leg_id].joint_cmd[motor_id].K_W = 0.0;
                _leg_cmd[leg_id].joint_cmd[motor_id].Pos = 0.0;
                _leg_cmd[leg_id].joint_cmd[motor_id].W = 0.0;
                _leg_cmd[leg_id].joint_cmd[motor_id].T = 0.0;
            }
        }
        std::cout << "[LowCmd] Init Success!" << std::endl;
    }

    void publishLegCmd() {
        _lcm.publish(topic_names[0], &_leg_cmd[0]);
        _lcm.publish(topic_names[1], &_leg_cmd[1]);
        _lcm.publish(topic_names[2], &_leg_cmd[2]);
        _lcm.publish(topic_names[3], &_leg_cmd[3]);
    }
    /**************************************/
    /**** Set Joint Command (q dq tau) ****/
    /**************************************/
    void setQ(int leg_id, Vec3 q,
              Vec3 qMaxLimit = Vec3(30 * M_PI / 180, 100 * M_PI / 180, -70 * M_PI / 180),
              Vec3 qMinLimit = Vec3(-30 * M_PI / 180, 0 * M_PI / 180, -165.0 * M_PI / 180)) {
        _leg_cmd[leg_id].joint_cmd[0].Pos = clip((float) q[0], Vec2(qMinLimit[0], qMaxLimit[0]));
        _leg_cmd[leg_id].joint_cmd[1].Pos = clip((float) q[1], Vec2(qMinLimit[1], qMaxLimit[1]));
        _leg_cmd[leg_id].joint_cmd[2].Pos = clip((float) q[2], Vec2(qMinLimit[2], qMaxLimit[2]));
    }

    void setQ(Vec12 q) {
        for (int i = 0; i < 4; ++i) {
            setQ(i, q.segment<3>(3 * i));
        }
    }

    void setDq(int leg_id, Vec3 dq, const Vec2 &dqLimit = Vec2(-20 * M_PI, 20 * M_PI)) {
        _leg_cmd[leg_id].joint_cmd[0].W = clip((float) dq[0], dqLimit);
        _leg_cmd[leg_id].joint_cmd[1].W = clip((float) dq[1], dqLimit);
        _leg_cmd[leg_id].joint_cmd[2].W = clip((float) dq[2], dqLimit);
    }

    void setDq(Vec12 dq) {
        for (int i = 0; i < 4; ++i) {
            setDq(i, dq.segment<3>(3 * i));
        }
    }

    void setDq(int leg_id, int motor_id, double dq, Vec2 dqLimit = Vec2(-20 * M_PI, 20 * M_PI)) {
        _leg_cmd[leg_id].joint_cmd[motor_id].W = clip((float) dq, std::move(dqLimit));
    }

    void setTau(int leg_id, Vec3 tau, const Vec2 &torqueLimit = Vec2(-25.0, 25.0)) {
        _leg_cmd[leg_id].joint_cmd[0].T = clip((float) tau[0], torqueLimit);
        _leg_cmd[leg_id].joint_cmd[1].T = clip((float) tau[1], torqueLimit);
        _leg_cmd[leg_id].joint_cmd[2].T = clip((float) tau[2], torqueLimit);
    }

    void setTau(Vec12 tau, const Vec2 &torqueLimit = Vec2(-25.0, 25.0)) {
        for (int i = 0; i < 4; ++i) {
            setTau(i, tau.segment<3>(3 * i), torqueLimit);
        }
    }
    /**************************************/
    /******** Set Real Motor Gain *********/
    /**************************************/
    /**
     * @brief Robot Fixed Stand Use
     * @param leg_id
     */
    void setRealStanceGain(int leg_id) {
        Vec3 Kp(0.12, 0.12, 0.12), Kd(1.5, 1.5, 1.5);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    /**
     * @brief Swing Leg Use (Trot)
     * @param leg_id
     */
    void setRealSwingGain(int leg_id) {
        Vec3 Kp(0.007, 0.007, 0.007), Kd(0.005, 0.005, 0.005);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    /**
     * @brief Stance Leg Use (FreeStand)
     * @param leg_id
     */
    void setRealFreeStanceGain(int leg_id) {
        Vec3 Kp(0.003, 0.003, 0.003), Kd(0.1, 0.1, 0.1);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    /**
     * @brief Stance Leg Use (FreeStand)
     * @param leg_id
     */
    void setRealFreeStanceGain() {
        for (int i = 0; i < 4; ++i) {
            setRealFreeStanceGain(i);
        }
    }

    void setPassiveGain(int leg_id) {
        Vec3 Kp(0, 0, 0), Kd(0.3, 0.3, 0.3);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    void setPassiveGain() {
        for (int i = 0; i < 4; ++i) {
            setPassiveGain(i);
        }
    }
/**************************************/
    /********* Set Sim Motor Gain *********/
    /**************************************/
    void setSimStanceGain(int leg_id) {
        Vec3 Kp(200, 200, 300), Kd(3.5, 3.5, 4.0);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0B;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    void setSimSwingGain(int leg_id) {
        Vec3 Kp(15, 15, 15), Kd(1.5, 1.5, 1.5);
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = (float) Kp(i);
            _leg_cmd[leg_id].joint_cmd[i].K_W = (float) Kd(i);
        }
    }

    void setSimVelocityGain(int leg_id, int motor_id) {
        _leg_cmd[leg_id].joint_cmd[motor_id].mode = 0x0C;
        _leg_cmd[leg_id].joint_cmd[motor_id].K_P = 0.0;
        _leg_cmd[leg_id].joint_cmd[motor_id].K_W = 0.0;
    }
    /**************************************/
    /********* Set Motor Cmd Zero *********/
    /**************************************/
    void setZeroDq(int leg_id) {
        _leg_cmd[leg_id].joint_cmd[0].W = 0;
        _leg_cmd[leg_id].joint_cmd[1].W = 0;
        _leg_cmd[leg_id].joint_cmd[2].W = 0;
    }

    void setZeroTau(int leg_id) {
        _leg_cmd[leg_id].joint_cmd[0].T = 0;
        _leg_cmd[leg_id].joint_cmd[1].T = 0;
        _leg_cmd[leg_id].joint_cmd[2].T = 0;
    }

    void setZeroGain(int leg_id) {
        for (int i = 0; i < 3; ++i) {
            _leg_cmd[leg_id].joint_cmd[i].mode = 0x0A;
            _leg_cmd[leg_id].joint_cmd[i].K_P = 0;
            _leg_cmd[leg_id].joint_cmd[i].K_W = 0;
        }
    }

    void setZeroDq() {
        for (int i = 0; i < 4; ++i) {
            setZeroDq(i);
        }
    }

    void setZeroTau() {
        for (int i = 0; i < 4; ++i) {
            setZeroTau(i);
        }
    }

    void setZeroGain() {
        for (int i = 0; i < 4; ++i) {
            setZeroGain(i);
        }
    }

private:
    // lcm
    std::string topic_names[4];
    lcm::LCM _lcm;
    doglcm::LegCmd_t _leg_cmd[4]{};
};

#endif //BUPT_DOG_CONTROLLER2_LOW_CMD_HPP
