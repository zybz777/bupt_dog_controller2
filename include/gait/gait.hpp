//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_GAIT_HPP
#define BUPT_DOG_CONTROLLER2_GAIT_HPP


#include <memory>
#include "doglcm/UserCmd_t.hpp"
#include "enum_gait.hpp"
#include "utils/math_types.hpp"

class Gait {
public:
    explicit Gait(std::shared_ptr<doglcm::UserCmd_t> user_cmd);

    void step();

    // get
    GaitType getGaitType() { return _gait_type; }

    double getTstance() { return _period * _st_ratio; }

    double getTswing() { return _period * (1 - _st_ratio); }

    double getT() { return _period; }

    const Vec4 &getPhase() { return _phase; }

    double getPhase(int leg_id) { return _phase[leg_id]; }

    const VecInt4 &getContact() { return _contact; }

    int getContact(int leg_id) { return _contact[leg_id]; }

    const std::vector<Vec4_i8> &getGaitList() { return _mpc_contact_list; }

private:
    void init();

    void checkGaitChange();

    void setGaitType(GaitType gait_type, double T);

    void calcWave(Vec4 &phase, VecInt4 &contact);

    void calcMpcWave(std::vector<Vec4_i8> &mpc_contact_list);

    std::shared_ptr<doglcm::UserCmd_t> _user_cmd;
    GaitType _gait_type;
    GaitType _next_gait_type;

    WaveStatus _wave;
    double _period;   // 周期
    double _st_ratio; // 站立 比例
    Vec4 _bias;       // 站立偏移时间(相位)
    Vec4 _normal_T;   // [0,1)
    Vec4 _phase;      // 当前足的相位，支撑摆动都是0-1
    VecInt4 _contact; // 1: contact 0: no contact

    double _pass_T;     // unit: second
    long long _start_T; // unit: us

    // mpc
    double _mpc_dt;
    std::vector<Vec4_i8> _mpc_contact_list;
};


#endif //BUPT_DOG_CONTROLLER2_GAIT_HPP
