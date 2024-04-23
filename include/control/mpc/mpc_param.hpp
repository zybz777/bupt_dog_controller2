//
// Created by zyb on 24-4-23.
//

#ifndef BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
#define MPC_FREQUENCY 100
#define HORIZON 100

const double inf = 1e6;
const double M = 15.851f; // 不含4条腿是13.895
const double M_BODY = 7.863f;
const double M_LEG = 1.937f;
const double I_diag[3] = {0.0657746146508966, 0.25734642693628385, 0.29240819661706724};
const double Body_Com[3] = {-0.00580, 0.0, -0.0158041};
// const double Body_Com[3] = {0.0, 0.0, 0.0};
#endif //BUPT_DOG_CONTROLLER2_MPC_PARAM_HPP
