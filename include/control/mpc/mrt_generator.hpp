//
// Created by zyb on 24-4-26.
//

#ifndef BUPT_DOG_CONTROLLER2_MRT_GENERATOR_HPP
#define BUPT_DOG_CONTROLLER2_MRT_GENERATOR_HPP


#include "utils/math_types.hpp"
#include "common/robot.hpp"
#include "gait/gait.hpp"
#include "common/estimator.hpp"

class MrtGenerator {
public:
    MrtGenerator();

    const std::vector<VecX> &getXtraj() { return _X_traj; }

    void step(const std::shared_ptr<Robot> &robot, const std::shared_ptr<Gait> &gait, const std::shared_ptr<Estimator> &estimator);

private:

    void resetXtraj(const Vec12 &X);

    double _dt;
    // X
    Vec12 _X;
    std::vector<VecX> _X_traj; // output
};


#endif //BUPT_DOG_CONTROLLER2_MRT_GENERATOR_HPP
