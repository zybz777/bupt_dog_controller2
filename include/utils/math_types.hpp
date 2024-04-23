//
// Created by zyb on 24-4-22.
//

#ifndef BUPT_DOG_CONTROLLER2_MATH_TYPES_HPP
#define BUPT_DOG_CONTROLLER2_MATH_TYPES_HPP

#include <eigen3/Eigen/Dense>
/************************/
/******** Vector ********/
/************************/
// 2x1 Vector
using Vec2 = typename Eigen::Matrix<double, 2, 1>;

// 3x1 Vector
using Vec3 = typename Eigen::Matrix<double, 3, 1>;
// 4x1 Integer Vector
using VecInt4 = typename Eigen::Matrix<int, 4, 1>;
// 4x1 Vector
using Vec4 = typename Eigen::Matrix<double, 4, 1>;
using Quat = typename Eigen::Matrix<double, 4, 1>;
using Vec4_i8 = typename Eigen::Matrix<int8_t, 4, 1>;

// 6x1 Vector
using Vec6 = typename Eigen::Matrix<double, 6, 1>;
// 12x1 Vector
using Vec12 = typename Eigen::Matrix<double, 12, 1>;
// 18x1 Vector
using Vec18 = typename Eigen::Matrix<double, 18, 1>;
// 24x1 Vector
using Vec24 = typename Eigen::Matrix<double, 24, 1>;
// Dynamic Length Vector
using VecX = typename Eigen::Matrix<double, Eigen::Dynamic, 1>;

/************************/
/******** Matrix ********/
/************************/
// Rotation Matrix
using RotMat = typename Eigen::Matrix<double, 3, 3>;
// 3x3 Matrix
using Mat3 = typename Eigen::Matrix<double, 3, 3>;
// 3x4 Matrix, each column is a 3x1 vector
using Vec34 = typename Eigen::Matrix<double, 3, 4>;
// 12x12 Matrix
using Mat12 = typename Eigen::Matrix<double, 12, 12>;
// 24x12 Matrix
using Mat24x12 = typename Eigen::Matrix<double, 24, 12>;
// Dynamic Size Matrix
using MatX = typename Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;

#define _I3 Eigen::MatrixXd::Identity(3, 3)
#define I12 Eigen::MatrixXd::Identity(12, 12)
#define I18 Eigen::MatrixXd::Identity(18, 18)
/************************/
/****** Functions *******/
/************************/
inline Vec34 vec12ToVec34(const Vec12 &vec12) {
    Vec34 vec34;
    for (int i(0); i < 4; ++i) {
        vec34.col(i) = vec12.segment(3 * i, 3);
    }
    return vec34;
}

inline Vec12 vec34ToVec12(const Vec34 &vec34) {
    Vec12 vec12;
    for (int i(0); i < 4; ++i) {
        vec12.segment(3 * i, 3) = vec34.col(i);
    }
    return vec12;
}

#endif //BUPT_DOG_CONTROLLER2_MATH_TYPES_HPP
