//
// Created by zyb on 24-4-22.
//

#ifndef BUPT_DOG_CONTROLLER2_MATH_TOOLS_HPP
#define BUPT_DOG_CONTROLLER2_MATH_TOOLS_HPP

#include "math_types.hpp"
#include <iostream>
#include <random>

inline MatX pinv(const MatX &mat) {
    return mat.completeOrthogonalDecomposition().pseudoInverse();
}

inline Mat3 skew(const Vec3 &v) {
    Mat3 m;
    m << 0, -v(2), v(1),
            v(2), 0, -v(0),
            -v(1), v(0), 0;
    return m;
}

inline RotMat rotMatEluerVel2BodyOmega(const Vec3 &rpy) {
    Mat3 m;
    double cy = cos(rpy[1]);
    double sy = sin(rpy[1]);
    double cx = cos(rpy[0]);
    double sx = sin(rpy[0]);
    m << 1, 0, -sy,
            0, cx, cy * sx,
            0, -sx, cy * cx;
    return m;
}

// 角速度旋转矩阵 欧拉角速率->世界系角速度
inline RotMat rotMatW(const Vec3 &rpy) {
    Mat3 m;
    double cy = cos(rpy[1]);
    double sy = sin(rpy[1]);
    double cz = cos(rpy[2]);
    double sz = sin(rpy[2]);
    m << cy * cz, -sz, 0,
            cy * sz, cz, 0,
            -sy, 0, 1;
    return m;
}

// 角速度旋转矩阵 世界系角速度->欧拉角速率
inline RotMat invRotMatW(const Vec3 &rpy) {
    RotMat mat;
    double c2 = cos(rpy[2]), c1 = cos(rpy[1]);
    double s2 = sin(rpy[2]);
    double t1 = tan(rpy[1]);
    mat << c2 / c1, s2 / c1, 0,
            -s2, c2, 0,
            c2 * t1, s2 * t1, 1;
    return mat;
}

inline Vec3 rotMatToRPY(const Mat3 &R) {
    Vec3 rpy;
    rpy(0) = atan2(R(2, 1), R(2, 2));
    rpy(1) = asin(-R(2, 0));
    rpy(2) = atan2(R(1, 0), R(0, 0));
    return rpy;
}

inline RotMat rotMatRz(const double &yaw) {
    RotMat mat;
    double s = sin(yaw);
    double c = cos(yaw);
    mat << c, -s, 0,
            s, c, 0,
            0, 0, 1;
    return mat;
}

inline RotMat rotMatRy(const double &pitch) {
    RotMat mat;
    double s = sin(pitch);
    double c = cos(pitch);
    mat << c, 0, s,
            0, 1, 0,
            -s, 0, c;
    return mat;
}

inline RotMat rotMatRx(const double &roll) {
    RotMat mat;
    double s = sin(roll);
    double c = cos(roll);
    mat << 1, 0, 0,
            0, c, -s,
            0, s, c;
    return mat;
}

inline RotMat rotMatRzyx(const Vec3 &rpy) {
    return rotMatRz(rpy[2]) * rotMatRy(rpy[1]) * rotMatRx(rpy[0]);
}

// limit func
template<typename T>
inline T clip(const T a, Vec2 limits) {
    T lowLim, highLim;
    if (limits(0) > limits(1)) {
        lowLim = limits(1);
        highLim = limits(0);
    } else {
        lowLim = limits(0);
        highLim = limits(1);
    }

    if (a < lowLim) {
        return lowLim;
    } else if (a > highLim) {
        return highLim;
    } else {
        return a;
    }
}

template<typename T>
inline T windowFunc(const T x, const T windowRatio, const T xRange = 1.0, const T yRange = 1.0) {
    if ((x < 0) || (x > xRange)) {
        std::cout << "[ERROR][windowFunc] The x=" << x << ", which should between [0, xRange]" << std::endl;
    }
    if ((windowRatio <= 0) || (windowRatio >= 0.5)) {
        std::cout << "[ERROR][windowFunc] The windowRatio=" << windowRatio << ", which should between [0, 0.5]"
                  << std::endl;
    }

    if (x / xRange < windowRatio) {
        return x * yRange / (xRange * windowRatio);
    } else if (x / xRange > 1 - windowRatio) {
        return yRange * (xRange - x) / (xRange * windowRatio);
    } else {
        return yRange;
    }
}

template<typename T1, typename T2>
inline void updateAverage(T1 &exp, T2 newValue, double n) {
    if (exp.rows() != newValue.rows()) {
        std::cout << "The size of updateAverage is error" << std::endl;
        exit(-1);
    }
    if (fabs(n - 1) < 0.001) {
        exp = newValue;
    } else {
        exp = exp + (newValue - exp) / n;
    }
}

template<typename T1, typename T2, typename T3>
inline void updateCovariance(T1 &cov, T2 expPast, T3 newValue, double n) {
    if ((cov.rows() != cov.cols()) || (cov.rows() != expPast.rows()) || (expPast.rows() != newValue.rows())) {
        std::cout << "The size of updateCovariance is error" << std::endl;
        exit(-1);
    }
    if (fabs(n - 1) < 0.1) {
        cov.setZero();
    } else {
        cov = cov * (n - 1) / n + (newValue - expPast) * (newValue - expPast).transpose() * (n - 1) / (n * n);
    }
}

template<typename T1, typename T2, typename T3>
inline void updateAvgCov(T1 &cov, T2 &exp, T3 newValue, double n) {
    // The order matters!!! covariance first!!!
    updateCovariance(cov, exp, newValue, n);
    updateAverage(exp, newValue, n);
}

class AvgCov {
public:
    AvgCov(unsigned int size, std::string name, bool avgOnly = false, unsigned int showPeriod = 1000,
           unsigned int waitCount = 30000, double zoomFactor = 10000) {
        _size = size;
        _valueName = name;
        _avgOnly = avgOnly;
        _showPeriod = showPeriod;
        _waitCount = waitCount;
        _zoomFactor = zoomFactor;

        _exp.resize(size);
        _cov.resize(size, size);
        _defaultWeight.resize(size, size);
        _defaultWeight.setIdentity();
        _measureCount = 0;
    }

    bool measure(VecX newValue) {
        ++_measureCount;
        if (_measureCount % 1000 == 0 && _measureCount < _waitCount) {
            std::cout << "prepareing :" << int(_measureCount) << std::endl;
        }
        if (_measureCount > _waitCount) {
            updateAvgCov(_cov, _exp, newValue, _measureCount - _waitCount);
            if (_measureCount % _showPeriod == 0) {
                std::cout << "******" << _valueName << " measured count: " << _measureCount - _waitCount << "******"
                          << std::endl;
                std::cout << _zoomFactor << " Times Average of " << _valueName << std::endl
                          << (_zoomFactor * _exp).transpose() << std::endl;
                if (!_avgOnly) {
                    // std::cout << _zoomFactor << " Times Covariance of " << _valueName << std::endl
                    //           << _zoomFactor * _cov << std::endl;
                    // RCLCPP_INFO_STREAM(rclcpp::get_logger(_valueName), _zoomFactor << " Times Covariance of " << _valueName << std::endl
                    //                                                                << _zoomFactor * _cov);
                }
                return true;
            }
        }
        return false;
    }

    std::vector<double> getCov() {
        std::vector<double> cov_vector = std::vector<double>(_cov.cols() * _cov.rows());
        memcpy(cov_vector.data(), _cov.data(), _cov.size() * sizeof(double));
        // std::cout << "Cov Vector" << std::endl;
        return cov_vector;
    }

private:
    VecX _exp;
    MatX _cov;
    MatX _defaultWeight;
    bool _avgOnly;
    unsigned int _size;
    unsigned int _measureCount;
    unsigned int _showPeriod;
    unsigned int _waitCount;
    double _zoomFactor;
    std::string _valueName;
};

class GaussianNoise {
public:
    GaussianNoise(double mean, double stddev) {
        _gen = std::mt19937(_rd());
        _distribution = std::normal_distribution<double>(mean, stddev);
    }

    double getNoise() {
        return _distribution(_gen);
    }

private:
    // 创建一个随机数生成器
    std::random_device _rd;
    std::mt19937 _gen;
    // 创建一个正态分布对象
    std::normal_distribution<double> _distribution;
};


class LPFilter {
public:
    LPFilter(double samplePeriod, double cutFrequency) {
        _weight = 1.0 / (1.0 + 1.0 / (2.0 * M_PI * samplePeriod * cutFrequency));
        _start = false;
    }

    void addValue(double newValue) {
        if (!_start) {
            _start = true;
            _pastValue = newValue;
            return;
        }
        _pastValue = _weight * newValue + (1 - _weight) * _pastValue;
    }

    double getValue() {
        return _pastValue;
    }

    void clear() {
        _start = false;
    }

private:
    double _weight;
    double _pastValue;
    bool _start;
};

#endif //BUPT_DOG_CONTROLLER2_MATH_TOOLS_HPP
