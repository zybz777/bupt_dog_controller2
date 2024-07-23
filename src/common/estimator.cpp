//
// Created by zyb on 24-4-23.
//

#include "common/estimator.hpp"

Estimator::Estimator(int ms, const std::shared_ptr<Gait> &gait, const std::shared_ptr<Robot> &robot) {
    _ms = ms;
    _dt = (double) ms / 1000.0;
    _gait = gait;
    _robot = robot;
    init();
#ifdef USE_ES_THREAD
    _thread = std::thread([this] { run(_ms); });
#endif
    std::cout << "[Estimator] Init Success!" << std::endl;
}

void Estimator::init() {
    _g << 0, 0, -9.81;
    _feetH_inWorld.setZero();
    // x u init
    _xhat.setZero();
    _u.setZero();
    // A init
    _A.setIdentity();
    _A.block<3, 3>(0, 3) = _I3 * _dt;
    // B init
    _B.setZero();
    _B.block<3, 3>(3, 0) = _I3 * _dt;
    // C init
    _C.setZero();
    _C.block<3, 3>(0, 0) = -_I3;
    _C.block<3, 3>(3, 0) = -_I3;
    _C.block<3, 3>(6, 0) = -_I3;
    _C.block<3, 3>(9, 0) = -_I3;
    _C.block<3, 3>(12, 3) = -_I3;
    _C.block<3, 3>(15, 3) = -_I3;
    _C.block<3, 3>(18, 3) = -_I3;
    _C.block<3, 3>(21, 3) = -_I3;
    _C.block<12, 12>(0, 6) = I12;
    _C(24, 8) = 1;
    _C(25, 11) = 1;
    _C(26, 14) = 1;
    _C(27, 17) = 1;
    // P init
    _large_variance = 10000;
    _P.setIdentity();
    // QR标定
    _R_check = std::make_shared<AvgCov>(Y_NUM, "R");
    _u_check = std::make_shared<AvgCov>(U_NUM, "u");
    readQR(_Cu, _R_init);
    // Q init
    for (int i = 0; i < _Q_diag.rows(); ++i) {
        if (i < 3) {
            // 位置估计 建模误差较小
            _Q_diag[i] = 2e-5;
        } else if (i < 6) {
            // 速度估计 建模误差较小
            _Q_diag[i] = 2e-6;
        } else {
            // 足端位置估计 足端触地时抖动带来较大误差
            _Q_diag[i] = 2e-4;
        }
    }
    _Q_init = _Q_diag.asDiagonal(); // 建模与离散化产生的过程噪声
    _Q_init += _B * _Cu * _B.transpose(); // 测量到的输入量过程噪声
    // R init
    _R_init(24, 24) = 1e-10;
    _R_init(25, 25) = 1e-10;
    _R_init(26, 26) = 1e-10;
    _R_init(27, 27) = 1e-10;
    // std::cout << "Q init " << std::endl
    //           << _Q_init << std::endl;
    // std::cout << "R init " << std::endl
    //           << _R_init << std::endl;
    // LP filter
    _vx_filter = std::make_shared<LPFilter>(_dt, 20.0);
    _vy_filter = std::make_shared<LPFilter>(_dt, 20.0);
    _vz_filter = std::make_shared<LPFilter>(_dt, 10.0);
    _px_filter = std::make_shared<LPFilter>(_dt, 3.0);
    _py_filter = std::make_shared<LPFilter>(_dt, 3.0);
    _pz_filter = std::make_shared<LPFilter>(_dt, 3.0);
    _fake_pitch_filter = std::make_shared<LPFilter>(_dt, 3.0);
    // lcm
    _es_data_topic_name = "es_data";
}


int Estimator::getContact(int leg_id) {
    return _gait->getContact(leg_id);
}

void Estimator::fakePitch() {
    RotMat R = rotMatRy(_robot->getRpy()[1]); // or Ry * Rx
    Vec34 foot_pos_2com_inWorld;
    for (int i = 0; i < LEG_NUM; ++i) {
        foot_pos_2com_inWorld.col(i) = R * _robot->getFootPosition_inBody(i);
    }
    static double P_Fx = _robot->getRobotStdFootPos_inBody().col(0)[0];
    static double P_Fz = _robot->getRobotStdFootPos_inBody().col(0)[2];
    static double P_Hx = _robot->getRobotStdFootPos_inBody().col(3)[0];
    static double P_Hz = _robot->getRobotStdFootPos_inBody().col(3)[2];
    for (int i = 0; i < 2; ++i) {
        if (getContact(i) == CONTACT) {
            P_Fx = foot_pos_2com_inWorld.col(i)[0];
            P_Fz = foot_pos_2com_inWorld.col(i)[2];
        }
    }
    for (int i = 2; i < 4; ++i) {
        if (getContact(i) == CONTACT) {
            P_Hx = foot_pos_2com_inWorld.col(i)[0];
            P_Hz = foot_pos_2com_inWorld.col(i)[2];
        }
    }
    double fake_pitch = atan2(P_Hz - P_Fz, P_Fx - P_Hx);
    _fake_pitch_filter->addValue(fake_pitch);
    // 地形适应 高度最低的足端高度为0 其余足端高度抬高
    double min_foot_pos = min(P_Hz, P_Fz);
    for (int i = 0; i < LEG_NUM; ++i) {
        if (getContact(i) == CONTACT) {
            _feetH_inWorld[i] = foot_pos_2com_inWorld.col(i)[2] - min_foot_pos;
        } else {
            _feetH_inWorld[i] = 0.0;
        }
    }
    // std::cout << foot_pos_2com_inWorld << std::endl;
}

#ifdef USE_ES_THREAD
void Estimator::begin() {
    while (!_thread.joinable()) {}
    _thread.join();
}
[[noreturn]] void Estimator::run(int ms) {
    std::cout << "[Estimator] Task Run!" << std::endl;
    std::chrono::microseconds period(ms * 1000);
    auto start_time = std::chrono::high_resolution_clock::now();
    while (true) {
        step();
        start_time += period;
        std::this_thread::sleep_until(start_time);
    }
}
#endif

void Estimator::step() {
    fakePitch();
    const RotMat &R = _robot->getRotMat();
    /* 观测量更新 */
    for (int i = 0; i < 4; ++i) {
        _feetPos2Body_inWorld.segment<3>(3 * i) = R * _robot->getFootPosition_inBody(i);
        _feetVel2Body_inWorld.segment<3>(3 * i) = R * (_robot->getFootVelocityFiltered_inBody(i) +
                                                       skew(_robot->getAngularVelocity()) *
                                                       _robot->getFootPosition_inBody(i));
    }
    _y << _feetPos2Body_inWorld, _feetVel2Body_inWorld, _feetH_inWorld;
    /* 输入量更新 */
    _u = _robot->getLinearAccelerometer_inWorld() + _g;
#ifdef USE_CALIBRATE
    calibrateQR(); // 标定噪声
#endif
    /* 噪声更新 */
    _Q = _Q_init;
    _R = _R_init;
    for (int i = 0; i < 4; ++i) {
        if (getContact(i) == SWING) {
            _Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) = _large_variance * _I3; // 摆动腿位置估计 大噪声
            _R.block<3, 3>(12 + 3 * i, 12 + 3 * i) = _large_variance * _I3; // 摆动腿速度测量 大噪声
            _R(24 + i, 24 + i) = _large_variance; // 摆动腿高度测量 大噪声
        } else {
            _trust = windowFunc2(_gait->getPhase(i), 0.25, 0.1);
            // 摆动腿位置估计噪声随触地相位增大而变小
            _Q.block<3, 3>(6 + 3 * i, 6 + 3 * i) =
                    (1 + (1 - _trust) * _large_variance) * _Q_init.block<3, 3>(6 + 3 * i, 6 + 3 * i);
            // 摆动腿速度测量噪声随触地相位增大而变小
            _R.block<3, 3>(12 + 3 * i, 12 + 3 * i) =
                    (1 + (1 - _trust) * _large_variance) * _R_init.block<3, 3>(12 + 3 * i, 12 + 3 * i);
            // 摆动腿高度测量噪声随触地相位增大而变小
            _R(24 + i, 24 + i) = (1 + (1 - _trust) * _large_variance) * _R_init(24 + i, 24 + i);
        }
    }
    /* kalman */
    _xhat = _A * _xhat + _B * _u;
    _yhat = _C * _xhat;

    _Ppriori = _A * _P * _A.transpose() + _Q;
    _S = _R + _C * _Ppriori * _C.transpose();
    _Slu = _S.lu();
    _Sy = _Slu.solve(_y - _yhat);
    _Sc = _Slu.solve(_C);
    _IKC = I18 - _Ppriori * _C.transpose() * _Sc;

    _xhat += _Ppriori * _C.transpose() * _Sy;
    _P = _IKC * _Ppriori;
    // lp filter
    _px_filter->addValue(_xhat[0]);
    _py_filter->addValue(_xhat[1]);
    _pz_filter->addValue(_xhat[2]);
    _vx_filter->addValue(_xhat[3]);
    _vy_filter->addValue(_xhat[4]);
    _vz_filter->addValue(_xhat[5]);
    // robot set estimator
    _robot->setComPosition_inWorld(getLpPosition());
    _robot->setComVelocity_inWorld(getVelocity());
    _robot->setComLpVelocity_inWorld(getLpVelocity());
    // lcm
    publishEsData();
}

void Estimator::publishEsData() {
    _es_data.pos[0] = getLpPosition()[0];
    _es_data.pos[1] = getLpPosition()[1];
    _es_data.pos[2] = getLpPosition()[2];
    memcpy(_es_data.vel, getVelocity().data(), sizeof(_es_data.vel));
    memcpy(_es_data.lp_vel, getLpVelocity().data(), sizeof(_es_data.lp_vel));
    _lcm.publish(_es_data_topic_name, &_es_data);
}

void Estimator::calibrateQR() {
    bool u_flag = false;
    bool R_flag = false;
    std::map<std::string, std::vector<double> > data;
    if (_u_check->measure(_u)) {
        data["u_cov"] = _u_check->getCov();
        u_flag = true;
    }
    if (_R_check->measure(_y)) {
        data["R_cov"] = _R_check->getCov();
        R_flag = true;
    }
    // data["Q"] = {1, 2, 3, 4}; // Replace this array with your data.
    if (!u_flag || !R_flag)
        return;
    // Create a YAML node and assign the map.
    YAML::Node node(data);

    // Create a YAML document and add the node.
    YAML::Emitter emitter;
    emitter << node;

    // Write the YAML document to a file.
    std::string es_config_path = CONFIG_PATH;
#ifdef USE_SIM
    es_config_path += "sim_es_config.yaml";
#else
    es_config_path += "real_es_config.yaml";
#endif
    std::ofstream file(es_config_path);
    file << emitter.c_str();
    file.close();

    std::cout << "config.yaml file generated with an array as a key-value pair." << std::endl;
}

void Estimator::readQR(Mat3 &Cu, Eigen::Matrix<double, 28, 28> &R_init) {
    // 从YAML文件加载配置
    std::string es_config_path = CONFIG_PATH;
#ifdef USE_SIM
    es_config_path += "sim_es_config.yaml";
#else
    es_config_path += "real_es_config.yaml";
#endif
    YAML::Node node = YAML::LoadFile(es_config_path);
    if (node["u_cov"]) {
        // 读取对应的数组
        auto buffer = node["u_cov"].as<std::vector<double> >();
        memcpy(Cu.data(), buffer.data(), sizeof(Cu));
        // std::cout << "Cu: " << std::endl
        //         << Cu << std::endl;
    } else {
        Cu.setZero();
        std::cerr << "The YAML file does not contain a u_cov key." << std::endl;
    }

    if (node["R_cov"]) {
        // 读取
        auto buffer = node["R_cov"].as<std::vector<double> >();
        memcpy(R_init.data(), buffer.data(), sizeof(R_init));
    } else {
        R_init.setZero();
        std::cerr << "The YAML file does not contain a R_cov key." << std::endl;
    }
}
