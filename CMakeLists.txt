cmake_minimum_required(VERSION 3.28)
project(bupt_dog_controller2)

set(CMAKE_CXX_STANDARD 17)

add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config/")
add_definitions(-DROBOT_PATH="${CMAKE_SOURCE_DIR}/config/")

set(hpipm-cpp_DIR "/usr/local/share/hpipm-cpp/cmake/hpipm-cpp-config.cmake")
find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(pinocchio REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)
find_package(hpipm-cpp REQUIRED)

include_directories(
        include
        ${LCM_USE_FILE}
        ../msg # lcm msg
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(bupt_dog_controller2 main.cpp
        include/common/low_state.hpp
        include/utils/math_types.hpp
        include/utils/math_tools.hpp
        src/common/robot.cpp
        include/common/robot.hpp
        src/common/estimator.cpp
        include/common/estimator.hpp
        include/common/low_cmd.hpp
        include/FSM/fsm_state.hpp
        include/common/ctrl_components.hpp
        include/FSM/fsm_enum.hpp
        src/FSM/state_fixed_down.cpp
        include/FSM/state_fixed_down.hpp
        src/FSM/state_fixed_stand.cpp
        include/FSM/state_fixed_stand.hpp
        include/gait/enum_gait.hpp
        include/gait/wave_generator.hpp
        include/utils/timer.hpp
        include/control/mpc/mpc_param.hpp
        src/gait/gait.cpp
        include/gait/gait.hpp
        src/FSM/fsm.cpp
        include/FSM/fsm.hpp
        src/FSM/state_passive.cpp
        include/FSM/state_passive.hpp)

# 动态库路径
link_directories(
        ${hpipm_DIR}/../lib
        ${blasfeo_DIR}/../lib
)

target_link_libraries(bupt_dog_controller2
        lcm
        pinocchio
        yaml-cpp::yaml-cpp
        hpipm
        blasfeo
        hpipm-cpp::hpipm-cpp
        -lpthread
)