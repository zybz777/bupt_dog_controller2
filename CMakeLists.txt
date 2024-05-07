cmake_minimum_required(VERSION 3.16)
project(bupt_dog_controller2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
# 设置 仿真模式(ON)或实物模式(OFF)
set(SIM_MODE OFF)# 设置为 ON 或者 OFF
set(REAL_MODE ON)
# 设置 状态估计标定噪声(ON OFF)
set(CALIBRATE_MODE OFF)

set(hpipm-cpp_DIR "/usr/local/share/hpipm-cpp/cmake/hpipm-cpp-config.cmake")
add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config/")
add_definitions(-DROBOT_PATH="${CMAKE_SOURCE_DIR}/config/")

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
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(${PROJECT_NAME}
        main.cpp
        src/common/robot.cpp
        src/common/estimator.cpp
        src/FSM/fsm.cpp
        src/FSM/state_fixed_down.cpp
        src/FSM/state_fixed_stand.cpp
        src/FSM/state_free_stand.cpp
        src/FSM/state_passive.cpp
        src/FSM/state_trot.cpp
        src/gait/gait.cpp
        src/control/vmc/vmc_controller.cpp
        src/control/mpc/mrt_generator.cpp
        src/control/mpc/mpc_controller.cpp
        src/control/wbc/wbc_controller.cpp
        src/control/wbc/wbc_optimizer.cpp)

# 动态库路径
link_directories(
        ${hpipm_DIR}/../lib
        ${blasfeo_DIR}/../lib
)

target_link_libraries(${PROJECT_NAME}
        lcm
        pinocchio
        yaml-cpp::yaml-cpp
        hpipm
        blasfeo
        hpipm-cpp::hpipm-cpp
        -lpthread
)

if (SIM_MODE)
    add_compile_definitions(USE_SIM)
endif ()
if (REAL_MODE)
    add_compile_definitions(USE_REAL)
endif ()
if (CALIBRATE_MODE)
    add_compile_definitions(USE_CALIBRATE)
endif ()