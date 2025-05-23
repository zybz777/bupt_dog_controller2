cmake_minimum_required(VERSION 3.16)
project(bupt_dog_controller2)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
##########################
######### OPTION #########
##########################
# 设置 仿真模式(ON)或实物模式(OFF)
set(SIM_MODE ON)# 设置为 ON 或者 OFF
set(REAL_MODE OFF)
# 设置 状态估计标定噪声(ON OFF)
set(CALIBRATE_MODE OFF)
# 设置 MPC1 MPC2
set(MPC_MODE1 ON)
set(MPC_MODE2 OFF)
# 设置 WBC 独立子线程
set(WBC_THREAD OFF) # 独立线程WBC好像不稳定
# 设置 机器人学pinocchio 独立子线程
set(PIN_THREAD ON)
# 设置 状态估计器 独立子线程
set(ES_THREAD OFF)
##########################
######### OPTION #########
##########################

##############################
######### DEPENDENCY #########
##############################
set(hpipm-cpp_DIR "/usr/local/share/hpipm-cpp/cmake/hpipm-cpp-config.cmake")
#set(Torch_DIR "/opt/libtorch/share/cmake/Torch/")
add_definitions(-DCONFIG_PATH="${CMAKE_SOURCE_DIR}/config/")
add_definitions(-DROBOT_PATH="${CMAKE_SOURCE_DIR}/config/")

find_package(lcm REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(pinocchio REQUIRED)
find_package(blasfeo REQUIRED)
find_package(hpipm REQUIRED)
find_package(hpipm-cpp REQUIRED)

include(FetchContent)
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
        GIT_TAG yaml-cpp-0.7.0 # Replace with the desired tag, commit hash, or branch
)
FetchContent_GetProperties(yaml-cpp)
if (NOT yaml-cpp_POPULATED)
    message(STATUS "Fetching yaml-cpp...")
    FetchContent_Populate(yaml-cpp)
    add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
    message(STATUS "yaml-cpp Source Dir: ${yaml-cpp_SOURCE_DIR}")
    message(STATUS "yaml-cpp Binary Dir: ${yaml-cpp_BINARY_DIR}")
endif ()

#find_package(Torch REQUIRED)
#message(STATUS "Torch library status:")
#message(STATUS "    version: ${TORCH_VERSION}")
#message(STATUS "    libraries: ${TORCH_LIBS}")
#message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}")
#message(STATUS "    torch lib : ${TORCH_LIBRARIES} ")

##############################
######### DEPENDENCY #########
##############################


include_directories(
        include
        ${LCM_USE_FILE}
        ${EIGEN3_INCLUDE_DIR}
        ${TORCH_INCLUDE_DIRS}
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
        src/control/mpc/mpc_controller2.cpp
        src/control/mpc/centroidal_force_mapper.cpp
        src/control/wbc/wbc_controller.cpp
        src/control/wbc/wbc_optimizer.cpp
        src/FSM/state_bridge_trot.cpp
        include/FSM/state_bridge_trot.hpp
        src/FSM/state_bridge_slow_trot.cpp
        include/FSM/state_bridge_slow_trot.hpp
)
# 动态库路径
link_directories(
        ${hpipm_DIR}/../lib
        ${blasfeo_DIR}/../lib
)

target_link_libraries(${PROJECT_NAME}
        pinocchio
        yaml-cpp::yaml-cpp
        lcm
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

if (MPC_MODE1)
    add_compile_definitions(USE_MPC1)
endif ()
if (MPC_MODE2)
    add_compile_definitions(USE_MPC2)
endif ()

if (WBC_THREAD)
    add_compile_definitions(USE_WBC_THREAD)
endif ()

if (PIN_THREAD)
    add_compile_definitions(USE_PIN_THREAD)
endif ()

if (ES_THREAD)
    add_compile_definitions(USE_ES_THREAD)
endif ()