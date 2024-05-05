# Bupt Dog Controller2
## INSTALL
1. Lcm
```bash
sudo apt install libeigen3-dev
sudo apt install build-essential
sudo apt install cmake
sudo apt install libglib2.0-dev
sudo apt install default-jdk
sudo apt install libjchart2d-java
sudo apt install python3-dev
git clone git@github.com:lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j8
sudo make install
```
2. Pinocchino
```bash
sudo apt update
sudo apt install ros-noetic-pinocchio # 通过ros安装
```
3. hpipm-cpp
```bash
git clone https://github.com/giaf/blasfeo
git clone https://github.com/giaf/hpipm
cd blasfeo && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF 
make -j8
sudo make install -j
cd ../../hpipm && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DHPIPM_TESTING=OFF 
make -j8
sudo make install -j
echo 'export LD_LIBRARY_PATH=/opt/blasfeo/lib:/opt/hpipm/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
git clone https://github.com/mayataka/hpipm-cpp
cd hpipm-cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release 
make -j8
sudo make install -j
```

## Common
1. low_state：读取电机数据和IMU数据 单独线程
2. low_cmd：发送电机指令 与状态机同一线程
3. robot+estimator:计算运动学数据和估计位姿
## Utils
数学工具及类型定义


## Usage
### REAL
1. 建立软链接将可执行文件放入/usr/local/bin中
```bash
sudo ln -s /home/zyb/CLionProjects/bupt_dog_controller2/cmake-build-release/bupt_dog_controller2 /usr/local/bin/
```
2. 建立软链接将库文件放入/usr/local/lib中
```bash
sudo ln -s /opt/blasfeo/lib/libblasfeo.so /usr/local/lib/
sudo ldconfig
```