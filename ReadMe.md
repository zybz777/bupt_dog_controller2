# Bupt Dog Controller2
## INSTALL
1. Lcm
```bash
sudo apt install libeigen3-dev
sudo apt install build-essential
sudo apt install cmake
sudo apt install libglib2.0-dev
sudo apt install default-jdk
sudo apt install python3-dev
git clone https://github.com/lcm-proj/lcm.git
cd lcm
mkdir build && cd build
cmake ..
make -j8
sudo make install
```
2. Pinocchino
```bash
sudo apt install libboost-all-dev
sudo apt install liburdfdom-dev
sudo apt install liburdfdom-headers-dev 
sudo apt install lib
git clone https://github.com/stack-of-tasks/pinocchio
cd pinocchio
git checkout v2.7.1
sed -i 's/OPTION(BUILD_PYTHON_INTERFACE "Build the Python bindings" ON)/OPTION(BUILD_PYTHON_INTERFACE "Build the Python bindings" OFF)/' CMakeLists.txt
grep 'OPTION(BUILD_PYTHON_INTERFACE "Build the Python bindings" OFF)' CMakeLists.txt
mkdir build && cd build
# TODO: 将CMakeLists.txt中82行的BUILD_PYTHON_INTERFACE设置为OFF 清空build，重新cmake..
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j8 # 多次编译
sudo make install
echo 'export PATH=/usr/local/bin:$PATH' >> ~/.bashrc
echo 'export PKG_CONFIG_PATH =/usr/local/lib/pkgconfig:$PKG_CONFIG_PATH' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
echo 'export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH' >> ~/.bashrc
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
4. yaml-cpp(跳过)
```bash
git clone git@github.com:jbeder/yaml-cpp.git
cd yaml-cpp && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_SHARED_LIBS=ON -DBLASFEO_EXAMPLES=OFF 
make -j8
sudo make install -j
```
5. libtorch(正在测试)
   1. https://pytorch.org/ 下载cxx11 ABI 2.3版本压缩包
   2. 移动libtorch文件夹到/opt/libtorch
   3. export LD_LIBRARY_PATH=/opt/libtorch/lib:$LD_LIBRARY_PATH
6. bupt_dog_msg
```bash
git clone git@github.com:zybz777/bupt_dog_msg.git
cd bupt_dog_msg
sudo chmod +x lcm_gen.sh
sudo ./lcm_gen.sh
echo 'export CLASSPATH=~/bupt_dog_msg/java/my_types.jar' >> ~/.bashrc
```
7. bupt_dog_controller2
```bash
sudo apt install libarmadillo-dev
git clone git@github.com:zybz777/bupt_dog_controller2.git
cd ~/bupt_dog_controller2
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j8
echo 'export PATH=~/bupt_dog_controller2/build:$PATH' >> ~/.bashrc
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
sudo ln -s ~/bupt_dog_controller2/build/bupt_dog_controller2 /usr/local/bin/
```
2. 建立软链接将库文件放入/usr/local/lib中
```bash
sudo ln -s /opt/blasfeo/lib/libblasfeo.so /usr/local/lib/
sudo ldconfig
```
3. 运行
```bash
sudo bupt_dog_controller2
```
