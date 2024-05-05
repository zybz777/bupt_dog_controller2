# Bupt Dog Controller2
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