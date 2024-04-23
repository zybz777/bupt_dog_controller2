# Bupt Dog Controller2
## Common
1. low_state：读取电机数据和IMU数据 单独线程
2. low_cmd：发送电机指令 与状态机同一线程
3. robot+estimator:计算运动学数据和估计位姿
## Utils
数学工具及类型定义