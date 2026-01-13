# Master命令

# 消息定义
Header                  header              # Header.stamp: 时刻
                                            # Header.frame_id: 模块名称

uint8                   motionmode          # 运动控制模式
bool                    lampauto            # 灯光自动

bool                    runSensor           # 运行传感器
bool                    runPerception       # 运行感知算法
bool                    runLocalization     # 运行定位算法
bool                    runPlanning         # 运行规划算法
bool                    runControl          # 运行控制算法
bool                    runCar              # 启动车辆