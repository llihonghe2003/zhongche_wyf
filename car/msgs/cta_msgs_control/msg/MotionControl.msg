# 运动控制消息，只用于控制车辆的运动机构，包括 挡位、加速器、刹车、方向盘
# 控制量归一化
#
# 存储了控制的参考量，包括参考位置/航向，参考线/角速度，参考线/角加速度

# 消息定义
Header                  header              # Header.stamp: 控制时刻
                                            # Header.frame_id: 控制算法模块名称

int8                    gear                # 挡位：0 空挡，+1 前进挡，-1 倒挡
float64                 throttle            # 取值 [0.0, 1.0]
float64                 brake               # 取值 [0.0, 1.0]
float64                 steerangle          # 取值 [-1.0, +1.0]
float64                 steerspeed          # 取值 [0.0, +1.0]