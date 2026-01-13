# 参考： autoware/autoware_msgs/ImageTracked.msg
#
# 目标跟踪，输出状态序列
# 基于图像实现或基于电云实现都应当生成该消息
#
# 所有输出目标信息均基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]
# 速度和加速度数据为相对值，以主车为原点视为静止


# 消息定义
Header                      header                      # Header.stamp: 获得目标时刻
                                                        # Header.frame_id: 数据来源（算法名称）


uint32                      id                          # 目标id
uint8                       label                       # 目标类型（标签）
cta_msgs/MotionState3d[]    states                      # 跟踪状态序列