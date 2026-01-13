# 参考： autoware/autoware_msgs/DetectedObject.msg
#
# 动态目标消息
# 动态目标指行驶环境中具有运动属性、可能与主车发生运动交互的目标，包括车辆、行人、自行车等；包括静止停放的车辆
#
# 该消息内可整合目标检测（类别、位置），目标跟踪（id），目标预测（轨迹）的信息
#
# 所有输出目标信息均基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]
# 速度和加速度数据为相对值，以主车为原点视为静止


# 消息定义
Header                      header                      # Header.stamp: ROS时间戳
                                                        # Header.frame_id: 数据来源

int32                       id                          # 目标id
uint8                       label                       # 标签，预定义
cta_msgs/ColorRGB           color                       # 目标颜色
cta_msgs/Vector3d           size                        # 目标3轴尺寸

cta_msgs/MotionState3d      state                       # 目标运动状态

uint8                       behavior                    # 目标行为，预定义
float64                     predstep                    # 预测时间间隔（连续状态间时间）
cta_msgs/MotionState3d[]    predstates                  # 目标预测状态估计

string[]                    usrinfo                     # 用户自定义信息