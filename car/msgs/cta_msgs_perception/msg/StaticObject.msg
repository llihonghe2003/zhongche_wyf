# 参考： autoware/autoware_msgs/DetectedObject.msg
#
# 静态目标消息
# 静态目标指行驶环境中静止的、但可能影响主车运动特性的目标，包括路障、陷坑、隔离带等
# 也包括环境中的交通标识牌、停止线等
#
# 该消息内整合ADAS地图、目标检测（类别、位置）的信息
#
# 所有输出目标信息均基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]
# 速度和加速度数据为相对值，以主车为原点视为静止

# 消息定义
Header                      header                      # Header.stamp: ROS时间戳
                                                        # Header.frame_id: 数据来源

uint32                      id                          # 目标id
uint8                       label                       # 标签，预定义    

cta_msgs/Point3dF64         position                    # 目标位置
cta_msgs/Vector3d           orientation                 # 目标朝向
cta_msgs/Vector3d           size                        # 目标3轴尺寸

bool                        ispassable                  # 是否可通行

string[]                    usrinfo                     # 用户自定义信息