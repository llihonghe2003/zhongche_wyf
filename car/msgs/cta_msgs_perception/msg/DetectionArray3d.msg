# 参考： autoware/autoware_msgs/ImageObj.msg
#
# 3D目标检测
#
# 所有数据基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]


# 消息定义
Header                      header                      # Header.stamp: 获得目标时刻
                                                        # Header.frame_id: 数据来源（算法名称）


uint8                       label                       # 标签，预定义
cta_msgs/DetectBox3d[]      boxes                       # 目标检测框列表