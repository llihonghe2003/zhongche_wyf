# 车道线消息，采用拟合多项式表示
#
# 所有输出目标信息均基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]


# 消息定义
Header                      header                      # Header.stamp: ROS时间戳
                                                        # Header.frame_id: 数据来源

int32                       laneid                      # 车道线所属车道id

cta_msgs/PolyLine2d         line                        # 车道线多项式表示
uint8                       linetype                    # 车道线线型
uint8                       linecolor                   # 车道线颜色


string[]                    usrinfo                     # 用户自定义信息