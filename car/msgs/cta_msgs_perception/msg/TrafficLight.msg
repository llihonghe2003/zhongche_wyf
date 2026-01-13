# 参考： autoware/autoware_msgs/TrafficLightResult.msg
#
# 交通信号灯消息
#
# 所有输出目标信息均基于车载坐标系：前（+X）-左（+Y）-上（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]


# 消息定义
Header                      header                      # Header.stamp: ROS时间戳
                                                        # Header.frame_id: 数据来源

uint32                      id                          # 信号灯id

cta_msgs/Point3dF64         position                    # 信号灯位置
cta_msgs/Vector3d           size                        # 信号灯尺寸
uint8                       color                       # 信号灯颜色，预定义

bool                        ispassable                  # 是否可通行

uint32                      roadid                      # 信号灯有效道路id
int32[]                     laneids                     # 信号灯有效车道id（可多个）

string[]                    usrinfo                     # 用户自定义信息