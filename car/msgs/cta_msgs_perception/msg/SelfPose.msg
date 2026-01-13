# 车辆自身位姿消息
# 根据传感器数据计算得到，数据来源可能为gnss，imu，lidar，camera
#
# 输出信息基于导航坐标系： 东（+X）-北（+Y）-天（+Z）坐标系
# 姿态角定义：滚转角（roll）绕X轴，俯仰角（pitch）绕Y轴，偏航角（yaw）绕Z轴
# 姿态角正旋方向服从右手定则；
# 姿态角取值范围：roll = (-pi/2, +pi/2), pitch = (-pi/2, pi/2), yaw = (-pi, +pi]

# 消息定义
Header                      header          # Header.stamp: ROS时间戳
                                            # Header.frame_id: 姿态数据来源，下示
                                            # "ins":    来自于gnss和imu
                                            # "lidar":  来自于lidar
                                            # "camera": 来自于camera

cta_msgs/GPSPoint           orggps          # 坐标原点经纬高信息

cta_msgs/GPSPoint           gps             # GPS记录信息
cta_msgs/MotionState3d      state           # 3轴运动信息

