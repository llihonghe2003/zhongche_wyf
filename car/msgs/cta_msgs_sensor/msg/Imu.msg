# 参考： ros/sensor_msg/Imu.msg
#
# 惯性测量单元消息， IMU (Inertial Measurement Unit)
#
# 惯导坐标系采用 前（+X）-右（+Y）-下（+Z）坐标系
# 3轴姿态角为：滚转角roll，绕X轴；俯仰角pitch，绕Y轴；偏航角yaw，绕Z轴
# 


# 消息定义
Header                      header          # Header.stamp: ROS时间戳，接收IMU信息时刻
                                            # Header.frame_id: 接收IMU设备名称

cta_msgs/Vector3d           rotation        # 3轴姿态角，单位 [rad].

cta_msgs/Vector3d           gyro            # 3轴陀螺仪输出，单位 [rad/sec]

cta_msgs/Vector3d           accm            # 3轴加速度计输出，单位 [m/s^2]