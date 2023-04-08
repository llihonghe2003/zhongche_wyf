# 参考： ros/sensor_msg/PointCloud.msg
#
# 3D点云消息定义
# 每个坐标点为float32点类型，并记录反射强度


# 消息定义
Header                      header      # Header.stamp: ROS时间戳，接收到点云时刻
                                        # Header.frame_id: 激光雷达设备名称

cta_msgs/Point3dF32[]       data        # 点云列表

float32[]                   intensity   # 激光反射强度