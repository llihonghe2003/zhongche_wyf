# 存储当前检测到的所有交通信号灯信息
# 交通信号灯属性参考 TrafficLight.msg

# 消息定义
Header                      header          # Header.stamp: ROS时间戳
                                            # Header.frame_id: 数据来源
                                                        
TrafficLight[]              lights          # 交通信号灯数据