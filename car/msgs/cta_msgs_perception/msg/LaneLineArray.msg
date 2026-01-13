# 存储当前检测到的所有车道线信息
# 车道线属性参考 LaneLine.msg

# 消息定义
Header                      header          # Header.stamp: ROS时间戳
                                            # Header.frame_id: 数据来源
                                                        
LaneLine[]                  lines           # 车道线数据，以车体为基准，从左向右排列  