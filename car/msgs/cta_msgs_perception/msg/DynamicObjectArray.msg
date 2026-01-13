# 存储当前检测到的所有动态目标信息
# 动态目标属性参考 DynamicObject.msg

# 消息定义
Header                      header          # Header.stamp: ROS时间戳
                                            # Header.frame_id: 数据来源

DynamicObject[]             objs            # 动态障碍物数据