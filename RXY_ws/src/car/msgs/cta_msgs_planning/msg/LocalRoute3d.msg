# 参考： autoware/autoware_msgs/WaypointArray.msg
#
# 局部路线，坐标系为本地东-北-天导航坐标系


# 消息定义
Header                      header                      # Header.stamp: 获得路线时刻
                                                        # Header.frame_id: 数据来源（算法名称）

uint8                       sampletype                  # 采样方式：0 等时采样；1 等距采样
float64                     samplestep                  # 采样步长：等时采样该值为时间间隔，单位【s】；等距采样该值为点距离，单位【m】

cta_msgs/WayPoint3d[]       waypoints                   # 道路锚点列表
