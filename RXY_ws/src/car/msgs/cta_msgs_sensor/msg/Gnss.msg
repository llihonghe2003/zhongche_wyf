# 参考： ros/sensor_msg/NavSatFix.msg
#
# 全球导航卫星系统消息定义，（GNSS， Global Navigation Satellite System）
#
# 约定使用 WGS84 坐标系

# 消息定义
Header              header              # Header.stamp: ROS时间戳，接收GNSS消息时刻
                                        # Header.frame_id: 接收GNSS设备名称

uint32              satellitenumber     # 当前卫星数

cta_msgs/GPSPoint   gps                 # GPS坐标 单位 lat[deg] lon[deg] alt[m]

float64             velnorth            # 北向速度，北向为正，单位 [m/s]
float64             veleast             # 东向速度，东向为正，单位 [m/s]
float64             velground           # 地向速度，地向为正，单位 [m/s]