# 参考： autoware/autoware_msgs/WaypointArray.msg
#
# 全局路线2D，坐标系为本地东-北导航坐标系，平面


# 消息定义
Header                      header                      # Header.stamp: 获得全局路线时刻
                                                        # Header.frame_id: 数据来源（算法名称）

cta_msgs/Point2dF64         startpos                    # 起点位置
cta_msgs/Point2dF64         goalpos                     # 目标位置
cta_msgs/WayPoint2d[]       waypoints                   # 道路锚点列表
