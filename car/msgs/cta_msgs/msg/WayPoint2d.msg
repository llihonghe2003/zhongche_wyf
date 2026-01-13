# 参考：autoware/autoware_msgs/Waypoint.msg
# 2d道路锚点，每一个点代表一个实际的道路点信息,视路面为平整的z=0


uint32          roadid          # 道路id
int32           laneid          # 车道id
Point2dF64      pos             # 2轴位置
float64         rot             # 航向角
float64         vel             # 参考合速度
float64         curv            # 曲率
float64         pla             # 角速度

