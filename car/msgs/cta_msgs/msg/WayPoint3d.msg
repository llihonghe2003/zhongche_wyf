# 参考：autoware/autoware_msgs/Waypoint.msg
# 3d道路锚点，每一个点代表一个实际的道路点信息


uint32          roadid          # 道路id
int32           laneid          # 车道id
Point3dF64      pos             # 3轴位置
Vector3d        rot             # 3轴姿态角
float64         vel             # 参考速度
float64         curv            # 曲率绝对值

