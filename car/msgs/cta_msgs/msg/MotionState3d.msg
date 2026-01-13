# 运动状态，3轴
# 坐标为换算到大地坐标系下得XYZ坐标;姿态角为东北天转车体前右上得姿态角
# 速度和加速度指沿或绕车体前右上三轴的速度

Point3dF64     pos                  # 中心位置
Vector3d       rot                  # 3轴旋转姿态角

Vector3d       vel                  # 3轴线速度
Vector3d       acc                  # 3轴线加速度
Vector3d       rotvel               # 3轴角速度
Vector3d       rotacc               # 3轴角加速度