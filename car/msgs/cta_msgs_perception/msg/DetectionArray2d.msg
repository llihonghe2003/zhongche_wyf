# 参考： autoware/autoware_msgs/ImageObj.msg
#
# 2D目标消检测
#
# 图像坐标系（x，y），原点位于图像左上方，水平向右为+x，竖直向下为+y
# （0,0）——————————————————————————————> [+x] 
#   |               |
#   |   ————————pixel[y][x]——————————
#   |               |
#   v
#  [+y]


# 消息定义
Header                      header                      # Header.stamp: 获得目标时刻
                                                        # Header.frame_id: 数据来源（算法名称）


# uint8                       label                       # 标签，预定义
# cta_msgs/DetectBox2d[]      boxes                       # 目标检测框列表


# 修改by lei_meg 为了相机使用 修改message格式

uint8[]                     label                       # 标签，预定义
cta_msgs/DetectBox2d[]      boxes                       # 目标检测框列表


float32[]                   identities                  # id Deepsort能够得到图像序列中的跟踪信息 可以直接确定一个id出来