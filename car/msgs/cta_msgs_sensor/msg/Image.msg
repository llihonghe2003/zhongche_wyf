# 参考： ros/sensor_msg/Image.msg
#
# 定义无压缩图片消息
#
# 图像坐标系（x，y），原点位于图像左上方，水平向右为+x，竖直向下为+y
# （0,0）——————————————————————————————> [+x] 
#   |               |
#   |   ————————pixel[y][x]——————————
#   |               |
#   v
#  [+y]


# 消息定义
Header              header      # header.stamp: ROS时间戳，接收图像数据时刻
                                # header.frame_id: 图像来源相机名称

uint8               encoding    # 图像编码格式，预定义

uint32              height      # 高度，图像行数
uint32              width       # 宽度，图像列数
uint32              channel     # 通道，图像深度

uint8[]             data        # 图像数据，size = ((channel * width) * height)