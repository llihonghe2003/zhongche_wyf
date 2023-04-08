# 车辆决策行为


# 消息定义
Header                      header                      # Header.stamp: 决策时刻
                                                        # Header.frame_id: 数据来源（算法名称）

uint8                       scenario                    # 当前场景，预定义
uint8                       behavior                    # 当前行为，预定义

float64                     speed                       # by @lei_meg
cta_msgs/Point2dF64         stoppoint                   # stop point 