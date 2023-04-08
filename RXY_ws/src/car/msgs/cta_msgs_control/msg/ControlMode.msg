# 控制模式消息，只用于调整控制模式

# 消息定义
Header                  header              # Header.stamp: 时刻
                                            # Header.frame_id: 模块名称

uint8                   motionmode          # 运动控制模式
bool                    islampauto          # 灯光自动