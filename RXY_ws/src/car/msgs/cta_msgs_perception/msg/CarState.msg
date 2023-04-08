# 车辆自身状态消息
# 根据车辆线控地盘返回得CAN报文解析得到

# 消息定义
Header                      header                  # Header.stamp: ROS时间戳
                                                    # Header.frame_id: 数据来源

uint64                      carid                   # 车辆编号
bool                        isempowered             # 是否已授权

uint8                       workmode                # 工作模式
bool                        isauto_steer            # 转向自动
bool                        isauto_brake            # 刹车自动
bool                        isauto_gear             # 档位自动
bool                        isauto_throttle         # 油门自动
bool                        isauto_park             # 驻车自动
bool                        isauto_lamp             # 灯光自动

uint16                      errcode                 # 错误代码

bool                        state_ispark            # 实时状态：是否驻车
bool                        state_isrun             # 实时状态：启动/停止
bool                        state_isstop            # 实时状态：急停开关

int8                        state_gear              # 实时状态：档位
float64                     state_throttle          # 实时状态：油门开度 %
float64                     state_brake             # 实时状态：刹车压力 Mpa
float64                     state_steerangle        # 实时状态：方向盘转角 deg
float64                     state_steerspeed        # 实时状态：方向盘转速 deg/s
float64                     state_speed             # 实时状态：车速 m/s
float64                     state_odem              # 实时状态：行驶里程 km

bool                        lamp_leftturn           # 灯光状态：左转向
bool                        lamp_rightturn          # 灯光状态：右转向
bool                        lamp_outline            # 灯光状态：示宽灯
bool                        lamp_shortlamp          # 灯光状态：近光灯
bool                        lamp_farlamp            # 灯光状态：远光灯
bool                        lamp_foglamp            # 灯光状态：雾灯
bool                        lamp_brakelamp          # 灯光状态：刹车灯
bool                        lamp_backofflamp        # 灯光状态：倒车灯
bool                        lamp_warninglamp        # 灯光状态：报警灯
bool                        lamp_whistle            # 灯光状态：鸣笛
bool                        lamp_wiper              # 灯光状态：雨刷器


