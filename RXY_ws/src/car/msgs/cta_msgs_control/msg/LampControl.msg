# 灯光控制消息，只用于控制车辆的灯光

# 消息定义
Header                  header              # Header.stamp: 控制时刻
                                            # Header.frame_id: 控制算法模块名称

bool                    leftture            # 左转灯
bool                    rightturn           # 右转灯

bool                    outline             # 示廓灯
bool                    shortlamp           # 近光转灯
bool                    farlamp             # 远光转灯
bool                    foglamp             # 雾灯灯

bool                    warninglamp         # 报警灯
bool                    brakelamp           # 刹车灯
bool                    backofflamp         # 倒车灯

bool                    whistle             # 鸣笛
bool                    wiper               # 雨刷
