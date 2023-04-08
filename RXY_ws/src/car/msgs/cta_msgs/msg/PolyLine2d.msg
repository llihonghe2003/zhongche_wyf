# 2d多项式曲线

cta_msgs/Point2dF64     start                   # 起始点坐标
cta_msgs/Point2dF64     end                     # 终止点坐标
uint32                  order                   # 多项式阶次
float64[]               param                   # 多项式系数，升阶排列
                                                # y = param[0] + param[1]*x + param[2]*x^2 + ... + param[order]*x^order
