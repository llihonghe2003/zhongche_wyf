# -*- coding: UTF-8 -*-
# 根据给出的型值点列反算B样条曲线的控制多边形，并画出均匀B样条曲线验证计算结果
import numpy as np
from scipy import linalg
import matplotlib.pyplot as plt
import math

c = []

# 反算控制多边形，Px，Py为已知的型值点列，dP_0_x, dP_0_y, dP_n_x,dP_n_y为端点处的一阶导数值
# 返回方程组的系数矩阵和右端常数项
def cal_back(Px, Py, dP_0_x, dP_0_y, dP_n_x, dP_n_y):
    n = len(Px) - 1
    A = np.zeros(shape=[n + 3, n + 3])  # 构造线性方程组的系数矩阵，除了即将赋值的元素，其余元素皆为0
    bx = np.zeros(n + 3)  # 构造方程组右端横坐标的常数值
    by = np.zeros(n + 3)  # 构造方程组右端纵坐标的常数值
    # 给系数矩阵和右端常数项赋值，第一行到第n+1行
    for i in range(n + 1):
        A[i][i] = 1
        A[i][i + 1] = 4
        A[i][i + 2] = 1
        bx[i] = Px[i]
        by[i] = Py[i]
    # 给系数矩阵赋值，第n+1行和n+2行
    A[n + 1][0] = 2
    A[n + 1][1] = 4
    A[n + 2][n + 1] = 4
    A[n + 2][n + 2] = 2
    # 计算方程右端最后两个值
    bx[n + 1] = Px[0] - 1 / 3 * dP_0_x
    bx[n + 2] = Px[n] + 1 / 3 * dP_n_x
    by[n + 1] = Py[0] - 1 / 3 * dP_0_y
    by[n + 2] = Py[n] + 1 / 3 * dP_n_y
    return A, bx, by


# 三次均匀B样条矩阵表达式，根据给定的控制顶点（x，y）画出三次均匀B样条曲线
# 使用该函数检验反算控制顶点的结果是否正确，如果曲线经过型值点，证明结果没有问题
def uniformBspline(x, y):

    # 将x,y转换成能够计算的数组
    x = np.array(x)
    y = np.array(y)
    # 输入矩阵
    A = [[1, 4, 1, 0], [-3, 0, 3, 0], [3, -6, 3, 0], [-1, 3, -3, 1]]
    A = np.array(A)

    # l为0和1之间的100个点
    l = np.linspace(0, 1, 450)
    # 初始化两个全零向量存放图像的横纵坐标
    RX = []
    RY = []
    rx = np.zeros(450)
    ry = np.zeros(450)

    # 每4个控制顶点确定一段函数，一共len(x) - 3段
    for i in range(len(x) - 3):
        for u in l:
            j = list(l).index(u)  # j为u在l中的索引值
            U = np.array([1, u, u**2, u**3])
            R = np.dot(1 / 6 * U, A)
            # 利用矩阵公式计算图像上的若干个点并拟合成一条曲线
            rx[j] = np.dot(
                R, np.array([x[i], x[i + 1], x[i + 2], x[i + 3]]).T
            )  # 三次曲线每次用到4个控制顶点
            ry[j] = np.dot(R, np.array([y[i], y[i + 1], y[i + 2], y[i + 3]]).T)

        print(rx, ry)
        RX.extend(rx)
        RY.extend(ry)
        plt.plot(rx, ry)  # 画出第i+1段曲线

    draw(RX, RY)


def draw(x, y):
    # plt.plot(x, y)
    # plt.show()
    h = []
    for m in range(0, len(x) - 1):
        h0 = math.atan2((y[m + 1] - y[m]), (x[m + 1] - x[m]))
        h.append(float(h0))
    fo = open("/home/xtark/RXY_ws/src/route/foo222.txt", "w")
    for t in range(0, len(x) - 1):
        fo.write(str(x[t] * 0.6))
        fo.write(" ")
        fo.write(str(y[t] * 0.6 + 0.6))
        fo.write(" ")
        fo.write(str(h[t]))
        fo.write(" ")
        fo.write("0")
        fo.write("\n")
    fo.close()


def main(Task):
    plt.figure()
    # Px = [0, 1, 2, 3, 4, 5]
    # Py = [1, 4, 2, 5, 1, 3]
    Px = []
    Py = []
    print(len(Task[0]))
    # 反算过程给定的型值点（插值点）
    for i in range(0, len(Task[0]) // 2):
        Px.append(Task[0][i * 2])
        Py.append(Task[0][i * 2 + 1])

    # 两端点处的一阶导数
    dP_0_x = 1
    dP_0_y = 1
    dP_n_x = -1
    dP_n_y = -1
    A, bx, by = cal_back(Px, Py, dP_0_x, dP_0_y, dP_n_x, dP_n_y)
    # 计算控制顶点的横纵坐标
    print(np.linalg.det(A))
    # a = []
    # for i in range(len(A)):
    #     a.append(0.000001)
    # c = np.diag(a)
    vx = linalg.solve(1 / 6 * A, bx)
    vy = linalg.solve(1 / 6 * A, by)
    # draw(vx, vy)
    plt.plot(vx, vy)  # 画出控制多边形
    # plt.show()
    plt.scatter(vx, vy, s=30)  # 描出控制多边形顶点
    # plt.show()
    uniformBspline(vx, vy)  # 画出由上述控制顶点确定的均匀B样条曲线
    # plt.show()
    plt.scatter(Px, Py)  # 描出型值点，如果曲线经过型值点说明反算结果正确
    plt.show()


if __name__ == "__main__":
    Task = []
    # task = [0, 1, 1, 4, 2, 2, 3, 5, 4, 1, 5, 3]
    task = [1, 0, 3, 1, 5, 1, 10, 3]

    Task.append(task)
    main(Task)
