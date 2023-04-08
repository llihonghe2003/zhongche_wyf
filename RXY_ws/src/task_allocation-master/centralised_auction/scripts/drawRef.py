#! python3
from tokenize import Number
import matplotlib.pyplot as plt
import numpy as np
import math


# x1 = 1
# y1 = 1

# x2 = 9
# y2 = 9

# x3 = 6
# y3 = 6

# taskList = [1, 0, 5, 4, 7, 3, 3, 1, 0, 1, 4, 6]

x = []
y = []
h = []


x3 = []
y3 = []
h3 = []

dt = 0.1
v = 0.2


def getLine(x1, y1, x2, y2):
    dis = math.sqrt(math.pow((x1 - x2), 2) + math.pow((y1 - y2), 2))

    for t in np.arange(0, 1, dt * v / dis):
        x0 = (x2 - x1) * t + x1
        y0 = (y2 - y1) * t + y1
        h0 = math.atan2((y2 - y1), (x2 - x1))
        x0 = "%.4f" % x0
        y0 = "%.4f" % y0
        h0 = "%.4f" % h0
        # print (type(x0))
        # print (type(y0))
        x.append(float(x0))
        y.append(float(y0))
        h.append(float(h0))
    return x, y


def draw():
    # plt.plot(x, y)
    plt.plot(x, y)
    plt.show()
    fo = open("/home/xtark/RXY_ws/src/route/foo333.txt", "w")
    for t in range(0, len(x)):
        fo.write(str(x[t]))
        fo.write(" ")
        fo.write(str(y[t] + 0.6))
        fo.write(" ")
        fo.write(str(h[t]))
        fo.write(" ")
        fo.write("0")
        fo.write("\n")
    fo.close()


def plan(task):
    print(len(task[0]) / 2)
    for i in range(0, len(task[0]) / 2 - 1):
        x, y = getLine(
            task[0][i * 2] * 0.6,
            task[0][i * 2 + 1] * 0.6,
            task[0][i * 2 + 2] * 0.6,
            task[0][i * 2 + 3] * 0.6,
        )

    print(x)
    print(y)
    draw()


def planB(task):

    for i in range(0, len(task) / 2 - 1):
        x, y = getLine(task[i * 2], task[i * 2 + 1], task[i * 2 + 2], task[i * 2 + 3])
    print(x)
    print(y)
    draw()


def printList(l):
    print("\n")
    print(len(l))
    for i in l:
        print(i)
    print("\n")


def planC():
    xx = []
    yy = []
    hh = []

    for i in range(0, 400):
        xx.append(0.02 * i)
        yy.append(2)
        hh.append(0)

    fo1 = open("/home/xtark/RXY_ws/src/route/foo333.txt", "w")
    for t in range(0, 400):
        fo1.write(str(xx[t]))
        fo1.write(" ")
        fo1.write(str(yy[t]))
        fo1.write(" ")
        fo1.write(str(hh[t]))
        fo1.write(" ")
        fo1.write("0")
        fo1.write("\n")
    fo1.close()


def main(l):
    # draw(l)
    # printList(l)
    # plan(l)  # 运行程序
    # print(l)
    # planB(taskList)
    planC()


if __name__ == "__main__":
    l = 1
    main(l)
