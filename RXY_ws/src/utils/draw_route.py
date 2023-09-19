#! python3
from tokenize import Number
import matplotlib.pyplot as plt
import numpy as np
import math

R=1.5
x=[]
y=[]
tt=[]
theta=[]


for t in range(0,1800):
    
    x.append(0.9+R+R*math.sin(0.2*t/180*3.14))
    y.append(0.9+R-R*math.cos(0.2*t/180*3.14))
    theta.append(0.2*t/180*3.14)
    tt.append(t/180*3.14)
# plt.plot(x, y)  
# # plt.plot(tt, y)  
# # plt.plot(tt, x)  
# plt.show()


def draw():
    # plt.plot(x, y)
    plt.plot(x, y)
    plt.show()
    fo = open("/home/yhs/xtark_01/RXY_ws/src/route/yuan.txt", "w")
    for t in range(0, len(x)):
        fo.write(str(x[t]))
        fo.write(" ")
        fo.write(str(y[t]))
        fo.write(" ")
        fo.write(str(theta[t]))
        fo.write(" ")
        fo.write("0")
        fo.write("\n")
    fo.close()
draw()