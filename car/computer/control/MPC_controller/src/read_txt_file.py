from os import sep
# from typing_extensions import runtime
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import mpl_toolkits.axisartist as ast



# export DISPLAY=:0.0
path = "/home/xtark/ros_ws/src/car/computer/control/MPC_controller/trajectory/ch_data.txt"
df_news = pd.read_csv(path, sep='\s', header=None,engine='python')
df_news = pd.DataFrame(df_news)
df_news
x = df_news[0]
y = df_news[1]
yaw = df_news[2]
v = df_news[3]
w = df_news[4]
xr = df_news[5]
yr = df_news[6]
yawr = df_news[7]
vc = df_news[8]    
wc = df_news[9]   
delta = df_news[10]   

x_avo = df_news[11]    
y_avo = df_news[12]   
theta_avo = df_news[13]   
v_avo = df_news[14]   

r1=0.5
r2=0.2

t=np.arange(0,2*np.pi,0.01)

x_ob=x_avo[0]+0.5+r1*np.cos(t) * np.cos(np.pi/4*5)-r2*np.sin(t) * np.sin(np.pi/4*5)
y_ob=y_avo[0]+r1*np.cos(t)* np.sin(np.pi/4*5)+r2*np.sin(t) * np.cos(np.pi/4*5)








plt.subplots(1)
plt.plot(x, y, '-r', linewidth=3)
plt.plot(xr, yr, '-g', linewidth=3, linestyle='--')
plt.plot(x_ob, y_ob, '-r', linewidth=3, linestyle='--')
plt.plot(x_avo, y_avo, '-r', linewidth=3)
plt.ylabel('global trajectory', fontsize=14, alpha=0.5)

# plt.subplots(1)
# plt.plot(range(len(yaw)), yaw, '-r', range(len(yawr)), yawr, '-g')
# plt.ylabel('yaw', fontsize=14, alpha=0.5)

# plt.subplots(1)
# plt.plot(range(len(vc)), vc, '-r')
# plt.ylabel('v', fontsize=14, alpha=0.5)

# plt.subplots(1)
# plt.plot(range(len(wc)), wc, '-r')
# plt.ylabel('w', fontsize=14, alpha=0.5)


# plt.subplots(1)
# plt.plot(range(len(delta)), delta, '-r')
# plt.ylabel('delta', fontsize=14, alpha=0.5)



# plt.figure()
# plt.subplot(3, 1, 1)
# plt.plot(range(len(x)), (xr-x)*np.cos(yaw)+(yr-y)*np.sin(yaw), '-r')
# plt.ylabel('x error', fontsize=14, alpha=0.5)
# plt.subplot(3, 1, 2)
# plt.plot(range(len(y)), (yr-y)*np.cos(yaw)-(xr-x)*np.sin(yaw), '-r')
# plt.ylabel('y error', fontsize=14, alpha=0.5)
# plt.subplot(3, 1, 3)
# plt.plot(range(len(yaw)), yaw-yawr, '-r')
# plt.ylabel('yaw error', fontsize=14, alpha=0.5)

plt.show()
