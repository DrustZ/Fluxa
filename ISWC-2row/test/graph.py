import serial
import numpy
import matplotlib.pyplot as plt
from quaternion import Quaternion
from vector import Vector
from drawnow import *
import math
import time

sensor = serial.Serial('/dev/cu.usbserial-AH02LQSB',19200)

g = [0,0,0]
v = Vector(0,0,0)
xs, ys, zs = [0],[0],[0]
lastv = [Vector(0,0,0),Vector(0,0,0),Vector(0,0,0)]
cnt = 0
timElapse = 0
vx,vy,vz = 0,0,0
interval = 0

line, = plt.plot([0]*50)

def makeFig():
    plt.ylim([-10,10])
    plt.title('My Live Sweet Data :P')
    plt.grid(True)
    plt.ylabel('Acc')
    plt.plot(xs,'r', label='Acc X')
    plt.legend(loc='upper left')
    plt.plot(ys,'g', label='Acc Y')
    plt.legend(loc='upper left')
    plt.plot(zs,'b', label='Acc Z')
    plt.legend(loc='upper left')

while True:
    try:
        msg = sensor.readline()
        msg = msg.strip()
        # tmp = msg.split('~')  
        # # print msg  
        # if len(tmp) != 8:
        #     continue
        # q = Quaternion(tmp[0],tmp[1],tmp[2],tmp[3])
        # v = Vector(tmp[4],tmp[5],tmp[6])
        # v = q.rotate(v)

        # alpha = 0.9
        # #high-pass filter
        # g[0] = alpha * g[0] + (1-alpha)*v.x
        # g[1] = alpha * g[1] + (1-alpha)*v.y
        # g[2] = alpha * g[2] + (1-alpha)*v.z

        # v.x -= g[0]
        # v.y -= g[1]
        # v.z -= g[2]

        # if abs(v.x) < 0.5:
        #     v.x = 0
        # if abs(v.y) < 0.5:
        #     v.y = 0
        # if abs(v.z) < 0.5:
        #     v.z = 0
        # if v.x == 0 and v.y == 0 and v.z == 0:
        #     timElapse += 1
        # else:
        #     timElapse = 0
        # if timElapse == 3:
        #     timElapse = 0
        #     vx = 0
        #     vy = 0
        #     vz = 0

        # # print("%.2f %.2f %.2f" % (v.x, v.y, v.z))
        # lastv.pop(0);
        # lastv.append(v)
        # cnt += 1
        # if cnt < 3:
        #     continue
        # else:
        #     cnt = 0
        #     interval += float(tmp[7])

        # vx += (lastv[0].x + lastv[1].x*4 + lastv[2].x) / 6 * interval;  
        # vy += (lastv[0].y + lastv[1].y*4 + lastv[2].y) / 6 * interval;  
        # vz += (lastv[0].z + lastv[1].z*4 + lastv[2].z) / 6 * interval;  
        # interval = 0
        xs.append(0)
        ys.append(float(msg))
        zs.append(0)

        drawnow(makeFig)
        # makeFig()

        if len(xs) > 50:
            xs.pop(0)
            ys.pop(0)
            zs.pop(0)

    except Exception, e:
        print 'all oh!'