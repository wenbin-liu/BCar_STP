import pyqtgraph as pg
import pyqtgraph.opengl as gl
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import serial
import sys


interval=30

win=pg.GraphicsWindow()
win.setWindowTitle("DataScope")
p1=win.addPlot()
p1.showGrid(True,True)
# p2=win.addPlot(row=1,col=0)
# p2.showGrid(True,True)
data=np.array([[0.0]*400]*3)
curve1=p1.plot(pen='r')
curve2=p1.plot(pen='g')
#curve3=p1.plot(pen='b')


boxwin=gl.GLViewWidget()
box=gl.GLBoxItem()
box.scale(0.5,0.8,1)
tr=box.transform()
boxwin.addItem(box)

ax=gl.GLAxisItem()
boxwin.addItem(ax)

boxwin.show()

pos=0
buff=b''

A=np.mat([[1,0.005],[0,1]])
Q=np.mat([[0.001,0.0],[0.,0.001]])
R=np.mat([[0.8514,0],[0,0.01281]])
x_hat=np.mat(np.zeros((2,1)))

K=np.mat(np.zeros((2,2)))
P=np.mat([[1.0,0.0],[0.0,1.0]])

def KalmanFilter(z):
    global P,K,Q,R,x_hat
    P_bar=A*P*A.T+Q
    K=P_bar*(P_bar+R).I
    x_bar=A*x_hat
    x_hat=x_bar+K*(z-x_bar)
    P=(np.eye(2)-K)*P_bar
    return x_hat
    
    

def dataLoad():
    global data,buff
    GYRO_BIAS=-0.0222
    val=[0.0,0.0,0.0]
    if s.inWaiting()==0:
        return
    lines=s.readlines()
    lines[0]=buff+lines[0]
    if lines[-1].decode("UTF-8")[-1]=='\n':
        buff=b''
    elif len(lines) == 1:
        buff=buff+lines[0]
        return
    else:
        buff=lines[-1]
        lines.pop()
    for line in lines:
        string = line.decode("UTF-8").strip().split()
        print(line.decode("UTF-8"))
        try:
            val[0]=float(string[0])
            val[1]=float(string[1])-GYRO_BIAS
        except (IndexError,ValueError):
            continue
            # print(line.decode().strip())
            # print(len(lines))
            # print(lines.index(line))

        #一阶滤波
#        val[2]=0.05*val[0]+0.95*(data[2][-1]+val[1]*0.005)
#        KalmanFilter(np.mat([val[0],val[1]]).T)
       
#        val[0]=x_hat[0]
#        val[1]=x_hat[1]
        data[0][:-1]=data[0][1:]
        data[1][:-1]=data[1][1:]
        data[2][:-1]=data[2][1:]
        data[0][-1]=val[0]
        data[1][-1]=val[1]
        data[2][-1]=val[2]
      #  print(val)


def update():
    dataLoad()
    global data
    global pos
    #print(data[-1])
    curve1.setData(data[0])
    curve2.setData(data[1])
    temp=pg.Transform3D(tr)
    temp.rotate(data[0][-1],0,1,0)
    box.setTransform(temp)
    if sys.stdin.buffer.peek()!='':
        a=input()
        s.write(a)
#    curve3.setData(data[2])
    # curve.setPos(pos,0)
try:    
    with serial.Serial("COM4",baudrate=115200) as s:
        s.timeout=0
        s.readline()
        timer= pg.QtCore.QTimer()
        timer.timeout.connect(update)
        timer.start(interval)
        QtGui.QApplication.instance().exec_()
except Exception:
    win.close()
