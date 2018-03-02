import pyqtgraph as pg
import numpy as np
from pyqtgraph.Qt import QtCore, QtGui
import serial

interval=20

win=pg.GraphicsWindow()
win.setWindowTitle("DataScope")
p1=win.addPlot()
p1.showGrid(True,True)
# p2=win.addPlot(row=1,col=0)
# p2.showGrid(True,True)
data=np.array([[0.0]*400]*3)
curve1=p1.plot(pen='r')
curve2=p1.plot(pen='g')
curve3=p1.plot(pen='b')
pos=0
buff=b''

A=np.mat([[1,-0.005],[0,1]])
B=np.mat([[0.005],[0]])
Q=np.mat([[0.001,0.0],[0.,0.003]])
R=np.mat([0.3514])
H=np.mat([1,0])
x_hat=np.mat(np.zeros((2,1)))

K=np.mat(np.zeros((2,1)))
P=np.mat([[1.0,0.0],[0.0,1.0]])

def KalmanFilter(angle,gyro):
    global P,K,Q,R,x_hat,H,B
    P_bar=A*P*A.T+Q
    K=P_bar*H.T*(H*P_bar*H.T+R).I
    x_bar=A*x_hat+gyro*B
    x_hat=x_bar+K*(angle-H*x_bar)
    P=(np.eye(2)-K*H)*P_bar
    return x_hat
    
    

def dataLoad():
    global data,buff
    val=[0.0,0.0,0.0]
    if s.inWaiting()==0:
        return
    lines=s.readlines()
    lines[0]=buff+lines[0]
    if lines[-1].decode("ASCII")[-1]=='\n':
        buff=b''
    else:
        buff=lines[-1]
        lines.pop()
    for line in lines:
        string = line.decode("ASCII").strip().split()
#        print(line)
        try:
            val[0]=float(string[0])
            val[1]=float(string[1])
        except IndexError:
            print(line)
            continue

        except ValueError:
            print(line)
            continue
        #一阶滤波
        val[2]=0.05*val[0]+0.95*(data[2][-1]+val[1]*0.005)
        #kalman滤波
        KalmanFilter(val[0],val[1])
       
        val[0]=x_hat[0]
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
    curve3.setData(data[2])
    # curve.setPos(pos,0)
    
with serial.Serial("COM4",baudrate=115200) as s:
    s.timeout=0
    s.readline()
    timer= pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(interval)
    QtGui.QApplication.instance().exec_()    # pos+=1
