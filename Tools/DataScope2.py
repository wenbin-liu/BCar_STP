import collections 
import serial
from pyqtgraph.Qt import QtGui,QtCore
import pyqtgraph as pg
import time
import numpy as np
buff=b''
graphNum=2

win=pg.GraphicsWindow()
curve=win.addPlot()
try:
    ser=serial.Serial('COM4',baudrate=115200)
except Exception:
    win.close()
    exit()
ser.readline()
data=[collections.deque(maxlen=100)]*graphNum
def update():
    global data,buff
    global ser
    if ser.inWaiting()==0:
        return
    lines = ser.readlines(5)
    lines[0]=buff+lines[0]
    if lines[-1].decode("ASCII")[-1]=='\n':
        buff=b''
    else:
        buff=lines[-1]
        lines.pop()
    for line in lines:
        try:
            string=line.decode("ASCII").strip().split()
            if(len(data[0])==data[0].maxlen):
                data[0].popleft()
                data[1].popleft()
            data[0].append(float(string[0]))
            data[1].append(float(string[1]))
            print(line)
            arr=np.array(data[0])
            curve.setData(arr)
        except (IndexError,ValueError):
            print("Lost one package")
            continue
def main():
    # while True:
    #     update()
    #     time.sleep(0.05)
    timer=pg.QtCore.QTimer()
    timer.timeout.connect(update)
    timer.start(50)
    QtGui.QApplication.instance().exec_()
if __name__=='__main__':
    main()


