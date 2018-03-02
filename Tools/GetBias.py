import string
import serial
import matplotlib.pyplot as plt 
s = serial.Serial("COM10")
strArray=["","","",""]
sum = 0
i=times=500
s.readline()
s.readline()
while i>0:
    i=i-1
    str = s.readline().decode("ASCII").strip()
    strArray = str.split()
    #Ax = int(strArray[1])
    Gx = int(strArray[0])
    sum+=Gx
print("Gyro biase")
print(sum/times)