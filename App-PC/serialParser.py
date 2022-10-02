import serial
import numpy as np

GROUPS_COUNT = 8
AXIS_COUNT = 3

ser = serial.Serial('COM5', 115200)

arr = np.empty((GROUPS_COUNT, AXIS_COUNT))

while(1):
    
    line = str(ser.readline(), 'UTF-8')
    
    arr[int(line[4])] = line[7:].removesuffix('\r\n').split('\t')

    print(arr[1], '\t', arr[5])