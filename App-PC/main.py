import serial

ser = serial.Serial('COM3', 9600, timeout=1)

while(1):
    
    line = ser.readline()
    
    print(line)