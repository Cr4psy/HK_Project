#!/usr/bin/env python
import time
import serial

ser = serial.Serial(
	port='/dev/ttyAMA0',
	baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=1
)
counter=0
#serialport = serial.Serial("/dev/ttyAMA0", 9600,8,'N',1, timeout=0.5)

while True:
	nbBytes = ser.inWaiting()
	if nbBytes>0:
		sensorData = ser.read(size=nbBytes)
		print(sensorData)
	#x=ser.readline()
        #print x
