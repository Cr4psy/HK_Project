#This program read the value of the maxbotic sonar through Serial

#On 7/13/15 John Bohlhuis contribute his own Python code that he is using to automate the reading of distance values. 
#We gladly share this for your use in your projects. 
#Thank you John for sharing. 


#***************************************************************************
#First, the Python module that does the actual work:
#***************************************************************************

#!/usr/bin/python3
# Filename: maxSonarTTY.py

# Reads serial data from Maxbotix ultrasonic rangefinders
# Gracefully handles most common serial data glitches
# Use as an importable module with "import MaxSonarTTY"
# Returns an integer value representing distance to target in millimeters

from time import time
from serial import Serial

serialDevice = "/dev/ttyAMA0" # default for RaspberryPi
maxwait = 3 # seconds to try for a good reading before quitting

def measure(portName):
    ser = Serial(portName, 9600, 8, 'N', 1, timeout=1)
    timeStart = time()
    valueCount = 0

    while time() < timeStart + maxwait:
        if ser.inWaiting():
            bytesToRead = ser.inWaiting()
 	    #print(bytesToRead)
            valueCount += 1
            if valueCount < 2: # 1st reading may be partial number; throw it out
                continue
            testData = ser.read(bytesToRead)
            if not testData.startswith(b'R'):
                print('data received did not startd with R')
                continue
            try:
                sensorData = testData.decode('utf-8').lstrip('R')
		#print(sensorData)
            except UnicodeDecodeError:
                print('data received could not be decoded properly')
                continue
            try:
                mm = int(sensorData)
		#print(mm)
            except ValueError:
                print("value is not a number")
                continue
            ser.close()
            return(mm)

    ser.close()
    raise RuntimeError("Expected serial data not received")

if __name__ == '__main__':
    while 1:
    	measurement = 2.54*measure(serialDevice)
    	print("distance =",measurement)


