# Simple demo of reading each analog input from the ADS1x15 and printing it to
# the screen.
# Author: Tony DiCola
# License: Public Domain
import time

# Import the ADS1x15 module.
import Adafruit_ADS1x15

#Math library
import math


# Create an ADS1115 ADC (16-bit) instance.
#adc = Adafruit_ADS1x15.ADS1115()

# Or create an ADS1015 ADC (12-bit) instance.
adc = Adafruit_ADS1x15.ADS1015() 
minDis = [50,50,50,50] 
maxDis = [500,500,500,999] 
minVolt = [-2,-2,-2,-2] 
iter = 10
# Note  you can change the I2C address from its default (0x48), and/or the I2C
# bus by passing in these optional parameters:
#adc = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

# Choose a gain of 1 for reading voltages from 0 to 4.09V.
# Or pick a different gain to change the range of voltages that are read:
#  - 2/3 = +/-6.144V
#  -   1 = +/-4.096V
#  -   2 = +/-2.048V
#  -   4 = +/-1.024V
#  -   8 = +/-0.512V
#  -  16 = +/-0.256V
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 1

print('Reading ADS1x15 values, press Ctrl-C to quit...')
# Print nice channel column headers.
print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*range(4)))
print('-' * 37)
# Main loop.
while True:
    average = [0]*4
    #average_1 = 0
    # Filtering loop (average of 10 samples)
    for f in range(iter):
    	# Read all the ADC channel values in a list.
    	values = [0]*4
    	for i in range(4):
        	# Read the specified ADC channel using the previously set gain value.
        	values[i] = adc.read_adc(i, gain=GAIN)*4.096/2047
		while (values[i]<minVolt[i]):
			time.sleep(0.04)
			values[i] = adc.read_adc(i, gain=GAIN)*4.096/2047
			
		# Note you can also pass in an optional data_rate parameter that controls
        	# the ADC conversion time (in samples/second). Each chip has a different
        	# set of allowed data rate values, see datasheet Table 9 config register
        	# DR bit values.
        	#values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        	# Each value will be a 12 or 16 bit signed integer value depending on the
        	# ADC (ADS1015 = 12-bit, ADS1115 = 16-bit).
    	# Print the ADC values.
    	#if (values[0] > 3):
        #        values[0] = 10
    	#elif (values[0] < 0.4):
        #        values[0] = 999
    	#else:
        #values[0] = (1/values[0]+0.0073)/0.017

    	#if (values[1] > 3):
	#	values[1] = 10
    	#elif (values[1] < 1.4):
	#	values[1] = 999
    	#else:
	for i in  range(4):		
		values[i] = math.exp((1/values[i]+0.4776)/0.193)
		average[i] = average[i] + values[i]
	#average_1 = average_1 + values[1]
	time.sleep(0.04)
   	
	for i in range(4):
		values[i] = average[i]/iter
    	#values[1] = average_1/iter

    for i in range(len(values)):
	if (values[i] > maxDis[i]):
		values[i]=999
	elif (values[i] < minDis[i]):
		values[i]=0


    print('| {0:>6} | {1:>6} | {2:>6} | {3:>6} |'.format(*values))
    # Pause for half a second.
    time.sleep(0.05)
