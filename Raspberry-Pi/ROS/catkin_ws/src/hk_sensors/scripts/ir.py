#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# 
# Script to read data from the ir sensor GP2Y0A02YK 

import time
# used to retrieve command line arguments
import sys
# Import the ADS1x15 module.
import Adafruit_ADS1x15

#Math library
import math

import rospy
import numpy as np
from hk_msgs.msg import ir as ir_msg
from hk_msgs.msg import user_mode as user_mode_msg
from hk_msgs.srv import *
from enum import Enum



# expected error (in meters) of the reported distance if the reported distance is lower or near to the current minimum range (approx. 15cm)
SENSOR_ERROR_LOW_RANGE = 15

# expected error (in meters) of the reported distance if the reported distance is between the current minimal range (approx. 15cm) and SRF02_MAX_RANGE
SENSOR_ERROR_GOOD_RANGE = 1

# expected error (in meters) of the reported distance if the reported distance is higher than SRF02_MAX_RANGE
SENSOR_ERROR_HIGH_RANGE = 100

#Maximum expected error that might possibly happen 
SENSOR_ERROR_MAX = 1000000


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
MSG_FAULTY_IR = - 100
NB_OF_FAULTY_MEASURES = 20
NB_RESET = 50

values = np.zeros(10)

class State(Enum):
	Not_read = 0
	Semi_automatic = 1
	Manual = -1

class Faulty_ir(Exception):
	def __init__(self) :
		self.msg_code = MSG_FAULTY_IR

class Ir:
	'Class to interact with the ir sensor'
	
	def __init__(self, ir_node_nb,rate,ref_voltages) :
		'Check that the ir sensor is working before creating the object'
		
		self.ros_rate = rospy.Rate(rate)
		self.ref_voltages = ref_voltages
		self.ADC_channel_nb = int(ir_node_nb)
		self.node_state = State.Not_read		
		self.dist_to_obj = 0
		self.was_in_semi_auto = True #consider as in semi-auto by default
		

	def start_init(self) :
		rospy.loginfo("Starting node ir_" + ir_node_nb)
		rospy.loginfo("Checking if the ir nb " + ir_node_nb + "is_working...")
		failure_count = 0
				
		for time in range(rate) : #initialization phase during 1s

			distance, error = self.read_sensor()
			
			#if the voltage is out of range, the sensor is not working properly
			if error == SENSOR_ERROR_LOW_RANGE:
				failure_count += 1

			self.ros_rate.sleep()
			
			# otherwise, the sensor is NOT working	
		if failure_count > 5 :
			raise Faulty_ir()
		
			# sensor is working if no more than 3 measures are out of range
		else :
			self.dist_to_obj = distance
		

	def process_data(self,pub):
		global values
		'Reads the data from the ir_sensor and sends it on its topic'
		failure_count = 0
		loop_count = 0
		msg = ir_msg()
		if self.node_state == State.Semi_automatic :
			rospy.logwarn("The semi-automatic mode is now active")
			self.was_in_semi_auto = True

		while (not rospy.is_shutdown()) and (self.node_state == State.Semi_automatic) :
			if failure_count > NB_OF_FAULTY_MEASURES:
				raise Faulty_ir()
			
			distance, error = self.read_sensor()

			#if the voltage is out of range, the sensor is not working properly
			if error == SENSOR_ERROR_MAX:
				failure_count += 1

			elif error == SENSOR_ERROR_HIGH_RANGE: #zone empty
				self.dist_to_obj = 0
				msg.dist_to_obj = self.dist_to_obj
#				rospy.loginfo("ir_" + str(self.ADC_channel_nb) + " talking : distance to obstacle : OUT OF RANGE") 
				pub.publish(msg) 
			#An object has been detected. Compute at which distance and send a message	
			else :
				values = np.roll(values,1)
				values[0] = distance
				med = np.median(values) 
				self.dist_to_obj = med
				msg.dist_to_obj = self.dist_to_obj
				pub.publish(msg) 

			if loop_count >= NB_RESET:
				loop_count = 0
				failure_count = 0

			loop_count += 1		
			self.ros_rate.sleep()


	def read_sensor(self):
		adc = Adafruit_ADS1x15.ADS1015()
		voltage = adc.read_adc(self.ADC_channel_nb,gain = GAIN)*4.096/2047

		if voltage < self.ref_voltages['faulty_low'] or voltage > self.ref_voltages['faulty_high']:
			error = SENSOR_ERROR_MAX
			distance = -1
		elif voltage < self.ref_voltages['no_object']:
			error = SENSOR_ERROR_HIGH_RANGE
			distance = 0 
		else:
			error = SENSOR_ERROR_GOOD_RANGE
			distance = math.exp((1/voltage+0.4776)/0.193) 

		return distance, error			

	def read_topic_user_mode(self,msg) :
		'Callback function for the topic_user_mode used to assess whether the semi-auto or manual mode is used'
		'If a message resend_data -1 is read, this node resends its init message, whatever it was'
		if msg.resend_data == 1 :
			to_msg = ir_msg()
			to_msg.dist_to_obj = self.dist_to_obj
			pub.publish(to_msg)
		else :
			if msg.semi_automatic == 0 :
				self.node_state = State.Manual
			else :
				self.node_state = State.Semi_automatic

if __name__ == '__main__':
	
	
	# SETTINGS THE PROPER VARIABLES
	
	ir_node_nb = sys.argv[1]
	
	rospy.init_node('ir_' + ir_node_nb, anonymous=True)
	
	msg = ir_msg()
	pub = rospy.Publisher('topic_ir_' + ir_node_nb, ir_msg,queue_size=10)


	rate = rospy.get_param("sensors/ir/reading_rate",25) #refresh at 25Hz if not provided otherwise
	retry_rate = rospy.get_param("sensors/ir/retry_rate",0.2) #if not provided otherwise, retry at 0.2 Hz to read data from the faulty sensor
	sleep_rate = rospy.get_param("/sensors/general/sensors_nodes_sleep_rate",1)

	voltage_fail_low = rospy.get_param("sensors/ir/voltage_fail_low",0.2) #consider the lowest output voltage to be 0.2 if not provided otherwise
	voltage_fail_high = rospy.get_param("sensors/ir/voltage_fail_high",3) #consider the highest output voltage to be 3 if not provided otherwise
	voltage_no_object = rospy.get_param("sensors/ir/voltage_no_object",0.5) #consider the no-object voltage to be 0.5 if not provided otherwise
	ref_voltages = {'faulty_low' : voltage_fail_low,'faulty_high' : voltage_fail_high, 'no_object' : voltage_no_object}
	
	fatal_init = False


	# INITIALIZATION PHASE
	
	ir = Ir(ir_node_nb,rate,ref_voltages)
        try :
		ir.start_init()
		
        except Faulty_ir as err :
		rospy.logfatal("Node ir_" + ir_node_nb + " has NOT initialized successfully")
		msg.dist_to_obj = err.msg_code
		pub.publish(msg)
		fatal_init = True
	except:
		rospy.logfatal("Node ir_" + ir_node_nb + " has failed to start")
		msg.dist_to_obj = MSG_FAULTY_IR
		pub.publish(msg)
		fatal_init = True 
		
        else : 
		rospy.loginfo("Node ir_" + ir_node_nb + " has initialized successfully")
		msg.dist_to_obj = ir.dist_to_obj
		pub.publish(msg)
	
#	INITIALIZATION OF THAT NODE DONE, WAITING FOR THE OTHER NODES INITIALIZATION TO FINISH AND TO RECEIVE WHETHER THE USER WANTS TO USE THE COLLISION AVOIDANCE ALGORITHM OR NOT
	# Confirming to the sensors_fusion node that it has finished its initialization

	rospy.wait_for_service('service_init_confirmation')
	rospy.loginfo("Node ir_" + ir_node_nb + " is waiting for the initialization of the other nodes")

	while not rospy.is_shutdown() :
		try :
			increment = rospy.ServiceProxy('service_init_confirmation', init_confirmation)
			increment()
			break
		except rospy.ServiceException, e:
			pass

	rospy.Subscriber("topic_user_mode", user_mode_msg, ir.read_topic_user_mode)
	ros_rate = rospy.Rate(rate)

	while not rospy.is_shutdown() :
		ros_rate.sleep()
		if ir.node_state != State.Not_read :
			break

	if fatal_init :
		rospy.logfatal("The sensor ir_" + ir_node_nb + " is not working")
		rospy.logfatal("Exiting the node ir_" + ir_node_nb  + " definitely")
		
	# READING PHASE
	
        try :
		rospy.loginfo("ir_" + ir_node_nb + " : starting processing data")

		# Try reading data from the ir. If the ir is working, process_data() reads the data at 'rate' frequency and 
		# transmits it on its associated topic at frequency 'rate/5'
		# If the ir is not working, Faulty_ir exception is raised and the node tries at 'retry_rate' frequency 
		# to process the data again (in case it miraculously starts reworking)

		ros_retry_rate = rospy.Rate(retry_rate)
		ros_sleep = rospy.Rate(sleep_rate)

		while not rospy.is_shutdown() :

			try :
			
				ir.process_data(pub)
			
			except Faulty_ir as err:
			
				rospy.logerr("Sensor ir_" + ir_node_nb + " has stopped working")
				rospy.logerr("Retrying in " + str(1/retry_rate) + "secs")
				msg.dist_to_obj = err.msg_code
				pub.publish(msg)
				ros_retry_rate.sleep()
			else :
				if ir.was_in_semi_auto :
					rospy.logwarn("The semi-automatic mode is now deactivated")
					ir.was_in_semi_auto = False
				ros_sleep.sleep()
				
        except rospy.ROSInterruptException :
	    pass
		
		
