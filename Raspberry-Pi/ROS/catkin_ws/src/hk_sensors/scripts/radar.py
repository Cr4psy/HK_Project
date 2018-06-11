#!/usr/bin/env  python


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
# Revision $Id$

## Script to read data from the radar

import sys
import math
import rospy
import os
from enum import Enum
from serial import Serial
import struct
from datetime import datetime, timedelta
from hk_msgs.msg import radar as radar_msg
from hk_msgs.msg import user_mode as user_mode_msg
from hk_msgs.srv import *
import numpy as np

import time



## Parameters, should be included in ROS params ##
#TODO: Put them in ros param
# maximum distance that can be measured in centimeter
#Not the actual value but define as the max
#MAX_RANGE = 800

#Minimum distance that can be measured in centimeter
#MIN_RANGE = 30


## Local fixed variable ##

# expected error (in meters) of the reported distance if the reported distance is lower or near to the current minimum range (approx. 15cm)
SENSOR_ERROR_LOW_RANGE = 15
# expected error (in meters) of the reported distance if the reported distance is between the current minimal range (approx. 15cm) and MAX_RANGE
SENSOR_ERROR_GOOD_RANGE = 1
# expected error (in meters) of the reported distance if the reported distance is higher than MAX_RANGE
SENSOR_ERROR_HIGH_RANGE = 100
#Maximum expected error that might possibly happen 
SENSOR_ERROR_MAX = 1000000


MSG_FAULTY = -100
NB_OF_FAULTY_MEASURES = 20
NB_RESET = 50

values = np.zeros(10)

# Exception :
class State(Enum):
	not_read = 0
	semi_automatic = 1
	manual = -1


class Faulty_radar(Exception):
	def __init__(self) :
		self.msg_code = MSG_FAULTY

# Class to communicate with the radar

class Radar:
    def __init__(self, rate,max_range,min_range):
        #self.__serial = Serial('/dev/ttyAMA0', 115200,8, 'N',1)
        #self.__serial.flushInput()
        #self.__serial.flushOutput()
	self.max_range = max_range
	self.min_range = min_range
	self.__ros_rate = rospy.Rate(rate)
	self.node_state = State.not_read
	self.dist_to_obj = 0
	self.was_in_semi_auto = True
	
    def start_init(self) :
	self.__serial = Serial('/dev/ttyAMA0', 115200,8, 'N',1)
	self.__serial.flushInput()
	self.__serial.flushOutput()


	failure_count = 0
    	for tick in range(rate) : # initialization phase during 1s,
			dis, err = self.read_bus()
	#			rospy.loginfo("The distance is :" + str(distance))
			if err == SENSOR_ERROR_MAX :
				failure_count += 1
			self.__ros_rate.sleep()

			if failure_count > 10 :
				raise Faulty_radar

    def process_data(self,pub):
		global values
		failure_count = 0
		loop_count = 0

		if self.node_state == State.semi_automatic:
			self.was_in_semi_auto = True

		while (not rospy.is_shutdown()) and (radar.node_state == State.semi_automatic):
			if failure_count > NB_OF_FAULTY_MEASURES:	
				raise Faulty_radar()

			dis, err = self.read_bus() # return the distance
			#print("Distance: " + str(dis) + "ERROR: " +str(err))

			if err == SENSOR_ERROR_MAX :
				failure_count += 1
			elif err == SENSOR_ERROR_LOW_RANGE:
				pass
			elif err == SENSOR_ERROR_HIGH_RANGE:
				self.dist_to_obj = 0
				msg.dist_to_obj = self.dist_to_obj
				pub.publish(msg)
			else :
				values = np.roll(values,1)
				values[0] = dis
				med = np.median(values) #take the median value of the 10 previous measurment.
#				rospy.loginfo("0 talking : distance to obstacle : " + str(med))
				self.dist_to_obj = med
				msg.dist_to_obj = self.dist_to_obj
				pub.publish(msg)

			if loop_count >= NB_RESET:
				loop_count = 0
				failure_count = 0

			loop_count += 1
			self.__ros_rate.sleep()


    def read_bus(self):
		nb_err = 0
		while (not rospy.is_shutdown()):
		   byte = struct.unpack('B', self.__serial.read(1))[0]

		   start = datetime.now()
		   while(byte != 0xFE): #Wait until it gets a signal
		        byte = struct.unpack('B', self.__serial.read(1))[0]
         	   	if (datetime.now()-start).total_seconds() > 2:
				distance = -1
 				error = SENSOR_ERROR_MAX
				rospy.logfatal("Watchdog communication time with radar")
				return distance, error 
		   byte2 = struct.unpack('B', self.__serial.read(1))[0]
	           byte3 = struct.unpack('B', self.__serial.read(1))[0]
		   byte4 = struct.unpack('B', self.__serial.read(1))[0]
	       	   byte5 = struct.unpack('B', self.__serial.read(1))[0]
		   byte6 = struct.unpack('B', self.__serial.read(1))[0]

	           header = byte
		   version_id = byte2
	           lsb_byte = byte3
	           msb_byte = byte4
	           snr = byte5
	           check_sum_transmitter = byte6
	           check_sum_receiver = (byte2 + byte3 + byte4 + byte5) & 0xFF
	           distance = byte3 + (byte4<<8)
		   self.__serial.flushInput()
		   self.__serial.flushOutput()
		   #print("Distance read_bus : " + str(distance))
	           if (check_sum_receiver == check_sum_transmitter):
#	          		rospy.loginfo( "checksum passed, ALTITUDE:  " + str(distance) + ", SNR:" + str(snr) + '\r\n')
					if distance <=  self.min_range:
						error = SENSOR_ERROR_LOW_RANGE
					elif distance > self.max_range:
						error = SENSOR_ERROR_HIGH_RANGE
					else:
						error = SENSOR_ERROR_GOOD_RANGE
	          			return distance, error
	           else:
#	          		rospy.logwarn( "checksum failed!!!" + '\r\n')
	          		nb_err += 1

		  		if nb_err == 5:	# Five consecutive error checksum
					distance = -1
					error = SENSOR_ERROR_MAX
					return distance, error

    def read_topic_mode(self,msg) :
	    if msg.resend_data == 1:
			to_msg = radar_msg()
			to_msg.dist_to_obj = self.dist_to_obj
			pub.publish(to_msg)
	    else:
	    	if msg.semi_automatic == 0 :
	 			self.node_state = State.manual
	    	else :
				self.node_state = State.semi_automatic


if __name__ == '__main__':
	os.system("sudo chmod 777 /dev/ttyAMA0") #Enable serial modification
    	node_nb = sys.argv[1]
    # ROS nodes
	rospy.init_node('radar_' + node_nb, anonymous = True)
	msg = radar_msg() #decalre msg type
	pub = rospy.Publisher('topic_radar_' + node_nb, radar_msg, queue_size=10)

	rate = rospy.get_param("sensors/radar/reading_rate",25)
	retry_rate = rospy.get_param("sensors/radar/retry_rate",0.2)
	sleep_rate = rospy.get_param("sensors/radar/sensors_nodes_sleep_rate", 1)
	max_range = rospy.get_param("/sensors/radar/max_range",10000)
	min_range = rospy.get_param("/sensors/radar/min_range",30)

    # INITIALIZATION phase

	fatal_init = False

	radar = Radar(rate,max_range,min_range)
    	try:
    		radar.start_init()
    	except Faulty_radar as err:
			rospy.logfatal("Node radar_ " + node_nb + " has NOT initialized successfully")
			msg.dist_to_obj = err.msg_code
			pub.publish(msg)
			fatal_init = True
	except:
			rospy.logfatal("Node radar_" + node_nb + " has failed to start")
			msg.dist_to_obj = MSG_FAULTY
			pub.publish(msg)
			fatal_init = True
    	else:
			rospy.loginfo("Node rada_" + node_nb +" has initialized successfully")
			msg.dist_to_obj = radar.dist_to_obj
			pub.publish(msg)
		

        # Waiting for acknowledgment from user_mode_msg
	rospy.loginfo("Wait for service initialisation")
	rospy.wait_for_service('service_init_confirmation')
	rospy.loginfo("Node " + node_nb + " is waiting for the initialization of the other nodes")
	while not rospy.is_shutdown():
		try :
			increment = rospy.ServiceProxy('service_init_confirmation', init_confirmation)
			increment()
			break
		except rospy.ServiceException, e:
			pass

	rospy.Subscriber("topic_user_mode", user_mode_msg, radar.read_topic_mode)
	ros_rate = rospy.Rate(rate)
	ros_sleep = rospy.Rate(sleep_rate)

	while not rospy.is_shutdown() : # Wait until it get an answer
		ros_rate.sleep()
		if radar.node_state != State.not_read:
			break

	if fatal_init :
		rospy.logfatal("The sensor " + node_nb + " is not working")
		rospy.logfatal("Exiting the node " + node_nb + " definitely")
		sys.exit()
	# READING PHASE
	try:
		rospy.loginfo("" + node_nb + " : starting processing data")
		ros_retry_rate = rospy.Rate(retry_rate)
		while not rospy.is_shutdown():
			try:
				radar.process_data(pub)
			except Faulty_radar as err: #If there are error msg then retry after 5sec
				rospy.logerr("Sensor " + node_nb + " has stopped working")
				rospy.logerr("Retrying in " + str(1/retry_rate) + "secs")
				msg.dist_to_obj = err.msg_code
				pub.publish(msg)
				ros_retry_rate.sleep()
			else:
				if radar.was_in_semi_auto :
					rospy.logwarn("The semi-automatic mode is now deactivated")
					radar.was_in_semi_auto = False
				ros_sleep.sleep()
	except rospy.ROSInterruptException :
		pass
