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

## Script to read data from the sonar sensor


import sys
import math
import rospy
from enum import Enum
from hk_msgs.msg import sonar as sonar_msg
from hk_msgs.msg import user_mode as user_mode_msg
from hk_msgs.srv import*
#import smbus,time,datetime
from datetime import datetime, timedelta
from smbus import SMBus
import numpy as np
import time


MSG_FAULTY_SONAR = -100 #Error message sent to the sensor fusion node
values = np.zeros(5)


###### FIXED PARAMETERS #####

# minimum wait time after a burst before we try to read
# This is not needed according to the specs. It is rather a workaround to prevent IOErrors that occur
# when we read although the measurement is not done yet.
MIN_TIME_BETWEEN_BURST_READ = 0.065

# From the data sheet: Do not initiate a ranging faster than every 65mS to give the previous burst time to fade away.
# specs say: 65ms
MIN_TIME_BETWEEN_BURSTS = 0.065

# specs say: 70ms
MAX_WAIT_TIME = 0.070

# expected error (in meters) of the reported distance if the reported distance is lower or near to the current minimum range (approx. 15cm)
SENSOR_ERROR_LOW_RANGE = 15

# expected error (in meters) of the reported distance if the reported distance is between the current minimal range (approx. 15cm) and MAX_RANGE
SENSOR_ERROR_GOOD_RANGE = 1

# expected error (in meters) of the reported distance if the reported distance is higher than MAX_RANGE
SENSOR_ERROR_HIGH_RANGE = 100

#Maximum expected error that might possibly happen 
SENSOR_ERROR_MAX = 1000000


#TODO: Should be set by rosparam
# maximum distance that can be measured in centimeter
MAX_RANGE = 500


# Exception :
class State(Enum):
	Not_read = 0
	Semi_automatic = 1
	Manual = -1

class Faulty_sonar(Exception):
	def __init__(self) :
		self.msg_code = MSG_FAULTY_SONAR

# Class to communicate to the sonar :
class Sonar:
	'Class to interact with the sonar'
	def __init__(self,address,sonar_node_nb,rate,max_range) :
		'Check that the sonar sensor is working before creating an object'
		self.max_range = max_range
		self._i2c = SMBus(1)
		self._address = address
		self._waiting_for_echo = False
		yesterday = datetime.now() - timedelta(1)
        	self._time_last_burst = yesterday

		# The last distance measurement
	        self.distance = None  # meters

		# On power up, the detection threshold is set to 28cm (11")
        	self.mindistance = 28  # cm
		
		#This is mostly for debugging and testing
		self.num_bursts_sent = 0
		
		# We want to check how often we get exceptions
		self.error_counter = 0

		self._ros_rate = rospy.Rate(rate)
		self.node_state = State.Not_read
		self.dist_to_obj = 0
		self.was_in_semi_auto = True

		#check that the sensor is present - read the version
		self.verion = self._read_version()

	def start_init(self) :

#		rospy.loginfo("The address used for sonar_" + sonar_node_nb + " is :"  + str(self._address))
		rospy.loginfo("Starting node sonar_" + sonar_node_nb)
		rospy.loginfo("Checking if the sonar nb " + sonar_node_nb + " is working...")

		failure_count = 0

		for tick in range(rate) : # initialization phase during 1s,
			distance, error = self.update()

#			rospy.loginfo("The distance is :" + str(distance))
			if error == SENSOR_ERROR_LOW_RANGE or error == SENSOR_ERROR_HIGH_RANGE:
				failure_count += 1

			self._ros_rate.sleep()

		if failure_count > 10 :
			raise Faulty_sonar


	def process_data(self,pub) :
		'Reads the data from the sonar_sensor and sends it on its topic'
		global values
		med = 0
		#values = np.zeros(5)
		msg = sonar_msg()
		if self.node_state == State.Semi_automatic :
			rospy.logwarn("The semi-automatic mode is now active")
			self.was_in_semi_auto = True
		while (not rospy.is_shutdown()) and (self.node_state == State.Semi_automatic) :
	   
      			distance, error = self.update()
			#rospy.loginfo("Distance: " + str(distance) + "  ERROR: " + str(error))
			if error==SENSOR_ERROR_GOOD_RANGE: #Take the value only if it's in the good range
				values = np.roll(values,1)
				values[0] = float(distance)
				med = np.median(values)
			#rospy.loginfo(values)


			if error == SENSOR_ERROR_LOW_RANGE:
				raise Faulty_sonar()

			elif error == SENSOR_ERROR_HIGH_RANGE:

				self.dist_to_obj = 0
				msg.dist_to_obj = self.dist_to_obj
#				rospy.loginfo("sonar_" + sonar_node_nb + " talking : distance to obstacle : OUT OF RANGE")
				pub.publish(msg)
			else :
				self.dist_to_obj = med
				msg.dist_to_obj = med
#				rospy.loginfo("sonar_" + sonar_node_nb + " talking : distance to obstacle : " + str(med))
				pub.publish(msg)

			self._ros_rate.sleep()

	def read_topic_user_mode(self,msg) :
		if msg.resend_data == 1 :
			to_msg = sonar_msg()
			to_msg.dist_to_obj = self.dist_to_obj
			pub.publish(to_msg)
		else :
			if msg.semi_automatic == 0 :
				self.node_state = State.Manual
			else :
				self.node_state = State.Semi_automatic




    # Should be called in some sensor loop, maybe at 100Hz. Is fast.
    # Will trigger an ultrasonic burst or check if we received an echo.
    # I we have a measurement, it is returned in meters.
	def update(self):

        	distance = None

	        now = datetime.now()
        	time_since_last_burst = (now - self._time_last_burst).total_seconds()

	        if self._waiting_for_echo:
            # make sure we wait at least some amount of time before we read
        	    if time_since_last_burst > MIN_TIME_BETWEEN_BURST_READ:
                # check if we have an echo
                	distance = self._read_echo()

            # Fallback if we don't get an echo, just stop waiting
            # from the data sheet:
            # The SRF02 will always be ready 70mS after initiating the ranging.
	            if distance is None and time_since_last_burst > MAX_WAIT_TIME:
                	#log.warn("Fallback! Waited longer than 70ms!")
        	        self._waiting_for_echo = False

	        if (not self._waiting_for_echo) and time_since_last_burst > MIN_TIME_BETWEEN_BURSTS:
        	    self._send_burst()

	        expected_error = self._calc_expected_error(distance)

        	return distance, expected_error

	def _send_burst(self):
        	self._i2c.write_byte_data(self._address, 0, 0x50) #0x51 in cm (Give stange values don't know why)  / 0x50 in inches
	        self._waiting_for_echo = True
        	self._time_last_burst = datetime.now()
	        self.num_bursts_sent += 1
        	#log.debug("Burst sent.")

	def _read_echo(self):
        # it must be possible to read all of these data in 1 i2c transaction
        # buf[0] software version. If this is 255, then the ping has not yet returned
        # buf[1] unused
        # buf[2] high byte range
        # buf[3] low byte range
        # buf[4] high byte minimum auto tuned range
        # buf[5] low byte minimum auto tuned range

        # We use the version information to detect if the result is there yet.
        # 255 is a dummy version for the case that no echo has been received yet. For me, the real version is "6".
        	if self._read_version() == 255:
	            #log.debug("Version is 255")
	            return None

        	self.distance = self._i2c.read_word_data(self._address, 2) / 255 *2.54
	        self.mindistance = self._i2c.read_word_data(self._address, 4) / 255 *2.54

	        # A value of 0 indicates that no objects were detected. We prefer None to represent this.
        	if self.distance == 0:
	            self.distance = None

        	self._waiting_for_echo = False
        #log.debug("echo received! distance is: {}".format(self.distance))
	
	        return self.distance

    # The version can be read from register 0.
    # Reading it has no real value for us, but we can use it to determine if a measurement is finished or not.
	def _read_version(self):
        	try:
	            return self._i2c.read_byte_data(self._address, 0)

            # 255 means that the unit is still measuring the distance
        	except IOError:
            #log.error("Recovering from IOError")
	            self.error_counter += 1
        	    return 255

    # find out what kind of error we expect (used in sensor fusion)
	def _calc_expected_error(self, distance):

        # no reading at all
        	if distance is None:
	            return SENSOR_ERROR_MAX

        # object too close
        	if distance <= self.mindistance:
	            return SENSOR_ERROR_LOW_RANGE

        # good distance, nice measurement
        	elif distance <= MAX_RANGE:
	            return SENSOR_ERROR_GOOD_RANGE

        # object too far
        	else:
	            return SENSOR_ERROR_HIGH_RANGE
 


if __name__ == '__main__':
	sonar_node_nb = sys.argv[1] #User input
	rospy.init_node('sonar_' + sonar_node_nb, anonymous = True) #ROS init
	msg = sonar_msg()
	pub = rospy.Publisher('topic_sonar_' + sonar_node_nb, sonar_msg, queue_size = 10)
	
	#Get params
	rate = rospy.get_param("sensors/sonar/reading_rate",25)
	retry_rate = rospy.get_param("sensors/sonar/retry_rate",0.2)
	sleep_rate = rospy.get_param("sensors/general/sensors_node_sleep_rate",1)
	max_range = rospy.get_param("/sensors/sonar/max_range",600)


	

	#I2C node address
	string_addr = '0x7' + sonar_node_nb 
	hex_addr = int(string_addr, 16)
	hex_addr += 1

	fatal_init = False

	### INITIALIZATION PHASE ###
	sonar = Sonar(hex_addr,sonar_node_nb,rate,max_range)
	try :
		sonar.start_init()

	except Faulty_sonar as err :
		rospy.logfatal("The node sonar_" + sonar_node_nb + "has NOT initialized successfully")
		msg.dist_to_obj = err.msg_code
		pub.publish(msg)
		fatal_init = True
	
	except:
		rospy.logfatal("The node sonar_" + sonar_node_nb + "has failed to start")
		msg.dist_to_obj = MSG_FAULTY_SONAR
		pub.publish(msg)
		fatal_init = True
	else :	
		rospy.loginfo("Node sonar_" + sonar_node_nb + " has initialized successfully")
		msg.dist_to_obj = sonar.dist_to_obj
		pub.publish(msg)

	### SENSOR FUSION NODE ACK ###
	# Confirming to the sensors_fusion node that it has finished its initialization
	rospy.wait_for_service('service_init_confirmation')
	rospy.loginfo("Node sonar_" + sonar_node_nb + " is waiting for the initialization of the other nodes")
	while not rospy.is_shutdown() :
		try :
			increment = rospy.ServiceProxy('service_init_confirmation', init_confirmation)
			increment()
			break
		except rospy.ServiceException, e:
			pass

	rospy.Subscriber("topic_user_mode", user_mode_msg, sonar.read_topic_user_mode)
	ros_rate = rospy.Rate(rate)
	while not rospy.is_shutdown() :
		ros_rate.sleep()
		if sonar.node_state != State.Not_read :
			break

	if fatal_init :
		rospy.logfatal("The sensor sonar_" + sonar_node_nb + "is not working")
		rospy.logfatal("Exiting the node" + sonar_node_nb + "definitely")
		sys.exit()

	### READING PHASE ###
	try:
		rospy.loginfo("sonar_" + sonar_node_nb + " : starting processing data")
		ros_retry_rate = rospy.Rate(retry_rate)
		ros_sleep = rospy.Rate(sleep_rate)

		while not rospy.is_shutdown() :

			try :
				sonar.process_data(pub)

			except Faulty_sonar as err :
				rospy.logerr("Sensor sonar_" + sonar_node_nb + " has stopped working")
				rospy.logerr("Retrying in " + str(1/retry_rate) + "secs")
				msg.dist_to_obj = err.msg_code
				pub.publish(msg)
				ros_retry_rate.sleep()
			else :
				if sonar.was_in_semi_auto :
					rospy.logwarn("The semi-automatic mode is now deactivated")
					sonar.was_in_semi_auto = False
				ros_sleep.sleep()
	except rospy.ROSInterruptException :
		pass

