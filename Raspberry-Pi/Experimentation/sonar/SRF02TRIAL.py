# Take readings from the I2C SRF02 Ultrasound range sensor

from datetime import datetime, timedelta
from smbus import SMBus
import time
import numpy as np



# The ultrasonic range finder SR02's i2c address as reported by "i2detect -y 1"
# For me, this is 0x70 or
SRF02_I2C_ADDRESS = 0x76

# minimum wait time after a burst before we try to read
# This is not needed according to the specs. It is rather a workaround to prevent IOErrors that occur
# when we read although the measurement is not done yet.
SRF02_MIN_TIME_BETWEEN_BURST_READ = 0.065

# From the data sheet: Do not initiate a ranging faster than every 65mS to give the previous burst time to fade away.
# specs say: 65ms
SRF02_MIN_TIME_BETWEEN_BURSTS = 0.065

# specs say: 70ms
SRF02_MAX_WAIT_TIME = 0.070

# maximum distance that can be measured in centimeter
SRF02_MAX_RANGE = 600

# expected error (in meters) of the reported distance if the reported distance is lower or near to the current minimum range (approx. 15cm)
SRF02_SENSOR_ERROR_LOW_RANGE = 15

# expected error (in meters) of the reported distance if the reported distance is between the current minimal range (approx. 15cm) and SRF02_MAX_RANGE
SRF02_SENSOR_ERROR_GOOD_RANGE = 1

# expected error (in meters) of the reported distance if the reported distance is higher than SRF02_MAX_RANGE
SRF02_SENSOR_ERROR_HIGH_RANGE = 100

#Maximum expected error that might possibly happen 
SENSOR_ERROR_MAX = 1000000


class SRF02:
    def __init__(self):

        self._i2c = SMBus(1)
        self._i2c_address = SRF02_I2C_ADDRESS
        self._waiting_for_echo = False

        yesterday = datetime.now() - timedelta(1)
        self._time_last_burst = yesterday

        # The last distance measurement
        self.distance = None  # meters

        # On power up, the detection threshold is set to 28cm (11")
        self.mindistance = 28  # meters

        # This is mostly for debugging and testing
        self.num_bursts_sent = 0

        # check that the sensor is present - read the version
        self.version = self._read_version()
        #log.info("SRF-02 Ultrasonic Sensor initialized. Version: %d" % self.version)

        # we want to check how often we get exceptions
        self.error_counter = 0

    # Should be called in some sensor loop, maybe at 100Hz. Is fast.
    # Will trigger an ultrasonic burst or check if we received an echo.
    # I we have a measurement, it is returned in meters.
    def update(self):

        distance = None

        now = datetime.now()
        time_since_last_burst = (now - self._time_last_burst).total_seconds()

        #        log.debug("time since last burst: {}".format(time_since_last_burst))

        if self._waiting_for_echo:
            # make sure we wait at least some amount of time before we read
            if time_since_last_burst > SRF02_MIN_TIME_BETWEEN_BURST_READ:
                # check if we have an echo
                distance = self._read_echo()

            # Fallback if we don't get an echo, just stop waiting
            # from the data sheet:
            # The SRF02 will always be ready 70mS after initiating the ranging.
            if distance is None and time_since_last_burst > SRF02_MAX_WAIT_TIME:
                #log.warn("Fallback! Waited longer than 70ms!")
                self._waiting_for_echo = False

        if (not self._waiting_for_echo) and time_since_last_burst > SRF02_MIN_TIME_BETWEEN_BURSTS:
            self._send_burst()

        expected_error = self._calc_expected_error(distance)

        return distance, expected_error

    def _send_burst(self):
        self._i2c.write_byte_data(self._i2c_address, 0, 0x50) #0x51 in cm (Give stange values don't know why)  / 0x50 in inches
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

        self.distance = self._i2c.read_word_data(self._i2c_address, 2) / 255 *2.54
        self.mindistance = self._i2c.read_word_data(self._i2c_address, 4) / 255 *2.54

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
            return self._i2c.read_byte_data(self._i2c_address, 0)

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
            return SRF02_SENSOR_ERROR_LOW_RANGE

        # good distance, nice measurement
        elif distance <= SRF02_MAX_RANGE:
            return SRF02_SENSOR_ERROR_GOOD_RANGE

        # object too far
        else:
            return SRF02_SENSOR_ERROR_HIGH_RANGE




if __name__ == '__main__':
    sonar1 = SRF02()
    values = np.zeros(5)
    while True:
      distance, error = sonar1.update()
      time.sleep(0.1)
      if error==SRF02_SENSOR_ERROR_GOOD_RANGE:
      	values = np.roll(values,1)
	values[0] = float(distance)
      med = np.median(values)
      #print(values)
      print("Distance in cm  " + str(med) + "  ERROR:  " + str(error))
