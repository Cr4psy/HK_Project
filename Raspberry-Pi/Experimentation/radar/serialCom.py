from serial import Serial
import struct
import numpy as np
import time


serial = Serial('/dev/ttyAMA0', 115200,8, 'N',1)
serial.flushInput()
serial.flushOutput()

values = np.zeros(5)



while(1):
  time.sleep(0.1)
  byte = 0
  byte = struct.unpack('B', serial.read(1))[0]
  if(byte == 0xFE):
    byte2 = struct.unpack('B', serial.read(1))[0]
    byte3 = struct.unpack('B', serial.read(1))[0]
    byte4 = struct.unpack('B', serial.read(1))[0]
    byte5 = struct.unpack('B', serial.read(1))[0]
    byte6 = struct.unpack('B', serial.read(1))[0]

    header = byte
    version_id = byte2
    lsb_byte = byte3
    msb_byte = byte4
    snr = byte5
    check_sum_transmitter = byte6
    check_sum_receiver = (byte2 + byte3 + byte4 + byte5) & 0xFF
    ALTITUDE = byte3 + (byte4<<8)
    serial.flushInput()
    serial.flushOutput()
    if (check_sum_receiver == check_sum_transmitter):
      #print "checksum passed, ALTITUDE:  " + str(ALTITUDE) + ", SNR:" + str(snr) + '\r\n'
      values = np.roll(values,1)
      values[0] = float(ALTITUDE)
      med = np.median(values)
      print "distance: " + str(med)
    else:
      print "checksum failed!!!" + '\r\n'
  else:
    continue
