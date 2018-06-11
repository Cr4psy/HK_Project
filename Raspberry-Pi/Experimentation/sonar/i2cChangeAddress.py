### This program is used to change the I2C address of the sonars

# Take readings from the I2C SRF02 Ultrasound range sensor
 # Address 8 bit -> 7 bit map
  
#  0xE0 -> 0x70
#  0xE2 -> 0x71  
#  0xE4 -> 0x72
#  0xE6 -> 0x73
#  0xE8 -> 0x74
#  0xEA -> 0x75
#  0xEC -> 0x76
#  0xEE -> 0x77
#  0xF0 -> 0x78
#  0xF2 -> 0x79
#  0xF4 -> 0x7A
#  0xF6 -> 0x7B
#  0xF8 -> 0x7C
#  0xFA -> 0x7D
#  0xFC -> 0x7E
#  0xFE -> 0x7F
import smbus,time,datetime

class srf02:
  def __init__(self, address):
    self.i2c = smbus.SMBus(1)
    # The address in hex, look it up in i2cdetect -y 1 if you're unsure
    self.addr = address #7bit address
  
  def getValues(self, numberOfValues):
    startTime = datetime.datetime.now()
    values = []

    for i in range(numberOfValues):
      self.i2c.write_byte_data(self.addr, 0, 81)
      time.sleep(0.08) # 80ms snooze whilst it pings
      #it must be possible to read all of these data in 1 i2c transaction
      #buf[0] software version. If this is 255, then the ping has not yet returned
      #buf[1] unused
      #buf[2] high byte range
      #buf[3] low byte range
      #buf[4] high byte minimum auto tuned range
      #buf[5] low byte minimum auto tuned range
      distance = self.i2c.read_word_data(self.addr, 2) / 255
      mindistance = self.i2c.read_word_data(self.addr, 4) / 255
      elapsed = datetime.datetime.now() - startTime
      values.append({"elapsed": elapsed, "distance": distance, "mindistance": mindistance}) 
      time.sleep(0.12) # 120ms snooze so we only take 5 readings per second

    return values

  def printValues(self, numberOfValues):
    print("time,range,minRange")
    values = self.getValues(numberOfValues)
    for value in values: 
      print (str(value["elapsed"].seconds) + "." + str(value["elapsed"].microseconds) + "," + str(value["distance"]) + "," + str(value["mindistance"]))


  def changeAddress(self, oldAddress, newAddress): #give the new address in 8bits
	print("Change sensor address")
	currentDeviceAddress = oldAddress
	commandRegister = 0x00
	changeCommand1 = 0xA0
	changeCommand2 = 0xAA
	changeCommand3 = 0xA5
	changeAddressTo = newAddress

	self.i2c.write_byte_data(currentDeviceAddress, commandRegister, changeCommand1)
	self.i2c.write_byte_data(currentDeviceAddress, commandRegister, changeCommand2)
	self.i2c.write_byte_data(currentDeviceAddress, commandRegister, changeCommand3)
	self.i2c.write_byte_data(currentDeviceAddress, commandRegister, changeAddressTo)


if __name__ == '__main__':
    sonar1 =  srf02(0x74) #Give 7bit address
    # while True:
    	#sonar1.printValues(10)
    sonar1.changeAddress(0x74,0xE2)
