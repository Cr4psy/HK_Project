# Description of directory
This package contains all the program related to the sensors. Which includes every individual program for every specific type of sensor, as well as the sensor fusion node. 
The different sensors are:
 - Infrared sensors: GP2Y0A710K0F
 - Sonars: SRF02
 - Radar: μSharp


## Dependencies
Several dependencies are necessary to be able to run the different program as they rely on external libraries.
The sensors are also using different communication protocole (I2C, Serial) than need to be activated on the Raspberry Pi.

### Enable I2C communication
Download the dependencies required:

	sudo apt-get -y update
	sudo apt-get -y upgrade
	sudo apt-get install -y i2c-tools
	sudo apt-get install libi2c-dev
	
(You may have to install wiringPi also: [here](https://projects.drogon.net/raspberry-pi/wiringpi/download-and-install/))

Enable I2C peripheral (the procedure may change depending on the RPi version):

    sudo raspi-config.
    
- Use the down arrow to select 3 Interfacing Options
- Arrow down to P4 I2C.
- Select yes when it asks you to enable I2C
- Also select yes when it tasks about automatically loading the kernel module.
- Use the right arrow to select the <Finish> button.
- Select yes when it asks to reboot.

When the system reboot, log in and enter the following command:

    ls /dev/*i2c*
The Pi should respond with:

**/dev/i2c-1**

Modify the peripheral configuration:

    sudo nano /boot/config.txt
At the end of the file insert: 

- **dtparam=i2c1=on**
- **dtparam=i2c0=on**
- **dtparam=i2c1_baudrate=50000**

To test that the Pi is seeing the peripherals connected to I2C, use the command:

    i2c-detect -y 1
	
	
### Enable UART communication
Download the dependencies required:
	
    sudo apt-get -y update
	sudo apt-get -y upgrade
	sudo apt-get install serial
	sudo apt-get install minicom
	
Search for available ports:

    dmesg | grep tty

The consol use the same port as for the serial communication, the last line indicates if it is enable for the consol or not. If it is the case, it must be disabled.

Run the configuration commmand

    sudo raspi-config
	
- Go to "Advanced Options"
- Then "Serial"
- Press "No", when asked if we would like a login shell to be accessible over serial.
- Then press <Ok> for the confirmation that the Serial is now disabled
	
Enable Serial Data on ttyAMA0

    sudo nano /boot/config.txt

Scroll to the end of the file and add the following lines to the file. Scroll to the end of the file and add the following lines to the file.
- **enable_uart=1**
- **dtoverlay=pi3-disable-bt**
- **dtoverlay=pi3-miniuart-bt**

It may happened that the acces to the port is limited to admin user. If it is the case use the following command to make it accessible by everybody:
 
    sudo chmod 777 /dev/ttyAMA0

 
### IR sensors 
The IR sensors are outputting an analog signal that is converted to a digital and then transmitted through I2C with an ADC converter.
The ADC used is a ADS1015 by Adafruit, the library can be installed with the following command:

    sudo apt-get install git build-essential python-dev
    cd ~
    git clone https://github.com/adafruit/Adafruit_Python_ADS1x15.git
    cd Adafruit_Python_ADS1x15
    sudo python setup.py install

Alternatively you can install from pip with:

    sudo pip install adafruit-ads1x15

Note that the pip install method **won't** install the example code.
 
The IR sensors follows a curve that is not linear and need calibration in order to give an accurate result. The function can be found in the code and is equal to:
	L = exp((1/V+0.4776)/0.193)
	With L the distance and V the voltage.

	
### Sonars
The sonars of type SRF02 are directly communicating with I2C and don't need any libraries or calibration. The code has been inspired by different source (see resources).
The address of every sensor can be modified using the program "i2cChangeAddress.py".
Note that the old address must be given in 7bits and the new address in 8bits, the convertion table is given in the code.

 
### Radar
The radar is using serial communication and don't require any specific libraries 
 
## Usage
The node can be run with a launch file that launches every program required:

    roslaunch hk_sensor sensors.launch

### Testing
In order to simulate the user acknowledgment, the following message can be sent:

    rostopic pub --once topic_user_mode hk_sensors/user_mode '{semi_automatic: 1,resend_data: 0}'


## File structure
The files are separated in different folders depending on their types. The messages are in /msg/, the C++ files can be found in the /src/ folder and finally the python programm are in /scripts/
Description of the file structure
 - **ir.py** Used for getting the values from the IR sensors 
 - **radar.py** Used for getting the measures from the radar
 - **sonar.py** Used for getting the measures from the sonar
 - **sensors_params.yaml** Contains all the parameters used in this node
 - **sensors.launch** Used to launch all the programs at once

## Resources
For more information about the different sensors and periph follow the different links:
### I2C:
https://learn.sparkfun.com/tutorials/raspberry-pi-spi-and-i2c-tutorial
https://www.maxbotix.com/Setup-Raspberry-Pi-Zero-for-i2c-Sensor-151

### Serial Port:
https://www.maxbotix.com/Raspberry-Pi-with-Ultrasonic-Sensors-144
http://www.instructables.com/id/Read-and-write-from-serial-port-with-Raspberry-Pi/

### ADS1x15
https://learn.adafruit.com/raspberry-pi-analog-to-digital-converters/ads1015-slash-ads1115

### SRF02
http://versasense.com/files/datasheets/SRF02.pdf
https://github.com/timdelbruegger/freecopter/blob/master/src/python3/sensors/range_finder_srf02.py
https://github.com/davstott/piTank/blob/master/range/srf02.py
http://davstott.me.uk/index.php/2013/04/07/raspberry-pi-sonar/

### Radar
https://aerotenna.readme.io/docs/usharp-patch-1


## Credits

