# Main package for Pyrothan
Interfaces between controller input, sensor input and pixhawk/gazebo output

## Installation
To run the ROS package you need to install the following:
For the pixhawk_interface node, Run the following in the terminal

	sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
	wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
	chmod +x ./install_geographiclib_datasets.sh
	./install_geographiclib_datasets.sh

## Usage
Run the launch file **main.launch**.
Specify fcu with fcu_url argument, default is raspberry-pi uart port.
Ex: 
	roslaunch hk_main main.launch fcu_url:="udp://:14551@127.0.0.1:14555" to launch with Gazebo

## Credits
Uses the mavros package for MAVLink communication

## File structure
Description of the file structure
 - **pixhawk_interface.cpp** Used for receiving flight cmd from decisions_taker.cpp and sending to pixhawk/gazebo.

## Resources
