using namespace std;

/*Messages*/
//Personal
#include "hk_msgs/flight_cmd.h"
#include "hk_msgs/radar.h"
#include "hk_msgs/user_mode.h"
#include "hk_msgs/zone.h"
#include "hk_msgs/ack.h" //WILL HAVE TO BE CHANGED TO PLACE THEM IN THE CORRECT PACKAGE!!!
#include <geometry_msgs/Twist.h>


/*Libraries*/
#include <iostream>
#include "ros/ros.h"
//Important note : std::array not supported by ROS messages. Uuse boost::array instead
#include "boost/bind.hpp"
#include "boost/array.hpp"





/* Namespace to contain all constants*/
namespace cst {
    constexpr char nb_zones = 6;
    constexpr char nb_proximity = 4;

    const boost::array<std::string, nb_proximity> params_proximity_name //static_cast<int>(Params::Nb_of_params)
	{
		"empty",
		"green",
        "yellow",
        "red"
	};

	
}


/*Variable*/
enum class Proximity{
    empty,
    green,
    yellow,
    red,
};
boost::array<Proximity,cst::nb_zones>		zones_proximity {Proximity::empty};     
boost::array<float, cst::nb_proximity> ratio_params;
boost::array<float, cst::nb_proximity> max_angle_params;
char g_zone = 0;  
Proximity g_proximity = Proximity::empty;
hk_msgs::flight_cmd cmd_radio_msg;
hk_msgs::flight_cmd cmd_PX_msg;
hk_msgs::ack ack;


int max_angle = 50;
float ratio = 0;


/*Function definition*/

void speed_control();
void read_zone(const hk_msgs::zone::ConstPtr& msg);
void remote_control(const hk_msgs::flight_cmd::ConstPtr& msg);
//void ack_send(int zone);
