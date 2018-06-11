#include "hk_msgs/ir.h"
#include "hk_msgs/sonar.h"
#include "hk_msgs/radar.h"
#include "hk_msgs/user_mode.h"
#include "hk_msgs/zone.h"
#include "hk_msgs/ack.h"
#include "hk_msgs/to_head_up_disp.h"
#include "hk_msgs/from_head_up_disp.h"

#include "hk_msgs/init_confirmation.h"
#include "hk_msgs/send_sensors_status_and_zones.h"
#include "hk_msgs/check_wifi_connectivity.h"

/* Namespace to contain all constants*/	

namespace cst {
	/* Constant(s) related to sensors */

	constexpr char radar_nb = 1;
	constexpr char sonar_nb = 5;
	constexpr char ir_nb = 3;
	
	constexpr char diff_sensors_nb = 3;
	constexpr char all_sensors_nb = ir_nb + sonar_nb + radar_nb;
	

	/* Constant(s) related to zones */

	constexpr char zones_nb = 6;
	//constexpr char nb_of_zones_with_redundancy = 3;

	constexpr boost::array<char,radar_nb>		radar_to_zones {0};
	constexpr boost::array<char,sonar_nb>		sonar_to_zones {1,2,3,4,5};
	constexpr boost::array<char,ir_nb>			ir_to_zones {0,2,4};
	//constexpr boost::array<char, nb_of_zones_with_redundancy> zones_with_redundancy{ 0, 1, 5 };

	/* Constant(s) related to messages */

	constexpr int faulty_sensor_msg = -100;
	constexpr int user_mode_msg_resend_data = -1;

}


/* Some enumeration*/

/* Sensors type */

enum class Sensors
{
	Radar = 1,
	Sonar,
	Ir
};

/* Proximity of the object in the zone of interest */

enum class Proximity
{
	Empty,
	Green,
	Yellow,
	Red
};

/* Used mainly when a confirmation is expected (for instance if the user wants to use the manual or semi-auto mode)*/

enum class State
{
	Unknown = -1,
	False = 0,
	True = 1,

};

/* Enum to access parameters in the all_params array. Note it's just an enum and not an enum class to avoid having to static cast every time we want to access
* a parameter. The Nb_of_params is used as a counter, works only because the first param starts at 0. */

enum Params
{
	Sensors_fusion_rate,
	Sensors_fusion_sleep_rate,
	Red_threshold,
	Yellow_threshold,
	Faulty_not_read_nb,
	Nb_of_params
};

/* This array contains the name of all the parameters stored in the param server*/

namespace cst{
	constexpr char nb_of_params = static_cast<char>(Params::Nb_of_params);

	const boost::array<std::string, nb_of_params> params_name //static_cast<int>(Params::Nb_of_params)
	{
		"general/sensors_fusion_rate",
		"general/sensors_fusion_sleep_rate",
		"general/red_threshold",
		"general/yellow_threshold",
		"general/faulty_not_read_nb"
	};
	
}


/* Functions definitions */

void read_sonar_msg(
	const hk_msgs::sonar::ConstPtr&								msg,
	const boost::array<float,cst::nb_of_params>&			 	all_params, 
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>&		all_zones_to_send,
	const char 													sonar_num);

void read_radar_msg(
	const hk_msgs::radar::ConstPtr& 							msg, 
	const boost::array<float,cst::nb_of_params>& 				all_params,
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>& 	all_zones_to_send,
	const char 													radar_num);

void read_ir_msg(
	const hk_msgs::ir::ConstPtr& 								msg, 
	const boost::array<float,cst::nb_of_params>& 				all_params,
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>& 	all_zones_to_send,
	const char													 ir_num);

void read_ack_msg(
	const hk_msgs::ack::ConstPtr&								msg,
	boost::array<std::pair<Proximity, bool>, cst::zones_nb>&	all_zones_to_send);

void read_head_up_disp_msg(
	const hk_msgs::from_head_up_disp::ConstPtr&					msg,
	State&														stop_node);

bool increment_init_counter(
	hk_msgs::init_confirmation::Request& a,
	hk_msgs::init_confirmation::Response& b);	

void subscribe_to_topics(
	ros::NodeHandle												n,
	boost::array<ros::Subscriber, cst::all_sensors_nb>&			all_sensors_sub,
	const boost::array<float, cst::nb_of_params>&				all_params,
	boost::array<std::pair<Proximity, bool>, cst::zones_nb>&	all_zones_to_send);

bool send_sensors_status_and_zones(
	hk_msgs::send_sensors_status_and_zones::Request& 			 a,
	hk_msgs::send_sensors_status_and_zones::Response&			 b,
	boost::array<std::pair<Proximity, bool>, cst::zones_nb>&	all_zones_to_send);	

bool check_wifi_connectivity(
	hk_msgs::check_wifi_connectivity::Request& a,
	hk_msgs::check_wifi_connectivity::Response& b);
	

/* This class is used to check the redundancy between two sensors*/

class Redundancy
{
public:
	static Proximity get_critical_proxi(const unsigned char zone, const Proximity new_proxi);
	static bool are_same_zones(const unsigned char zone, const Proximity proxi);

private:
	Redundancy() = default;
	static std::map<unsigned char,bool> are_detected;
	static boost::array<Proximity, cst::zones_nb> previous_proximities;
};

/* This class is used to assess whether a sensor node hasn't sent messages over its topic (and hence is considered faulty)*/

class Unread_counter
{
public:
	static void increment(const boost::array<float,cst::nb_of_params>& all_params);
	static void reset(const Sensors sensor_type,const unsigned char sensor_num);
	static bool isNegative();

private:
	Unread_counter() = default;
	static boost::array<int,cst::radar_nb> radar_unread_counter;
	static boost::array<int,cst::sonar_nb> sonar_unread_counter;
	static boost::array<int,cst::ir_nb> ir_unread_counter;
};

/* The following classes are for exceptions. */

/*Faulty_sensor is a ABSTRACT class*/
class Faulty_sensor 
{
public:
	Faulty_sensor(unsigned char num) : _num_faulty_sensor{num}{};
	virtual boost::array<unsigned char, 2> get_msg() const = 0;
	static bool isFaulty(const Sensors sensor_type, const unsigned char sensor_num); 
	static void reset(const Sensors sensor_type, const unsigned char sensor_num);
	

protected:
	virtual ~Faulty_sensor() = default;	

	const unsigned char _num_faulty_sensor;
	static boost::array<bool,cst::radar_nb> all_faulty_radar;
	static boost::array<bool,cst::sonar_nb> all_faulty_sonar;
	static boost::array<bool,cst::ir_nb> all_faulty_ir;
	
};

class Faulty_radar : public Faulty_sensor

{
public:

	Faulty_radar(unsigned char num) : Faulty_sensor{ num } { all_faulty_radar[num] = true; };
	boost::array<unsigned char, 2>get_msg() const override{
		return boost::array<unsigned char, 2>{static_cast<unsigned char>(Sensors::Radar), _num_faulty_sensor};
	};
};


class Faulty_sonar : public Faulty_sensor
{
public:

	Faulty_sonar(unsigned char num) : Faulty_sensor{ num } { all_faulty_sonar[num] = true; };
	boost::array<unsigned char, 2>get_msg() const override{
		return boost::array<unsigned char, 2>{static_cast<unsigned char>(Sensors::Sonar), _num_faulty_sensor};
	};

};


class Faulty_ir : public Faulty_sensor
{
public :
	Faulty_ir(unsigned char num) : Faulty_sensor{ num } {all_faulty_ir[num] = true;};
	boost::array<unsigned char,2>get_msg() const override{
		return boost::array<unsigned char,2>{static_cast<char>(Sensors::Ir),_num_faulty_sensor};
	};
};

/* This is a class used as an exception when a faulty sensor starts miracously re-working */

class Reworking_sensor 
{
public:
	Reworking_sensor(const Sensors sensor_type, const unsigned char sensor_num) : _sensor_type{ sensor_type }, _sensor_num{ sensor_num } {};
	boost::array<unsigned char, 2>get_msg() const  {
		return boost::array<unsigned char, 2>{static_cast<unsigned char>(_sensor_type), _sensor_num};
	};
private:
	const Sensors _sensor_type;
	const unsigned char _sensor_num;
};

/* That's a class used as an exception when the sensor is unknown */

class Unknown_sensor
{
public:
	Unknown_sensor(Sensors sensor_type,unsigned char num) : _sensor_type{sensor_type},_sensor_num{num} {};
	boost::array<unsigned char, 2>get_msg() const  {
		return boost::array<unsigned char, 2>{static_cast<unsigned char>(_sensor_type), _sensor_num};
	};
private:
	Sensors _sensor_type;
	const unsigned char _sensor_num;
};


/*Contains some functions and variables used for the initialization process*/

class Initialization
{
public:
	static bool increment_init_counter(
		hk_msgs::init_confirmation::Request& a,
		hk_msgs::init_confirmation::Response& b);

	static int get_nb_of_nodes_init(){return nb_of_nodes_init;}	
	static bool is_init_time_expired(){return init_time_expired;}
	static void time_has_expired(const ros::TimerEvent&){init_time_expired = true;}
private:
	static int nb_of_nodes_init;
	static bool init_time_expired;
};
