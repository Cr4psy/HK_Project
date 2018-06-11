#include "ros/ros.h"
#include "sensors_fusion.h"

#include <iostream>
//#include <array> : Important note : std::array not supported by ROS messages. Use boost::array instead
#include <vector>
#include <string>
#include <map>
#include "boost/bind.hpp"
#include "boost/array.hpp"


/* TO DO (?):
 * Checking that the message from the head_up_display is sent after the sensors_fusion node have sent the faulty sensors as a msg)
*/

/* Static variables declared in the Redundancy class used to assess if another sensor has already computed the proximity in that zone previously during
* the current iteration of spinOnce() and whether the previous zone calculated is the same as the new one.
*/

std::map<unsigned char,bool> Redundancy::are_detected{ {{0,false},{2,false},{4,false} } };
boost::array<Proximity, cst::zones_nb> Redundancy::previous_proximities{};

/* Arrays containing all faulty sensors (at first, none are considered faulty). */

boost::array<bool, cst::radar_nb>	Faulty_sensor::all_faulty_radar{};
boost::array<bool, cst::sonar_nb>	Faulty_sensor::all_faulty_sonar{};
boost::array<bool, cst::ir_nb>		Faulty_sensor::all_faulty_ir{};

/* Initializing the arrays that are used to determine if a message from a working sensor hasn't been received for a certain amount of the main loop iteration
 * (this value is stored as the parameter 'Faulty_not_read_nb'. If it's the case, throw a faulty_sensor exception and stop incrementing the counter for
 * that specific sensor. During normal working, the value can only be 0 or above. (each callback of the specific sensor reset the counter to 0 while the
 * value is always incremented in the main loop. Hence, if the counter goes above the specified value, an exception is thrown). The -1 is used during the 
 * iteration to make sure all sensors have been read. If it's -1, continue looping till the sensor is assessed to 'not working' by a faulty communication
 * instead of by receiving a -100 message */

boost::array<int,cst::radar_nb>		Unread_counter::radar_unread_counter {-1};
boost::array<int,cst::sonar_nb>		Unread_counter::sonar_unread_counter{-1,-1,-1,-1,-1};
boost::array<int,cst::ir_nb>		Unread_counter::ir_unread_counter{-1,-1,-1};

/* Initialize the static variables of the class 'Initialization */

int Initialization::nb_of_nodes_init{};
bool Initialization::init_time_expired = false;
/* Get the critical proximity of two redundant sensors assessing different proximities for a given zone*/

Proximity Redundancy::get_critical_proxi(
	const unsigned char zone_num, 
	const Proximity new_proxi)
{
	try { //if trying to check the redundancy of a zone with only one sensor, throw an error 'out of range' (as irrelevant). Note that this error will be thrown often
		if (are_detected.at(zone_num)) {
			are_detected[zone_num] = false;
			if (new_proxi > previous_proximities[zone_num])
				previous_proximities[zone_num] = new_proxi;
			
		}
		else {
			are_detected[zone_num] = true;
			previous_proximities[zone_num] = new_proxi;
		}
	}
	catch (std::out_of_range) {
		
	}
	return previous_proximities[zone_num];
}

/* Assessing whether the object is still in the same zone as detected previously.
* If it's the case, there is no need to re-send a zone msg to the decisions_taker node
*/

bool Redundancy::are_same_zones(
	const unsigned char zone_num,
	const Proximity proxi)
{
	if (proxi != previous_proximities[zone_num]) {
		previous_proximities[zone_num] = proxi;
		return false;
	}
	else {
		return true;
	}
}

/* The counter of each sensor is incremented after each main loop iteration. The counter is reset to zero when the callback of the sensor is finally
called. If it's not called for a too long period of time, the counter will reach the top value defined by the parameter 'Faulty_not_read_nb' and
* the sensor will therefore be considered as faulty.
*/

void Unread_counter::increment(
	const boost::array<float,cst::nb_of_params>& all_params)
{
	for(int i = 0; i < radar_unread_counter.size(); ++i){
		if(!Faulty_sensor::isFaulty(Sensors::Radar,i)){
			++radar_unread_counter[i];
			if(radar_unread_counter[i] >= all_params[Params::Faulty_not_read_nb]){
			//	ROS_WARN("radar is faulty by no read msg");
				throw Faulty_radar(i);
			}
		}	
	}
	for(int i = 0; i < sonar_unread_counter.size(); ++i){
		if(!Faulty_sensor::isFaulty(Sensors::Sonar,i)){
			++sonar_unread_counter[i];
			if(sonar_unread_counter[i] >= all_params[Params::Faulty_not_read_nb]){
			//	ROS_WARN("SONAR is faulty by no read msg");
				throw Faulty_sonar(i);
			}
		}	
	}
	for(int i = 0; i < ir_unread_counter.size(); ++i){
		if(!Faulty_sensor::isFaulty(Sensors::Ir,i)){
			++ir_unread_counter[i];
			if(ir_unread_counter[i] >= all_params[Params::Faulty_not_read_nb]){
			//	ROS_WARN("IR is faulty by no read msg");
				throw Faulty_ir(i);
			}
		}	
	}
}

/* Used to reset the value of the counter when the callback is processed*/

void Unread_counter::reset(
	const Sensors sensor_type,
	const unsigned char sensor_num)
{
	try{
		switch (sensor_type){
			case Sensors::Radar : {
				radar_unread_counter.at(sensor_num) = 0;
				break;
			}
			case Sensors::Sonar : {
				sonar_unread_counter.at(sensor_num) = 0;
				break;
			}
			case Sensors::Ir :{
				ir_unread_counter.at(sensor_num) = 0;
				break;
			}
		}
	}
	catch(std::out_of_range){
		throw Unknown_sensor(sensor_type,sensor_num);
	}
}

/* Used during the initialization to define if all sensors have sent their messages or not.*/

bool Unread_counter::isNegative()
{
	for(auto i : radar_unread_counter){ if(i < 0) return true;}
	for(auto i : sonar_unread_counter){ if(i < 0) return true;}
	for(auto i : ir_unread_counter){ if(i < 0) return true;}
	return false;
}

/* Used to determine if a specific sensor is faulty or not. Indeed, if the sensor is already considered as faulty, there is no point
* in incrementing its counter.
*/

bool Faulty_sensor::isFaulty(
	const Sensors sensor_type, 
	const unsigned char sensor_num)
{
	switch(sensor_type){
		case Sensors::Radar : return all_faulty_radar[sensor_num];
		case Sensors::Sonar : return all_faulty_sonar[sensor_num];
		case Sensors::Ir :    return all_faulty_ir[sensor_num];
	}
}

/* Used to reset the sensor as a working one (if it suddenly restarts working out of a miracle)*/

void Faulty_sensor::reset(
	const Sensors sensor_type, 
	const unsigned char sensor_num)
{
	switch(sensor_type){
		case Sensors::Radar :  all_faulty_radar[sensor_num] = false; break;
		case Sensors::Sonar :  all_faulty_sonar[sensor_num] = false; break;
		case Sensors::Ir :     all_faulty_ir[sensor_num] = false; break;
	}
}


/* Determine the proximity in a given zone. Throw a Faulty_sonar exception if the sensor is faulty (message received is equal to cst::faulty_sensor_msg)*/
/* Throw a rework exception if a sensor is reworking */

Proximity get_proximity(
	const Sensors sensor_type,
	const unsigned char sensor_num, 
	float dist,
	const boost::array<float,cst::nb_of_params>& all_params)
{
	Proximity proximity;
	if (dist == cst::faulty_sensor_msg) {
		switch (sensor_type){
		case Sensors::Sonar : throw Faulty_sonar(sensor_num);
		case Sensors::Radar : throw Faulty_radar(sensor_num);
		case Sensors::Ir	: throw Faulty_ir(sensor_num);
		}
	}
	else {
		/* Check if the sensor has started re-working from a previous faulty state
		* Note that when a sensor starts re-working, the first message is just used
		* to tell so to the user, not yet to assess the proximity of the zone. 
		* (A second working message has to arrive for that)
		*/
		if (Faulty_sensor::isFaulty(sensor_type, sensor_num)) {
			Faulty_sensor::reset(sensor_type, sensor_num);
			//ROS_WARN("Get proxi : A sensor is reworking");
			throw Reworking_sensor(sensor_type,sensor_num); 							  
		}

		/* Assess the proximity*/
			
		if (dist == 0)
			proximity = Proximity::Empty;

		else if (dist > all_params[Params::Yellow_threshold])
			proximity = Proximity::Green;

		else if (dist > all_params[Params::Red_threshold])
			proximity = Proximity::Yellow;

		else
			proximity = Proximity::Red;
	}
	return proximity;
}




/* Process the messages sent by the sonar nodes and determine the proximity of the object in the related zone 
 * The sensor number corresponds to the zone number (hence sonar_2 monitors zone 2). Check first if the zone is known, if
 * not, throw an Unknown_zone exception. Then, get the proximity of that zone based on the distance and assess if the sensor
 * is working correctly. If not, throw a Faulty_sonar exception. Afterwards, determine if the previous proximity calculated
 * in that zone is identical to the new one. If it's the case, don't do anything (no need to compute several times identical values).
 * Lastely, determine if another sensor had already assessed the proximity of an object in that zone. If it's the case, check if
 * the two sensors detect the object at the same distance. If NOT, consider the more critical proximity value as true
 * and the one that will be sent to the decisions_taker node. There is no mean to assess if a sensor is faulty or not with just
 * two sensors, so don't throw any faulty exception message in that case. Finally, update the proximity of the zone in the 
 * all_zones_to_send vector and set the flag bit to true if the message has to be sent after spinOnce() finishes processing
 * all callbacks.
 */

void read_sonar_msg(
	const hk_msgs::sonar::ConstPtr& msg,
	const boost::array<float,cst::nb_of_params>& all_params, 
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>& all_zones_to_send,
	const char sonar_num)
{
	unsigned char zone_num;
	try{
		zone_num = cst::sonar_to_zones.at(sonar_num);
	}
	catch(std::out_of_range){
		throw Unknown_sensor(Sensors::Sonar,sonar_num);
	}
	
	Unread_counter::reset(Sensors::Sonar,sonar_num);

	const float dist = msg->dist_to_obj; 

	Proximity proxi = get_proximity(Sensors::Sonar, sonar_num, dist, all_params);
	if (!Redundancy::are_same_zones(zone_num,proxi)) {
		/*NOTE : instead of storing a static array 'previous priority' in the class Redundancy, I could just pass in the all_zones_to_send array in those functions
		* and update it directly there. I'm afraid it would abstract away the clarity of what's happening if I do so though.*/
		all_zones_to_send[zone_num].first = Redundancy::get_critical_proxi(zone_num, proxi); 
		all_zones_to_send[zone_num].second = true;
	}
}

/* Process the messages sent by the radar node(s) and perform similar actions as described for the sonar callback ('read_sonar_msg') */

void read_radar_msg(
	const hk_msgs::radar::ConstPtr& msg, 
	const boost::array<float,cst::nb_of_params>& all_params,
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>& all_zones_to_send,
	const char radar_num)
{

	unsigned char zone_num;
	try{
		zone_num = cst::radar_to_zones.at(radar_num);
	}
	catch(std::out_of_range){
		throw Unknown_sensor(Sensors::Radar,radar_num);
	}


	Unread_counter::reset(Sensors::Radar,radar_num);

	const float dist = msg->dist_to_obj;

	Proximity proxi = get_proximity(Sensors::Radar, radar_num, dist, all_params);

	if (!Redundancy::are_same_zones(zone_num, proxi)) {
		all_zones_to_send[zone_num].first = Redundancy::get_critical_proxi(zone_num, proxi);
		all_zones_to_send[zone_num].second = true;
	}
}

/* Process the messages sent by the ir nodes and perform similar actions as described for the sonar callback ('read_sonar_msg') */

void read_ir_msg(
	const hk_msgs::ir::ConstPtr& msg, 
	const boost::array<float,cst::nb_of_params>& all_params,
	boost::array<std::pair<Proximity, bool>,cst::zones_nb>& all_zones_to_send,
	const char ir_num)
{
	
	unsigned char zone_num;
	try{
		zone_num = cst::ir_to_zones.at(ir_num);
	}
	catch(std::out_of_range){
		throw Unknown_sensor(Sensors::Ir,ir_num);
	}

	Unread_counter::reset(Sensors::Ir,ir_num);

	const float dist = msg->dist_to_obj;

	Proximity proxi = get_proximity(Sensors::Ir, ir_num, dist, all_params);

	if (!Redundancy::are_same_zones(zone_num, proxi)) {
		all_zones_to_send[zone_num].first = Redundancy::get_critical_proxi(zone_num, proxi);
		all_zones_to_send[zone_num].second = true;
	}
}



/* Read the acknowledgement messages sent by the decisions_taker node to confirm that it has received
 * the zone messages sent by the sensor_fusions node. If it's the case, set the flag bit of the corresponding
 * zones in the all_zones_to_send vector to 'false'. If the message hasn't been read, it is re-sent till it's
 * received. If an object is detected in zone 4 and a msg with thas zone number is sent over the topic_zones, 
 * the decisions_taker sends an integer of value '4' to acknowledge he has read that peculiar msg.
 */

void read_ack_msg(
	const hk_msgs::ack::ConstPtr& msg, 
	boost::array<std::pair<Proximity,bool>,cst::zones_nb>& all_zones_to_send)
{
	try {
		all_zones_to_send.at(msg->ack_zone).second = false;
	}
	catch (std::out_of_range) {
		ROS_WARN("An invalid zone was acknowledged");
	}

}


/* A sensor can be detected faulty both if the sensor_fusions node receives a msg '-100' on its associated topic or if
 * the sensor_fusions node doesn't read any messages from this sensor for a long enough time. This doesn't hold if the sensor
 * is already faulty naturally.
*/


/* Read messages sent from the head_up_disp */

void read_head_up_disp_msg(
	const hk_msgs::from_head_up_disp::ConstPtr& msg, 
	State& stop_node)
{
	stop_node = static_cast<State>(!msg->use_semi_automatic_mode);
}

/* This service function is called by all the sensors' nodes when they're done with their initialization. Therefore
 * the nb_of_nodes_init variable stores the number of nodes that've finished initializing and the sensors_fusion node
 * waits till all the nodes are done initializing before moving further*/

bool Initialization::increment_init_counter(
	hk_msgs::init_confirmation::Request& a,
	hk_msgs::init_confirmation::Response& b)
{
	++nb_of_nodes_init;
	ROS_INFO("The nb of nodes initialized is : %i",nb_of_nodes_init);
	return true;
}

/*This service is called by the GUI interface during its initialization. Indeed, if the GUI boots after the 
 * sensors_fusion nodes, it has still to retrieve the information of all faulty sensors and current zones*/

bool send_sensors_status_and_zones(
	hk_msgs::send_sensors_status_and_zones::Request& 			a,
	hk_msgs::send_sensors_status_and_zones::Response& 			b,
	boost::array<std::pair<Proximity, bool>, cst::zones_nb>&	all_zones_to_send)
{
	/*Not the best design choice as the publisher is re-created everytime the function is called. This function is
	 *however rarely called as it's called only when the gui interface is booted so it's acceptable */
	ros::NodeHandle n;
	ros::Publisher to_head_up_disp_pub	= n.advertise<hk_msgs::to_head_up_disp>("topic_to_head_up_disp",50);
	ros::Publisher zones_pub			= n.advertise<hk_msgs::zone>("topic_zones", 50);

	hk_msgs::to_head_up_disp 		msg;
	hk_msgs::zone					zone_msg;

	for (int i = 0; i < cst:: radar_nb; i++){
		if(Faulty_sensor::isFaulty(Sensors::Radar,i)){
			msg.faulty_sensors = boost::array<int,2>{static_cast<int>(Sensors::Radar),i};
			to_head_up_disp_pub.publish(msg);   
			msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
		}
	}
	for (int i = 0; i < cst::sonar_nb; i++){
		if(Faulty_sensor::isFaulty(Sensors::Sonar,i)){
			msg.faulty_sensors = boost::array<int,2>{static_cast<int>(Sensors::Sonar),i};
			to_head_up_disp_pub.publish(msg);
			msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
		}
	}
	for (int i = 0; i < cst::ir_nb; i++){
		if(Faulty_sensor::isFaulty(Sensors::Ir,i)){
			msg.faulty_sensors = boost::array<int,2>{static_cast<int>(Sensors::Ir),i};
			to_head_up_disp_pub.publish(msg);
			msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
		}
	}		
	msg.faulty_sensors = boost::array<int,2>{0,0};
	msg.init_success = true;
	to_head_up_disp_pub.publish(msg);
	/* Re-publish all the zones at the init  of the GUI application as well*/
	{
		int i = 0;
		for (auto zone_to_send : all_zones_to_send) {
			zone_msg.zone = i;
			zone_msg.proximity = static_cast<int>(zone_to_send.first);
			zones_pub.publish(zone_msg);
			++i;
		}
	}
	
	return true;
}

bool check_wifi_connectivity(
	hk_msgs::check_wifi_connectivity::Request& a,
	hk_msgs::check_wifi_connectivity::Response& b)
{
	return true;
}

bool process_callbacks(
	ros::Publisher& to_head_up_disp_pub)
{
	bool is_faulty = false;
	hk_msgs::to_head_up_disp to_head_up_disp_msg;
	while(ros::ok()){
		try {
			ros::spinOnce();
			break;
		}
		catch (Faulty_sensor &msg) {
			to_head_up_disp_msg.faulty_sensors = msg.get_msg();
			to_head_up_disp_pub.publish(to_head_up_disp_msg);
			to_head_up_disp_msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
			is_faulty = (is_faulty || true);
			//ROS_WARN("A SENSOR IS FAULTY");
		}
		catch (Reworking_sensor &msg) {
			to_head_up_disp_msg.reworking_sensors = msg.get_msg();
			to_head_up_disp_pub.publish(to_head_up_disp_msg);
			to_head_up_disp_msg.reworking_sensors = boost::array<unsigned char,2>{0,0};
			//ROS_WARN("A sensor is reworking");
		}
		catch (Unknown_sensor &msg) {
			boost::array<unsigned char, 2> a_msg = msg.get_msg();
			ROS_WARN("An unknown sensor is communicating. Sensor type :%u, Sensor number %u", a_msg[0], a_msg[1]);
		}
		catch(...){
			ROS_WARN("Unknown error have been thrown");
		}
	}
	
	return is_faulty;
}



/* Subscribing to all sensors data topic in one place. The topics are numeroted according to the zone they're monitoring. Hence topic_ir_5 means that 
 * this topic is used by that sensor to detect if there is an object in zone 5. By retrieving the last character of the topic name, one can know the 
 * zone that's being monitor by this message and process it accordingly.*/

//NOTE : creating a nodehandle is cheap and all nodehandle points to the same node. Therefore, instead of passing it, it would have been probably clearer to re-create 
//a nodehandle object inside the function.
void subscribe_to_topics(
	ros::NodeHandle n,
	boost::array<ros::Subscriber, cst::all_sensors_nb>& all_sensors_sub,
	const boost::array<float,cst::nb_of_params>& all_params,
	boost::array<std::pair<Proximity,bool>,cst::zones_nb>& all_zones_to_send)
{

	for (int i = 0; i < cst::all_sensors_nb; ++i){
		if (i < cst::radar_nb){
			all_sensors_sub[i] = n.subscribe<hk_msgs::radar>("topic_radar_" + std::to_string(i),10, boost::bind(read_radar_msg, _1, boost::ref(all_params),boost::ref(all_zones_to_send),i));
			continue;
		}
		else if (i < (cst::radar_nb + cst::sonar_nb)){
			all_sensors_sub[i] = n.subscribe<hk_msgs::sonar>("topic_sonar_" + std::to_string(i-cst::radar_nb), 10, boost::bind(read_sonar_msg, _1, boost::ref(all_params), boost::ref(all_zones_to_send),i-cst::radar_nb));
			continue;
		}
		else
			all_sensors_sub[i] = n.subscribe<hk_msgs::ir>("topic_ir_" + std::to_string(i-cst::radar_nb-cst::sonar_nb), 10, boost::bind(read_ir_msg, _1, boost::ref(all_params), boost::ref(all_zones_to_send),i-cst::radar_nb-cst::sonar_nb));	
	}

}

/*------------------------------------------
 * MAIN FUNCTION. 
 * -----------------------------------------*/

int main(int argc, char **argv)
{
	/* Used to decide whether to continue using the semi-automatic flying mode or not */

	State stop_node = State::Unknown;

	/* Ros initialization */

	ros::init(argc, argv, "sensors_fusion");
	ros::NodeHandle n;

	/*
	* Get all the parameters. They are first stored in a non-const array called temp_params
	* and then moved to a const array called all_params. Each param can easily be accessed
	* using the enum Params, for instance, to retrieve red_threshold, one can do :
	* all_params[Params::Red_thresold]
	*/
	
	boost::array<float, cst::nb_of_params> temp_params;
	bool all_params_exist = true;
	for (int i = 0; i < cst::nb_of_params; ++i)
	{
		all_params_exist = all_params_exist && n.getParam("sensors/" + cst::params_name[i], temp_params[i]);
		if (!all_params_exist)
			break;
	}

	/*
	* If one parameter is missing, throw a fatal error message and quit the node. Note that the
	* following if condition was NOT placed in the loop for clarity purposes. All the sensors' nodes
	* will also be killed because of the property "required" set to true in the sensors.launch file
	*/

	if (!all_params_exist) {
		ROS_FATAL("All the parameters required for the sensors_fusion node couldn't be retrieved \n");
		ROS_FATAL("Sensors_fusion exiting...");
		
		ros::shutdown();
		return EXIT_FAILURE;
	}
	else{
		ROS_INFO("Success : all required parameters retrieved");
	}
	
	const boost::array<float, cst::nb_of_params> all_params = temp_params;

	/* The array used to store the proximity related to each zone and whether the message has been sent. Has to be passed to the callback functions */

	boost::array<std::pair<Proximity, bool>,cst::zones_nb> all_zones_to_send{};
	boost::array<bool, cst::all_sensors_nb> faulty_sensors{}; // keep an array of all faulty sensors

	/* Defining the subscribers */

	boost::array<ros::Subscriber, cst::all_sensors_nb> all_sensors_sub;
	subscribe_to_topics(n,all_sensors_sub, all_params, all_zones_to_send);

	
	ros::Subscriber ack_sub					= n.subscribe<hk_msgs::ack>("topic_ack", 10, boost::bind(read_ack_msg,_1,boost::ref(all_zones_to_send)));
	ros::Subscriber from_head_up_disp_sub	= n.subscribe<hk_msgs::from_head_up_disp>("topic_from_head_up_disp", 10, boost::bind(read_head_up_disp_msg,_1, boost::ref(stop_node)));
	//ros::Subscriber user_mode_sub 			= n.subscribe<hk_msgs::user_mode>("topic_user_mode",5);

	

	/* Defining the publishers */	

	ros::Publisher user_mode_pub		= n.advertise<hk_msgs::user_mode>("topic_user_mode",50);
	ros::Publisher zones_pub			= n.advertise<hk_msgs::zone>("topic_zones", 50);
	ros::Publisher to_head_up_disp_pub	= n.advertise<hk_msgs::to_head_up_disp>("topic_to_head_up_disp",50);

	/* Advertising the service */

	ros::ServiceServer service = n.advertiseService("service_init_confirmation",&Initialization::increment_init_counter);
	ros::ServiceServer service_sensors_status = n.advertiseService<hk_msgs::send_sensors_status_and_zones::Request,hk_msgs::send_sensors_status_and_zones::Response>("service_sensors_status_and_zones",boost::bind(send_sensors_status_and_zones,_1,_2,boost::ref(all_zones_to_send)));
	ros::ServiceServer service_wifi = n.advertiseService("service_check_wifi_connectivity",&check_wifi_connectivity);
	/* Create the messages that have to be sent */

	hk_msgs::to_head_up_disp		to_head_up_disp_msg;
	hk_msgs::user_mode				user_mode_msg;
	hk_msgs::zone					zone_msg;

	
	/* Defining some sleeping rate. 
	 * - init_rate : use once at the beginning to wait for the initialization of all sensors node. 
	 * - loop_rate : default frequency at which the node is executed
	 * - sleep_rate : when the semi-automatic mode is deactivated, the node is executing in slow mode, 
	 *				  just checking whether the user want to reactive the semi-automatic mode or not
	 */

	ros::Rate loop_rate(all_params[Params::Sensors_fusion_rate]);
	ros::Rate sleep_rate(all_params[Params::Sensors_fusion_sleep_rate]);

	/* Wait till all nodes have finished their initialization (that is, have incremented the init_node counter). If after 5 seconds,
	 * all nodes haven't finished their init, just move on because it probably means some nodes are faulty. Then, process all messages sent
	 * by the sensors during the init phase.
	 * If some messages haven't been received, ask the sensor nodes to resend their messages. If after a certain time, the nodes haven't send any
	 * messages, consider the sensor to be faulty. */



	
	
	ROS_INFO("Sensors_fusion : waiting for sensors's nodes initialization...");
	ros::Timer timer = n.createTimer(ros::Duration(5), Initialization::time_has_expired); //set the variable init_time_expired to true if 5 secs have elapsed
	bool one_faulty_sensor = false;

	/* Wait for all nodes to init */

	while (ros::ok()) {
		one_faulty_sensor = process_callbacks(to_head_up_disp_pub);
		if(Initialization::get_nb_of_nodes_init() == cst::all_sensors_nb){ROS_WARN("Sensors_fusions : all sensors's nodes have finished their initialization"); break;}
		if(Initialization::is_init_time_expired()){
			ROS_WARN("Sensors_fusions : Some sensors haven't finished their initialization. Moving on because 5 secs have passed by"); 
			one_faulty_sensor = true; 
			break;
			}
		loop_rate.sleep();
	}

	/* Read all messages and determining how many sensors are not working. If a certain node hasn't sent a message, we ask it to resend the message a certain number of time
	* If after that time, the node hasn's still communicated, we consider the sensor as faulty.
	*/

	{
		int i = 0;
		while(Unread_counter::isNegative() && (i <= all_params[Params::Faulty_not_read_nb] + 10) && ros::ok()){
			user_mode_msg.resend_data = true;
			user_mode_pub.publish(user_mode_msg);
			while(ros::ok()){ //required to get the exceptions of all the sensors if several stop working simultaneously
				try{
					Unread_counter::increment(all_params);
					break;
				}
				catch(Faulty_sensor &msg){
					to_head_up_disp_msg.faulty_sensors = msg.get_msg();
					to_head_up_disp_pub.publish(to_head_up_disp_msg);
					to_head_up_disp_msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
					one_faulty_sensor = true;		
				}
			}
			++i;
			one_faulty_sensor = (one_faulty_sensor || process_callbacks(to_head_up_disp_pub));
			loop_rate.sleep();
		}
		
	}
	

	/* Reset the resend data to zero after the init phase. Critical in order to receive the semi-auto message in the other nodes */
	user_mode_msg.resend_data = false;
	ROS_INFO("Initialization done");

	/*If one sensor is faulty, loop till the user sends a message from the head up display deciding whether he wants to use the 
		*semi-automatic mode or not */

	/* INIT_SUCCESS HAS TO BE MODIFIED AS IT'S ACTING AS INIT_DONE FOR NOW. SEE HOW TO MODIFY THIS*/ 




	if(one_faulty_sensor){
		ROS_WARN("One or more sensor are not working properly");
		ROS_WARN("Waiting for the user's response to know if he wants to still use the semi-automatic flying mode");
	}
	else{
		ROS_WARN("All sensors are working correctly.");
		ROS_WARN("Waiting for the user's response to know if he wants to use the semi-automatic flying mode");
		
		
	}	
	to_head_up_disp_msg.init_success = true;
	to_head_up_disp_pub.publish(to_head_up_disp_msg);	

	while(stop_node == State::Unknown && ros::ok()){ 
		process_callbacks(to_head_up_disp_pub);
		loop_rate.sleep();
	}

	/* Doesn't want to use the semi-automatic flying mode*/
	if(stop_node == State::True){
		ROS_WARN("The semi-automatic flying mode is now deactivated");	
		user_mode_msg.semi_automatic = false;
	}
		

	/* Does want to use the semi-automatic flying mode*/
	else{
		ROS_WARN("The semi-automatic flying mode is now active");	
		user_mode_msg.semi_automatic = true;
	}
	
		/* By default the semi-automatic flying mode is set up if all sensors are working*/
	
	
	user_mode_pub.publish(user_mode_msg);
	to_head_up_disp_msg.init_success = false;
	
	
		
	/* Main loop. If the user decides to switch to manual mode, the node enters a sleeping mode and checks at a 
	 * frequency of sleep_rate if the user wants to reactive the semi-automatic mode.
	 */
	
	while (ros::ok()){
		//process_callbacks(to_head_up_disp_pub, to_head_up_disp_msg, true);
		
		try{ //should be uncesseray but just to be on the safe side if a callback throw unexpectedly while in sleep mode. 
			 //In sleep mode, the sensors status is not monitored
			ros::spinOnce();
		}
		catch(...){

		}

		bool was_in_semi_auto = false;
		if (stop_node != State::True){
			ROS_WARN("The semi-automatic flying mode is now active");
			user_mode_msg.semi_automatic = true;
			user_mode_pub.publish(user_mode_msg);
		}		

		while (ros::ok() && stop_node != State::True){
			process_callbacks(to_head_up_disp_pub);
		

			try{ //if several sensor stops working simultaneously, only one is detected. The other will be at the next loop iteration
				Unread_counter::increment(all_params);
			}
			catch(Faulty_sensor &err){
				to_head_up_disp_msg.faulty_sensors = err.get_msg();
				to_head_up_disp_pub.publish(to_head_up_disp_msg);
				to_head_up_disp_msg.faulty_sensors = boost::array<unsigned char,2>{0,0};
			}
			
			/* Sending the zones */
			{
				int i = 0;
				for (auto zone_to_send : all_zones_to_send) {
					if (zone_to_send.second) {
						zone_msg.zone = i;
						zone_msg.proximity = static_cast<int>(zone_to_send.first);
						zones_pub.publish(zone_msg);
					}
					++i;
				}
			}
			was_in_semi_auto = true;
			loop_rate.sleep();
		}
	
		if(was_in_semi_auto){
			ROS_WARN("The semi-automatic flying mode is now deactivated");	
			user_mode_msg.semi_automatic = false;
			user_mode_pub.publish(user_mode_msg);
		}
			

		sleep_rate.sleep();
	}

	return EXIT_SUCCESS;
}
	
