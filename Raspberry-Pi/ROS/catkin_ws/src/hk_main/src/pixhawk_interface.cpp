#include <ros/ros.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <sensor_msgs/BatteryState.h>
#include <hk_msgs/flight_cmd.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Temperature.h>
#include <hk_msgs/pixhawk_status.h>

#include <math.h>

#define rate_freq 20
#define degToRad M_PI/180

/*
 * Class for interfacing between command signals from
 * decision taker on the raspberry piand flight on the
 * pixhawk.
 *
 */
class PixhawkInterface{
    public:

        /*
         * Default constructor for the PixhawkInterface
         *
         * Creates the nodehandle, subscribers, publishers and services
         * Check FCU connection
         */
        PixhawkInterface()
        {
            ros::NodeHandle nh;

            // Parameters
            nh.param<double>("pixhawk_interface/max_thrust", _max_thrust, 20.0);
            nh.param<double>("pixhawk_interface/max_roll_angle", _max_roll_angle, 45.0);
            nh.param<double>("pixhawk_interface/max_pitch_angle", _max_pitch_angle, 45.0);
            nh.param<double>("pixhawk_interface/max_yaw_rate", _max_yaw_rate, 1.0);

            // Publishers
            _attitude_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude", 10);
            _pixhawk_status_pub = nh.advertise<hk_msgs::pixhawk_status>("topic_pixhawk_status", 10);

            // Subscribers
            _state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &PixhawkInterface::stateCallback, this);
            _battery_sub = nh.subscribe<sensor_msgs::BatteryState>("mavros/battery", 10, &PixhawkInterface::batteryCallback, this);
            _temperature_sub = nh.subscribe<sensor_msgs::Temperature>("mavros/imu/temperature", 10, &PixhawkInterface::temperatureCallback, this);
            _heading_sub = nh.subscribe<std_msgs::Float64>("mavros/global_position/compass_hdg", 10, &PixhawkInterface::headingCallback, this);
            _decision_sub = nh.subscribe<hk_msgs::flight_cmd>("topic_decisions_cmd", 10, &PixhawkInterface::decisionCallback, this);
            _joy_sub = nh.subscribe<hk_msgs::flight_cmd>("topic_decisions_cmd", 10, &PixhawkInterface::joyCallback, this);

            // Serivces
            _set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
            _arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");

            _last_request = ros::Time::now();
            _sm.request.custom_mode = "GUIDED_NOGPS";
            _kill = false;
		//_arm_cmd.request.value = true;

        }

        /*
         * Callback function for pixhawk state messages
         *
         */
        void stateCallback(const mavros_msgs::State::ConstPtr& msg_in){
            _current_state = *msg_in;
            _pixhawk_status.armed = _current_state.armed;
            _pixhawk_status.connected = _current_state.connected;
            _pixhawk_status.mode = _current_state.mode;
        }

        /*
         * Callback function for pixhawk battery messages
         *
         */
        void batteryCallback(const sensor_msgs::BatteryState::ConstPtr& msg_in){
            _battery_state = *msg_in;
            _pixhawk_status.voltage = _battery_state.voltage*100;
            _pixhawk_status.current = _battery_state.current;
            _pixhawk_status.percentage = _battery_state.percentage;
        }

        /*
         * Callback function for pixhawk imu temperature
         *
         */
        void temperatureCallback(const sensor_msgs::Temperature::ConstPtr& msg_in){
            _temperature = *msg_in;
            _pixhawk_status.temperature = _temperature.temperature;
        }

        /*
         * Callback function for pixhawk heading
         *
         */
        void headingCallback(const std_msgs::Float64::ConstPtr& msg_in){
            _heading = *msg_in;
            _pixhawk_status.heading = _heading.data;
        }

        /*
         * Callback function for decision taker flight cmd
         *
         * Converts euler angles to quaternions
         */
        void decisionCallback(const hk_msgs::flight_cmd::ConstPtr& msg_in){

            _attitude_cmd.thrust = msg_in->throttle;//0.5 -_max_thrust + (_max_thrust/100) * msg_in->throttle; // 0-1;
            double yaw = 0; // ignore
            double roll = _max_roll_angle*degToRad * msg_in->roll; // + right - left
            double pitch = _max_pitch_angle*degToRad * msg_in->pitch; // + forward - backward
            _attitude_cmd.body_rate.z = _max_yaw_rate*msg_in->yaw; // + cw, - ccw

            // Converts yaw, pitch and roll to quaternions
            double cy = cos(yaw * 0.5);
            double sy = sin(yaw * 0.5);
            double cr = cos(roll * 0.5);
            double sr = sin(roll * 0.5);
            double cp = cos(pitch * 0.5);
            double sp = sin(pitch * 0.5);

            _attitude_cmd.orientation.w = cy * cr * cp + sy * sr * sp;
            _attitude_cmd.orientation.x = cy * sr * cp - sy * cr * sp;
            _attitude_cmd.orientation.y = cy * cr * sp + sy * sr * cp;
            _attitude_cmd.orientation.z = sy * cr * cp - cy * sr * sp;
        }

        /*
         * Callback function for joy flight cmd
         *
         */

        void joyCallback(const hk_msgs::flight_cmd::ConstPtr& msg_in){
            // kill
            if(msg_in->kill){
                _kill = true;
                _arm_cmd.request.value = false;
            }

            // arming
            if(msg_in->start){
                _arm_cmd.request.value = true;
            }

            // disarm
            if(msg_in->reset){
                _arm_cmd.request.value = false;
            }
            // disable
            if(msg_in->arm){
                _kill = false;
            }
        }

        /*
         * Controls flight mode and arm then publishes topic
         */
        void start()
        {
            // Checks flight mode
            if( _current_state.mode != "GUIDED_NOGPS" && (ros::Time::now() - _last_request > ros::Duration(5.0))){
                if(_set_mode_client.call(_sm) && _sm.response.mode_sent){
                    ROS_INFO("GUIDED_NOGPS enabled");
                }
                _last_request = ros::Time::now();
            // Checks arm
            } else {
                if(!_current_state.armed && _arm_cmd.request.value){
                    if(_arming_client.call(_arm_cmd) && _arm_cmd.response.success){
                        ROS_INFO("Vehicle armed");
                    }
                } else if(_current_state.armed && !_arm_cmd.request.value){
                    if(_arming_client.call(_arm_cmd) && _arm_cmd.response.success){
                        ROS_INFO("Vehicle disarmed");
                     }
                }
            }

            // Checks kill switch
            if(_kill) {

                _attitude_cmd.thrust = 0;
                _attitude_cmd.orientation.w = 1;
                _attitude_cmd.orientation.x = 0;
                _attitude_cmd.orientation.y = 0;
                _attitude_cmd.orientation.z = 0;
                _attitude_cmd.body_rate.z = 0;
            }

            // Publish attitude command
            _attitude_pub.publish(_attitude_cmd);

            // Publish pixhawk status
            _pixhawk_status_pub.publish(_pixhawk_status);
        }

        /*
         * Get function for _current_state
         */
        mavros_msgs::State getState(){
            return _current_state;
        }

    private:
        // Publishers
        ros::Publisher _attitude_pub;
        ros::Publisher _pixhawk_status_pub;

        // Subscribers
        ros::Subscriber _state_sub;
        ros::Subscriber _battery_sub;
        ros::Subscriber _temperature_sub;
        ros::Subscriber _decision_sub;
        ros::Subscriber _joy_sub;
        ros::Subscriber _heading_sub;

        // Services
        ros::ServiceClient _set_mode_client;
        ros::ServiceClient _arming_client;

        // Messages
        mavros_msgs::AttitudeTarget _attitude_cmd;
        mavros_msgs::State _current_state;
        sensor_msgs::BatteryState _battery_state;
        mavros_msgs::SetMode _sm;
        mavros_msgs::CommandBool _arm_cmd;
        sensor_msgs::Temperature _temperature;
        std_msgs::Float64 _heading;
        hk_msgs::pixhawk_status  _pixhawk_status;

        // Time
        ros::Time _last_request;
        // Attitude maximum parameters
        double _max_thrust;
        double _max_roll_angle;
        double _max_pitch_angle;
        double _max_yaw_rate;

        // Kill switch
        bool _kill;
};


/*
 * Main function
 *
 * Creates PixhawkInterface object and then check FCU connection
 *
 */
int main(int argc, char **argv){
    ros::init(argc, argv, "pixhawk_interface");
    PixhawkInterface pi;

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(rate_freq);

    // Check for FCU connection
    while(ros::ok() && pi.getState().connected){
        ros::spinOnce();
        rate.sleep();
    }

    while(ros::ok()){
        pi.start();
        ros::spinOnce();
        rate.sleep();
    }
}
