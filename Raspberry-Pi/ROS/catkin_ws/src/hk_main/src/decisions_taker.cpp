#include "decisions_taker.h"

/*Variables declaration*/

ros::Publisher topic_ack_pub;
ros::Publisher topic_control_cmd_pub;

void read_zone(const hk_msgs::zone::ConstPtr& msg)
{
  g_zone = msg->zone;
  g_proximity = static_cast<Proximity>(msg->proximity);  
  ROS_INFO("Zone and proximity: [%d, %d]", msg->zone, msg->proximity);

  //Update the zone array
  zones_proximity[g_zone] = g_proximity;
  ack.ack_zone = g_zone;
  topic_ack_pub.publish(ack);
  speed_control();
  
}


void remote_control(const hk_msgs::flight_cmd::ConstPtr& msg)
{
  ROS_INFO("Radio control: [%lf]", msg->roll);
  /*Read the value from the remote control*/
  cmd_radio_msg = *msg;
  //cmd_radio_msg.yaw = msg->yaw;
  //cmd_radio_msg.pitch = msg->pitch;
  //cmd_radio_msg.roll = msg->roll;
  speed_control();

}

void speed_control()
{

  float max_angle;
  float ratio;

  cmd_PX_msg = cmd_radio_msg; //Pass by all the messages and modify them if necessary

  /*All color check at once */
  for(int i = 0; i< cst::nb_proximity; i++){
      Proximity fooProx = static_cast<Proximity>(i);
      max_angle = max_angle_params[i];
      ratio = ratio_params[i];

      //Forward / Backward
      if ((zones_proximity[0] == fooProx) && (cmd_radio_msg.pitch >= 0)) {//Front zone
        if (fooProx == Proximity::red){
          cmd_PX_msg.pitch = max_angle;
        }
        else{
          cmd_PX_msg.pitch = ratio*cmd_radio_msg.pitch;
        }
        if(cmd_PX_msg.pitch > max_angle){
          cmd_PX_msg.pitch = max_angle;
        }
      } 
      if ((zones_proximity[3] == fooProx) && (cmd_radio_msg.pitch <= 0)){//Back zone
        if (fooProx == Proximity::red){
          cmd_PX_msg.pitch = -max_angle;
        }
        else{
          cmd_PX_msg.pitch = ratio*cmd_radio_msg.pitch;
        }
        if(cmd_PX_msg.pitch < -max_angle){
          cmd_PX_msg.pitch = -max_angle;
        }
      } 
    
      //Right / left
      if ((zones_proximity[4] == fooProx) && (cmd_radio_msg.roll >= 0)) {//Left zone
        if (fooProx == Proximity::red){
          cmd_PX_msg.roll = max_angle;
        }
        else{
          cmd_PX_msg.roll = ratio*cmd_radio_msg.roll;
        }
        if(cmd_PX_msg.roll > max_angle){
          cmd_PX_msg.roll = max_angle;
        }
      } 
      if ((zones_proximity[2] == fooProx) && (cmd_radio_msg.roll <= 0)){//Right zone
        if (fooProx == Proximity::red){
          cmd_PX_msg.roll = -max_angle;
        }
        else{
          cmd_PX_msg.roll = ratio*cmd_radio_msg.roll;
        }
        if(cmd_PX_msg.roll < -max_angle){
          cmd_PX_msg.roll = -max_angle;
        }
      } 
  }

  topic_control_cmd_pub.publish(cmd_PX_msg);

}

int main(int argc, char *argv[])
{
    ROS_INFO("Welcome in the decision taker node");
    ros::init(argc,argv, "Decision_taker");
    ros::NodeHandle n;

    /*Subsriber*/
    //Receive Zone messages
    ros::Subscriber topic_zones_sub = n.subscribe("topic_zones",50,read_zone);
    //Receive Remote control messages
    ros::Subscriber topic_remote_control_sub = n.subscribe("topic_cmd",50,remote_control);
    //ros::Subscriber
    /*Publisher*/
    //Publish the speed to the PixHawk  
    //ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    topic_control_cmd_pub = n.advertise<hk_msgs::flight_cmd>("topic_decisions_cmd",50);
    //Publish the acknowledgement message
    topic_ack_pub = n.advertise<hk_msgs::ack>("topic_ack",50);

    /*Get params*/
    //Get the parameters
    //n.getParam("control/max_angle_yellow", max_angle_yellow);
    //n.getParam("control/ratio_yellow", ratio_yellow);
    //ROS_INFO("Success?: [%d]",n.getParam("control/max_angle_yellow", max_angle_yellow)); //#### CREATE THE PARAM FILE
    /*
      * Get all the parameters. They are first stored in a non-const array called temp_params
      * and then moved to a const array called all_params. Each param can easily be accessed
      * using the enum Params, for instance, to retrieve red_threshold, one can do :
      * all_params[Params::Red_thresold]
      */

      bool all_params_exist = true;
      for (int i = 0; i < cst::nb_proximity; ++i)
      {
        all_params_exist = all_params_exist && n.getParam("control/ratio_" + cst::params_proximity_name[i], ratio_params[i]);//If one is 0 then become 0
        all_params_exist = all_params_exist && n.getParam("control/max_angle_" + cst::params_proximity_name[i], max_angle_params[i]);
        if (!all_params_exist)
          break;
      }

      /*
      * If one parameter is missing, throw a fatal error message and quit the node. Note that the
      * following if condition was NOT placed in the loop for clarity purposes. All the sensors' nodes
      * will also be killed because of the property "required" set to true in the sensors.launch file
      */

      if (!all_params_exist) {
        ROS_FATAL("All the parameters required for the decision_talker couldn't be retrieve \n");
        ROS_FATAL("Decision_taker exiting...");
        
    //		to_head_up_disp_msg.use_semi_automatic_mode = false;
    //		to_head_up_pub.publish(msg);
        ros::shutdown();
        return EXIT_FAILURE;
      }
      else{
        ROS_INFO("Success : all required parameters retrieved");
      }

     ros::Rate loop_rate(50);

     while(ros::ok())
     {
       //ROS_INFO("In the loop");
       ros::spinOnce();
       loop_rate.sleep();
     }    
   

    /* TODO 
    Figure out the messages receive from the remote control and the message that need
    to be sent to the PXH.
    For the message send to the PXH, we can use geometry_msgs/Twist
    Check with Guillaume what ACK is needed for the sensor fusion node
    Create the different states for the zones and impact on the velocity control 
    of the PXH
    */


      
    



    return 0;
}
