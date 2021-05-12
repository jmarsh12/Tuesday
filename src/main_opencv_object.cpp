#include "ros/ros.h"
#include "my_id_robot/FindObjectOpenCV.h"
#include "std_msgs/String.h"

#include <cstdlib>

ros::ServiceClient client;
ros::Publisher servoControl;
std_msgs::String servo_msg;
std::stringstream ss_message;


void findObjectCallback(const std_msgs::String::ConstPtr& msg)
{
  my_id_robot::FindObjectOpenCV srv;
  srv.request.id_object = "object";

  ROS_INFO("Got request to find Object");
  if (client.call(srv))
  {
    int position = 400;
    ROS_INFO("X: %d", srv.response.x);
    ROS_INFO("Y: %d", srv.response.y);
    ss_message.clear();
    ss_message.str("");
    if (srv.response.x > 500)
      ss_message << "3, " << 300;
    else
      ss_message << "3, " << 700;
    servo_msg.data = ss_message.str();
    ROS_INFO("sending to servo %s", servo_msg.data);
    servoControl.publish(servo_msg);
  }
  else
  {
    ROS_ERROR("Failed to call service find_object_opencv");
    return;
  }
  return;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_object_client");
  if (argc != 1)
  {
    ROS_INFO("usage: main_object_client");
    return 1;
  }

  ROS_INFO("Initializing main_object node");
  
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("opencv_find", 100, findObjectCallback); 
  client = n.serviceClient<my_id_robot::FindObjectOpenCV>("my_id_robot");

  servoControl = n.advertise<std_msgs::String>("servo_control", 100);

  ros::spin();

  return 0;
}
