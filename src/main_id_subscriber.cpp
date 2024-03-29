#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cstdlib>

ros::Subscriber sub;
std_msgs::String opencv_msg;
ros::Publisher opencv_pub;
std::stringstream ss_message;

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void voiceCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
  if (strcmp(msg->data.c_str(), "hello") == 0)
  {
      std::system("flite -voice rms -t \"Hello\"");
  }
  if (strcmp(msg->data.c_str(), "object") == 0)
  {
      std::system("flite -voice rms -t \"Searching out object\"");
      ss_message.clear();
      ss_message.str("");
      ss_message << "object";
      opencv_msg.data = ss_message.str();
      ROS_INFO("%s", opencv_msg.data.c_str());
      opencv_pub.publish(opencv_msg);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "main_id_subscriber");
  ros::NodeHandle n;

  ROS_INFO("Starting main_id_subscriber");
  sub = n.subscribe("voice", 1000, voiceCallback);
  opencv_pub = n.advertise<std_msgs::String>("voice_commands", 1000);
  
  ros::spin();

  return 0;
}
