#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Vector3.h"

void chatter1Callback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

void chatter3Callback(const geometry_msgs::Vector3::ConstPtr& msg)
{
  ROS_INFO("I got a Vector: [%f, %f, %f]", msg->x, msg->y, msg->z);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;
  ros::Subscriber sub1 = n.subscribe("chatter1", 1000, chatter1Callback);
  ros::Subscriber sub2 = n.subscribe("chatter3", 1000, chatter3Callback);

  ros::spin();

  return 0;
}
