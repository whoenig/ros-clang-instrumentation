#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Point.h"

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

  ros::NodeHandle n;
  ros::Publisher chatter1_pub = n.advertise<std_msgs::String>("chatter1", 1000);
  ros::Publisher chatter2_pub = n.advertise<geometry_msgs::Point>("chatter2", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    std_msgs::String msg1;

    std::stringstream ss;
    ss << "hello world " << count;
    msg1.data = ss.str();

    ROS_INFO("%s", msg1.data.c_str());
    chatter1_pub.publish(msg1);

    if (count % 2 == 0) {
      geometry_msgs::Point msg2;
      msg2.x = count;
      msg2.y = 0;
      msg2.z = 1;
      chatter2_pub.publish(msg2);
    }

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
