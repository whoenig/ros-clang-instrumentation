#include <thread>

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"

ros::Publisher g_chatter3_pub;

void chatter2Callback(const geometry_msgs::Point::ConstPtr& msg)
{
  static uint32_t counter = 0;
  if (counter % 2 == 0) {
    // emulate slow computation
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  } else {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // publish something based on this message.
  geometry_msgs::Vector3 m;
  m.x = msg->x;
  m.y = msg->y + msg->z;
  m.z = 0.5;
  g_chatter3_pub.publish(m);

  ++counter;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "repeater");

  ros::NodeHandle n;

  g_chatter3_pub = n.advertise<geometry_msgs::Vector3>("chatter3", 1000);

  ros::Subscriber sub2 = n.subscribe("chatter2", 1000, chatter2Callback);

  ros::spin();

  return 0;
}
