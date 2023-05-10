#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_controller_node");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);

  ros::Rate loop_rate(10);

  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = 0.2;
    cmd_vel.linear.y = 0.2;

    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
