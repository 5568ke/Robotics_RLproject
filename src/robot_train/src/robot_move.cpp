#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>


float velocity{};
void callback(const std_msgs::Float64 vel);

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlebot_controller_node");
  ros::NodeHandle nh;
  ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
  ros::Subscriber vel_sub_=nh.subscribe("action", 1000, &callback);

  ros::Rate loop_rate(1000);
  while (ros::ok())
  {
    geometry_msgs::Twist cmd_vel;
    cmd_vel.linear.x = velocity;
    cmd_vel.linear.y = 0.0;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    cmd_vel_pub.publish(cmd_vel);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}


void callback(const std_msgs::Float64 vel){
  velocity=vel.data;
}



