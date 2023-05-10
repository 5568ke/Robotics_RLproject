#include <memory>
#include <ros/ros.h>
#include <cmath>
#include <chrono>
#include <utility>
#include <thread>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <iostream>
#include <time.h>
#include <mutex>

float y_origin{};
std::mutex m_;
void shoot_ball(ros::NodeHandle& );
void set_y_origin(const gazebo_msgs::ModelStates::ConstPtr& );
void set_ball_pos_to_origin(ros::NodeHandle& );

int main(int argc, char** argv)
{
  ros::init(argc, argv, "apply_wrench");
  ros::NodeHandle nh;
  ros::Subscriber model_states_sub_;
  model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, set_y_origin);
  while(1){
    std::this_thread::sleep_for(std::chrono::seconds(1));
    ros::spinOnce();
    shoot_ball(nh);
    std::this_thread::sleep_for(std::chrono::seconds(3));
    set_ball_pos_to_origin(nh);
  }
  return 0;
}

void shoot_ball(ros::NodeHandle& nh){
  srand(time(NULL));
  std::pair<float,float> velocity(-7.f,rand()%4 - 2.f - y_origin);
  float magnitude{ std::sqrt(pow(velocity.first,2)+pow(velocity.second,2))*3.f};
  velocity.first /= magnitude;
  velocity.second /= magnitude;
  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
  gazebo_msgs::ApplyBodyWrench srv;
  srv.request.body_name = "unit_sphere::link";  
  srv.request.reference_frame = "world";  
  srv.request.wrench.force.x = velocity.first;   
  srv.request.wrench.force.y = velocity.second;  
  srv.request.wrench.force.z = 0; 
  ROS_INFO("x : %f " ,velocity.first);
  ROS_INFO("y : %f ", velocity.second);
  srv.request.start_time = ros::Time::now();
  srv.request.duration = ros::Duration(0.005f); 
  client.call(srv);
}

void set_y_origin(const gazebo_msgs::ModelStates::ConstPtr& msg) {
  int model_index = -1;
  for (int i = 0; i < msg->name.size(); i++) {
    if (msg->name[i] == "unit_sphere") {
      model_index = i;
      break;
    }
  }
  if (model_index >= 0) 
    y_origin=msg->pose[model_index].position.y;
  ROS_INFO("Model position: y_origin=%f",y_origin);
}


void set_ball_pos_to_origin(ros::NodeHandle& nh){

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;
  geometry_msgs::Twist model_twist;
  modelstate.model_name = "unit_sphere";
  modelstate.reference_frame = "world";
  srand(time(NULL));
  model_twist.linear.x = 0.f;
  model_twist.linear.y = rand()%16 - 8.f;
  y_origin=model_twist.linear.y;
  model_twist.linear.z = 0.f;
  modelstate.twist = model_twist;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);
}
