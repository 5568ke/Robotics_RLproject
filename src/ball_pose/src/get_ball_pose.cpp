#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

class MyNode {
  public:
    MyNode() {
      ros::NodeHandle nh;
      model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &MyNode::modelStatesCallback, this);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
      int model_index = -1;
      for (int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "unit_sphere") {
        // if (msg->name[i] == "turtlebot3_burger") {
          model_index = i;
          break;
        }
      }

      if (model_index >= 0) {
        ROS_INFO("Model position: x=%f, y=%f, z=%f", 
                 msg->pose[model_index].position.x,
                 msg->pose[model_index].position.y,
                 msg->pose[model_index].position.z);
      } else {
        ROS_WARN("Could not find model!!");
      }
    }

  private:
    ros::Subscriber model_states_sub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "my_node");

  MyNode node;

  ros::spin();
}

