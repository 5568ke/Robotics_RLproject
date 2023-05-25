#include <iostream>
#include <thread>
#include <vector>
#include <random>
#include <fstream>
#include <cmath>
#include <ctime>
#include <chrono>
#include <memory>
#include <ros/ros.h>
#include <chrono>
#include <utility>
#include <gazebo_msgs/ApplyBodyWrench.h>
#include <gazebo_msgs/ModelStates.h>
#include <gazebo_msgs/SetModelState.h>
#include <geometry_msgs/Wrench.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Float64.h>
#include <iostream>
#include <time.h>
#include <mutex>
#include "Q_learning.h"
#include <geometry_msgs/Pose.h>

#define M_PI 3.14159265358979323846
using namespace std;

void cmd_vel_pubish(float velocity/*,ros::NodeHandle& nh*/){
    std::cout<<"fk"<<std::endl;
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);
    ros::Publisher cmd_vel_pub = nh.advertise<std_msgs::Float64>("action", 10,1);
    std_msgs::Float64 vel;
    vel.data=velocity;
    cmd_vel_pub.publish(vel);
    loop_rate.sleep(); 

    // while(ros::ok()){
    //   geometry_msgs::Twist cmd_vel;
    //   std::cout<<"velocity : "<<velocity<<std::endl;
    //   cmd_vel.linear.x = velocity;
    //   cmd_vel_pub.publish(cmd_vel);
    // }
}

void cmd_reset_pubish(float a){

  ros::NodeHandle nh;

  // Create a publisher to publish the model state
  ros::Publisher modelStatePub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);

  // Create a service client to reset the model state
  ros::ServiceClient modelStateClient = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

  // Create a model state message
  gazebo_msgs::ModelState modelState;
  modelState.model_name = "turtlebot3_burger";  // Set the model name to reset

  // Set the desired position for the model
  modelState.pose.position.x = a;  // Set the desired X position
  modelState.pose.position.y = 0.0;  // Set the desired Y position
  modelState.pose.position.z = 0.0;  // Set the desired Z position

//   Set other desired attributes of the model state if needed
  modelState.pose.orientation.x = 0;
  modelState.pose.orientation.y = 0;
  modelState.pose.orientation.z = 0;
  modelState.pose.orientation.w = 1;

  // Publish the initial model state
  modelStatePub.publish(modelState);

  // Wait for the model state to be set
  ros::Duration(1.0).sleep();
}

float reward_decide(float x, float y, bool flag){
    if(-10.0<=x<=-7.5&&-2.2<=y<=2.2){
        if(flag){    
            float r = -10; 
            return r;
        }else{
            float r = -1; 
            return r;
        }
    }else if(flag){
        float r = 1;
        return r;
    }else{
        float r = -0.001;
        return r;
    }
}

bool collision_detect(BotNode bot_node,float x_next,float y_next){
    float dis = sqrt(pow((bot_node.x-x_next),2)+pow((bot_node.y-y_next),2));
    if(dis<=0.3){
        return 1;
    }
    return 0;
}


// Global 
vector<vector<double>> Q_w;
vector<vector<double>> Reward;
vector<vector<double>> Q_utility;
int num_actions = 5;
int Q_w_length =4;
float x_origin{1.685f},y_origin{-0.2f};
float x_target{-8.f},y_target{};

void shoot_ball(ros::NodeHandle& );
void set_ball_pos_to_origin(ros::NodeHandle& );
void set_robot_to_origin(ros::NodeHandle& );


int main(int argc, char** argv) {
    // Set hyperparameters
    double epsilon = 0.25;
    double alpha = 0.5;
    double gamma = 0.95;
    int Episode = 100;
    int num_states = 1000;
    double b = -2.0;

    // Initialize Q and Reward
    init_Q(Episode+1);
    init_Reward(Episode, num_states);
    init_Q_utility(Episode, num_states);

    ros::init(argc, argv, "my_node");
    ros::NodeHandle nh;
    BotNode bot_node;
    BallNode ball_node;

    std::thread t([](){
        ros::spin();
    });

    // Train Q for episodes, each episode takes num_states to learn
    vector<double> Total(Episode);
    for (int i = 0; i < Episode; i++) { 
        cout<< "123"<<endl;
        Total[i] = simulate_episode(i, epsilon, alpha, gamma, num_states, b,ball_node,bot_node,nh);
        cout << "Episode " << i << " Total Reward: " << Total[i] << endl;
        cout <<"=================================="<<endl;
    }

    // Print all reward 
    // for (int i = 0; i < Episode; i++) { 
    //     cout << "Episode " << i << " Total Reward: " << Total[i] << endl;
    // }
    // cout <<"----------------------------------"<<endl;

    // Print all Q_utility
    // for (int i = 0; i < Episode; i++) { 
    //     cout << "Q_utility: Episode " << i<<endl;
    //     for (int j = 0; j < num_states; j++) {
    //         cout << Q_utility[i][j] << " ";
    //     }
    //     cout<<endl;
    // }
    // cout <<"----------------------------------"<<endl;

    // Print all Q
    cout << "All Episode Q_w:" << endl;
    for (int i = 0; i <Episode; i++) {
        for (int j =0;j<Q_w_length;j++){
            cout << Q_w[i][j] << " ";
        }
        cout << endl;
    }
    t.join();
    return 0;
}


// Function to initialize Q with random values
void init_Q(int episode) {
    cout<<"Start init_Q"<<endl;
    Q_w = vector<vector<double>>(episode, vector<double>(Q_w_length));
    for (int j =0;j<Q_w_length;j++){
        Q_w[1][j] = 5;
    }
    cout<<"Done init_Q"<<endl;
}

// Function to initialize Reward with 0.0
void init_Reward(int episode, int num_states) {
    Reward = vector<vector<double>>(episode, vector<double>(num_states));
    for (int j =0;j<episode;j++){
        for (int i = 0; i < num_states; i++) {
            Reward[j][i] = 0.0;
        }
    }
    cout<<"Done init_Reward"<<endl;
}

// Function to initialize Q with guess
void init_Q_utility(int episode, int num_states) {
    Q_utility = vector<vector<double>>(episode, vector<double>(num_states));
    for (int j =0;j<episode;j++){
        for (int i = 0; i < num_states; i++) {
            Q_utility[j][i] = 0.0;
        }
    }
    cout<<"Done init_Q_utility"<<endl;
}

// Function to choose an action based on exploration/exploitation trade-off
pair<Action, double> choose_action(int episode, State s, double epsilon) {
    // Generate a random double value
    auto time_now = chrono::high_resolution_clock::now();
    auto seed = time_now.time_since_epoch().count();
    mt19937_64 rng(seed);
    uniform_real_distribution<double> distribution(0.0, 1.0);
    double random_number = distribution(rng);
    
    // cout<<"    epsilon random:"<<random_number<<endl;
    Action a;
    // Epsilon greedy
    if (random_number < epsilon) {
        // Exploration: choose a random action
        cout<<"    >> choose random"<<endl;
        uniform_int_distribution<int> action_distribution(0, 4);
        int action_index = action_distribution(rng);
        a = {action_index};
        pair<Action, int> a_Q_prime(a, action_index);
        return a_Q_prime;
    }
    else {
        cout<<"    >> choose max Q"<<endl;
        // Exploitation: choose action with maximum Q-value
        int best_action_index = 0;
        double best_q_value = 0;
        for (int i = 1; i < num_actions; i++) {
            double q_value = Q_w[episode][0]+Q_w[episode][1]*(1/s.d)+Q_w[episode][2]*s.time*(i-2)*0.5+Q_w[episode][3]*1;
            if (q_value > best_q_value) {
                best_q_value = q_value;
                best_action_index = i;
            }
        }
        a = {best_action_index};
        pair<Action, double> a_Q_prime(a, best_q_value);
        return a_Q_prime;
    }
}

// Function to update Q_w
void q_learning_step(int episode, double alpha, double Q_utility) {
    for (int j =0;j<Q_w_length;j++){
        Q_w[episode][j] =  Q_w[episode][j] + alpha * Q_utility;
    }
    cout <<"  Q_w value:"<<endl;
    cout <<"    ";
    for (int j =0;j<Q_w_length;j++){
        cout << Q_w[episode][j] << " ";
    }
    cout<<endl;
}

// Function to simulate the robot goalkeeper and compute the total reward for an episode
double simulate_episode(int episode, double epsilon, double alpha, double gamma, int num_states, double b,BallNode& ball_node,BotNode& bot_node,
                        ros::NodeHandle& nh) {
    cout<<"Episode "<<episode<<endl;
    cout <<"----------------------------------"<<endl;
    // get state s
    shoot_ball(nh);
    float x_now = ball_node.x;
    float y_now = ball_node.y;
    float x_next{},y_next{};
    while(1){
      if(ball_node.x!=x_now && ball_node.y!=y_now){
        x_next=ball_node.x;
        y_next=ball_node.y;
        break;
      }
    }
    float bot_x = bot_node.x;
    float bot_y = bot_node.y;
    cout<<"x_now:"<<x_now<<"   x_next:"<<x_next<<endl;
    cout<<"y_now:"<<y_now<<"   y_next:"<<y_next<<endl;
    State s = {(bot_x-x_now)/(x_now-x_next), (bot_y-y_now)/(y_now-y_next)};
    //initial parameter for compute
    double accumalate_loss = 0.0;
    bool flag_collision =0;
    for (int t = 0; t < num_states; t++) {    // episode length
        cout << "  num_states:  " << t <<endl;
        cout<<endl;
        cout <<"    state distence:        "<< s.d<<endl;
        cout <<"    state time:            "<< s.time<<endl;
        //Get Q and a by max Q(s,a)
        pair<Action, double> a_Q = choose_action(episode, s, epsilon);
        Action a = a_Q.first;
        double Q_value = a_Q.second;
        cout <<"      a velocity:"<< (a.velocity-2)*0.5<<endl;
        cout <<"      Q value:"<<Q_value<<endl<<endl;
        // Do Action by a
        cmd_vel_pubish((a.velocity-2)*0.5/*,nh*/);

        // Get r based on block success/failure
        while(1){
            if(ball_node.x!=x_now && ball_node.y!=y_now){
                x_next=ball_node.x;
                y_next=ball_node.y;
                break;
            }
        }
        //Collision detect
        if(collision_detect(bot_node, x_next, y_next)){
            flag_collision = 1;
        }
        //Get reward
        float r = reward_decide(x_next,y_next,flag_collision);
        cout <<"    r:"<< r <<endl;
        cout<<endl;
        //Get s'
        x_now = ball_node.x;
        y_now = ball_node.y;
        while(1){
            if(ball_node.x!=x_now && ball_node.y!=y_now){
                x_next=ball_node.x;
                y_next=ball_node.y;
                break;
            }
        }
        bot_x = bot_node.x;
        bot_y = bot_node.y;
        cout<<"x_now:"<<x_now<<"   x_next:"<<x_next<<endl;
        cout<<"y_now:"<<y_now<<"   y_next:"<<y_next<<endl;
        State s_prime = {(bot_x-x_now)/(x_now-x_next), (bot_y-y_now)/(y_now-y_next)};
        cout <<"    state_prime distence:   "<< s_prime.d<<endl;
        cout <<"    state_prime time:       "<< s_prime.time<<endl;
        //Get Q' and a' by max Q(s',a')
        pair<Action, double> a_Q_prime = choose_action(episode, s, epsilon);
        Action a_prime = a_Q_prime.first;
        double Q_value_prime = a_Q_prime.second;
        cout <<"      a' velocity:"<< (a_prime.velocity-2)*0.1<<endl;
        cout <<"      Q' value:"<<Q_value_prime<<endl;
        cout<<endl;
        
        //update Q_w
        Reward[episode][t] = r;
        accumalate_loss = accumalate_loss*gamma+r;
        Q_utility[episode][t] = accumalate_loss + gamma * Q_value_prime - Q_value;
        cout <<"    Q_utility value:"<<Q_utility[episode][t]<<endl<<endl;
        q_learning_step(episode, alpha, Q_utility[episode][t]);
        
        s=s_prime;
        cout<<"flag_collision"<<flag_collision<<endl;
        cout <<"----------------------------------"<<endl;
        if ((flag_collision)||(-10.0<x_next && x_next<-7.5 && -2.2<y_next && y_next<2.2)){
            std::cout<<"setsetsetsetset"<<std::endl;
            set_ball_pos_to_origin(nh);
            // set_robot_to_origin(nh);
            //cmd_reset_pubish(-6.5f);
            break;
        }
    }
    for (int j =0;j<Q_w_length;j++){
        Q_w[episode+1][j] = Q_w[episode][j];
    }
    return accumalate_loss;
}

void shoot_ball(ros::NodeHandle& nh){
  srand(time(NULL));
  x_target=-7.5f;
  y_target=rand()%3-1.5f;
  std::pair<float,float> velocity(x_target - x_origin , y_target - y_origin);
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
  srv.request.duration = ros::Duration(0.002f); 
  client.call(srv);
}



void set_ball_pos_to_origin(ros::NodeHandle& nh){

  geometry_msgs::Twist model_twist;
  geometry_msgs::Pose start_pose;
  x_origin=0.f;
  y_origin=rand()%8-4.f;
  start_pose.position.x = x_origin;
  start_pose.position.y = y_origin;
  start_pose.position.z = 0.0;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;

  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = "unit_sphere";
  modelstate.reference_frame = "world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);
}


void set_robot_to_origin(ros::NodeHandle& nh){

  geometry_msgs::Twist model_twist;
  geometry_msgs::Pose start_pose;
  x_origin=-6.5f;
  y_origin=-10;
  start_pose.position.x = x_origin;
  start_pose.position.y = y_origin;
  start_pose.position.z = 0.0;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.0;
  start_pose.orientation.w = 0.0;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = "turtlebot3_burger";
  modelstate.reference_frame = "world";
  modelstate.pose = start_pose;

  ros::ServiceClient client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

    // ros::Publisher pub = nh.advertise<gazebo_msgs::ModelState>("/gazebo/set_model_state", 10);
    // gazebo_msgs::ModelState msg;
    // msg.model_name = "turtlebot3_burger";  // Replace with the name of your TurtleBot3 model in Gazebo
    // msg.pose.position.x = -6.5f;  // Set the x-coordinate of the position
    // msg.pose.position.y = 0.0;  // Set the y-coordinate of the position
    // msg.pose.position.z = 0.0;  // Set the z-coordinate of the position

    // msg.pose.orientation.x = 0.0;  // Set the x-component of the orientation quaternion
    // msg.pose.orientation.y = 0.0;  // Set the y-component of the orientation quaternion
    // msg.pose.orientation.z = 0.0;  // Set the z-component of the orientation quaternion
    // msg.pose.orientation.w = 1.0;  // Set the w-component of the orientation quaternion
    // pub.publish(msg);
    // ros::Duration(1.0).sleep();  // Delay for 1 second
}
