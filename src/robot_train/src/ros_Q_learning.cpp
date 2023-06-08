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
#include <std_msgs/Float32MultiArray.h>
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

// function declaration
// void cmd_vel_pubish(float);
float reward_decide(float , float, bool,State);
bool collision_detect(BotNode,float ,float );
void shoot_ball(ros::NodeHandle&);
void set_ball_pos_to_origin(ros::NodeHandle&);
void set_robot_to_origin(ros::NodeHandle&);
void init_Q(int);
void init_Reward(int,int);
void init_Q_utility(int,int,int);
double q_function(int,State,int);
void q_learning_step(int,double,double,int );
double simulate_episode(int, double , double , double , int , double ,BallNode& ,BotNode& ,ros::NodeHandle& );
float sigmoid (float);
// void pubish_graphic(float ,float ,float );

// Global 
vector<vector<double>> Q_w;
vector<vector<double>> Reward;
vector<vector<vector<double>>> Q_utility;
int num_actions = 5;
int Q_w_length =4;
float x_origin{1.685f},y_origin{-0.2f};
float x_target{-8.f},y_target{};
double State::ball_x_vel = 0;
double State::predict_point_y_pos=0;


int main(int argc, char** argv) {
    // Set hyperparameters
    double epsilon = 0.45;
    double alpha = 0.3;
    double gamma = 0.8;
    int Episode = 140;
    int num_states = 35;
    double b = -2.0;

    // Initialize Q and Reward
    init_Q(Episode+1);
    init_Reward(Episode, num_states);
    init_Q_utility(Episode, num_states, Q_w_length);

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
        epsilon*=0.9;
        Total[i] = simulate_episode(i, epsilon, alpha, gamma, num_states, b,ball_node,bot_node,nh);
        cout << "Episode " << i << " Total Reward: " << Total[i] << endl;
        cout <<"=================================="<<endl;
    }

    // Print all reward 
    for (int i = 0; i < Episode; i++) { 
        cout << "Episode " << i << " Total Reward: " << Total[i] << endl;
    }
    cout <<"----------------------------------"<<endl;

    t.join();
    return 0;
}


// Function to simulate the robot goalkeeper and compute the total reward for an episode
double simulate_episode(int episode, double epsilon, double alpha, double gamma, int num_states, double b,BallNode& ball_node,BotNode& bot_node,
                        ros::NodeHandle& nh) {
    cout<<"Episode "<<episode<<endl;
    cout <<"----------------------------------"<<endl;

    //episode start -> shoot ball
    shoot_ball(nh);
    //calculate some static state data and init
    double prev_ball_x{ball_node.x};
    std::this_thread::sleep_for(std::chrono::milliseconds(300)); 
    double ball_x{ball_node.x};
    State::ball_x_vel = (ball_x-prev_ball_x)/0.3f;
    State::predict_point_y_pos=y_target;
    

    //initial parameter for compute
    double accumalate_loss = 0.0;
    bool flag_collision =0;
    for (int t = 0; t < num_states; t++) {  

        //print num_state info
        State s{ball_node.x,bot_node.y};
        cout << "  num_states:  " << t <<endl;
        cout<<endl;
        s.PrintMe();
        cout<<endl;
        ////////////////////////////////////////

    
        //Get Q and a by max Q(s,a), execute action
        pair<Action, double> a_Q = choose_action(episode, s, epsilon, accumalate_loss);
        Action a = a_Q.first;
        // cmd_vel_pubish((a.velocity-3)*(-0.5)/*,nh*/);
      
        ros::Publisher cmd_vel_pub = nh.advertise<std_msgs::Float64>("action", 100,1);
        ros::Publisher graphic_pub = nh.advertise<std_msgs::Float32MultiArray>("graphic",100,1);
        ros::Rate loop_rate(10);
        std_msgs::Float64 vel;
        std_msgs::Float32MultiArray msg;


        vel.data=(a.velocity-3)*(-0.5);
        cmd_vel_pub.publish(vel);

        //std::cout<<"move dis :  "<<s.d-(a.velocity-3)*(0.5)*s.time<<std::endl;
        cout <<"      action:  "<<(a.velocity-3)*(0.5)<<endl;
        cout<<endl;
        double Q_value = a_Q.second;
        /////////////////////////////////////////////////////////////////
      
        

        //check q_value normal
        if (!std::isfinite(Q_value))
            Q_value = 100;

        cout <<"      Q  value:"<<Q_value<<endl;
        cout<<endl;
        /////////////////////////////
        

        // Get r based on block success/failure
        if(collision_detect(bot_node,ball_node.x,ball_node.y))
            flag_collision = 1;
        float r = reward_decide(ball_node.x,ball_node.y,flag_collision,s);
        // cout <<"    r:"<< r <<endl;
        //////////////////////////////////////////////////////


        std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        //Get s'
        State s_prime{ball_node.x,bot_node.y};
        cout<<endl;
        s_prime.PrintMe();
        cout<<endl;
        //Get Q' and a' by max Q(s',a')
        pair<Action, double> a_Q_prime = choose_action(episode, s, epsilon, accumalate_loss);
        Action a_prime = a_Q_prime.first;
        double Q_value_prime = a_Q_prime.second;
        
        if (!std::isfinite(Q_value_prime))
            Q_value_prime = 100;
        
        cout <<"      Q' value:"<<Q_value_prime<<endl;
        cout<<endl;
        
        //update Q_w
        Reward[episode][t] = r;
        accumalate_loss = accumalate_loss*gamma+r;
        // pubish_graphic(a.velocity,r,accumalate_loss);
        // 
        msg.data={a.velocity,r-2,accumalate_loss};
        graphic_pub.publish(msg);
        loop_rate.sleep(); 
        
        
        for (int i = 0; i < Q_w_length; i++) {
            double q_value = q_function(episode,s,i);
            Q_utility[episode][t][i] = accumalate_loss + gamma * Q_value_prime - Q_value;
            cout <<"    Q_utility value:"<<Q_utility[episode][t][i]<<endl<<endl;
            q_learning_step(episode, alpha, Q_utility[episode][t][i],i);
        }
        
        //Q_utility[episode][t] = accumalate_loss + gamma * Q_value_prime - Q_value;
        //q_learning_step(episode, alpha, Q_utility[episode][t],1);
        
        
        s=s_prime;
        cout<<"flag_collision"<<flag_collision<<endl;
        cout <<"----------------------------------"<<endl;
        if ((flag_collision)||(-10.0<ball_node.x && ball_node.x<-7.5 && -2.2<ball_node.y&& ball_node.y<2.2)||t==num_states-1){
            std::cout<<"Meet boundary conditions, start new episode"<<std::endl;
            set_robot_to_origin(nh);
            //std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
            set_ball_pos_to_origin(nh);
            break;
        }
    }

    for (int j =0;j<Q_w_length;j++){
        Q_w[episode+1][j] = Q_w[episode][j];
    }

    return accumalate_loss;
}


// Function to choose an action based on exploration/exploitation trade-off
pair<Action, double> choose_action(int episode, State s, double epsilon,double accumalate_loss) {
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
        for (int i = 1; i <= num_actions; i++) {
            double q_value = q_function(episode,s,i);
            //std::cout<<"q_value of "<<i<<" : "<<q_value<<std::endl;
            if (q_value > best_q_value) {
                best_q_value = q_value;
                best_action_index = i;
            }
        }
        //std::cout<<"best action index : "<<best_action_index<<std::endl;
        //std::cout<<"best q value : "<<best_q_value<<std::endl;
        a = {best_action_index};
        pair<Action, double> a_Q_prime(a, best_q_value);
        return a_Q_prime;
    }
}

double q_function(int episode, State s,int i){
    return Q_w[episode][0] +Q_w[episode][1]*-(abs((s.d)-(i-3)*(0.5)*s.time)) + Q_w[episode][2]*(i-3)*(s.d);
}

// Function to update Q_w
void q_learning_step(int episode, double alpha, double Q_utility,int i) {
    Q_w[episode][i] =  Q_w[episode][i] + alpha * sigmoid(Q_utility);
    cout <<"  Q_w value:"<<endl;
    cout <<"    ";
    for (int j =0;j<Q_w_length;j++){
        cout << Q_w[episode][j] << " ";
    }
    cout<<endl;
}

float sigmoid (float x) {
    return 1 / (1 + exp(-x));
}

void shoot_ball(ros::NodeHandle& nh){
  static int counter=0;
  srand(time(NULL));
  x_target=-7.5f;
  //y_target=rand()%3-1.5f;
  if(counter<=110)
    y_target=-1.5f+(counter/10)*0.3f;
  else
    y_target=-1.5+((counter-110)/3)*0.3f;
  std::pair<float,float> velocity(x_target - x_origin , y_target - y_origin);
  float magnitude{ std::sqrt(pow(velocity.first,2)+pow(velocity.second,2))*9.f};
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
  counter++;
}
void set_ball_pos_to_origin(ros::NodeHandle& nh){
  geometry_msgs::Pose start_pose1;
  x_origin=0.f;
  //y_origin=rand()%8-4.f;
  y_origin=0.f;
  start_pose1.position.x = x_origin;
  start_pose1.position.y = y_origin;
  start_pose1.position.z = 0.0;
  start_pose1.orientation.x = 0.0;
  start_pose1.orientation.y = 0.0;
  start_pose1.orientation.z = 0.0;
  start_pose1.orientation.w = 0.0;

  geometry_msgs::Twist start_twist1;
  start_twist1.linear.x = 0.0;
  start_twist1.linear.y = 0.0;
  start_twist1.linear.z = 0.0;
  start_twist1.angular.x = 0.0;
  start_twist1.angular.y = 0.0;
  start_twist1.angular.z = 0.0;

  gazebo_msgs::ModelState modelstate1;
  modelstate1.model_name = "unit_sphere";
  modelstate1.reference_frame = "world";
  modelstate1.pose = start_pose1;
  modelstate1.twist = start_twist1;

  ros::ServiceClient client1 = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate1;
  setmodelstate1.request.model_state = modelstate1;
  client1.call(setmodelstate1);
}
void set_robot_to_origin(ros::NodeHandle& nh){
  geometry_msgs::Twist model_twist;
  geometry_msgs::Pose start_pose;
  x_origin=-7.f;
  y_origin=0;
  start_pose.position.x = x_origin;
  start_pose.position.y = y_origin;
  start_pose.position.z = 0.0;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 0.7071;
  start_pose.orientation.w = 0.7071;

  gazebo_msgs::ModelState modelstate;
  modelstate.model_name = "turtlebot3_burger";
  modelstate.reference_frame = "world";
  modelstate.pose = start_pose;

  ros::ServiceClient client2 = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate2;
  setmodelstate2.request.model_state = modelstate;
  client2.call(setmodelstate2);
}
// void cmd_vel_pubish(float velocity/*,ros::NodeHandle& nh*/){
//     ros::NodeHandle nh;
//     ros::Rate loop_rate(20);
//     ros::Publisher cmd_vel_pub = nh.advertise<std_msgs::Float64>("action", 10,1);
//     std_msgs::Float64 vel;
//     vel.data=velocity;
//     cmd_vel_pub.publish(vel);
//     loop_rate.sleep(); 
// }

// void pubish_graphic(float Action,float Score,float Reward){
//     ros::NodeHandle nhg;
//     ros::Rate loop_rate(20);
//     ros::Publisher graphic_pub = nhg.advertise<std_msgs::Float32MultiArray>("graphic",10,1);
//     std_msgs::Float32MultiArray msg;
//     msg.data={Action,Score,Reward};
//     graphic_pub.publish(msg);
//     loop_rate.sleep(); 
// }

bool collision_detect(BotNode bot_node,float x_next,float y_next){
    float dis = sqrt(pow((bot_node.x-x_next),2)+pow((bot_node.y-y_next),2));
    if(dis<=0.3){
        return 1;
    }
    return 0;
}
float reward_decide(float x, float y, bool flag, State s){
    if(-10.0<=x<=-7.5&&-2.2<=y<=2.2){
        if(flag){    
            float r = 5; 
            return r;
        }else{
            float r = -10*abs(s.d); 
            return r;
        }
    }else if(flag){
        float r = 20;
        return r;
    }else{
        float r = -0.25;
        return r;
    }
}

// Function to initialize Q with random values
void init_Q(int episode) {
    cout<<"Start init_Q"<<endl;
    Q_w = vector<vector<double>>(episode, vector<double>(Q_w_length));
    Q_w[1][0] = 0.1;
    Q_w[1][1] = 0.1;
    Q_w[1][2] = 0.1;
    Q_w[1][3] = 0.001;
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
void init_Q_utility(int episode, int num_states, int Q_w_length) {
    Q_utility = vector<vector<vector<double>>>(episode, vector<vector<double>>(num_states,vector<double>(Q_w_length)));
    for (int j =0;j<episode;j++){
        for (int i = 0; i < num_states; i++) {
            for (int k=0;j<Q_w_length;j++){
                Q_utility[j][i][k] = 0.0;
            }
        }
    }
    cout<<"Done init_Q_utility"<<endl;
}
