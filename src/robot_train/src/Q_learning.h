#include <iostream>
#include <vector>
#include <random>
#include <fstream>
#include <cmath>
#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>

using namespace std;

class BallNode
{
public:
    float x,y;

    BallNode()
    {
        ros::NodeHandle nh;
        model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &BallNode::modelStatesCallback, this);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        int model_index = -1;
        for (int i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "unit_sphere")
            {
                model_index = i;
                break;
            }
        }

        if (model_index >= 0)
        {
            x=msg->pose[model_index].position.x;
            y=msg->pose[model_index].position.y;
        }
        else
        {
            ROS_WARN("Could not find model!!");
        }
    }

private:
    ros::Subscriber model_states_sub_;
};

class BotNode
{
public:
    float x,y;

    BotNode()
    {
        ros::NodeHandle nh;
        model_states_sub_ = nh.subscribe("/gazebo/model_states", 10, &BotNode::modelStatesCallback, this);
    }

    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &msg)
    {
        int model_index = -1;
        for (int i = 0; i < msg->name.size(); i++)
        {
            if (msg->name[i] == "turtlebot3_burger")
            {
                model_index = i;
                break;
            }
        }

        if (model_index >= 0)
        {
            x=msg->pose[model_index].position.x;
            y=msg->pose[model_index].position.y;
        }
        else
        {
            ROS_WARN("Could not find model!!");
        }
    }

private:
    ros::Subscriber model_states_sub_;
};

/*
// State definition
struct State {
public:
    State(double Ball_x,double Robot_y)
    {
        time=abs(-7.5-Ball_x)/ball_x_vel;
        d=
    }

    double d;   //  distance of robot's position to locus
    double time;    // time to reach locus
    static double ball_x_vel; //ball's x-axis velocity , use this to calculate time
    static double ball
    void PrintMe(){
        std::cout<<" time : "<<time<<std::endl;
        std::cout<<" distance to predict point : "<<d<<std::endl;
    }
};
*/

// State definition
struct State {
public:
    double d;   //  distance of robot's position to locus
    double ball_x;
    double time;    // time to reach locus
    void PrintMe(){
        std::cout<<" time : "<<time<<std::endl;
        std::cout<<" distance to predict point : "<<d<<std::endl;
    }
};

// Action definition
struct Action {
    int velocity;
};

// Function to choose an action using an epsilon-greedy policy
pair<Action, double> choose_action(int , State , double );

// Function to update the Q-table using the Q-learning algorithm
void q_learning_step(int , double , double );

// Function to simulate the robot goalkeeper and compute the total reward for an episode
double simulate_episode(int , double , double , double , int , double ,BallNode& ,BotNode&,ros::NodeHandle&);

// initial Q
void init_Q(int);
// initial Reward Q-utility
void init_Reward(int , int );
void init_Q_utility(int , int );
