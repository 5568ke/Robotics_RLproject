#include <iostream>
#include <thread>
#include <vector>
#include <random>
#include <fstream>
#include <cmath>
#include <ctime>
#include <chrono>
#define M_PI 3.14159265358979323846
using namespace std;
#include "Q_learning.h"



// Global 
vector<vector<double>> Q_w;
vector<vector<double>> Reward;
vector<vector<double>> Q_utility;
int num_actions = 5;
int Q_w_length =4;


int main(int argc, char** argv) {
    // Set hyperparameters
    double epsilon = 0.35;
    double alpha = 0.5;
    double gamma = 0.95;
    int Episode = 2;
    int num_states = 66;
    double b = -2.0;

    // Initialize Q and Reward
    init_Q(Episode+1);
    init_Reward(Episode, num_states);
    init_Q_utility(Episode, num_states);

    ros::init(argc, argv, "my_node");
    BotNode bot_node;
    BallNode ball_node;
    std::thread t([](){
        ros::spin();
    });

    // Train Q for episodes, each episode takes num_states to learn
    vector<double> Total(Episode);
    for (int i = 0; i < Episode; i++) { 
        Total[i] = simulate_episode(i, epsilon, alpha, gamma, num_states, b,ball_node,bot_node);
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
        Q_w[1][j] = 0.0;
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
double simulate_episode(int episode, double epsilon, double alpha, double gamma, int num_states, double b,BallNode& ball_node,BotNode& bot_node) {
    cout<<"Episode "<<episode<<endl;
    cout <<"----------------------------------"<<endl;
    // get state s
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
    float s_1 = (bot_x-x_now)/(x_now-x_next);
    float s_2 = (bot_y-y_now)/(y_now-y_next);
    State s = {s_1, s_2};
    //initial parameter for compute
    double accumalate_loss = 0.0;
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
        // Get r based on block success/failure
        double r = -0.01;
        cout <<"    r:"<< r <<endl;
        cout<<endl;
        //Get s'
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
        float s_prime_1 = (bot_x-x_now)/(x_now-x_next);
        float s_prime_2 = (bot_y-y_now)/(y_now-y_next);
        State s_prime = {s_prime_1, s_prime_2};
        cout <<"    state_prime distence:   "<< s_prime.d<<endl;
        cout <<"    state_prime time:       "<< s_prime.time<<endl;
        //Get Q' and a' by max Q(s',a')
        pair<Action, double> a_Q_prime = choose_action(episode, s, epsilon);
        Action a_prime = a_Q_prime.first;
        double Q_value_prime = a_Q_prime.second;
        cout <<"      a' velocity:"<< (a_prime.velocity-2)*0.5<<endl;
        cout <<"      Q' value:"<<Q_value_prime<<endl;
        cout<<endl;
        
        //update Q_w
        Reward[episode][t] = r;
        accumalate_loss = accumalate_loss*gamma+r;
        Q_utility[episode][t] = accumalate_loss + gamma * Q_value_prime - Q_value;
        cout <<"    Q_utility value:"<<Q_utility[episode][t]<<endl<<endl;
        q_learning_step(episode, alpha, Q_utility[episode][t]);
        
        s=s_prime;
        cout <<"----------------------------------"<<endl;
    }
    for (int j =0;j<Q_w_length;j++){
        Q_w[episode+1][j] = Q_w[episode][j];
    }
    return accumalate_loss;
}
