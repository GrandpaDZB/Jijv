/*
 * @Author: your name
 * @Date: 2021-08-30 22:07:12
 * @LastEditTime: 2021-09-01 22:37:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/src/demo.cpp
 */
#include <WalkingGait.h>
#include <iostream>
#include <time.h>
#include <RangeComputer.h>
#include <MotorCenterV2.h>
#include <sensor_msgs/Joy.h>
using namespace std;


void Callback_joy(const sensor_msgs::JoyConstPtr & msg){
    float x = -msg->axes[0];
    float y = msg->axes[1];
    if(x == 0 && y == 0){
        cout << "center" << endl;
        return;
    }
    float theta = acos(x/sqrt(x*x+y*y));
    if(y < 0){
        theta = 2*3.1415926 - theta;
    }
    cout << theta*(180/3.1415926) << endl;
}


int main(int argc, char** argv){
    // MotorCenter model;
    // model.set_waiting_time(1.0);
    // time_t timer = clock();
    // time_t long_timer = clock();
    // StateInfo state_info;
    // state_info.motor_flow = Tail::READY0;
    // state_info.forward_distance = 1.0;
    // while(true){
    //     if((clock()-long_timer)/(float)CLOCKS_PER_SEC >= 5.0){
    //         state_info.forward_distance = 0.0;
    //     }
    //     if((clock()-timer)/(float)CLOCKS_PER_SEC >= 0.02){
    //         model.run(&state_info);
    //         timer = clock();
    //         cout << state_info.motor_flow << endl;
    //         cout << model.leg_set[0].cast_shadow_length*cos(model.leg_set[0].arm_angle_1) << " " <<model.leg_set[0].cast_shadow_length*sin(model.leg_set[0].arm_angle_1) << endl;
    //     }
    //     else{}
    // }

    ros::init(argc, argv, "test_node");
    ros::NodeHandle nh;

    

    ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>("joy", 10, Callback_joy);
    ros::spin();

    return 0;
}