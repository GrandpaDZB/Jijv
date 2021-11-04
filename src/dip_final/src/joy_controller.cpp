#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <iostream>

float max_linear_velocity = 0.1;
float max_angular_velocity = 0.1;

geometry_msgs::Twist msg_vel;
ros::Subscriber sub_joy;
ros::Publisher pub_vel;
void Callback_joy(const sensor_msgs::JoyConstPtr& msg){
    msg_vel.linear.x = msg->axes[1]*max_linear_velocity;
    msg_vel.linear.y = msg->axes[0]*max_linear_velocity;
    msg_vel.angular.z = msg->axes[3]*max_angular_velocity;
    pub_vel.publish(msg_vel);
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "joy_controller_node");
    ros::NodeHandle nh;

    sub_joy = nh.subscribe<sensor_msgs::Joy>("/joy", 10, Callback_joy);
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::spin();
    return 0;
}