/*
 * @Author: your name
 * @Date: 2021-08-30 22:07:12
 * @LastEditTime: 2021-09-03 13:39:59
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/src/demo.cpp
 */
#include <WalkingGait.h>
#include <iostream>
#include <time.h>
#include <RangeComputer.h>
#include <MotorCenterV2.h>
#include <Communication.h>
#include <sensor_msgs/Joy.h>
#include <serial/serial.h>

using namespace std;

// serial::Serial* sp = Communication_Create_SerialPort();



// void Callback_joy(const sensor_msgs::JoyConstPtr & msg){
//     float x = -msg->axes[0];
//     float y = msg->axes[1];
//     if(x == 0 && y == 0){
//         cout << "center" << endl;
//         return;
//     }
//     float theta = acos(x/sqrt(x*x+y*y));
//     if(y < 0){
//         theta = 2*3.1415926 - theta;
//     }

//     theta = theta*(180/3.1415926);
//     if(theta > 180){
//         theta = 180;
//     }
//     Communication_Servo_control(sp, Tail::LEG_1_ARM_1, theta);
//     cout << theta << endl;
// }


int main(int argc, char** argv){

    ros::init(argc ,argv, "test_node");
    ros::NodeHandle nh;

    

    // ros::Subscriber sub_joy = nh.subscribe<sensor_msgs::Joy>("joy", 10, Callback_joy);
    // ros::spin();

    // string str = "#12P1200\r\n";
    // uint8_t buffer[1000] = {0};
    // int num = 0;
    // num = Communication_string_to_uint8array(str, buffer);
    // //serial::Serial* sp = Communication_Create_SerialPort();
    // serial::Serial sp;
    // serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    // sp.setPort("/dev/ttyUSB0");
    // sp.setBaudrate(115200);
    // sp.setTimeout(timeout);
    // sp.open();
    // if(!sp.isOpen()){
    //     ROS_ERROR("Fail to open serial port. Please check settings!");
    // }
    // sp.write(buffer, num);

    serial::Serial* sp;
    legState leg_set[6];
    Initialize_leg_set(leg_set);
    MotorCenter model_MOTOR;
    model_MOTOR.set_waiting_time(1.0);

    model_MOTOR.interval_counter = model_MOTOR.max_interval;
    model_MOTOR.run(leg_set, 90);
    cout << endl;

    model_MOTOR.interval_counter = model_MOTOR.max_interval;
    model_MOTOR.run(leg_set, 90);
    cout << endl;

    model_MOTOR.interval_counter = model_MOTOR.max_interval;
    model_MOTOR.run(leg_set, 90);
    cout << endl;

    model_MOTOR.interval_counter = model_MOTOR.max_interval;
    model_MOTOR.run(leg_set, 90);
    cout << endl;

    model_MOTOR.interval_counter = model_MOTOR.max_interval;
    model_MOTOR.run(leg_set, 90);
    cout << endl;
    

    return 0;
}