/*
 * @Author: your name
 * @Date: 2021-09-01 18:41:30
 * @LastEditTime: 2021-09-01 22:07:30
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/include/GeometryComputer/MotorCenterV1.h
 */

#ifndef MOTORCENTER_V2
#define MOTORCENTER_V2

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <common_parameters.h>
#include <common_defination.h>
#include <Communication.h>
#include <MsgJar.h>
#include <WalkingGait.h>
#include <ros/ros.h>

#define pi 3.1415926

class MotorCenter
{
public:
    int interval_counter = 0;
    int max_interval = 0;
    int lift_part = Tail::EVEN_PART, down_part = Tail::ODD_PART;
    float init_l[6];
    float init_angles[6][3];

    int motor_flow;

    float C_x[6], C_y[6], T_x[6], T_y[6];

    void set_waiting_time(float wait_sec);
    void run(legState* leg_set, float theta, float velocity_rate = 1.0, float support_height = Tail::SUPPORT_HEIGHT);
    void alter_lift_part();
    void compute_Arm_Angles_from_Orientation(legState* leg_set, float theta, float velocity_rate = 1.0);

    MotorCenter(/* args */);
    ~MotorCenter();
};


void MotorCenter::compute_Arm_Angles_from_Orientation(legState* leg_set, float theta, float velocity_rate){
    // compute unit direction vector
    Vector2f unit_direct = Vector2f(cos(theta*3.1415926/180.0), sin(theta*3.1415926/180.0));
    // obtain optimized delta_f from table
    float delta_f = Tail::FO_LIB[(int)theta]*velocity_rate;
    
    for(int i = this->lift_part; i < 4; i += 2){
        Vector2f Ti = Vector2f(this->T_x[i],this->T_y[i]);
        Vector2f Ci = Vector2f(this->C_x[i],this->C_y[i]);
        Vector2f li = Ti+delta_f*unit_direct-Ci;
        leg_set[i].cast_shadow_length = li.norm();
        float ac = (li.dot(Vector2f(1,0)))/leg_set[i].cast_shadow_length;
        float alpha = acos(ac);
        if( li[1] >= 0){
            if(alpha > pi/2){
                alpha = pi - alpha;
            }else{}
        }else{
            if(alpha > pi/2){
                alpha = - pi + alpha;
            }else{
                alpha = -alpha;
            }
        }
        leg_set[i].arm_angle_1 = alpha;
        WalkingGait_Update_Angles_by_ShadowLength(leg_set[i].cast_shadow_length, &leg_set[i].arm_angle_2, &leg_set[i].arm_angle_3);
        
    }
    for(int i = this->down_part; i < 4; i += 2){
        leg_set[i].arm_angle_2 = this->init_angles[i][0];
        leg_set[i].arm_angle_2 = this->init_angles[i][1];
        leg_set[i].arm_angle_2 = this->init_angles[i][1];
        leg_set[i].cast_shadow_length = this->init_l[i];
    }
    return;
}

void MotorCenter::alter_lift_part(){
    if(this->lift_part == Tail::EVEN_PART){
        this->lift_part = Tail::ODD_PART;
        this->down_part = Tail::EVEN_PART;
    }else{
        this->lift_part = Tail::EVEN_PART;
        this->down_part = Tail::ODD_PART;
    }
}

MotorCenter::MotorCenter(/* args */){
    this->init_l[0] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_1[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_1[2]);
    this->init_l[1] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_2[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_2[2]);
    this->init_l[2] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_3[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_3[2]);
    this->init_l[3] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_4[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_4[2]);
    this->init_l[4] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_5[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_5[2]);
    this->init_l[5] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_6[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_6[2]);

    this->C_x[0] = Tail::C1[0];    this->C_y[0] = Tail::C1[1];
    this->C_x[1] = Tail::C2[0];    this->C_y[1] = Tail::C2[1];
    this->C_x[2] = Tail::C3[0];    this->C_y[2] = Tail::C3[1];
    this->C_x[3] = Tail::C4[0];    this->C_y[3] = Tail::C4[1];
    this->C_x[4] = Tail::C5[0];    this->C_y[4] = Tail::C5[1];
    this->C_x[5] = Tail::C6[0];    this->C_y[5] = Tail::C6[1];

    this->T_x[0] = Tail::T1[0];    this->T_y[0] = Tail::T1[1];
    this->T_x[1] = Tail::T2[0];    this->T_y[1] = Tail::T2[1];
    this->T_x[2] = Tail::T3[0];    this->T_y[2] = Tail::T3[1];
    this->T_x[3] = Tail::T4[0];    this->T_y[3] = Tail::T4[1];
    this->T_x[4] = Tail::T5[0];    this->T_y[4] = Tail::T5[1];
    this->T_x[5] = Tail::T6[0];    this->T_y[5] = Tail::T6[1];

    this->init_angles[0][0] = Tail::ARM_ANGLES_1[0];    this->init_angles[0][1] = Tail::ARM_ANGLES_1[1];    this->init_angles[0][2] = Tail::ARM_ANGLES_1[2];
    this->init_angles[1][0] = Tail::ARM_ANGLES_2[0];    this->init_angles[1][1] = Tail::ARM_ANGLES_2[1];    this->init_angles[1][2] = Tail::ARM_ANGLES_2[2];
    this->init_angles[2][0] = Tail::ARM_ANGLES_3[0];    this->init_angles[2][1] = Tail::ARM_ANGLES_3[1];    this->init_angles[2][2] = Tail::ARM_ANGLES_3[2];
    this->init_angles[3][0] = Tail::ARM_ANGLES_4[0];    this->init_angles[3][1] = Tail::ARM_ANGLES_4[1];    this->init_angles[3][2] = Tail::ARM_ANGLES_4[2];
    this->init_angles[4][0] = Tail::ARM_ANGLES_5[0];    this->init_angles[4][1] = Tail::ARM_ANGLES_5[1];    this->init_angles[4][2] = Tail::ARM_ANGLES_5[2];
    this->init_angles[5][0] = Tail::ARM_ANGLES_6[0];    this->init_angles[5][1] = Tail::ARM_ANGLES_6[1];    this->init_angles[5][2] = Tail::ARM_ANGLES_6[2];
    
}

MotorCenter::~MotorCenter()
{
}

/**
 * @description: Since we do not have any feedback telling whether the legs should have already touched ground, MotorCenter has to 
 * wait a moment for legs completing its motion.   
 * @param {float} wait_sec
 * @return {*} none
 */
void MotorCenter::set_waiting_time(float wait_sec){
    this->max_interval = (int)(wait_sec/0.02);
    return;
}

/**
 * @description: Drive servos to work
 * @param {StateInfo*} state_info_ptr
 * @param {float} support_height
 * @return {*}
 */
void MotorCenter::run(legState* leg_set, float theta, float velocity_rate, float support_height){
    if(this->interval_counter != this->max_interval){
        this->interval_counter ++; 
        return;
    }else if(this->interval_counter == this->max_interval){
        this->interval_counter = 0;
        switch (this->motor_flow)
        {
            case Tail::MOTOR_FLOW_READY:{
                if(velocity_rate <= Tail::MIN_VELOCITY_RATE){ return; }
                this->compute_Arm_Angles_from_Orientation(leg_set, theta, velocity_rate);
                WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(leg_set, this->lift_part);
                Communication_Post_Servo_angles(leg_set);
                this->motor_flow ++;
                break;
            }
            case Tail::MOTOR_FLOW_WALKING1:{
                WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(leg_set, this->lift_part);
                Communication_Post_Servo_angles(leg_set);
                this->motor_flow ++;
                break;
            }
            case Tail::MOTOR_FLOW_WALKING2:{
                this->alter_lift_part();
                if(velocity_rate <= Tail::MIN_VELOCITY_RATE){
                    Initialize_leg_set(leg_set);
                    WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(leg_set, this->lift_part);
                    Communication_Post_Servo_angles(leg_set);
                    this->motor_flow = Tail::MOTOR_FLOW_RESET;
                }else{
                    this->compute_Arm_Angles_from_Orientation(leg_set, theta, velocity_rate);
                    WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(leg_set, this->lift_part);
                    Communication_Post_Servo_angles(leg_set);
                    this->motor_flow = Tail::MOTOR_FLOW_WALKING1;
                }
                break;
            }
            case Tail::MOTOR_FLOW_RESET:{
                WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(leg_set, this->lift_part);
                Communication_Post_Servo_angles(leg_set);
                this->motor_flow = Tail::MOTOR_FLOW_READY;
                break;
            }
            default:{
                ROS_ERROR("Wrong type of MOTOR_FLOW!!");
                break;
            }
        }

    }

}


#endif