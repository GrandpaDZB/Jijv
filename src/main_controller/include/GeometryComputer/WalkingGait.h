#ifndef WALKING_GAIT 
#define WALKING_GAIT

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <common_parameters.h>
#include <common_defination.h>
#include <Communication.h>
#include <MsgJar.h>
#include <ros/ros.h>


/**
 * @description: A single cast-shadow leg length could uniquely reflect into a set of angles --- angle_2 & angle_3. This function does help with updating 
 * angles' data according to a given shadow length.
 * @param {float} shadow_length ; Cast-shadow leg length
 * @param {float*} angle_2 ; leg angle 2
 * @param {float*} angle_3 ; leg angle 3
 * @param {float} support_height = Tail::SUPPORT_HEIGHT ; Default parameter should be modified when using another support height
 * @return {*} none
 */
void WalkingGait_Update_Angles_by_ShadowLength(float shadow_length, float* angle_2, float* angle_3, float support_height = Tail::SUPPORT_HEIGHT){
    float A = shadow_length - Tail::ARM_LENGTH_1;
    float h = support_height;
    float b = Tail::ARM_LENGTH_2;
    float c = Tail::ARM_LENGTH_3;
    float K = (A*A+h*h+b*b-c*c)/(2*b*sqrt(h*h+A*A));
    *angle_2 = asin(-K) + atan(A/h);
    *angle_3 = acos((h+b*sin(*angle_2))/c);
    return;
}



/**
 * @description: Update the angles of a given part of legs according to a forward_distance
 * @param {legState*} leg_set ; Array of legState
 * @param {int} half ; Tail::ODD_PART or Tail::EVEN_PART
 * @param {float} forward_distance ; The max value of it is Tail::MAX_FORWARD_DISTANCE
 * @param {float} support_height ; Default parameter should be modified when using another support height
 * @return {*} none
 */
void WalkingGait_Update_halfPart_Angles_by_forwardDistance(legState* leg_set, int half, float forward_distance, float support_height = Tail::SUPPORT_HEIGHT){
    int init_index = 0;
    if(half == Tail::ODD_PART){
        init_index = 1;
    }else if(half == Tail::EVEN_PART){
        init_index = 2;
    }else{
        init_index = 1;
        ROS_ERROR("Wrong type of HALF_LEG_TYPE!");
    }
    if(forward_distance > Tail::MAX_FORWARD_DISTANCE){
        forward_distance = Tail::MAX_FORWARD_DISTANCE;
    }

    for(int i = 0; i < 3; init_index += 2, i++){
        float y = leg_set[init_index-1].cast_shadow_length*sin(leg_set[init_index-1].arm_angle_1);
        float x = leg_set[init_index-1].cast_shadow_length*cos(leg_set[init_index-1].arm_angle_1);
        float new_y = y + forward_distance;
        leg_set[init_index-1].arm_angle_1 = atan(new_y / x);
        leg_set[init_index-1].cast_shadow_length = sqrt(new_y*new_y + x*x);
        WalkingGait_Update_Angles_by_ShadowLength(
            leg_set[init_index-1].cast_shadow_length,
            &leg_set[init_index-1].arm_angle_2,
            &leg_set[init_index-1].arm_angle_3,
            support_height    
        );
    }
    return;
}


/**
 * @description: Add a bias angle on arm_angle_2 so that the leg could lift up.
 * @param {legState*} leg_set
 * @param {int} half
 * @return {*} none
 */
void WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(legState* leg_set, int half){
    int init_index = 0;
    if(half == Tail::ODD_PART){
        init_index = 1;
    }else if(half == Tail::EVEN_PART){
        init_index = 2;
    }else{
        init_index = 1;
        ROS_ERROR("Wrong type of HALF_LEG_TYPE!");
    }
    for(int i = 0; i < 3; init_index += 2, i++){
        leg_set[init_index-1].arm_angle_2 += Tail::LIFT_ANGLE;
    }
    return;
}

/**
 * @description: Add a minus bias angle on arm_angle_2 so that the leg could fall down.
 * @param {legState*} leg_set
 * @param {int} half
 * @return {*} none
 */
void WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(legState* leg_set, int half){
    int init_index = 0;
    if(half == Tail::ODD_PART){
        init_index = 1;
    }else if(half == Tail::EVEN_PART){
        init_index = 2;
    }else{
        init_index = 1;
        ROS_ERROR("Wrong type of HALF_LEG_TYPE!");
    }
    for(int i = 0; i < 3; init_index += 2, i++){
        leg_set[init_index-1].arm_angle_2 -= Tail::LIFT_ANGLE;
    }
    return;
}


/**
 * @description: This function helps you initialize legState data when program first runs
 * @param {legState*} leg_set ; Array of legState
 * @return {*} none
 */
void Initialize_leg_set(legState* leg_set){
    leg_set[0].arm_angle_1 = Tail::ARM_ANGLES_1[0];
    leg_set[0].arm_angle_2 = Tail::ARM_ANGLES_1[1];
    leg_set[0].arm_angle_3 = Tail::ARM_ANGLES_1[2];

    leg_set[1].arm_angle_1 = Tail::ARM_ANGLES_2[0];
    leg_set[1].arm_angle_2 = Tail::ARM_ANGLES_2[1];
    leg_set[1].arm_angle_3 = Tail::ARM_ANGLES_2[2];

    leg_set[2].arm_angle_1 = Tail::ARM_ANGLES_3[0];
    leg_set[2].arm_angle_2 = Tail::ARM_ANGLES_3[1];
    leg_set[2].arm_angle_3 = Tail::ARM_ANGLES_3[2];

    leg_set[3].arm_angle_1 = Tail::ARM_ANGLES_4[0];
    leg_set[3].arm_angle_2 = Tail::ARM_ANGLES_4[1];
    leg_set[3].arm_angle_3 = Tail::ARM_ANGLES_4[2];

    leg_set[4].arm_angle_1 = Tail::ARM_ANGLES_5[0];
    leg_set[4].arm_angle_2 = Tail::ARM_ANGLES_5[1];
    leg_set[4].arm_angle_3 = Tail::ARM_ANGLES_5[2];
    
    leg_set[5].arm_angle_1 = Tail::ARM_ANGLES_6[0];
    leg_set[5].arm_angle_2 = Tail::ARM_ANGLES_6[1];
    leg_set[5].arm_angle_3 = Tail::ARM_ANGLES_6[2];

    leg_set[0].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_1[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_1[2]);
    leg_set[1].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_2[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_2[2]);
    leg_set[2].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_3[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_3[2]);
    leg_set[3].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_4[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_4[2]);
    leg_set[4].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_5[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_5[2]);
    leg_set[5].cast_shadow_length = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_6[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_6[2]);
    
    return;
}

// =========================================== class Motor CTail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_1[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_1[2]);enter ===================================================
class MotorCenter
{
public:
    int interval_counter = 0;
    int max_interval = 0;
    int lift_part = 0, down_part = 0;
    float init_l[6];
    bool reset_lock = false;
    legState leg_set[6];

    void set_waiting_time(float wait_sec);
    void run(StateInfo* state_info_ptr, float support_height = Tail::SUPPORT_HEIGHT);

    MotorCenter(/* args */);
    ~MotorCenter();
};

MotorCenter::MotorCenter(/* args */){
    Initialize_leg_set(this->leg_set);
    this->init_l[0] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_1[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_1[2]);
    this->init_l[1] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_2[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_2[2]);
    this->init_l[2] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_3[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_3[2]);
    this->init_l[3] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_4[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_4[2]);
    this->init_l[4] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_5[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_5[2]);
    this->init_l[5] = Tail::ARM_LENGTH_1 + Tail::ARM_LENGTH_2*cos(Tail::ARM_ANGLES_6[1]) + Tail::ARM_LENGTH_3*sin(Tail::ARM_ANGLES_6[2]);
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
void MotorCenter::run(StateInfo* state_info_ptr, float support_height){
    if(this->interval_counter != this->max_interval){
        this->interval_counter ++; 
        return;
    }else if(this->interval_counter == this->max_interval && state_info_ptr->forward_distance != 0.0 && this->reset_lock == false){
        this->interval_counter = 0;
        switch (state_info_ptr->motor_flow)
        {
        case Tail::READY0:{
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::ODD_PART, state_info_ptr->forward_distance, support_height);
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::ODD_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::READY1:{
            WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(this->leg_set, Tail::ODD_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::READY2:{
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::EVEN_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::READY3:{
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::EVEN_PART, state_info_ptr->forward_distance, support_height);
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::EVEN_PART);
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::ODD_PART, -2*state_info_ptr->forward_distance, support_height);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING1:{
            WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(this->leg_set, Tail::EVEN_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING2:{
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::ODD_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING3:{
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::ODD_PART, 2*state_info_ptr->forward_distance, support_height);
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::ODD_PART);
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::EVEN_PART, -2*state_info_ptr->forward_distance, support_height);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING4:{
            WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(this->leg_set, Tail::ODD_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING5:{
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::EVEN_PART);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow ++; 
            break;
        }
        case Tail::WALKING6:{
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::EVEN_PART, 2*state_info_ptr->forward_distance, support_height);
            WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, Tail::EVEN_PART);
            WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, Tail::ODD_PART, -2*state_info_ptr->forward_distance, support_height);
            Communication_Post_Servo_angles(this->leg_set);
            state_info_ptr->motor_flow = Tail::WALKING1; 
            break;
        }
        default:
            ROS_ERROR("Wrong type of Motor flow");
            break;
        }
    }else{
        if(state_info_ptr->motor_flow == Tail::READY0){return;}
        // change back to ready state
        if(this->reset_lock == false){
            this->reset_lock = true;
            this->lift_part = 0;
            if(
                state_info_ptr->motor_flow == Tail::READY1   ||
                state_info_ptr->motor_flow == Tail::WALKING3 ||
                state_info_ptr->motor_flow == Tail::WALKING4){
                this->lift_part = Tail::ODD_PART;
                this->down_part = Tail::EVEN_PART;
                state_info_ptr->motor_flow = Tail::RESET1;
            }else if(
                state_info_ptr->motor_flow == Tail::READY3   ||
                state_info_ptr->motor_flow == Tail::WALKING1 ||
                state_info_ptr->motor_flow == Tail::WALKING6){
                this->lift_part = Tail::EVEN_PART;
                this->down_part = Tail::ODD_PART;
                state_info_ptr->motor_flow = Tail::RESET1;
            }else{
                this->lift_part = Tail::EVEN_PART;
                this->down_part = Tail::ODD_PART;
                state_info_ptr->motor_flow = Tail::RESET0;
            }
            this->interval_counter = this->max_interval;
            return;
        }else{
            switch (state_info_ptr->motor_flow)
            {
            case Tail::RESET0:{
                float diff_y = (
                    (this->init_l[1]*sin(Tail::ARM_ANGLES_2[0]) - this->leg_set[1].cast_shadow_length*sin(this->leg_set[1].arm_angle_1))+
                    (this->init_l[3]*sin(Tail::ARM_ANGLES_4[0]) - this->leg_set[3].cast_shadow_length*sin(this->leg_set[3].arm_angle_1))+
                    (this->init_l[5]*sin(Tail::ARM_ANGLES_6[0]) - this->leg_set[5].cast_shadow_length*sin(this->leg_set[5].arm_angle_1))
                )/3.0;
                WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, this->lift_part, diff_y, support_height);
                WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, this->lift_part);
                Communication_Post_Servo_angles(this->leg_set);
                state_info_ptr->motor_flow = Tail::RESET2;
            }
            case Tail::RESET1:{
                float diff_y;
                if(this->lift_part == Tail::EVEN_PART){
                    diff_y = (
                        (this->init_l[1]*sin(Tail::ARM_ANGLES_2[0]) - this->leg_set[1].cast_shadow_length*sin(this->leg_set[1].arm_angle_1))+
                        (this->init_l[3]*sin(Tail::ARM_ANGLES_4[0]) - this->leg_set[3].cast_shadow_length*sin(this->leg_set[3].arm_angle_1))+
                        (this->init_l[5]*sin(Tail::ARM_ANGLES_6[0]) - this->leg_set[5].cast_shadow_length*sin(this->leg_set[5].arm_angle_1))
                    )/3.0;
                }else{
                    diff_y = (
                        (this->init_l[0]*sin(Tail::ARM_ANGLES_1[0]) - this->leg_set[0].cast_shadow_length*sin(this->leg_set[0].arm_angle_1))+
                        (this->init_l[2]*sin(Tail::ARM_ANGLES_3[0]) - this->leg_set[2].cast_shadow_length*sin(this->leg_set[2].arm_angle_1))+
                        (this->init_l[4]*sin(Tail::ARM_ANGLES_5[0]) - this->leg_set[4].cast_shadow_length*sin(this->leg_set[4].arm_angle_1))
                    )/3.0;
                }
                WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, this->lift_part, diff_y, support_height);
                Communication_Post_Servo_angles(this->leg_set);
                state_info_ptr->motor_flow = Tail::RESET2;
            }
            case Tail::RESET2:{
                WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(this->leg_set, this->lift_part);
                Communication_Post_Servo_angles(this->leg_set);
                state_info_ptr->motor_flow ++;
            }
            case Tail::RESET3:{
                float diff_y;
                if(this->down_part == Tail::EVEN_PART){
                    diff_y = (
                        (this->init_l[1]*sin(Tail::ARM_ANGLES_2[0]) - this->leg_set[1].cast_shadow_length*sin(this->leg_set[1].arm_angle_1))+
                        (this->init_l[3]*sin(Tail::ARM_ANGLES_4[0]) - this->leg_set[3].cast_shadow_length*sin(this->leg_set[3].arm_angle_1))+
                        (this->init_l[5]*sin(Tail::ARM_ANGLES_6[0]) - this->leg_set[5].cast_shadow_length*sin(this->leg_set[5].arm_angle_1))
                    )/3.0;
                }else{
                    diff_y = (
                        (this->init_l[0]*sin(Tail::ARM_ANGLES_1[0]) - this->leg_set[0].cast_shadow_length*sin(this->leg_set[0].arm_angle_1))+
                        (this->init_l[2]*sin(Tail::ARM_ANGLES_3[0]) - this->leg_set[2].cast_shadow_length*sin(this->leg_set[2].arm_angle_1))+
                        (this->init_l[4]*sin(Tail::ARM_ANGLES_5[0]) - this->leg_set[4].cast_shadow_length*sin(this->leg_set[4].arm_angle_1))
                    )/3.0;
                }
                WalkingGait_Update_halfPart_Angles_by_forwardDistance(this->leg_set, this->down_part, diff_y, support_height);
                WalkingGait_Update_halfPart_Angles_with_LIFT_ANGLE(this->leg_set, this->down_part);
                Communication_Post_Servo_angles(this->leg_set);
                state_info_ptr->motor_flow ++;
            }
            case Tail::RESET4:{
                WalkingGait_Update_halfPart_Angles_with_FALL_ANGLE(this->leg_set, this->down_part);
                Communication_Post_Servo_angles(this->leg_set);
                state_info_ptr->motor_flow = Tail::READY0;
                this->reset_lock = false;
            }
            default:
                ROS_ERROR("Wrong type of RESET motor flow");
                break;
            }
        }

    }
}



#endif 
