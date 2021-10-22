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
 * @description: Change radian measure to degree measure
 * @param {float} radian
 * @return {float} degree
 */
float radian2degree(float radian){
    return radian*180.0/3.1415926;
}
/**
 * @description: Change degree measure to radian measure 
 * @param {float} degree
 * @return {float} radian
 */
float degree2radian(float degree){
    return degree*3.1415926/180.0;
}


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
    }else if(forward_distance < -Tail::MAX_FORWARD_DISTANCE){
        forward_distance = -Tail::MAX_FORWARD_DISTANCE;
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





#endif 
