#ifndef WALKING_GAIT 
#define WALKING_GAIT

#include <iostream>
#include <eigen3/Eigen/Dense>
#include <common_parameters.h>


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












#endif 
