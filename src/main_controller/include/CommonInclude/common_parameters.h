#ifndef COMMON_PARAMETERS
#define COMMON_PARAMETERS

float pi = 3.1415926;

typedef struct legState{
    // Servo angles for each arm
    float arm_angle_1 = 0.0;
    float arm_angle_2 = 0.0;
    float arm_angle_3 = 0.0;
    // Cast shadow leg length
    float cast_shadow_length = 0.1;
}legState; 

namespace Tail{

// The height of Spider body
float SUPPORT_HEIGHT = 0.1;

// Initial serovo angles for each arm
float ARM_ANGLES_1[3] = {0.0, pi/4, pi/4};
float ARM_ANGLES_2[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_3[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_4[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_5[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_6[3] = {0.0, 0.0, 0.0};

// Initial physical parameters about the mechanical design
float ARM_LENGTH_1 = 1.0;
float ARM_LENGTH_2 = 1.414;
float ARM_LENGTH_3 = 2.828;

// Max forward distance.
float MAX_FORWARD_DISTANCE = 100;

// LIFT_ANGLE would add on arm_angle_2 when lifting
float LIFT_ANGLE = pi/6;










}
#endif