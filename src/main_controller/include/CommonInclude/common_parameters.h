#ifndef COMMON_PARAMETERS
#define COMMON_PARAMETERS

#define pi 3.1415926

typedef struct legState{
    // Physical parameters about the mechanical design
    float arm_length_1 = 0.1;
    float arm_length_2 = 0.1;
    float arm_length_3 = 0.1;
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
float ARM_ANGLES_1[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_2[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_3[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_4[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_5[3] = {0.0, 0.0, 0.0};
float ARM_ANGLES_6[3] = {0.0, 0.0, 0.0};

// Initial physical parameters about the mechanical design
float ARM_LENGTH_1 = 1.0;
float ARM_LENGTH_2 = 1.414;
float ARM_LENGTH_3 = 2.828;














}
#endif