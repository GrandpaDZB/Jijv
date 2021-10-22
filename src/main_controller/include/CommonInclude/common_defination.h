#ifndef COMMON_DEFINATION
#define COMMON_DEFINATION
namespace Tail{



// StateMode
enum STATE_TYPE{
    MANUAL,
    END,
};

// two half of legs
enum HALF_LEGS_TYPE{
    ODD_PART,
    EVEN_PART,
};

// Motor flow
enum MOTOR_FLOW{
    MOTOR_FLOW_READY,
    MOTOR_FLOW_WALKING1,
    MOTOR_FLOW_WALKING2,
    MOTOR_FLOW_RESET,
};

// Servo index
enum SERVO_INDEX{
    LEG_1_ARM_1 = 12,
    LEG_1_ARM_2 = 13,
    LEG_1_ARM_3 = 14,
    LEG_2_ARM_1 = 0,
    LEG_2_ARM_2 = 1,
    LEG_2_ARM_3 = 2,
    LEG_3_ARM_1 = 3,
    LEG_3_ARM_2 = 4,
    LEG_3_ARM_3 = 5,
    LEG_4_ARM_1 = 6,
    LEG_4_ARM_2 = 7,
    LEG_4_ARM_3 = 8,
    LEG_5_ARM_1 = 18,
    LEG_5_ARM_2 = 19,
    LEG_5_ARM_3 = 20,
    LEG_6_ARM_1 = 15,
    LEG_6_ARM_2 = 16,
    LEG_6_ARM_3 = 17,
};


}
#endif
