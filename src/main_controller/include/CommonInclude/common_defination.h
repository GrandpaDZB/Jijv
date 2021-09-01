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


}
#endif
