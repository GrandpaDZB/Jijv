#ifndef COMMUNICATION
#define COMMUNICATION

#include <common_parameters.h>
#include <common_defination.h>
#include <ros/ros.h>
#include <string.h>
#include <serial/serial.h>
#include <iostream>
using namespace std;

typedef union SerialTrans{
    const char* str;
    uint8_t buffer[1000];

}SerialTrans;


/**
 * @description: Create a serial port ptr with given parameters. You should delete this ptr after its work done.
 * @param {string} Portname ; default :"/dev/ttyUSB0" 
 * @param {int} Baudrate ; default: 115200
 * @return {*}
 */
serial::Serial* Communication_Create_SerialPort(string Portname = "/dev/ttyUSB0", int Baudrate = 115200){
    serial::Serial* sp = new serial::Serial;
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    sp->setPort(Portname);
    sp->setBaudrate(Baudrate);
    sp->setTimeout(timeout);
    sp->open();
    if(!sp->isOpen()){
        ROS_ERROR("Fail to open serial port. Please check settings!");
    }
    return sp;
}

/**
 * @description: This function do a part work of servo control - add on a control msg about the given servo
 * @param {uint8_t*} buffer; uint8_t array
 * @param {int} index; the position where uint8_t array need to add on new msg.
 * @param {int} Servo_index
 * @param {float} angle; radian measure
 * @return {int} updated index
 */
int Communication_Servo_control_part_work(uint8_t* buffer, int index, int Servo_index, float angle){
    angle = angle*180.0/3.1415926;
    cout << angle << endl;
    int PWM = (int)(11.11*angle + 500);
    buffer[index] = (int)'#'; index++;
    if(Servo_index >= 10){
        buffer[index] = (int)(Servo_index/10%10)+48; index++;
        buffer[index] = (int)(Servo_index%10)+48; index++;
    }else{
        buffer[index] = (int)(Servo_index%10)+48; index++;
    }
    buffer[index] = (int)'P'; index++;
    if(PWM >= 1000){
        buffer[index] = (int)(PWM/1000)+48; index++;
        buffer[index] = (int)(PWM/100%10)+48; index++;
        buffer[index] = (int)(PWM/10%10)+48; index++;
        buffer[index] = (int)(PWM/1%10)+48; index++;
    }else{
        buffer[index] = (int)(PWM/100%10)+48; index++;
        buffer[index] = (int)(PWM/10%10)+48; index++;
        buffer[index] = (int)(PWM/1%10)+48; index++;
    }
    return index;
}

void Communication_Post_Servo_angles(legState* leg_set){
    uint8_t buffer[1000];
    int index = 0;
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_1_ARM_1, leg_set[0].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_1_ARM_2, leg_set[0].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_1_ARM_3, leg_set[0].arm_angle_3);

    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_2_ARM_1, leg_set[1].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_2_ARM_2, leg_set[1].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_2_ARM_3, leg_set[1].arm_angle_3);

    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_3_ARM_1, leg_set[2].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_3_ARM_2, leg_set[2].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_3_ARM_3, leg_set[2].arm_angle_3);

    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_4_ARM_1, leg_set[3].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_4_ARM_2, leg_set[3].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_4_ARM_3, leg_set[3].arm_angle_3);

    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_5_ARM_1, leg_set[4].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_5_ARM_2, leg_set[4].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_5_ARM_3, leg_set[4].arm_angle_3);

    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_6_ARM_1, leg_set[5].arm_angle_1);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_6_ARM_2, leg_set[5].arm_angle_2);
    index = Communication_Servo_control_part_work(buffer, index, Tail::LEG_6_ARM_3, leg_set[5].arm_angle_3);

    for(int i = 0; i < index; i++){
        cout << buffer[i];
    }

    buffer[index] = (int)'\r'; index++;
    buffer[index] = (int)'\n'; index++;

    return;
    

}


/**
 * @description: Control a servo to rotate the given angle
 * @param {serial::Serial*} sp; serial port ptr
 * @param {int} Servo_index
 * @param {float} angle ; range-[0, 180]
 * @return {*}
 */
void Communication_Servo_control(serial::Serial* sp,int Servo_index, float angle){
    int PWM = (int)(11.11*angle + 500);
    uint8_t serial_data[100];
    int index = 0;
    serial_data[index] = (int)'#'; index++;
    if(Servo_index >= 10){
        serial_data[index] = (int)(Servo_index/10%10)+48; index++;
        serial_data[index] = (int)(Servo_index%10)+48; index++;
    }else{
        serial_data[index] = (int)(Servo_index%10)+48; index++;
    }
    serial_data[index] = (int)'P'; index++;
    if(PWM >= 1000){
        serial_data[index] = (int)(PWM/1000)+48; index++;
        serial_data[index] = (int)(PWM/100%10)+48; index++;
        serial_data[index] = (int)(PWM/10%10)+48; index++;
        serial_data[index] = (int)(PWM/1%10)+48; index++;
    }else{
        serial_data[index] = (int)(PWM/100%10)+48; index++;
        serial_data[index] = (int)(PWM/10%10)+48; index++;
        serial_data[index] = (int)(PWM/1%10)+48; index++;
    }
    serial_data[index] = (int)'\r'; index++;
    serial_data[index] = (int)'\n'; index++;
    sp->write(serial_data, index);
    return;
}




/**
 * @description: Transform string to uint8_t array, and return the data size of it.
 * @param {string} str
 * @param {uint8_t*} buffer
 * @return {*}
 */
int Communication_string_to_uint8array(string str, uint8_t* buffer){
    SerialTrans ST;
    ST.str = str.data();
    int str_index = 0;
    while((int)ST.str[str_index] != 0x0a){
        cout << hex << (int)ST.str[str_index] << endl;
        buffer[str_index] = (int)ST.str[str_index];
        str_index ++;
    }
    buffer[str_index] = (int)ST.str[str_index];
    str_index ++;
    return str_index;
}








#endif