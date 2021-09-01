/*
 * @Author: your name
 * @Date: 2021-08-30 22:07:12
 * @LastEditTime: 2021-09-01 22:04:09
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/src/demo.cpp
 */
#include <WalkingGait.h>
#include <iostream>
#include <time.h>
#include <RangeComputer.h>
#include <MotorCenterV2.h>
using namespace std;

int main(int argc, char** argv){
    // MotorCenter model;
    // model.set_waiting_time(1.0);
    // time_t timer = clock();
    // time_t long_timer = clock();
    // StateInfo state_info;
    // state_info.motor_flow = Tail::READY0;
    // state_info.forward_distance = 1.0;
    // while(true){
    //     if((clock()-long_timer)/(float)CLOCKS_PER_SEC >= 5.0){
    //         state_info.forward_distance = 0.0;
    //     }
    //     if((clock()-timer)/(float)CLOCKS_PER_SEC >= 0.02){
    //         model.run(&state_info);
    //         timer = clock();
    //         cout << state_info.motor_flow << endl;
    //         cout << model.leg_set[0].cast_shadow_length*cos(model.leg_set[0].arm_angle_1) << " " <<model.leg_set[0].cast_shadow_length*sin(model.leg_set[0].arm_angle_1) << endl;
    //     }
    //     else{}
    // }

    float L_max = 4.0, L_min = 0.5;
    RangeComputer model_RC(0.0, 3.0, 20, 0.5, 4.0);
    model_RC.run(40, 3.1415926/6.0);

    return 0;
}