/*
 * @Author: your name
 * @Date: 2021-08-30 22:07:12
 * @LastEditTime: 2021-08-31 09:55:50
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/src/demo.cpp
 */
#include <WalkingGait.h>
#include <iostream>
using namespace std;

int main(int argc, char** argv){
    legState leg_set[6];
    Initialize_leg_set(leg_set);

    WalkingGait_Update_halfPart_Angles_by_forwardDistance(leg_set, Tail::ODD_PART, 4.0, 1.0);
    cout << leg_set[0].cast_shadow_length << endl;

    return 0;
}