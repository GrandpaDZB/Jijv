/*
 * @Author: your name
 * @Date: 2021-08-30 22:07:12
 * @LastEditTime: 2021-08-30 23:23:04
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: /Jijv/src/main_controller/src/demo.cpp
 */
#include <WalkingGait.h>
#include <iostream>
using namespace std;

int main(int argc, char** argv){
    float angle_2;
    float angle_3;
    WalkingGait_Update_Angles_by_ShadowLength(4, &angle_2, &angle_3, 1);

    cout << angle_2 << " " << angle_3 << endl;

    return 0;
}