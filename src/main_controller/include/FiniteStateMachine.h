#ifndef FINITE_STATE_MACHINE
#define FINITE_STATE_MACHINE


#include <stdarg.h>
#include <ros/ros.h>
#include <iostream>
#include <workers_common_include.h>
#include <common_defination.h>
#define INFINITY 99999
using namespace std;

class FSM{
private:
    // main loop function runs in a specific frequency set by timer
    ros::Timer FSM_timer;
    // subscriber & publisher
    // ros::Subscriber sub_state, sub_vel, sub_rc, sub_position;
    // ros::Publisher servo_load_pub;

public:
    ros::NodeHandle nh;
    vector<StateWorker*> Workers;
    int flow = 0;
    StateInfo state_info;

    void loop(const ros::TimerEvent &);
    void build_ScheduleTable(int Schedule, ...);



    // Callback Functions
/*     void StateCallback(const mavros_msgs::StateConstPtr &msg);
    void VelocityCallback(const geometry_msgs::TwistStampedConstPtr &msg);
    void RCCallback(const mavros_msgs::RCInConstPtr &msg);
    void PositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
    void ButtonCallback(const std_msgs::BoolConstPtr &msg); */
    
    void set_timer();


    FSM(ros::NodeHandle &nh);
    ~FSM();
};

// ===================== Realization =====================


FSM::FSM(ros::NodeHandle &nh){
    this->nh = nh;
    // Initialize subscriber & publisher
/*     this->sub_state = nh.subscribe("/mavros/state", 10, &FSM::StateCallback, this);
    this->sub_vel = nh.subscribe("/mavros/local_position/velocity", 10, &FSM::VelocityCallback, this);
    this->sub_rc = nh.subscribe("/mavros/rc/in", 10, &FSM::RCCallback, this);
    this->sub_position = nh.subscribe("/mavros/vision_pose/pose", 10, &FSM::PositionCallback, this);
    this->servo_load_pub = nh.advertise<geometry_msgs::Vector3>("/sun/servo_ctl", 10); */
}

FSM::~FSM(){
    // free memory
    for(auto each:this->Workers){
        delete each;
    }
}

void FSM::set_timer(){
    this->FSM_timer = nh.createTimer(ros::Duration(0.02), &FSM::loop, this); 
    return;   
}

// main loop func, where the selected worker works
void FSM::loop(const ros::TimerEvent &){
    // running by schedule table
    if(this->Workers[this->flow]->is_finished()){
        this->flow ++;
        if(this->flow == this->Workers.size()){
            ROS_INFO("Finish ScheduleTable!");
            exit(0);
        }
    }else{
        this->Workers[this->flow]->run(this->state_info);
    }
    // update state_info
/*     update_bluetooth_data(&this->state_info);
    update_serial_data(&this->state_info); */
}



/* @brief build schedule table
@param sun::DOCKING, int type, int level 
@param sun::ADAPTIVE_LEARNING, int max_iter, float expected_height
@param sun::DETECTING, float direction_vector_x, float direction_vector_y, float max_sec, int figure_type
@param sun::LANDING, None
@param sun::TAKEOFF, float height
@param sun::WAITING, None
@param sun::HEIGHTSERV, float expected_height, float max_speed
*/
void FSM::build_ScheduleTable(int Schedule, ...){
    va_list arg_ptr;
    va_start(arg_ptr, Schedule);
    while(Schedule != Tail::END){
        switch (Schedule){
            case Tail::MANUAL:{
/*                 int type = va_arg(arg_ptr, int);
                int level = va_arg(arg_ptr, int);
                int bias_x = va_arg(arg_ptr, int);
                int bias_y = va_arg(arg_ptr, int);
                DockingWorker* tmp_worker = new DockingWorker(this->nh, type, level, bias_x, bias_y);
                this->Workers.push_back((StateWorker*)tmp_worker); */
                break;
            }
            default:
                ROS_ERROR("Wrong type of Schedule Table!");
                exit(0);
                break;
        }
        Schedule = va_arg(arg_ptr, int);
    }
    return;
}

// ================ callback ===================
/* void FSM::StateCallback(const mavros_msgs::StateConstPtr &msg){
    this->state_info.armed = msg->armed;
    this->state_info.connected = msg->connected;
    this->state_info.mode = msg->mode;
    return;
} */



#endif
