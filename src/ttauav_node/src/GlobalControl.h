#pragma once
#ifndef GLOBALCONTROL_H
#define GLOBALCONTROL_H
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>
#include <string>

enum UGVStatus{
    start = 1,
    follow,
    first_scan,
    temp_landing,
    second_takeoff,
    final_landing
};

/**
 * @brief 全局控制发布器
 * 用于在无人车上控制无人机。
 * 通过获取参数服务器中无人车的状态，
 * 设定参数服务器参数，
 * 让uavControler节点来控制无人机
*/
class GlobalControl
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    int ugvStatus;
    bool alreadyTakeoff;
    int  GimbalControl;
    bool ifFollow;
    bool ifScan;
    int ScanFlag;
    int ScanIndex;
    void GlobalControlCallback(const std_msgs::Bool::ConstPtr& msg);
    void GlobalControlUpdate();

public:
    GlobalControl();
    void GlobalParamUpdate();
};



#endif // GLOBALCONTROL_H