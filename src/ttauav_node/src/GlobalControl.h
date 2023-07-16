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
    follow = 1,
    first_takeoff,
    first_scan,
    first_landing,
    second_takeoff,
    second_scan,
    second_landing
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
    int ugvStatus;
    bool alreadyTakeoff;
    int  GimbalControl;
    bool GimbalisDown;
    bool ifFollow;
    bool ifScan;
    int ScanFlag;
    int ScanIndex;
    void set_follow();
    void set_scan(int index);
    void set_takeoff();
    void set_landing();

public:
    GlobalControl();
    void GlobalControlUpdate();
    void GlobalParamUpdate();

};



#endif // GLOBALCONTROL_H