#pragma once
#ifndef UAVCONTROL_H
#define UAVCONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ttauav_node/action.h>
#include <vector>

#define MOVE 3

/**
 * @brief 无人机控制发布器
 * 用于在无人车上控制无人机，
 * 包括起飞降落、飞行、云台控制等
 */
class uavControl
{
public:
    uavControl();
    void sendUpdate();
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    int  takeoffOrLanding;
    int  GimbalControl;
    bool alreadyTakeoff;
    bool ifFollow;
    bool ifScan;
    std::vector<std::vector<float>> ScanPoints;
    // n*4的数组，分别对应NED和Yaw
    int ScanFlag;
    bool ActionDown;
    void sendtakeoffOrLanding(int takeoffOrLanding);
    void sendflightByVel(float velN, float velE, float velD, float velYaw,int fly_time);
    void sendgimbalControl(float GimbalPitch);
    void sendfollow();
};

#endif // UAVCONTROL_H