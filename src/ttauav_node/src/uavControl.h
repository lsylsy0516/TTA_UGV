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
    void paramUpdate();
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    // 无人机控制参数
    bool ActionDown;
    // 起飞/降落
    int  takeoffOrLanding;
    bool alreadyTakeoff;
    // 跟随无人车
    bool ifFollow;
    // 扫码
    bool ifScan;
    int ScanFlag;
    int ScanIndex;
    std::vector<std::vector<float>> ScanPoints_1; //第一次扫码对应Points
    std::vector<std::vector<float>> ScanPoints_2; //第二次扫码对应Points
    int  GimbalControl;
    void sendtakeoffOrLanding(int takeoffOrLanding);
    void sendflightByVel(float velN, float velE, float velD, float velYaw,int fly_time);
    void sendgimbalControl(float GimbalPitch);
    void sendfollow();
};

#endif // UAVCONTROL_H