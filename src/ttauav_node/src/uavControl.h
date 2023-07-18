#pragma once
#ifndef UAVCONTROL_H
#define UAVCONTROL_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ttauav_node/action.h>
#include <vector>
#include <cmath>


#define TIMETHRE 20 // 无人机动作控制时间阈值
#define X_c 0.77    // 无人机相对于无人车的位置前馈补偿——x
#define Y_c 0.42    // 无人机相对于无人车的位置前馈补偿——y
#define Kp 0.7      // 比例系数，用于计算无人机速度
#define THETA (270-246) //度数
#define RAD2DGR 0.01744 //角度转弧度的转换系数
#define VEL 0.3     // 无人机速度控制在0.3m/s


enum Action{ 
    FbV = 1,    // 速度控制
    ToL,    // 起飞降落
    G_C    // 云台控制
};

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
    // 跟随无人车
    bool ifFollow;
    // 扫码
    bool ifScan;
    int ScanFlag;
    int ScanIndex;
    int Action;
    std::vector<std::vector<float>> ScanPoints_0; //第一次起飞到第一次扫码
    std::vector<std::vector<float>> ScanPoints_1; //第一次扫码
    std::vector<std::vector<float>> ScanPoints_2; //第一次扫码到第一次降落
    
    std::vector<std::vector<float>> ScanPoints_3; //第二次起飞到第二次扫码
    std::vector<std::vector<float>> ScanPoints_4; //第二次扫码
    std::vector<std::vector<float>> ScanPoints_5; //第二次扫码到第二次降落
    int  GimbalControl;
    void sendtakeoffOrLanding(int takeoffOrLanding);
    void sendflightByVel(float velN, float velE, float velD, float velYaw,int fly_time);
    void sendflightByDis(float disN, float disE, float disD);
    void sendgimbalControl(float GimbalPitch);
    void sendfollow();
};

#endif // UAVCONTROL_H