#include "GlobalControl.h"


/**
 * @brief 用于更新参数服务器中的参数
 * 频率为10Hz
 * 逻辑：
 * 读取参数服务器中的参数，根据参数服务器中的参数，更新无人机控制发布器中的参数
 */
void GlobalControl::GlobalParamUpdate()
{
    // 读取参数
    ugvStatus = ros::param::param("ugvStatus", 0);
    alreadyTakeoff = ros::param::param("alreadyTakeoff", false);
    ifFollow = ros::param::param("ifFollow", false);
    ifScan = ros::param::param("ifScan", false);
    ScanFlag = ros::param::param("ScanFlag", 0);
    ScanIndex = ros::param::param("ScanIndex", 0);
    GimbalControl = ros::param::param("GimbalControl", 0);
}


/**
 * @brief 用于更新无人机状态
 */
void GlobalControl::GlobalControlUpdate(){
    switch (ugvStatus) {
        case start:
            if (!alreadyTakeoff) {
                ros::param::set("takeoffOrLanding", 1);
                ROS_INFO("Start game, Uav takeoff");
            }
            break;

        case follow:
            if (alreadyTakeoff) {
                ros::param::set("ifFollow", true);
                ROS_INFO("Uav follow Ugv");
            }
            break;

        case first_scan:
            if (alreadyTakeoff && ifFollow && GimbalControl) {
                ros::param::set("GimbalControl", 1);
                ROS_INFO("GimbalUp");
            }

            if (alreadyTakeoff && ifFollow && ScanIndex == 0) {
                ros::param::set("ifFollow", false);
                ros::param::set("ifScan", true);
                ros::param::set("ScanIndex", 1);
                ROS_INFO("Uav scan");
            }
            break;

        case temp_landing:
            if (alreadyTakeoff) {
                ros::param::set("ifFollow", false);
                ros::param::set("takeoffOrLanding", 2);
                ROS_INFO("Uav temp_landing");
            }
            break;

        case second_takeoff:
            if (!alreadyTakeoff) {
                ros::param::set("takeoffOrLanding", 1);
                ROS_INFO("Uav second_takeoff");
            }
            break;

        case final_landing:
            if (alreadyTakeoff && ifFollow && ScanIndex == 2) {
                ros::param::set("ifFollow", false);
                ros::param::set("takeoffOrLanding", 2);
                ROS_INFO("Uav final_landing");
            }
            break;

        default:
            break;
    }
}

void GlobalControl::GlobalControlCallback(const std_msgs::Bool::ConstPtr& msg){
    GlobalControlUpdate();
    ROS_INFO("status Update!");
}

GlobalControl::GlobalControl()
{
    // 初始化参数
    alreadyTakeoff = false;
    ifFollow = false;
    ifScan = false;
    ScanFlag = 0;
    ugvStatus = 0;
    ROS_INFO("GlobalControl_Inited!");
    // 订阅更新状态请求
    sub = nh.subscribe<std_msgs::Bool>("updateStatus", 10, &GlobalControl::GlobalControlCallback, this);
}

int main(int argc, char  *argv[]){
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"global_control_node");
    GlobalControl globalControl;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        globalControl.GlobalParamUpdate();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}