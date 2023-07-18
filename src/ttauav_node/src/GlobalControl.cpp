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
    ugvStatus = ros::param::param("StatusUpdate", 0);
}


/**
 * @brief 用于更新无人机状态
 */
void GlobalControl::GlobalControlUpdate(){
    switch (ugvStatus) {
        case takeoff:
            ros::param::set("takeoffOrLanding", 1);
            ros::param::set("GimbalControl",0);
            
            ros::param::set("ifScan", false);
            ros::param::set("ifFollow",false);
            ROS_INFO("Uav Takeoff and GimbalDown"); 
            break;

        case follow:
            ros::param::set("ifFollow", true);

            ros::param::set("takeoffOrLanding", 0);
            ros::param::set("GimbalControl",2);
            ros::param::set("ifScan", false);
            ROS_INFO("Start Following");
            break;

        case landing:
            ros::param::set("takeoffOrLanding", 2);

            ros::param::set("GimbalControl",0);
            ros::param::set("ifFollow", false);
            ros::param::set("ifScan", false);
            ROS_INFO("Uav Landing");
            break;

        case first_scan:
            ros::param::set("GimbalControl",1);
            ros::param::set("ScanIndex",0); //第一次初始化扫码序列为0
            ros::param::set("ifScan", true);

            ros::param::set("takeoffOrLanding", 0);
            ros::param::set("ifFollow", false);
            ROS_INFO("First_Scaning");
            break;

        case second_scan:
            ros::param::set("GimbalControl",1);
            ros::param::set("ifScan", true);
            // ros::param::set("ScanIndex",2); ,不设置ScanIndex，ScanIndex由uavControl中的scan函数自动更新

            ros::param::set("takeoffOrLanding", 0);
            ros::param::set("ifFollow", false);
            ROS_INFO("Second_Scaning");
            break;

        default:
            break;
    }
}

void GlobalControl::StatusUpdate_cb(const std_msgs::Bool::ConstPtr& msg_p){
    GlobalControlUpdate();
}


GlobalControl::GlobalControl()
{
    ugvStatus = 0;
    sub = nh.subscribe<std_msgs::Bool>("updateStatus",10,&GlobalControl::StatusUpdate_cb,this);
    ROS_INFO("GlobalControl_Inited!");
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