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
        case first_takeoff:
            if (!alreadyTakeoff) {                          // 先起飞
                ros::param::set("takeoffOrLanding", 1);
                ROS_INFO("Start game, Uav takeoff");
            }
            else if (!GimbalControl && !GimbalisDown){      // Gimbal need Control and is not Down
                ros::param::set("GimbalControl",2);
                ROS_INFO("GimbalDown");
                GimbalisDown = true ;
            }
            break;

        case follow:
            if (alreadyTakeoff && GimbalisDown ) {
                ros::param::set("ifFollow", true);
                ROS_INFO("Uav follow Ugv");
            }
            break;

        case first_scan:
            if (alreadyTakeoff && ifFollow && !GimbalControl && GimbalisDown) { // last status is follow and GimbalisDown
                ros::param::set("GimbalControl", 1);
                ROS_INFO("GimbalUp");
                break;
            }
            if (alreadyTakeoff && ifFollow && ScanIndex == 0) {
                ros::param::set("ifFollow", false);
                ros::param::set("ifScan", true); 
                ros::param::set("ScanIndex", 1); //第一次的轨迹
                ROS_INFO("Uav scan 1");
            }
            break;

        case first_landing:
            if (alreadyTakeoff && !ifScan && ifFollow) { //扫码已经结束
                ros::param::set("ifFollow", false);
                ros::param::set("takeoffOrLanding", 2);
                // 降落时会自动回正
                // ros::param::set("GimbalControl", 1);
                GimbalisDown = false ;
                ROS_INFO("Uav temp_landing");
            }
            break;

        case second_takeoff:
            if (!alreadyTakeoff) {                          // 先起飞
                ros::param::set("takeoffOrLanding", 1);
                ROS_INFO("Start game, Uav takeoff");
            }
            else if (!GimbalControl && !GimbalisDown){      // Gimbal need Control and is not Down
                ros::param::set("GimbalControl",2);
                ROS_INFO("GimbalDown");
                GimbalisDown = true ;
            }
            break;

        case second_scan:
            if (alreadyTakeoff && ifFollow && !GimbalControl && GimbalisDown) { // last status is follow and GimbalisDown
                ros::param::set("GimbalControl", 1);
                ROS_INFO("GimbalUp");
                break;
            }
            if (alreadyTakeoff && ifFollow && ScanIndex == 1) {
                ros::param::set("ifFollow", false);
                ros::param::set("ifScan", true); 
                ros::param::set("ScanIndex", 2); //第二次的轨迹
                ROS_INFO("Uav scan 1");
            }
            break;


        case second_landing:
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



GlobalControl::GlobalControl()
{
    // 初始化参数
    alreadyTakeoff = false;
    ifFollow = false;
    ifScan = false;
    ScanFlag = 0;
    ugvStatus = 0;
    GimbalisDown = false;

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
        globalControl.GlobalControlUpdate();
        loop_rate.sleep();
    }
    return 0;
}