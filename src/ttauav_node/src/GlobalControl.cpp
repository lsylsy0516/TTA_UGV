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
    ugvStatus = ros::param::param<std::string>("ugvStatus", " ");
    alreadyTakeoff = ros::param::param("alreadyTakeoff", false);
    ifFollow = ros::param::param("ifFollow", false);
    ifScan = ros::param::param("ifScan", false);
    ScanFlag = ros::param::param("ScanFlag", 0);
    ScanIndex = ros::param::param("ScanIndex", 0);
}

/**
 * @brief 用于更新无人机状态
 */
void GlobalControl::GlobalControlUpdate()
{    
    // 比赛开始时，无人机先起飞
    if (ugvStatus == "start" && !alreadyTakeoff ){
        ros::param::set("takeoffOrLanding",1);
        ROS_INFO("Start game,Uav takeoff");    
    }

    // 无人机起飞之后，进入跟随模式
    if (alreadyTakeoff && ugvStatus == "follow"){
        ros::param::set("ifFollow",true);
        ROS_INFO("Uav follow Ugv");
    }

    // 无人机跟随无人车，直到无人车到达指定位置，开始第一次扫码
    if (alreadyTakeoff && ifFollow && ugvStatus=="first_scan"&& ScanIndex == 0){
        // 停止跟随
        ros::param::set("ifFollow",false);
        // 开始扫码
        ros::param::set("ifScan",true);
        // 设为第一次扫码
        ros::param::set("ScanIndex",1);
        ROS_INFO("Uav scan");
    }
    
    // 扫码已经结束，无人机再次开始跟随无人车
    if (alreadyTakeoff && !ifFollow && ugvStatus == "follow" && !ifScan){
        // 开始跟随
        ros::param::set("ifFollow",true);
        ROS_INFO("Uav follow Ugv");
    }
    
    // 临时降落，无人车已经到达指定地点
    if (alreadyTakeoff && ugvStatus == "temp_landing"){
        // 停止跟随
        ros::param::set("ifFollow",false);
        // 降落
        ros::param::set("takeoffOrLanding",2);
        ROS_INFO("Uav temp_landing");
    }
    
    // 降落之后等待5s，无人机再次起飞
    if (!alreadyTakeoff && ugvStatus == "second_takeoff"){
        // 起飞
        ros::param::set("takeoffOrLanding",1);
        ROS_INFO("Uav second_takeoff");
    }
    
    // 无人机起飞之后，进入跟随模式（和比赛开始时一样）
    if (alreadyTakeoff && ugvStatus == "follow"){
        ros::param::set("ifFollow",true);
        ROS_INFO("Uav follow Ugv");
    }
    
    // 无人机跟随无人车，直到无人车到达指定位置，开始第二次扫码(类似第一次扫码)
    if (alreadyTakeoff && ifFollow && ugvStatus=="second_scan"&& ScanIndex == 1){
        // 停止跟随
        ros::param::set("ifFollow",false);
        // 开始扫码
        ros::param::set("ifScan",true);
        // 设为第二次扫码
        ros::param::set("ScanIndex",2);
        ROS_INFO("Uav scan");
    }
    
    // 扫码已经结束，无人机再次开始跟随无人车(和第一次扫码结束时一样)
    if (alreadyTakeoff && !ifFollow && ugvStatus == "follow" && !ifScan){
        // 开始跟随
        ros::param::set("ifFollow",true);
        ROS_INFO("Uav follow Ugv");
    }
    
    // 无人机跟随无人车，直到无人车到达指定位置，完成最终降落
    if (alreadyTakeoff && ifFollow && ugvStatus=="final_landing"&& ScanIndex == 2){
        // 停止跟随
        ros::param::set("ifFollow",false);
        // 降落
        ros::param::set("takeoffOrLanding",2);
        ROS_INFO("Uav final_landing");
    }

}

void GlobalControl::GlobalControlCallback(const std_msgs::Bool::ConstPtr& msg){
    GlobalControlUpdate();
}

GlobalControl::GlobalControl()
{
    // 初始化参数
    alreadyTakeoff = false;
    ifFollow = false;
    ifScan = false;
    ScanFlag = 0;
    ugvStatus = "inited";
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