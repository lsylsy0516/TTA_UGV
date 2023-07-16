#include "uavControl.h"

void uavControl::sendtakeoffOrLanding(int takeoffOrLanding){
    ttauav_node::action msg;
    msg.mode = takeoffOrLanding;
    ROS_INFO("msg=%d",msg.mode);
    pub.publish(msg);
}

void uavControl::sendflightByVel(float velN, float velE, float velD, float velYaw,int fly_time){
    ttauav_node::action msg;
    msg.mode = 3;
    msg.n = velN;
    msg.e = velE;
    msg.d = velD;
    msg.yaw = velYaw;
    msg.fly_time = fly_time;
    pub.publish(msg);
}

void uavControl::sendgimbalControl(float GimbalPitch){
    ttauav_node::action msg;
    msg.mode = 4;
    msg.PTZ_pitch = GimbalPitch;
    pub.publish(msg);
}

void uavControl::sendfollow(){
        ttauav_node::action msg;
        float x = ros::param::param("x", 0.0);
        float y = ros::param::param("y", 0.0);
        float yaw = ros::param::param("yaw", 0.0);
        float n, e, neyaw;
        float epsilon = 0.0001; // 误差范围

        if (fabs(x) < epsilon && fabs(y) < epsilon && fabs(yaw) < epsilon) {
            n = e = neyaw = 0.0;
        } else {
            n = (x - 0.755) * 0.7;
            e = (y - 0.50) * 0.7;
            neyaw = yaw;
        }

        // 在这里可以使用n、e和yaw变量进行后续操作

        int fly_time = 500;

        msg.mode = 3;
        msg.n = n;
        msg.e = e;
        msg.yaw = neyaw;
        msg.fly_time = fly_time;
        pub.publish(msg);
}
// 上面所有的函数都是瞬时的，只会发送一次

/**
 * @brief 用于更新参数服务器中的参数
 * 频率为10Hz
 * 逻辑：
 * 读取参数服务器中的参数，根据参数服务器中的参数，更新无人机控制发布器中的参数
*/
void uavControl::paramUpdate(){
    // 读取参数
    ActionDown = ros::param::param("IfActionDown", false);
    takeoffOrLanding = ros::param::param("takeoffOrLanding", 0);
    alreadyTakeoff = ros::param::param("alreadyTakeoff", false);
    ifFollow = ros::param::param("ifFollow", false);
    ifScan = ros::param::param("ifScan", false);
    ScanIndex = ros::param::param("ScanIndex", 0);
    GimbalControl =ros::param::param("GimbalControl",0);
}

/**
 * @brief 用于更新无人机控制发布
 * 频率为1Hz
 * 逻辑：
 * 若takeoffOrLanding为1，则发送起飞指令，若为2，则发送降落指令，若为0，则不发送
 * 若在已经起飞且ifFollow为true的情况下，发送follow指令
 * 若在扫码模式下，则按照flag发送动作序列指令
*/
void uavControl::sendUpdate(){
    // 如果此时无人机正在执行动作，或者参数服务器没有初始化好，则不发送指令
    if (!ActionDown){
        if (takeoffOrLanding ){ //确定无人机在起飞或降落了,takeoffOrLanding 清零
            ros::param::set("takeoffOrLanding",0);
        }
        if (GimbalControl){ //确定无人机在云台控制了,GimbalControl 清零
            ros::param::set("GimbalControl",0);
        }
        return;
    }
    // 起飞/降落
    if (takeoffOrLanding != 0){ 
        sendtakeoffOrLanding(takeoffOrLanding);

        //更新takeoffOrLanding 和 alreadyTakeoff
        if (takeoffOrLanding == 1){
            alreadyTakeoff = true;
            ros::param::set("alreadyTakeoff",true);
        }else{
            alreadyTakeoff = false;
            ros::param::set("alreadyTakeoff",false);
        }
        // ros::param::set("takeoffOrLanding",0);
        // 不能立马清零，因为无人机可能接收失败
        return;
    }
    // 跟随无人车
    if (alreadyTakeoff && ifFollow){
        ROS_INFO("ifFollow: %s", ifFollow ? "true" : "false");

        sendfollow();
        return;
    }
    // 扫码模式
    if( alreadyTakeoff && ifScan){
        // 根据ScanIndex和ScanFlag发送动作序列
        std::vector<float>* scanPoint = nullptr;
        if (ScanIndex == 1)
            scanPoint = &ScanPoints_1[ScanFlag];
        else 
            scanPoint = &ScanPoints_2[ScanFlag];
        ROS_INFO("Scaning---Index: %d---Flag: %d", ScanIndex, ScanFlag);
        
        sendflightByVel((*scanPoint)[0],(*scanPoint)[1],(*scanPoint)[2],(*scanPoint)[3],(*scanPoint)[4]);
        ScanFlag++;
        if (ScanFlag==2){ //扫码结束，关闭扫码模式
            ros::param::set("ifScan",false);
            ScanFlag = 0;   //动作序列清零
        }
    }
    // 云台控制
    if (GimbalControl){
        ROS_INFO("GimbalControl: %d", GimbalControl);
        if (GimbalControl == 1){
            sendgimbalControl(90);
        }else{
            sendgimbalControl(-90);
        }
        // 更新GimbalControl伴随的参数
        // ros::param::set("GimbalControl",0);
        return;
    }

}

/**
 * @brief 无人机控制发布器的构造函数
 * 初始化发布器
 * 初始化参数
*/
uavControl::uavControl(){
    // 初始化
    pub = nh.advertise<ttauav_node::action>("uavAction", 10);
    ScanPoints_1 = {
        {0.0, 0.0, 0.10, 0.0,1000},
        {0.0, 0.0, -0.10, -90.0,1000}
    };
    ScanPoints_2 = {
        {0.0, 0.0, 0.10, 0.0,1000},
        {0.0, 0.0, -0.10, -90.0,1000}
    };
    ScanFlag = 0;
    takeoffOrLanding = 0;
    alreadyTakeoff = false;
    ifFollow = false;
    ifScan = false;
    ros::param::set("takeoffOrLanding", 0);
    ros::param::set("ifScan", false);
    ros::param::set("ifFollow", false);
    ros::param::set("GimbalControl",0);
    ros::param::set("alreadyTakeoff",false);
    ROS_INFO("uav_control_node inited!");
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uavControler");
    uavControl uavControl;
    ros::Rate loop_rate(100);
    while (ros::ok())
    {
        uavControl.paramUpdate();
        uavControl.sendUpdate();
        loop_rate.sleep();
    }
    return 0;
}

