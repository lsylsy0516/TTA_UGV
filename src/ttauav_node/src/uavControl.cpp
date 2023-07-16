#include "uavControl.h"

void uavControl::sendtakeoffOrLanding(int takeoffOrLanding){
    ttauav_node::action msg;
    msg.mode = takeoffOrLanding;
    ROS_INFO("msg=%d",msg.mode);
    // 假设开始无人机初始化好，IfActionDown = true ，那么我们停止发送takeoffOrLanding
    // 若没有初始化好，IfActionDown不存在，那么我们一直发送takeoffOrLanding
    // 之后若已初始化好，则只要IfActionDown = false，那么我们停止发送takeoffOrLanding
    // 先发送一次，再判断IfActionDown是否存在，若存在且为false，则停止发送
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
        float n = (ros::param::param("x", 0.0)-0.435)*(0.5);
        float e = (ros::param::param("y", 0.0)-0.113)*(0.5);
        float yaw = ros::param::param("yaw", 0.0);
        int fly_time = 100;

        msg.mode = 3;
        msg.n = n;
        msg.e = e;
        msg.yaw = yaw;
        msg.fly_time = fly_time;
        pub.publish(msg);
}
// 上面所有的函数都是瞬时的，只会发送一次


/**
 * @brief 用于更新无人机控制发布
 * 频率为1Hz
 * 逻辑：
 * 若takeoffOrLanding为1，则发送起飞指令，若为2，则发送降落指令，若为0，则不发送
 * 若在已经起飞且ifFollow为true的情况下，发送follow指令
 * 若在扫码模式下，则按照flag发送动作序列指令
*/
void uavControl::sendUpdate(){
    // 读取参数
    takeoffOrLanding = ros::param::param("takeoffOrLanding", 0);
    ifScan = ros::param::param("ifScan", false);
    ActionDown = ros::param::param("IfActionDown", false);
    ifFollow = ros::param::param("ifFollow", false);
    GimbalControl =ros::param::param("GimbalControl",0);

    // 如果此时无人机正在执行动作，或者参数服务器没有初始化好，则不发送指令
    if (!ActionDown){
        return;
    }
    // 起飞/降落
    if (takeoffOrLanding != 0){ 
        ROS_INFO("takeoffOrLanding: %d", takeoffOrLanding);
        sendtakeoffOrLanding(takeoffOrLanding);
        //更新takeoffOrLanding 和 alreadyTakeoff

        if (takeoffOrLanding == 1){
            alreadyTakeoff = true;
        }else{
            alreadyTakeoff = false;
        }
        ros::param::set("takeoffOrLanding",0);
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
        ROS_INFO("ifScan: %s", ifScan ? "true" : "false");
        const std::vector<float>& scanPoint = ScanPoints[ScanFlag];
        sendflightByVel(scanPoint[0],scanPoint[1],scanPoint[2],scanPoint[3],scanPoint[4]);
        ScanFlag++;
        if (ScanFlag==2){
            ros::param::set("ifScan",false);
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
        ros::param::set("GimbalControl",0);
        return;
    }

}

uavControl::uavControl(){
    // 初始化
    pub = nh.advertise<ttauav_node::action>("uavAction", 10);
    ScanPoints = {
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
    // ros::param::set("IfActionDown", false);
    ros::param::set("ifFollow", false);
    ros::param::set("GimbalControl",0);
    ROS_INFO("control_node inited!");
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uavControler");
    uavControl uavControl;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        uavControl.sendUpdate();
        loop_rate.sleep();
    }
    return 0;
}

