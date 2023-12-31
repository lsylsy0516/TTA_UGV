#include "uavControl.h"

void uavControl::sendtakeoffOrLanding(int takeoffOrLanding){
  
    ttauav_node::action msg;
    msg.mode = takeoffOrLanding;
    ROS_INFO("msg=%d",msg.mode);
    pub.publish(msg);
}

void uavControl::sendgimbalControl(float GimbalPitch){
 
    ttauav_node::action msg;
    msg.mode = 4;
    msg.PTZ_pitch = GimbalPitch;
    pub.publish(msg);
    
}

void uavControl::sendflightByVel(float velN, float velE, float velD, float velYaw,int fly_time){

    ttauav_node::action msg;
    msg.mode = 3;
    // 这段放在无人机上就好
    // float sin_theta = std::sin(THETA*RAD2DGR); 
    // float cos_theta = std::cos(THETA*RAD2DGR); 
    // msg.n = velN*cos_theta+velE*sin_theta;
    // msg.e = velE*cos_theta-velN*sin_theta;
    msg.n = velN;
    msg.e = velE;
    msg.d = velD;
    msg.yaw = velYaw;
    msg.fly_time = fly_time;
    ROS_INFO("NED=%f,%f,%f,time=%d",velN,velE,velD,fly_time);
    pub.publish(msg);
}

void uavControl::sendflightByDis(float disN, float disE, float disD){ //单位米
    //单位秒
    float time = sqrt(disN*disN+disE*disE+disD*disD)/VEL;
    sendflightByVel(disN/time,disE/time,disD/time,0,time*1000);
}

void uavControl::sendfollow(){
        ttauav_node::action msg;
        float x = ros::param::param("x", 0.0);
        float y = ros::param::param("y", 0.0);
        float yaw = ros::param::param("yaw", 0.0);
        float n, e, neyaw;
        float epsilon = 0.0001; // 误差范围
        float total_velocity = 0.2;

        if (fabs(x) < epsilon && fabs(y) < epsilon && fabs(yaw) < epsilon) {
            return;
        } else {
            n = (x - X_c) * Kp;
            e = (y - Y_c) * Kp;
            neyaw = yaw;
            ROS_INFO("右,前,yaw = %f,%f,%f",n,e,neyaw);
        }

        // Calculate total distance
        float total_distance = sqrt(n*n + e*e);

        // Calculate the proportions of the velocity
        float velocity_n = total_velocity * (n / total_distance);
        float velocity_e = total_velocity * (e / total_distance);

        // fly_time = total distance / total velocity
        int fly_time = int(total_distance / total_velocity)*1000;
        float velocity_yaw =  neyaw / fly_time;

        msg.mode = 3;
        msg.n = velocity_n;  // 更新为速度值
        msg.e = velocity_e;  // 更新为速度值
        msg.yaw = velocity_yaw;
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
    ifFollow = ros::param::param("ifFollow", false);
    ifScan = ros::param::param("ifScan", false);
    ScanIndex = ros::param::param("ScanIndex", 0);
    GimbalControl =ros::param::param("GimbalControl",0);
    Action = ros::param::param("uavAction",0);

}

/**
 * @brief 用于更新无人机控制发布
 * 频率为10Hz
 * 逻辑：
 * 若takeoffOrLanding为1，则发送起飞指令，若为2，则发送降落指令，若为0，则不发送
 * 若在已经起飞且ifFollow为true的情况下，发送follow指令
 * 若在扫码模式下，则按照flag发送动作序列指令
*/
void uavControl::sendUpdate(){
    // 如果此时无人机正在执行动作，或者参数服务器没有初始化好，则不发送指令
    if (!ActionDown){
        if (takeoffOrLanding && Action == ToL){ //确定无人机在起飞或降落了,takeoffOrLanding 清零
            return;
        }
        if (GimbalControl && Action == G_C){ //确定无人机在云台控制了,GimbalControl 清零
            ros::param::set("takeoffOrLanding",0);
            ros::param::set("GimbalControl",0);
            return;
        }
    }

    // 起飞/降落
    if (takeoffOrLanding){ 
        ros::param::set("uavAction",ToL);
        sendtakeoffOrLanding(takeoffOrLanding);
        // ros::param::set("takeoffOrLanding",0);
        return;
    }
    // 云台控制
    if (GimbalControl){
        ros::param::set("uavAction",G_C);
        if(GimbalControl != Last_status) {
            if (GimbalControl == 1) 
                sendgimbalControl(90);
            else    
                sendgimbalControl(-90);
            ROS_INFO("GimbalControl: %d", GimbalControl);
            ros::param::set("GimbalControl",0);
            Last_status = GimbalControl;
            return;
        }
    }
    // 扫码模式
    if(ifScan){
        if(ActionDown)
        {
            // 根据ScanIndex和ScanFlag发送动作序列
            std::vector<float>* scanPoint = nullptr;
            int scanSize = 0;
            switch (ScanIndex)
            {
                case 0:
                    scanPoint = &ScanPoints_0[ScanFlag];
                    scanSize = ScanPoints_0.size();
                    break;
                case 1:
                    scanPoint = &ScanPoints_1[ScanFlag];
                    scanSize = ScanPoints_1.size();
                    break;
                case 2:
                    scanPoint = &ScanPoints_2[ScanFlag];
                    scanSize = ScanPoints_2.size();
                    break;
                case 3:
                    scanPoint = &ScanPoints_3[ScanFlag];
                    scanSize = ScanPoints_3.size();
                    break;
                case 4:  
                    scanPoint = &ScanPoints_4[ScanFlag];
                    scanSize = ScanPoints_4.size();
                    break;  
                case 5:  
                    scanPoint = &ScanPoints_5[ScanFlag];
                    scanSize = ScanPoints_5.size();
                    break;
            }

            ROS_INFO("Scaning---Index: %d---Flag: %d", ScanIndex, ScanFlag);
            float disN = (*scanPoint)[0];
            float disE = (*scanPoint)[1];
            float disD = (*scanPoint)[2];
            sendflightByDis(disN,disE,disD);
            ros::Duration((sqrt(disN*disN+disE*disE+disD*disD)/VEL)).sleep();
            ScanFlag++;
            if (ScanFlag ==  scanSize){ // 一个动作序列发送完毕
                ScanFlag = 0;
                ifScan = false;
                ros::param::set("ifScan",false); 
                
                ScanIndex++;
                ros::param::set("ScanIndex",ScanIndex);
            }
        }
        ros::param::set("uavAction",FbV);
        return;
    }

    // 跟随无人车
    if (ifFollow){
        sendfollow();
        ROS_INFO("ifFollow: %s", ifFollow ? "true" : "false");
        ros::param::set("uavAction",FbV);
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
    ScanPoints_0 = {    // 向右移动1.0m
        {1.0,0,0},
    };
    ScanPoints_2 = {    // 向前移动2.3m
        {0.0,-2.3,0.0},
    };
    ScanPoints_3 = {    // 向左移动1.0m
        {-1.0,0.0,0.0},
    };
    ScanPoints_5 = {    // 向后移动2.3m
        {0.0,2.3,0.0},
    };
    scanPoints_1 = {    //向右扫码

        {0.75,0.0,0.0},{0.0,0.0,-0.6},{0.75,0.0,0.0},{0.0,0.0,0.6}, //第一个货架 1.5m

        {0.75,0.0,0.0},{0.0,0.0,-0.6},{0.75,0.0,0.0},{0.0,0.0,0.6}, //第二个货架 1.5m

        {0.75,0.0,0.0},{0.0,0.0,-0.6},{0.75,0.0,0.0},{0.0,0.0,0.6}, //第三、四个货架 1.5m 

        {1.50,0.0,0.0}, //到达货架外
    };
    ScanPoints_4 = {    //向左扫码
        {-0.75,0.0,0.0},{0.0,0.0,-0.6},{-0.75,0.0,0.0},{0.0,0.0,0.6}, //第一个货架 1.5m

        {-0.75,0.0,0.0},{0.0,0.0,-0.6},{-0.75,0.0,0.0},{0.0,0.0,0.6}, //第二个货架 1.5m

        {-0.75,0.0,0.0},{0.0,0.0,-0.6},{-0.75,0.0,0.0},{0.0,0.0,0.6}, //第三、四个货架 1.5m 

        {-1.50,0.0,0.0}, //到达货架外
    };

    ScanFlag = 0;
    takeoffOrLanding = 0;
    ifFollow = false;
    ifScan = false;
    Last_status = 1;
    ROS_INFO("uav_control_node inited!");
}

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"uavControler");
    uavControl uavControl;
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        uavControl.paramUpdate();
        uavControl.sendUpdate();
        loop_rate.sleep();
    }
    return 0;
}

