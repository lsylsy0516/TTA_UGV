#include <ros/ros.h>
#include <std_msgs/Bool.h>

/**
 * @brief 本程序用于测试GlobalControl
 * 假设自己为无人机，通过修改参数服务器中的参数，来模拟无人机的状态
 * 具体设置何种状态由argv[1]决定
 * @param takeoff 无人机起飞
 * @param landing 无人机临时降落
 * @param follow 无人机跟随无人车
 * @param first_scan 无人机第一次扫码
 * @param second_scan 无人机第二次扫码
*/

enum UGVStatus{
    takeoff = 1,
    landing,
    follow,
    first_scan,
    second_scan,
};

const char* str_msgs[7]  ={
    "takeoff",
    "landing",
    "follow",
    "first_scan",
    "second_scan",
};

int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "test_comm_control");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<std_msgs::Bool>("updateStatus", 10);
    std_msgs::Bool msg;
    // 读取参数
    int status = atoi (argv[1]);
    // 设置参数
    ros::param::set("StatusUpdate",status);
    msg.data = true;
    ros::Duration(2).sleep();
    pub.publish(msg);
    ROS_INFO("new status is %s",str_msgs[status-1]);
    return 0;
}