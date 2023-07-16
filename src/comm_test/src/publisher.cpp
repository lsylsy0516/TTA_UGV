/*
    需求: 循环发布人的信息

*/

#include "ros/ros.h"
#include "comm_test/Val.h"

int main(int argc, char *argv[])
{
    setlocale(LC_ALL,"");

    //1.初始化 ROS 节点
    ros::init(argc,argv,"val_publisher");

    //2.创建 ROS 句柄
    ros::NodeHandle nh;

    //3.创建发布者对象
    ros::Publisher pub = nh.advertise<comm_test::Val>("val",1000);

    //4.组织被发布的消息，编写发布逻辑并发布消息
    comm_test::Val v;
    // v.n = 0. ;
    // v.e = 0. ;
    // v.d = 0. ;
    // v .yaw = 0. ;

    ros::Rate r(10);
    while (ros::ok())
    {
        pub.publish(v);
        ros::spinOnce();
        r.sleep();
    }



    return 0;
}
