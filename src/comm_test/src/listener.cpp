#include "ros/ros.h"
#include "comm_test/Val.h" 

void doMsg(const comm_test::Val::ConstPtr& v_p){
    ROS_INFO("n=   %f",v_p->n);
    ROS_INFO("e=   %f",v_p->e);
    ROS_INFO("d=   %f",v_p->d);
    ROS_INFO("yaw= %f",v_p->yaw);
}
int main(int argc, char  *argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"val_listener");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<comm_test::Val>("val",10,doMsg);
    ros::spin();
    return 0;
}  