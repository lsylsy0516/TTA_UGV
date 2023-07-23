from robomaster import robot
import time 
import rospy
from std_msgs.msg import Bool

# enum UGVStatus{
#     takeoff = 1,
#     landing,
#     follow,
#     first_scan,
#     second_scan,
# };

if __name__ == '__main__':
    rospy.init_node('ugv_node', anonymous=True)
    pub = rospy.Publisher("updateStatus",Bool,queue_size=10)
    msg = Bool()

    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_chassis = ep_robot.chassis
    rospy.loginfo("start the ugv node")  

    # 无人机起飞
    rospy.set_param("StatusUpdate", 1) # 开始，控制无人机起飞
    pub.publish(msg)
    time.sleep(1)

    #scanPoints_1
    while rospy.get_param("IfActionDown",False) == False: 
        pass
    rospy.set_param("StatusUpdate", 4)  #move to start point
    pub.publish(msg)
    ep_chassis.move(x=1.5, y=0, z=0, xy_speed=0.7).wait_for_completed()

    rospy.loginfo("move to the first point")

    #scanPoints_2
    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 5) 
    pub.publish(msg)
    ep_chassis.move(x=1.5, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=1.5, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=1.0, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=1.0, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)


    rospy.loginfo("finished the first scan")

    # 到达停车点
    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 5) 
    pub.publish(msg)
    ep_chassis.move(x=1.0, y=0, z=0, xy_speed=0.6).wait_for_completed()
    ep_chassis.move(x=0.0, y=-2.3, z=0, xy_speed=0.7).wait_for_completed()


    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 3) # follow mode
    pub.publish(msg)

    # 降落
    while True:
        if abs(rospy.get_param("x")-0.77)<0.05 and abs(rospy.get_param("y")-0.42)<0.05 :
            rospy.set_param("StatusUpdate", 2)
            pub.publish(msg)
            break
    time.sleep(5)

    rospy.set_param("StatusUpdate", 1) 
    pub.publish(msg)
    time.sleep(1)

    # 到扫码区域
    while rospy.get_param("IfActionDown",False) == False: 
        pass
    rospy.set_param("StatusUpdate", 5) 
    pub.publish(msg)
    ep_chassis.move(x=-1.0, y=0.0, z=0, xy_speed=0.7).wait_for_completed()

    # 扫码
    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 5) 
    pub.publish(msg)

    ep_chassis.move(x=-1.0, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=-1.0, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=-1.5, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    ep_chassis.move(x=-1.5, y=0, z=0, xy_speed=0.6).wait_for_completed()
    time.sleep(1)
    # 停车
    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 5) 
    pub.publish(msg)
    ep_chassis.move(x=-1.5, y=0, z=0, xy_speed=0.6).wait_for_completed()
    ep_chassis.move(x=0.0, y=2.3, z=0, xy_speed=0.7).wait_for_completed()


    while rospy.get_param("ifScan",True) == True:
        pass
    rospy.set_param("StatusUpdate", 3) # follow mode
    pub.publish(msg)

    # 降落
    while True:
        if abs(rospy.get_param("x")-0.77)<0.05 and abs(rospy.get_param("y")-0.42)<0.05 :
            rospy.set_param("StatusUpdate", 2)
            pub.publish(msg)
            break
            
    ep_robot.close()
