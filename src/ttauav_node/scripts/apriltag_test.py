import apriltag
import cv2
import numpy as np
import rospy

# 设置相机内参
focal_length_x = 555.18770321 # x轴焦距
focal_length_y = 555.86044578  # y轴焦距
principal_point_x = 401.58568147  # x轴光心坐标
principal_point_y = 294.43733787  # y轴光心坐标

def get_camera_pose(frame):
    # 创建 AprilTag 检测器对象
    detector = apriltag.apriltag(family='tag36h11')

    # 转换图像为灰度
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.imshow("gray",gray)
    cv2.waitKey(10)
    # 检测 AprilTag
    result = detector.detect(gray)

    if len(result) == 0:
        print("未检测到 AprilTag.")
        return 0,0,0

    # 获取相机内参
    camera_matrix = np.array([[focal_length_x, 0, principal_point_x],
                              [0, focal_length_y, principal_point_y],
                              [0, 0, 1]])

    # 获取 AprilTag 相对于相机的姿态
    corners = result[0]["lb-rb-rt-lt"]

    # 计算相机姿态
    # 定义AprilTag标签的三维坐标
    tag_points_3d = np.array([[-0.06, -0.06, 0.0],
                          [0.06,-0.06, 0.0],
                          [0.06, 0.06, 0.0],
                          [-0.06, 0.06, 0.0]])

    _, camera_rotation, camera_position = cv2.solvePnP(tag_points_3d, corners, camera_matrix, None)

    rotation_matrix, _ = cv2.Rodrigues(camera_rotation)
    yaw = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])
    yaw = np.degrees(yaw)
    x, y, _ = camera_position.flatten()
    x = float(x)
    y = float(y)
    yaw = float(yaw)
    return x,y,yaw # 返回的是实际的相对位移

if __name__ == "__main__":
    rospy.init_node("position_pub")
    cap = cv2.VideoCapture('rtsp://192.168.73.195:8554/live')
    #每隔0.1s发布一次相机位姿
    rate = rospy.Rate(10)
    cnt = 0
    while True:
        try:
            # 读取一帧图像
            ret, frame = cap.read()
            cnt += 1 
            if cnt % 10 == 0 : 
                x,y,yaw = get_camera_pose(frame)
                # 用参数服务器发布相机位姿,可以实时更新
                rospy.set_param("x", x)
                rospy.set_param("y", y)
                rospy.set_param("yaw", yaw)
                rate.sleep()
        except KeyboardInterrupt:
            print("exit")
            break
    cap.release()
    cv2.destroyAllWindows()
     
