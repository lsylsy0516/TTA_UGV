from robomaster import robot
import cv2
import time
import pyzbar.pyzbar as pyzbar
# import rospy
# from std_msgs.msg import String

def frame_process(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    decoded_qr_codes = pyzbar.decode(gray)
    for qr_code in decoded_qr_codes:
        print("QR code data: ", qr_code.data.decode())
        # pub.publish(qr_code.data.decode())

if __name__ == '__main__':
    # rospy.init_node('ugv_qrcode_puber', anonymous=True)
    # pub = rospy.Publisher('/qrcode_data', String, queue_size=10)
    
    ep_robot = robot.Robot()
    # ep_robot.initialize(conn_type="rndis")
    ep_robot.initialize(conn_type="ap")
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    
    while True:
        try:
            img = ep_camera.read_cv2_image()
            frame_process(img)
            cv2.waitKey(1)
            img = cv2.flip(img,0) 
            img = cv2.flip(img,1) 
            cv2.imshow("gray",img)
        except KeyboardInterrupt:
            print("Shutting down")
            break
    cv2.destroyAllWindows()
    ep_camera.stop_video_stream()