
import cv2
import pyzbar.pyzbar as pyzbar
import rospy
from std_msgs.msg import String
rtsp_url = "rtsp://192.168.1.104:8554/live"
FrameRate = 20


def frame_process(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    decoded_qr_codes = pyzbar.decode(gray)
    for qr_code in decoded_qr_codes:
        print("QR code data: ", qr_code.data.decode())
        pub.publish(qr_code.data.decode())

if __name__ == '__main__':
    rospy.init_node('uav_qrcode_puber', anonymous=True)
    pub = rospy.Publisher('/qrcode_data', String, queue_size=10)
    cap = cv2.VideoCapture(rtsp_url)
    cnt = 0
    while True:
        try:
            ret, img = cap.read()
            cnt += 1
            if ret and cnt % FrameRate == 0:
                frame_process(img)
                cv2.waitKey(1)
        except KeyboardInterrupt:
            print("Shutting down")
            break
    cv2.release()
    cv2.destroyAllWindows()