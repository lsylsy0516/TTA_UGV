import pandas as pd
import rospy
from std_msgs.msg import String

def qrcode_cb(data):
    
    df = pd.read_csv('qrcode.csv')
    # 如果data不在df中，就添加到df中
    if data not in df.values:
        # 直接添加文本到csv文件中
        df = df.append([data], ignore_index=True) #ignore_index=True表示不按原来的索引，从0开始自动递增
    df.to_csv('qrcode.csv', index=False)
    
if __name__ == '__main__':
    rospy.init_node('qrcode_suber', anonymous=True)
    rospy.Subscriber('/qrcode_data', String, qrcode_cb)
    rospy.spin()