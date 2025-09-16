#!/usr/bin/env python3
import rospy
from apoc_pkg.msg import detection_data

def simple_publisher():
    rospy.init_node('simple_detection_pub')
    pub = rospy.Publisher('/detection/data', detection_data, queue_size=10)
    rate = rospy.Rate(10)
    
    while not rospy.is_shutdown():
        msg = detection_data()
        msg.detection_id = 1  # 强制有目标
        msg.detection_x = 320.0  # 固定坐标（无波动）
        msg.detection_y = 320.0
        pub.publish(msg)
        rospy.loginfo(f"有目标 | id=1, x=320.0, y=320.0")
        rate.sleep()

if __name__ == '__main__':
    try:
        simple_publisher()
    except rospy.ROSInterruptException:
        pass