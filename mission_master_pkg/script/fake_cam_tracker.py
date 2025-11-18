#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped

def pose_callback(msg):
    """回调函数：接收并打印local_position/pose消息"""
    # 使用ROS日志系统打印消息（会同时输出到控制台和ROS日志文件）
    rospy.loginfo("收到本地位置信息:")
    rospy.loginfo(f"时间戳: {msg.header.stamp}")
    rospy.loginfo(f"坐标系: {msg.header.frame_id}")
    rospy.loginfo(f"位置(x,y,z): ({msg.pose.position.x:.4f}, {msg.pose.position.y:.4f}, {msg.pose.position.z:.4f})")
    rospy.loginfo(f"姿态(x,y,z,w): ({msg.pose.orientation.x:.4f}, {msg.pose.orientation.y:.4f}, "
                  f"{msg.pose.orientation.z:.4f}, {msg.pose.orientation.w:.4f})")
    rospy.loginfo("----------------------------------------")

def main():
    # 初始化节点，节点名为"local_position_listener"（可自定义）
    rospy.init_node('local_position_listener', anonymous=True)
    
    try:
        # 订阅/mavros/local_position/pose话题，消息类型为PoseStamped，指定回调函数
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
        
        # 保持节点运行，等待回调
        rospy.loginfo("开始监听/mavros/local_position/pose话题...")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        rospy.logwarn("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")

if __name__ == '__main__':
    main()