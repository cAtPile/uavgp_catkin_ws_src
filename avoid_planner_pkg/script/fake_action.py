#!/usr/bin/env python3
import rospy
import actionlib
from avoid_planner_pkg.msg import ApocAvoidAction, ApocAvoidGoal

def send_continuous_goals():
    # 初始化节点
    rospy.init_node('continuous_goal_sender', anonymous=True)
    
    # 创建Action客户端，连接到avoid_planner_action服务器
    client = actionlib.SimpleActionClient('/pcp_test_node/avoid_planner_action', ApocAvoidAction)
    
    # 等待服务器连接
    rospy.loginfo("等待avoid_planner_action服务器连接...")
    client.wait_for_server()
    rospy.loginfo("已连接到action服务器")
    
    # 设置发布频率（10Hz）
    rate = rospy.Rate(10)
    
    # 持续发送目标直到节点关闭
    while not rospy.is_shutdown():
        # 创建目标消息
        goal = ApocAvoidGoal()
        
        # 设置当前位置
        goal.current_pose_x = 0.0
        goal.current_pose_y = 0.0
        goal.current_pose_z = 0.0
        
        # 设置目标位置
        goal.goal_x = 1.0
        goal.goal_y = 0.0
        goal.goal_z = 1.0
        
        # 设置命令（1为启动命令）
        goal.cmd = 1
        
        # 发送目标
        rospy.loginfo("发送目标: 当前位置(0,0,0) -> 目标位置(1,0,0), cmd=1")
        client.send_goal(goal)
        
        # 等待结果（超时时间1秒，因为需要持续发送）
        client.wait_for_result(rospy.Duration(1.0))
        
        # 检查是否到达目标（可选）
        if client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("已到达目标位置!")
        
        # 按照设定频率发送
        rate.sleep()

if __name__ == '__main__':
    try:
        send_continuous_goals()
    except rospy.ROSInterruptException:
        rospy.loginfo("节点被中断")
    except Exception as e:
        rospy.logerr(f"发生错误: {str(e)}")