/**
 * @file 避障相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 避障任务开始
 */
void MissionMaster::avoidStart()
{
    ROS_INFO("A start");

    setPoint(AVOID_START_WAYPOINT);

    while (ros::ok())
    {
        ROS_INFO("A Loop");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(AVOID_START_WAYPOINT))
        {
            ROS_INFO("Arrived at avoid Start Point");
            current_mission_state = EXECUTE_AVOID_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 避障执行
 */
void MissionMaster::avoidExecute()
{
    ROS_INFO("A exe");

    // 预留
    current_mission_state = SUCCEED_AVOID_STATE;
}

/**
 * @brief 避障检查
 */
void MissionMaster::avoidCheck()
{
    ROS_INFO("A check");

    setPoint(AVOID_END_WAYPOINT);
    while (ros::ok())
    {
        ROS_INFO("Ac Loop");

        setpoint_pub_.publish(temp_pose);
        if (reachCheck(AVOID_END_WAYPOINT))
        {
            current_mission_state = START_TRACE_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

void MissionMaster::avoidLoop(double avoid_goal_x,double avoid_goal_y,double avoid_goal_z){

    //以geometry_msgs/PoseStamped的形式发送到/mission_avoid/goal
    geometry_msgs::PoseStamped::avoid_goal;
    avoid_goal.header.stamp=ros::Time::now();
    avoid_goal.header.frame_id="map";
    avoid_gaol.pose.position.x=avoid_goal_x;
    avoid_gaol.pose.position.y=avoid_goal_y;
    avoid_gaol.pose.position.z=avoid_goal_z;
    avoid_gaol.pose.orientation.x=0;
    avoid_gaol.pose.orientation.y=0;
    avoid_gaol.pose.orientation.z=0;
    avoid_gaol.pose.orientation.w=1;

    geometry_msgs::PoseStamped::avoid_cmd;
    avoid_cmd.header.stamp=ros::Time::now();
    avoid_cmd.header.frame_id="map";
    avoid_gaol.pose.position.x=avoid_goal_x;
    avoid_gaol.pose.position.y=avoid_goal_y;
    avoid_gaol.pose.position.z=avoid_goal_z;
    avoid_gaol.pose.orientation.x=0;
    avoid_gaol.pose.orientation.y=0;
    avoid_gaol.pose.orientation.z=0;
    avoid_gaol.pose.orientation.w=1;


    while (ros::ok())
    {
        temp_pose.pose.position.x=0;//
        temp_pose.pose.position.x=0;//
        temp_pose.pose.position.x=0;//

    }
    
    
    //接受ego消息
    /**
     * #PositionCommand.msg

Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
float64 yaw
float64 yaw_dot
float64[3] kx
float64[3] kv 

uint32 trajectory_id

uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7

# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag
     */

    //ego消息转换
    
    //发布到mav

    //到达退出
}