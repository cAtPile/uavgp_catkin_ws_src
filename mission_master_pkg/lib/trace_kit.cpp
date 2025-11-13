/**
 * @file trace_kit.cpp
 * @brief 跟踪相关
 */
#include "mission_master_pkg/mission_master.h"

/**
 * @brief 跟踪任务开始
 */
void MissionMaster::traceStart()
{
    ROS_INFO("T Start");
    setPoint(TRACE_START_WAYPOINT);

    while (ros::ok())
    {
        ROS_INFO("T Loop");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_START_WAYPOINT))
        {
            ROS_INFO("Arrived at trace Start Point");
            current_mission_state = EXECUTE_TRACE_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}

/**
 * @brief 跟踪执行
 */
void MissionMaster::traceExecute()
{
    ROS_INFO("T exe");
    /*
      # CamTrack.msg

    std_msgs/Header header   # 时间戳、坐标系等信息，备用。
    bool system_ok           # 系统健康状态（True=正常运行，False=检测异常）

    int32 ball_num #检测到的ball数量 下面的xy坐标是节点决策后最适合抓取的哪个球的坐标 所以只输出一个坐标
    float32 ball_x
    float32 ball_y
    float32 ball_dis #到ball的距离，由d455测量，留作后续开发的接口，现在都输出0

    int32 car_num #检测到的car数量 下面的
    float32[] car_x
    float32[] car_y
    float32[] car_dis #到car的距离，由d455测量，留作后续开发的接口，现在都输出0

    bool in_gripper #用于检测夹爪是否成功抓取，True为夹爪有球，False为夹爪无球

     */
    //
   // while (ros::ok())
    {

        /*
                if (current_camtrack.system_ok && current_camtrack.car_num > 0)
                {

                    //汽车位置读取
                    double car_x = current_camtrack.car_x[0];
                    double car_y = current_camtrack.car_x[0];


                }*/
    }

    // 预留
    current_mission_state = SUCCEED_TRACE_STATE;
}

/**
 * @brief 跟踪检查
 */
void MissionMaster::traceCheck()
{
    ROS_INFO("T c");

    setPoint(TRACE_END_WAYPOINT);
    while (ros::ok())
    {
        ROS_INFO("TC L");
        setpoint_pub_.publish(temp_pose);
        if (reachCheck(TRACE_END_WAYPOINT))
        {

            ROS_INFO("Arrived at trace End Point")
            current_mission_state = START_LAND_STATE;
            break;
        }

        ros::spinOnce();
        rate_.sleep();
    }
}