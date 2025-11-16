#include "mission_master_pkg/mission_master.h"
/**
 * @brief 任务主循环
 */
void MissionMaster::missionExecute()
{

    switch (current_mission_state)
    {
    case WAITING_TAKEOFF_STATE:
        waitingTakeoff(); // 等待起飞
        break;

    case EXECUTE_TAKEOFF_STATE:
        takeoffExecute(); // 执行起飞
        break;

    case SUCCEED_TAKEOFF_STATE:
        takeoffCheck(); // 起飞成功检查
        break;

    case START_PICKUP_STATE:
        pickupStart();
        break;

    case EXECUTE_PICKUP_STATE:
        pickupExecute();
        break;

    case SUCCEED_PICKUP_STATE:
        pickupCheck();
        break;

    case START_AVOID_STATE:
        avoidStart();
        break;

    case EXECUTE_AVOID_STATE:
        avoidExecute();
        break;

    case SUCCEED_AVOID_STATE:
        avoidCheck();
        break;

    case START_TRACE_STATE:
        traceStart();
        break;

    case EXECUTE_TRACE_STATE:
        traceExecute();
        break;

    case SUCCEED_TRACE_STATE:
        traceCheck();
        break;

    case START_LAND_STATE:
        landStart();
        break;

    case EXECUTE_LAND_STATE:
        landExecute();
        break;

    case SUCCEED_LAND_STATE:
        landCheck();
        break;

    case ERROR_STATE:
        ROS_ERROR("ERROR");
        break;

    case MISSION_SUCCEED_STATE:
        ROS_INFO("MISSION SUCCEED");
        break;

    default:
        break;
    }
}

void MissionMaster::run()
{
    while (ros::ok())
    {
        if(current_mission_state==MISSION_SUCCEED_STATE){
            break;
        }
        missionExecute();
        ros::spinOnce();
        rate_.sleep();
    }
}
