#include "mission_master_pkg/mission_master.h"
/**
 * @brief 任务主循环
 */
void MissionMaster::missionExecute()
{

    switch (current_mission_state)
    {
    case WAITING_TAKEOFF_STATE:
        waitingTakeoff();
        break;

    case EXECUTE_TAKEOFF_STATE:
        takeoffExecute();
        break;

    case SUCCEED_TAKEOFF_STATE:
        takeoffCheck();
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

        //其他异常状态

    default:
        // local_pos_pub.publish(trace_start_pose);
        break;
    }
}