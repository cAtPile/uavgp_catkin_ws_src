/**
 * @brief 任务执行器
 */
#include"mission_master_pkg/mission_master.h"

void MissionMaster::missionExecutor(){

    switch (current_mission_state_)
    {
    //等待起飞
    case : ENUM_WATTING_TAKEOFF
        waitingTakeoff();//完成航点导入和心跳信号
        break;

    //状态回调更新 offb后跟新到 ENUM_TAKEOFF
    //非offb则更新到ENUM_WATTING_TAKEOFF

    //执行起飞
    case: ENUM_TAKEOFF
        takeoffExcute();//定点起飞模式，到达检查后跟新 ENUM_FLYTO_PICKUP_POINT
        break;
    
    //成功起飞，飞到下一个航点
    case ENUM_FLYTO_PICKUP_POINT:
        flytoExcute();//飞行到下一个任务航点，并更新状态 ENUM_FLYTO_PICKUP_POINT
        break;

        case ENUM_PICKUP_POINT:
            if (pickExecute())
                current_mission_state_ = ENUM_PICKUP_SUCCEED;
            break;

        case ENUM_PICKUP_SUCCEED:
            setPoint(PICKUP_END_POSE_XYZ);
            if (reachCheck(PICKUP_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_AVOID_POINT;
            break;

        case ENUM_FLYTO_AVOID_POINT:
            setPoint(AVOID_START_POSE_XYZ);
            if (reachCheck(AVOID_START_POSE_XYZ))
                current_mission_state_ = ENUM_AVOID_POINT;
            break;

        case ENUM_AVOID_POINT:
            if (avoidExecute())
                current_mission_state_ = ENUM_AVOID_SUCCEED;
            break;

        case ENUM_AVOID_SUCCEED:
            setPoint(AVOID_END_POSE_XYZ);
            if (reachCheck(AVOID_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_TRACE_POINT;
            break;

        case ENUM_FLYTO_TRACE_POINT:
            setPoint(TRACE_START_POSE_XYZ);
            if (reachCheck(TRACE_START_POSE_XYZ))
                current_mission_state_ = ENUM_TRACE_POINT;
            break;

        case ENUM_TRACE_POINT:
            if (traceExecute())
                current_mission_state_ = ENUM_TRACE_SUCCEED;
            break;

        case ENUM_TRACE_SUCCEED:
            setPoint(TRACE_END_POSE_XYZ);
            if (reachCheck(TRACE_END_POSE_XYZ))
                current_mission_state_ = ENUM_FLYTO_LAND_POINT;
            break;

        case ENUM_FLYTO_LAND_POINT:
            setPoint(TAKEOFF_POSE_XYZ);
            if (reachCheck(TAKEOFF_POSE_XYZ))
                current_mission_state_ = ENUM_LAND_POINT;
            break;

        case ENUM_LAND_POINT:
            if (landExecute())
                current_mission_state_ = ENUM_LAND_SUCCEED;
            break;

        case ENUM_LAND_SUCCEED:
            // disarm
            ROS_INFO("Mission Succeed!!");
            break;

        default:
            // local_pos_pub.publish(trace_start_pose);
            break;
        }
    default:
        break;
    }




}

void MissionMaster::waitingTakeoff(){}
