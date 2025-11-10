/**
 * @brief 任务执行器
 */
#include "mission_master_pkg/mission_master.h"

void MissionMaster::missionExecutor()
{

    switch (current_mission_state_)
    {
    case ENMU_TAKEOFF_WAITING: // 等待飞行
        waitingTakeoff();      // 心跳信号
        break;

    case ENMU_TAKEOFF_EXECUTE: // 执行起飞
        takeoffExecute();      // 执行起飞，到达检查 检查点 ，跟新状态 状态
        break;

    case ENMU_TAKEOFF_SUCCEED: // 起飞成功
        // 飞行 航点
        break;

    case ENMU_PICK_START: // 飞到抓取点
        // 飞行 航点
        break;

    case ENMU_PICK_EXECUTE: // 执行抓取
        pickExecute();      // 执行抓取，更新状态 状态
        break;

    case ENMU_PICK_SUCCEED: // 抓取结束
        // 飞行 航点
        break;

    case ENMU_AVOID_START: // 飞到避障点
        // 飞行 航点
        break;

    case ENMU_AVOID_EXECUTE: // 执行避障
        avoidExecute();      // 执行避障，跟新状态 状态
        break;

    case ENMU_AVOID_SUCCEED: // 避障成功
        // 飞行 航点
        break;

    case ENMU_TRACE_START: // 飞到跟踪点
        // 飞行 航点
        break;

    case ENMU_TRACE_EXECUTE: // 执行跟踪
        traceExecute();      // 执行跟踪，跟新状态 状态
        break;

    case ENMU_TRACE_SUCCEED: // 跟踪成功
        // 飞行 航点
        break;

    case ENMU_PASS_TP: // 起飞抓取中间防撞
        // 飞行 航点
        break;

    case ENMU_PASS_PA: // 抓取避障中间防撞
        // 飞行 航点
        break;

    case ENMU_PASS_AT: // 避障跟踪中间防撞
        // 飞行 航点
        break;

    case ENMU_PASS_TPA: // 跟踪抓取防撞
        // 飞行 航点
        break;

    case ENMU_PASS_TL: // 跟踪降落防撞
        // 飞行 航点
        break;

    case ENMU_LAND_START: // 降落开始
        // 飞行 航点
        break;

    case ENMU_LAND_EXECUTE: // 降落执行
        landExecute();      // 执行降落，更新状态 状态
        break;

    case ENMU_LAND_SUCCEED: // 降落成功
        // 飞行 航点
        break;

    default:
        // 默认状态
        break;
    }
}

/**
 * @brief 等待起飞
 * @details 发布心跳信号
 */
void MissionMaster::waitingTakeoff()
{

    // 发送心跳信号
    local_pos_pub.publish(current_pose);

    //
    ROS_INFO("Waiting for taking off...");
}

/**
 * @brief 执行起飞
 * @details 发布解锁信号,记录home点,航点处理,发布起飞点,更新状态为
 */
void MissionMaster::takeoffExecute()
{
    // 航点导入，home点确定
    loadWaypoints();

    // 创建解锁服务请求
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true; // true表示解锁，false表示上锁
    arming_client.call(arm_cmd);

    // 发布航点
    // setPoint(TAKEOFF_POSE_XYZ);
    if (reachCheck(/*起飞目标点*/))
    {
        current_mission_state_ = ENMU_TAKEOFF_SUCCEED;
    }
}

void MissionMaster::pickExecute() {}
void MissionMaster::avoidExecute() {}
void MissionMaster::traceExecute() {}
void MissionMaster::landExecute() {}