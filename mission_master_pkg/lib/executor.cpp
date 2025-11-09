/**
 * @brief 任务执行器
 */
#include "mission_master_pkg/mission_master.h"

void MissionMaster::missionExecutor()
{

    switch (current_mission_state_)
    {
    // 等待起飞
    case ENUM_WATTING_TAKEOFF:
        waitingTakeoff(); // 完成航点导入和心跳信号
        break;

    // 状态回调更新 offb后跟新到 ENUM_TAKEOFF
    // 非offb则更新到ENUM_WATTING_TAKEOFF

    // 执行起飞
    case ENUM_TAKEOFF_EXECUTE:
        takeoffExcute(); // 定点起飞模式，到达检查后跟新 ENUM_FLYTO_PICKUP_POINT
        break;

    //飞到成功起飞点
    case ENUM_TAKEOFF_SUCCEED:
        //飞到 成功起飞点 ，到达检查后更新状态 

    // 成功起飞，飞到下一个航点
    case ENUM_FLYTO_PICKUP_POINT:
        flytoExcute(); // 飞行到下一个任务航点，到达检查更新状态 ENUM_PICKUP_POINT
        break;

    // 执行抓取任务
    case ENUM_PICKUP_EXECUTE:
        pickExecute(); // 执行抓取任务，成功后飞到抓取结束航点，跟新状态
        break;

    // 飞到抓取结束点
    case 
    // 飞行到避障开始点
    case ENUM_FLYTO_AVOID_POINT:
        flytoExcute(); // 飞行到 任务航点 ，到达检查后跟新状态
        break;

    // 执行避障任务
    case ENUM_AVOID_POINT :
        avoidExecute();//执行避障任务，完成后更新状态
        break;

    //飞到避障结束点

    //飞到跟踪开始点

    //执行跟踪任务

    //飞到跟踪结束点

    //飞到途径防撞点

    //飞到降落点

    //执行降落


default:
    break;
}

}


void MissionMaster::waitingTakeoff() {}
