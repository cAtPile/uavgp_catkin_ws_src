/**
 * @brief 定点飞行
 */
#include"mission_master_pkg/mission_master.h"

  HOME_POSE;//HOME点
  TAKEOFF_POSE;//起飞目标点
  PICK_START_POSE;//爪取开始
  PICK_END_POSE;//抓取结束
  AVOID_START_POSE;//避障开始
  AVOID_END_POSE;//避障结束
  TRACE_START_POSE;//跟踪开始
  TRACE_END_POSE;//跟踪结束
  LAND_POSE;//降落点
  PASS_TP_POSE;//起飞抓取途径
  PASS_PA_POSE;//抓取避障途径
  PASS_AT_POSE;//避障跟踪途径
  PASS_TPA_POSE;//跟踪抓取途径
  PASS_TL_POSE;//跟踪降落途径

/**
 * @brief 飞到起飞点
 */
void MissionMaster::flytoTakeoff(){
//定义点
//循环
    //发布点
    //到达检查
    //更新状态
}