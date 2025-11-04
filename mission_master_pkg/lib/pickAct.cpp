/**
 * @file pickAct.cpp
 * @brief pick相关启动反馈
 */
#include "mission_master_pkg/mission_master_node.h"

void MissionMaster::pickFBCB(){

}

void MissionMaster::pickDoneCB(){

}

bool MissionMaster::pickAct(){
    // 发送目标
    mission_master_pkg::CountGoal goal;
    goal.target = 5;  // 设置目标字段
    client.sendGoal(goal, ...);  // client是Action客户端实例
    
} 