/**
 * @file  client.cpp
 * @brief 
 * @date
 */
#include"mission_master_pkg/mission_master_node.h"

bool MissionMaster::pickClient() {

    // 首次调用
    if (client.getState().isDone()) {  
        pick_server::PickGoal goal;
        goal.pick_enable = true;
        client.sendGoal(goal);
        return false; 
    }

    // 非首次调用
    pick_server::PickResultConstPtr result = client.getResult();
    if (!result) {  
        return false;
    }

    bool success = result->pick_success;
    return success;
}

bool MissionMaster::avoidClient(){
    
}
bool MissionMaster::traceClient(){
    
}
bool MissionMaster::landClient(){
    
}