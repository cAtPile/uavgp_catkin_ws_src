/**
 * @file  client.cpp
 * @brief
 * @date
 */
#include "mission_master_pkg/mission_master_node.h"

bool MissionMaster::pickClient()
{
    /*
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
        */
    retrn true;
}

bool MissionMaster::avoidClient()
{
    retrn true;
}
bool MissionMaster::traceClient()
{
    retrn true;
}
bool MissionMaster::landClient()
{
    set_mode_srv.request.custom_mode = "AUTO.LAND";
    if (set_mode_client.call(set_mode_srv) && set_mode_srv.response.mode_sent)
    {
        return true;
    }
    else
    {
        return false;
    }
}