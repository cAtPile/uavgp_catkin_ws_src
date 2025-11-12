#include"mission_master_pkg/mission_master.h"
/**
 * @brief 起飞成功
 */
void MissionMaster::takeoffCheck(){

    while(ros::ok()){

        //保持起飞定点
        setpoint_pub_.publishi(temp_pose);

        if(reachCheck(TAKEOFF_WAYPOINT)){
            current_mission_state = SUCCEED_TAKEOFF_STATE;
            break;
        }
    }

}