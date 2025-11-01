/**
 * @file callback.cpp
 * @brief 回调函数
 * @date 2025/11/2
 */
#include<mission_master_pkg/mission_master_node.h>

void MissionMaster::localPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose_ = msg;
}

void MissionMaster::stateCheckCB(const mavros_msgs::State::ConstPtr& msg){

    //记录无人机状态
    current_vehicle_state_=msg;

    //判断解锁
    if (current_vehicle_state_.armed == ARM){

        //判断当前状态
        switch (current_mission_state_) {

            case 'ENUM_WATTING_TAKEOFF':
                home_pose = current_pose_;
                local_pos_pub.publish(takeoff_pose);
                current_mission_state_=ENUM_TAKEOFF;
                break;

            case 'ENUM_TAKEOFF':
                local_pos_pub.publish(takeoff_pose);
                if(missionStateCheck(takeoff_pose)) current_mission_state_=ENUM_TAKEOFF_SUCCEED;
                break;

            case 'ENUM_TAKEOFF_SUCCEED':
                local_pos_pub.publish(pickup_start_pose);
                current_mission_state_=ENUM_FLYTO_PICKUP_POINT;
                break;

            case 'ENUM_FLYTO_PICKUP_POINT':
                local_pos_pub.publish(pickup_start_pose);
                if(missionStateCheck(pickup_start_pose)) current_mission_state_=ENUM_PICKUP_POINT;
                break;

            case 'ENUM_PICKUP_POINT':
                if(pickClient())current_mission_state_=ENUM_PICKUP_SUCCEED;
                break;

            case 'ENUM_PICKUP_SUCCEED':
                local_pos_pub.publish(pickup_end_pose);
                if(missionStateCheck(pickup_end_pose)) current_mission_state_=ENUM_FLYTO_AVOID_POINT;
                break;

            case 'ENUM_FLYTO_AVOID_POINT':
                local_pos_pub.publish(avoid_start_pose);
                if(missionStateCheck(avoid_start_pose)) current_mission_state_=ENUM_AVOID_POINT;
                break;

            case 'ENUM_AVOID_POINT':
                if(pickClient())current_mission_state_=ENUM_AVOID_SUCCEED;
                break;

            case 'ENUM_AVOID_SUCCEED':
                local_pos_pub.publish(avoid_end_pose);
                if(missionStateCheck(avoid_end_pose)) current_mission_state_=ENUM_FLYTO_TRACE_POINT;
                break;

            case 'ENUM_FLYTO_TRACE_POINT':
                local_pos_pub.publish(trace_start_pose);
                if(missionStateCheck(trace_start_pose)) current_mission_state_=ENUM_TRACE_POINT;
                break;

            case 'ENUM_TRACE_POINT':
                if(avoidClient())current_mission_state_=ENUM_TRACE_SUCCEED;
                break;

            case 'ENUM_TRACE_SUCCEED':                
                local_pos_pub.publish(trace_end_pose);
                if(missionStateCheck(trace_end_pose)) current_mission_state_=ENUM_FLYTO_LAND_POINT;
                break;            
                
            case 'ENUM_FLYTO_LAND_POINT':
                local_pos_pub.publish(land_pose);
                if(missionStateCheck(land_pose)) current_mission_state_=ENUM_LAND_POINT;
                break;            
                
            case 'ENUM_LAND_POINT':
                if(landClient())current_mission_state_=ENUM_LAND_SUCCEED;
                break;            
                
            case 'ENUM_LAND_SUCCEED':
                //切降落
                ROS_INFO("Mission Succeed!!");
                break;           

            default:
                local_pos_pub.publish(trace_start_pose);
                break;
        }

    }

}