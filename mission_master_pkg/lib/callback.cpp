/**
 * @file callback.cpp
 * @brief 回调函数
 * @date
 */
#include<mission_master_pkg/mission_master_node.h>

void MissionMaster::localPoseCB(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose_ = msg;
}
void MissionMaster::stateCheckCB(const mavros_msgs::State::ConstPtr& msg){
    current_vehicle_state_=msg;
    if (current_state.armed == ARM){
        switch (mission_state) {
            case 'ENUM_WATTING_TAKEOFF':
                //准备起飞
                break;

            case 'ENUM_TAKEOFF':
                //起飞任务
                break;

            case 'ENUM_TAKEOFF_SUCCEED':
                //起飞成功
                break;

            case 'ENUM_FLYTO_PICKUP_POINT':
                //飞到拾取点
                break;

            case 'ENUM_PICKUP_POINT':
                //进行拾取
                break;

            case 'ENUM_PICKUP_SUCCEED':
                //拾取成功
                break;

            case 'ENUM_FLYTO_AVOID_POINT':
                //飞到避障
                break;

            case 'ENUM_AVOID_POINT':
                //进行避障
                break;

            case 'ENUM_AVOID_SUCCEED':
                //完成避障
                break;

            case 'ENUM_FLYTO_TRACE_POINT':
                //飞到跟踪点
                break;

            case 'ENUM_TRACE_POINT':
                //进行跟踪
                break;  

            case 'ENUM_TRACE_SUCCEED':
                //追踪成功
                break;            
                
            case 'ENUM_FLYTO_LAND_POINT':
                //飞到降落点
                break;            
                
            case 'ENUM_LAND_POINT':
                //进行降落
                break;            
                
            case 'ENUM_LAND_SUCCEED':
                //降落成功
                break;           

            default:
                //维持当前位置
                //超时后自动降落
                break;
        }

    }

}