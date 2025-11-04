/**
 * @file avoidAct.cpp
 * @brief avoid相关action
 */
#include "mission_master_pkg/mission_master.h"
#include <ros/console.h>

/**
 * @brief 避障Action激活回调函数
 * @details 当避障Action服务器开始处理目标时调用
 */
void MissionMaster::avoidActiveCB() {
    ROS_INFO("Avoid action server has accepted the goal and started processing");
}

/**
 * @brief 避障Action完成回调函数
 * @param state 动作客户端状态
 * @param result 避障结果
 */
void MissionMaster::avoidDoneCB(const actionlib::SimpleClientGoalState &state,
                 const mission_master_pkg::AvoidResultConstPtr &result) {
    ROS_INFO("Avoid action finished with state: %s", state.toString().c_str());
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->avoid_success) {
        ROS_INFO("Avoid mission completed successfully");

    } else {
        ROS_WARN("Avoid mission failed or was preempted");
        // 可以根据需要添加失败处理逻辑，如重试或进入错误状态
    }
}

/**
 * @brief 避障Action反馈回调函数
 * @param feedback 避障过程中的反馈信息
 */
void MissionMaster::avoidFeedbackCB(const mission_master_pkg::AvoidFeedbackConstPtr &feedback) {
    ROS_INFO("Avoid current step: %d", feedback->avoid_current_step);
    
    // 可以根据当前步骤添加进度提示或中间处理逻辑
    switch(feedback->avoid_current_step) {
        case 0:
            ROS_INFO("Avoid initializing...");
            break;
        case 1:
            ROS_INFO("Detecting obstacles...");
            break;
        case 2:
            ROS_INFO("Planning avoidance path...");
            break;
        case 3:
            ROS_INFO("Executing avoidance maneuver...");
            break;
        default:
            ROS_INFO("Avoid proceeding with step %d", feedback->avoid_current_step);
    }
}

/**
 * @brief 执行避障Action
 * @return 是否成功发送避障目标
 */
void MissionMaster::avoidAct() {

    // 创建并设置避障目标
    mission_master_pkg::AvoidGoal goal;
    goal.avoid_enable = true; // 启动避障

    // 发送目标并注册回调函数
    avoid_clientor.sendGoal(goal,
        boost::bind(&MissionMaster::avoidDoneCB, this, _1, _2),
        boost::bind(&MissionMaster::avoidActiveCB, this),
        boost::bind(&MissionMaster::avoidFeedbackCB, this, _1));

    ROS_INFO("Avoid goal sent to action server");
}

/**
 * 
 */
bool MissionMaster::avoidExecute(){
    
    // 判断避障Action客户端是否正在运行任务
    if (avoid_clientor.getState().isActive()) {
        ROS_INFO("Previous avoid action is still active, waiting for completion...");
        return false; // 上个避障未完成，不重复执行
    }

    actionlib::SimpleClientGoalState state = avoid_clientor.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        bool avoid_succeed_result = avoid_clientor.getResult()->avoid_success;
        ROS_INFO("Avoid action completed, success: %s", avoid_succeed_result ? "true" : "false");
        return avoid_succeed_result;
    } else {
        ROS_ERROR("Avoid action failed with state: %s", state.toString().c_str());
        return false;
    }
}