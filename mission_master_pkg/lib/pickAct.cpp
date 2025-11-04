/**
 * @file pickAct.cpp
 * @brief pick相关action
 */
#include "mission_master_pkg/mission_master.h"
#include <ros/console.h>
#include <boost/bind.hpp>

/**
 * @brief 拾取Action激活回调函数
 * @details 当拾取Action服务器开始处理目标时调用
 */
void MissionMaster::pickActiveCB() {
    ROS_INFO("Pick action server has accepted the goal and started processing");
    current_mission_state_ = ENUM_PICKUP_POINT; // 更新任务状态为执行拾取
}

/**
 * @brief 拾取Action完成回调函数
 * @param state 动作客户端状态
 * @param result 拾取结果
 */
void MissionMaster::pickDoneCB(const actionlib::SimpleClientGoalState &state,
               const mission_master_pkg::PickResultConstPtr &result) {
    ROS_INFO("Pick action finished with state: %s", state.toString().c_str());
    
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED && result->pick_success) {
        ROS_INFO("Pick mission completed successfully");
        current_mission_state_ = ENUM_PICKUP_SUCCEED; // 更新状态为拾取成功
    } else {
        ROS_WARN("Pick mission failed or was preempted");
        // 失败处理逻辑可根据需求添加
    }
}

/**
 * @brief 拾取Action反馈回调函数
 * @param feedback 拾取过程中的反馈信息
 */
void MissionMaster::pickFeedbackCB(const mission_master_pkg::PickFeedbackConstPtr &feedback) {
    ROS_INFO("Pick current step: %d", feedback->pick_current_step);
    
    // 根据步骤添加进度提示
    switch(feedback->pick_current_step) {
        case 0:
            ROS_INFO("Pick initializing...");
            break;
        case 1:
            ROS_INFO("Detecting target object...");
            break;
        case 2:
            ROS_INFO("Performing pickup operation...");
            break;
        case 3:
            ROS_INFO("Verifying pickup success...");
            break;
        default:
            ROS_INFO("Pick proceeding with step %d", feedback->pick_current_step);
    }
}

/**
 * @brief 启动拾取Action
 */
void MissionMaster::pickAct() {
    // 创建并设置拾取目标
    mission_master_pkg::PickGoal goal;
    goal.pick_enable = true; // 启动拾取

    // 发送目标并注册回调函数
    pick_clientor.sendGoal(goal,
        boost::bind(&MissionMaster::pickDoneCB, this, _1, _2),
        boost::bind(&MissionMaster::pickActiveCB, this),
        boost::bind(&MissionMaster::pickFeedbackCB, this, _1));

    ROS_INFO("Pick goal sent to action server");
}

/**
 * @brief 执行拾取任务逻辑
 * @return 拾取是否成功
 */
bool MissionMaster::pickExecute() {
    // 判断拾取Action客户端是否正在运行任务
    if (pick_clientor.getState().isActive()) {
        ROS_INFO("Previous pick action is still active, waiting for completion...");
        return false; // 上个拾取未完成，不重复执行
    }

    // 检查Action服务器是否就绪
    if (!pick_clientor.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Pick action server not available within 5 seconds");
        return false;
    }

    // 发送拾取目标
    pickAct();

    // 等待任务完成（超时时间30秒）
    if (!pick_clientor.waitForResult(ros::Duration(30.0))) {
        ROS_ERROR("Pick action timed out");
        return false;
    }

    // 获取拾取结果
    actionlib::SimpleClientGoalState state = pick_clientor.getState();
    if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        bool pick_succeed_result = pick_clientor.getResult()->pick_success;
        ROS_INFO("Pick action completed, success: %s", pick_succeed_result ? "true" : "false");
        return pick_succeed_result;
    } else {
        ROS_ERROR("Pick action failed with state: %s", state.toString().c_str());
        return false;
    }
}