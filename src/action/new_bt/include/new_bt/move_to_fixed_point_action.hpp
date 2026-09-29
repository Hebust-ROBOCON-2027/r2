// Copyright 2026 RoboCon Team
// Licensed under the Apache License, Version 2.0

#ifndef NEW_BT__MOVE_TO_FIXED_POINT_ACTION_HPP_
#define NEW_BT__MOVE_TO_FIXED_POINT_ACTION_HPP_

#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace new_bt
{

/**
 * @brief MoveToFixedPointAction 行为树动作节点
 * 
 * 专门用于使底盘移动到指定固定点位姿 (x, y, yaw, frame_id)。
 * 封装了对 Nav2 NavigateToPose 动作服务器的调用与状态监听。
 */
class MoveToFixedPointAction : public BT::StatefulActionNode
{
public:
  using NavigateToPose = nav2_msgs::action::NavigateToPose;
  using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;

  MoveToFixedPointAction(
    const std::string & name,
    const BT::NodeConfig & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr action_client_;
  GoalHandleNavigateToPose::SharedPtr goal_handle_;
  bool goal_result_available_{false};
  rclcpp_action::ClientGoalHandle<NavigateToPose>::WrappedResult result_;

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_goal_pub_;

  geometry_msgs::msg::PoseStamped createGoalMessage(
    double x, double y, double yaw, const std::string & frame_id);
};

}  // namespace new_bt

#endif  // NEW_BT__MOVE_TO_FIXED_POINT_ACTION_HPP_
