// Copyright 2026 RoboCon Team
// Licensed under the Apache License, Version 2.0

#include "new_bt/move_to_fixed_point_action.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace new_bt
{

MoveToFixedPointAction::MoveToFixedPointAction(
  const std::string & name,
  const BT::NodeConfig & config)
: BT::StatefulActionNode(name, config),
  goal_result_available_(false)
{
}

BT::PortsList MoveToFixedPointAction::providedPorts()
{
  return {
    BT::InputPort<rclcpp::Node::SharedPtr>("node", "ROS2 node (optional, if omitted reads from blackboard)"),
    BT::InputPort<double>("x", "Target X coordinate in meters"),
    BT::InputPort<double>("y", "Target Y coordinate in meters"),
    BT::InputPort<double>("yaw", "Target Yaw orientation in radians"),
    BT::InputPort<std::string>("frame_id", "map", "Reference frame id"),
    BT::InputPort<std::string>("action_name", "/AT_R2/navigate_to_pose", "Nav2 Action server name"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "{goal}", "Published goal pose")
  };
}

BT::NodeStatus MoveToFixedPointAction::onStart()
{
  // 1. 获取 ROS 节点句柄 (优先从端口输入获取，若无则从黑板获取)
  if (!node_) {
    if (!getInput("node", node_)) {
      if (!config().blackboard->get<rclcpp::Node::SharedPtr>("node", node_)) {
        RCLCPP_ERROR(rclcpp::get_logger("MoveToFixedPointAction"),
                     "Missing ROS node: neither port [node] nor blackboard key [node] was found.");
        return BT::NodeStatus::FAILURE;
      }
    }
  }

  if (!node_) {
    RCLCPP_ERROR(rclcpp::get_logger("MoveToFixedPointAction"), "ROS Node pointer is null.");
    return BT::NodeStatus::FAILURE;
  }

  // 2. 获取 action server 名称 (默认 /AT_R2/navigate_to_pose)
  std::string action_name = "/AT_R2/navigate_to_pose";
  getInput("action_name", action_name);

  if (!action_client_) {
    action_client_ = rclcpp_action::create_client<NavigateToPose>(node_, action_name);
  }

  // 广播当前目标给可能存在的恢复行为节点
  if (!current_goal_pub_) {
    auto goal_qos = rclcpp::QoS(1).transient_local().reliable();
    current_goal_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>(
      "/AT_R2/current_nav_goal", goal_qos);
  }

  // 3. 读取目标点坐标
  double x{0.0}, y{0.0}, yaw{0.0};
  std::string frame_id{"map"};

  if (!getInput("x", x)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToFixedPointAction] Missing required input [x]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("y", y)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToFixedPointAction] Missing required input [y]");
    return BT::NodeStatus::FAILURE;
  }
  if (!getInput("yaw", yaw)) {
    RCLCPP_ERROR(node_->get_logger(), "[MoveToFixedPointAction] Missing required input [yaw]");
    return BT::NodeStatus::FAILURE;
  }
  getInput("frame_id", frame_id);

  // 4. 等待 Nav2 动作服务端可用
  if (!action_client_->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node_->get_logger(),
      "[MoveToFixedPointAction] Nav2 Action server '%s' not available.", action_name.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // 5. 构造目标消息并发送
  auto goal_msg = createGoalMessage(x, y, yaw, frame_id);
  auto action_goal = NavigateToPose::Goal();
  action_goal.pose = goal_msg;

  RCLCPP_INFO(node_->get_logger(),
    "[MoveToFixedPointAction] 正在发送目标点: (x=%.3f, y=%.3f, yaw=%.3f) frame='%s'",
    x, y, yaw, frame_id.c_str());

  auto send_goal_options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    [this](const GoalHandleNavigateToPose::SharedPtr & goal_handle) {
      if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "[MoveToFixedPointAction] 目标被导航服务端拒绝！");
        goal_handle_ = nullptr;
      } else {
        RCLCPP_INFO(node_->get_logger(), "[MoveToFixedPointAction] 目标已被服务端接收，底盘开始移动...");
        goal_handle_ = goal_handle;
      }
    };

  send_goal_options.result_callback =
    [this](const GoalHandleNavigateToPose::WrappedResult & result) {
      goal_result_available_ = true;
      result_ = result;

      switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
          RCLCPP_INFO(node_->get_logger(), "✓ [MoveToFixedPointAction] 导航成功到达固定点！");
          break;
        case rclcpp_action::ResultCode::ABORTED:
          RCLCPP_ERROR(node_->get_logger(), "✗ [MoveToFixedPointAction] 导航被中止！");
          break;
        case rclcpp_action::ResultCode::CANCELED:
          RCLCPP_WARN(node_->get_logger(), "[MoveToFixedPointAction] 导航已被取消。");
          break;
        default:
          RCLCPP_ERROR(node_->get_logger(), "[MoveToFixedPointAction] 导航返回未知状态码。");
          break;
      }
    };

  send_goal_options.feedback_callback =
    [this](GoalHandleNavigateToPose::SharedPtr,
           const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
      RCLCPP_DEBUG(node_->get_logger(),
        "[MoveToFixedPointAction] 剩余距离: %.2f 米", feedback->distance_remaining);
    };

  goal_result_available_ = false;
  action_client_->async_send_goal(action_goal, send_goal_options);

  setOutput("goal", goal_msg);
  if (current_goal_pub_) {
    current_goal_pub_->publish(goal_msg);
  }

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus MoveToFixedPointAction::onRunning()
{
  if (goal_result_available_) {
    if (result_.code == rclcpp_action::ResultCode::SUCCEEDED) {
      return BT::NodeStatus::SUCCESS;
    } else {
      return BT::NodeStatus::FAILURE;
    }
  }
  return BT::NodeStatus::RUNNING;
}

void MoveToFixedPointAction::onHalted()
{
  if (goal_handle_) {
    RCLCPP_INFO(node_->get_logger(), "[MoveToFixedPointAction] 节点被终止，正在取消导航目标...");
    action_client_->async_cancel_goal(goal_handle_);
    goal_handle_ = nullptr;
  }
  goal_result_available_ = false;
}

geometry_msgs::msg::PoseStamped MoveToFixedPointAction::createGoalMessage(
  double x, double y, double yaw, const std::string & frame_id)
{
  geometry_msgs::msg::PoseStamped goal_msg;
  goal_msg.header.stamp = node_->now();
  goal_msg.header.frame_id = frame_id;
  goal_msg.pose.position.x = x;
  goal_msg.pose.position.y = y;
  goal_msg.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  goal_msg.pose.orientation = tf2::toMsg(q);

  return goal_msg;
}

}  // namespace new_bt

// 注册节点到 BehaviorTree.CPP 工厂
#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<new_bt::MoveToFixedPointAction>("MoveToFixedPoint");
  // 同时别名注册为 PublishGoal，保持对现有树的最大兼容
  factory.registerNodeType<new_bt::MoveToFixedPointAction>("PublishGoal");
}
