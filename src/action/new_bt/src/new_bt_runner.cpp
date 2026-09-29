// Copyright 2026 RoboCon Team
// Licensed under the Apache License, Version 2.0

#include <chrono>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/bt_factory.h"
#include "new_bt/move_to_fixed_point_action.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("new_bt_runner");

  RCLCPP_INFO(node->get_logger(), "=== new_bt 固定点导航执行器已启动 ===");

  // 1. 创建行为树工厂并注册节点
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<new_bt::MoveToFixedPointAction>("MoveToFixedPoint");
  factory.registerNodeType<new_bt::MoveToFixedPointAction>("PublishGoal");

  // 2. 确定要执行的 XML 文件路径
  std::string bt_file = "nav_to_fixed_point.xml";
  if (argc > 1) {
    bt_file = argv[1];
  }

  std::string full_xml_path = bt_file;
  // 若传入的是相对文件名且本地不存在，尝试从 share 目录中寻找
  if (!std::filesystem::exists(full_xml_path)) {
    try {
      std::string pkg_share = ament_index_cpp::get_package_share_directory("new_bt");
      std::string candidate = pkg_share + "/behavior_trees/" + bt_file;
      if (std::filesystem::exists(candidate)) {
        full_xml_path = candidate;
      }
    } catch (const std::exception &) {
      // 忽略 share 路径定位异常，后续直接报错
    }
  }

  if (!std::filesystem::exists(full_xml_path)) {
    RCLCPP_ERROR(node->get_logger(), "找不到指定的行为树 XML 文件: %s", full_xml_path.c_str());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "正在加载行为树: %s", full_xml_path.c_str());

  // 3. 配置全局黑板
  auto blackboard = BT::Blackboard::create();
  blackboard->set("node", node);

  // 4. 解析并创建行为树
  BT::Tree tree;
  try {
    tree = factory.createTreeFromFile(full_xml_path, blackboard);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(node->get_logger(), "行为树 XML 解析失败: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "行为树构建成功，开始执行...");

  // 5. 循环 Tick 树，直到完成或失败
  rclcpp::WallRate loop_rate(20);  // 20Hz
  BT::NodeStatus status = BT::NodeStatus::RUNNING;

  while (rclcpp::ok() && status == BT::NodeStatus::RUNNING) {
    rclcpp::spin_some(node);
    status = tree.tickOnce();
    loop_rate.sleep();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "✓ 行为树顺利执行完毕 (SUCCESS)");
  } else {
    RCLCPP_WARN(node->get_logger(), "✗ 行为树退出，状态: %s", BT::toStr(status).c_str());
  }

  rclcpp::shutdown();
  return (status == BT::NodeStatus::SUCCESS) ? 0 : 1;
}
