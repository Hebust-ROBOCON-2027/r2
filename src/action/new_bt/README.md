# new_bt — 固定点导航行为树包与节点

本项目专门用于控制机器人底盘**只执行固定点导航移动**任务，无机械臂动作、无视觉传感器或抓取流程依赖。包含完整的 C++ 行为树动作节点、可复用 XML 子树节点以及直接可运行的固定点导航行为树。

---

## 一、目录结构

```text
src/action/new_bt/
├── CMakeLists.txt                                  # 构建配置文件
├── package.xml                                     # 软件包依赖定义
├── README.md                                       # 使用与开发说明
├── include/new_bt/
│   └── move_to_fixed_point_action.hpp              # MoveToFixedPoint 动作节点头文件
├── src/
│   ├── move_to_fixed_point_action.cpp              # 核心节点实现 (对接 Nav2 NavigateToPose)
│   └── new_bt_runner.cpp                           # 独立的轻量级行为树运行器
├── behavior_trees/
│   ├── nav_to_fixed_point.xml                      # 【主树】单固定点导航行为树
│   ├── move_to_fixed_point_subnode.xml             # 【子树节点】可复用的移动到固定点行为树节点
│   └── multi_points_patrol.xml                     # 【巡航树】多固定点顺序导航示例
└── launch/
    └── new_bt_launch.py                            # ROS2 Launch 启动脚本
```

---

## 二、行为树节点说明

### 1. C++ 核心动作节点：`MoveToFixedPoint` (别名 `PublishGoal`)
* **对应类**：[`new_bt::MoveToFixedPointAction`](include/new_bt/move_to_fixed_point_action.hpp)
* **实现原理**：继承自 `BT::StatefulActionNode`，作为客户端异步调用 Nav2 的 `/AT_R2/navigate_to_pose` 动作服务端，在后台监控底盘移动状态，到达目标点返回 `SUCCESS`，遇到阻塞或失败返回 `FAILURE`，被外部中断时自动取消目标。
* **端口定义**：
  | 端口名 | 类型 | 必需 | 默认值 | 说明 |
  | :--- | :--- | :--- | :--- | :--- |
  | `x` | double | 是 | - | 目标点在参考坐标系下的 X 坐标（米） |
  | `y` | double | 是 | - | 目标点在参考坐标系下的 Y 坐标（米） |
  | `yaw` | double | 是 | - | 目标点底盘朝向偏航角（弧度，`0` 表示沿地图 X 轴正向） |
  | `frame_id` | string | 否 | `"map"` | 参考坐标系 |
  | `action_name` | string | 否 | `"/AT_R2/navigate_to_pose"` | Nav2 动作服务名 |
  | `node` | Node::SharedPtr | 否 | 从黑板获取 | ROS2 节点指针 |
  | `goal` | PoseStamped | 输出 | `"{goal}"` | 输出发布的目标 Pose |

### 2. XML 可复用子树节点：`MoveToFixedPointSubTree`
定义于 [`move_to_fixed_point_subnode.xml`](behavior_trees/move_to_fixed_point_subnode.xml)，可通过 `<include>` 引入并在任何其他复杂任务树中像普通节点一样被调用。

---

## 三、编译与运行

### 1. 编译本包
在工作空间根目录执行：
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select new_bt --symlink-install
source install/setup.bash
```

### 2. 前置准备：启动底层导航栈（终端 1）
使用既有脚本启动定位、Costmap 与控制器：
```bash
./run_navigation_blue.sh    # 蓝区
# 或 ./run_navigation_red.sh (红区)
```

### 3. 运行固定点导航行为树（终端 2）

* **方式一：通过 launch 文件启动（推荐）**
  ```bash
  ros2 launch new_bt new_bt_launch.py
  ```
  如果需要指定其他行为树（如多点巡航）：
  ```bash
  ros2 launch new_bt new_bt_launch.py bt_xml:=$(ros2 pkg prefix new_bt)/share/new_bt/behavior_trees/multi_points_patrol.xml
  ```

* **方式二：通过 `new_bt_runner` 节点直接启动**
  ```bash
  ros2 run new_bt new_bt_runner nav_to_fixed_point.xml
  ```

---

## 四、如何修改固定点坐标

打开 [`behavior_trees/nav_to_fixed_point.xml`](behavior_trees/nav_to_fixed_point.xml)，修改 `x`、`y`、`yaw` 属性值即可：

```xml
<MoveToFixedPoint x="2.00"
                  y="1.50"
                  yaw="0.00"
                  frame_id="map"/>
```
修改后无需重新编译，直接再次运行即可生效（因为使用的是 `--symlink-install`）。

---

## 五、在其他行为树中调用

在团队的任意其他行为树 XML 中：
```xml
<root BTCPP_format="4">
  <include path="new_bt/move_to_fixed_point_subnode.xml"/>

  <BehaviorTree ID="MyMission">
    <Sequence>
      <!-- 调用固定点移动节点 -->
      <SubTree ID="MoveToFixedPointSubTree"
               target_x="3.8"
               target_y="-1.8"
               target_yaw="0.0"
               frame_id="map"/>
    </Sequence>
  </BehaviorTree>
</root>
```
