# Moveit2的使用

> 记录本人使用moveit2的一些经历

> 本人环境linux-mint22.1（基于ubuntu24.04）
>
> ${ROS_DISTRO}为jazzy

## 安装ros2

这部分内容为`mint-use.md`的`ros2`安装部分

在这里就不做过多介绍了

## 安装Moveit2

这里为使用Moveit2的二进制包安装

不打算使用编译安装（后期可能考虑弄以下编译安装），原因如下

1. 使用二进制包安装更方便，操作步骤更少，更不容易出错。
2. 使用二进制包安装更节省时间。
3. moveit2二进制包安装和ros2二进制包的地方更集中，我导入ros2的环境进入我的IDE中后，不需要重新把我moveit2的环境在导入我的IDE中。

以下为二进制包安装的步骤

>  以下步骤都是在安装完ros2-jazzy后进行的

```bash
# 安装ros-jazzy-moveit
sudo apt install ros-jazzy-moveit
# 安装RVIZ(这个似乎一般不需要再次安装，似乎好像安装ros时会自动安装)
sudo apt install ros-jazzy-rviz2
# 安装ros2_control
sudo apt install ros-jazzy-ros2-control ros-jazzy-ros2-controllers
# 安装Cyclone DDS（Moveit2官方推荐的DDS中间件）（可选，不过我没安装）
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
# 若选择此DDS要添加环境
echo "export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp" >> ~/.bashrc
```

以下为安装一些教程和示例的包（可选）

```bash
sudo apt install ros-jazzy-moveit-resources ros-jazzy-moveit-msgs ros-jazzy-moveit-visual-tools
```

## Launch再学

### 动态构造文件路径

我目前已知的可以动态的构造文件路径的函数有

`PathJoinSubstitution`，`FindPackageShare`，`LaunchConfiguration`

他们的引入

```python
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
```



` FindPackageShare`：用来查找功能包的路径，返回的值是动态的值。

`PathJoinSubstitution`：用来拼接路径的。可以接收静态字符串，也可以接收动态的值作为参数。例如

```python
# 获取包的共享目录
pkg_share = FindPackageShare('my_package')

# 使用 PathJoinSubstitution 构造文件的完整路径
config_file_path = PathJoinSubstitution([pkg_share, 'config', 'params.yaml'])
```

所以通常和`FindPackageShare`一起使用。

`LaunchConfiguration`：用于在运行中获取参数的值。这些参数的值可以由用户传入或者默认值或者其他设置。例如

```python
def generate_launch_description():
    return LaunchDescription([
        # 声明一个参数
        DeclareLaunchArgument(
            name='robot_name',
            default_value='turtlebot',
            description='Name of the robot'
        ),
        # 使用 LaunchConfiguration 引用参数
        Node(
            package='my_package',
            executable='my_node',
            name='my_node',
            parameters=[{
                'robot_name': LaunchConfiguration('robot_name')
            }]
        )
    ])
```

当值是动态的文件路径时，就无法通过直接用`print`来打印出文件的路径了，因为这是这是的值要以来当前`launch`的上下文环境才能确定。一种比较简单的办法来打印出动态文件路径的办法。使用`OpaqueFunction`来获得运行时的上下文环境参数。

```python
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import OpaqueFunction

def evaluate_path(context, *args, **kwargs):
    # 获取 PathJoinSubstitution 的值
    pkg_share = FindPackageShare('own_gz_demo')
    path = PathJoinSubstitution([pkg_share, 'config', 'bridge.yaml'])
    resolved_path = path.perform(context)  # 解析为实际路径

    print(f"Resolved PathJoinSubstitution: {resolved_path}")
    print(f"Resolved PathJoinSubstitution: {pkg_share.perform(context)}")
    return []  # OpaqueFunction 必须返回一个列表

def generate_launch_description():

    return LaunchDescription([
        OpaqueFunction(function=evaluate_path),
    ])

```

如此便可以打印出动态的文件路径。

> `FindPackageShare`和`get_package_share_directory`都可以获取文件路径，`FindPackageShare`和是动态的，而`get_package_share_directory`是静态的。故当函数参数为动态的时候只能使用`FindPackageShare`比如
>
> ```python
> from launch import LaunchDescription
> from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
> from launch.actions import DeclareLaunchArgument
> from launch_ros.substitutions import FindPackageShare
> 
> def generate_launch_description():
>     return LaunchDescription([
>         DeclareLaunchArgument('pkg_name', default_value='my_package'),
>         PathJoinSubstitution([
>             FindPackageShare(LaunchConfiguration('pkg_name')),  # 动态包名
>             'config',
>             'params.yaml'
>         ])
>     ])
> ```
>
> 使用动态包名，这时候只能使用`FindPackageShare`。

## Gazebo仿真

> 在linux-mint22.1上即（ubuntu24.04上），对应的Gazebo是Harmonic版

同理也是二进制包安装

```bash
# 下载密钥，开启Gazebo源
sudo curl https://packages.osrfoundation.org/gazebo.gpg --output /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
# 添加源，注意此处要选择$(. /etc/os-release && echo $UBUNTU_CODENAME)而不要选择$(lsb_release -cs)，在Linux-mint（这种非标准的Ubuntu/Debian中）中直接使用lsb_release -cs无法正确识别我们是基于哪个Ubuntu发行版的，而使用. /etc/os-release && echo $UBUNTU_CODENAME可以识别我们的发行版是基于哪个发行版的
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(. /etc/os-release && echo $UBUNTU_CODENAME)  main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
# 更新
sudo apt update
# 安装gz-harmonic
sudo apt install gz-harmonic
```

到这里已经安装完了`gz-harmonic`，接下来为安装`ros2`和`gz-harmonic`的接口

```bash
sudo apt install ros-jazzy-ros-gz
```

## 使用Moveit2的Cpp的API

### 规划组

创建一个规划组对象

```c++
static const std::string PLANNING_GROUP = "panda_arm";
moveit::planning_interface::MoveGroupInterface move_group(move_group_node, PLANNING_GROUP);
```

显示规划组的基坐标系的名称

```c++
move_group.getPlanningFrame();	//const std::string&
```

显示末端执行器的名称

```c++
move_group.getEndEffectorLink();	//const std::string&
```

显示所有规划组的名称

```c+=
move_group.getJointModelGroupNames()
```

规划一个Pose目标

```c++
// 设置一个Pose点作为目标
geometry_msgs::msg::Pose target_pose1;
target_pose1.orientation.w = 1.0;
target_pose1.position.x = 0.28;
target_pose1.position.y = -0.2;
target_pose1.position.z = 0.5;
move_group.setPoseTarget(target_pose1);

// 判断这点能否规划抵达
moveit::planning_interface::MoveGroupInterface::Plan my_plan;

bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

RCLCPP_INFO(LOGGER, "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");
// 执行这次规划
move_group.execute();

// 规划并执行这次路径
// move_group.move()
```

规划一个关节的状态空间作为目标

```c++
moveit::core::RobotStatePtr current_state = move_group.getCurrentState(10);

std::vector<double> joint_group_positions;
current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);

joint_group_positions[0] = -1.0;  // radians	// 设置第一个关节的目标角度
move_group.setJointValueTarget(joint_group_positions);	// 作为目标进行规划

move_group.setMaxVelocityScalingFactor(0.05);	// 把允许的最大速度降低为最大值的5%。(默认是5%)
move_group.setMaxAccelerationScalingFactor(0.05);	// 把允许的最大加速度降低为最大值的5%。(默认是5%)
// 最大值的默认首选是在joint_limits.yaml

success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
RCLCPP_INFO(LOGGER, "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
```

带有约束条件的规划

```c++
moveit_msgs::msg::OrientationConstraint ocm;
ocm.link_name = "panda_link7";	// 约束的目标Link，末端Link
ocm.header.frame_id = "panda_link0";	// 参考的坐标系，姿态相对于panda_link0定义
ocm.orientation.w = 1.0;	// 四元数表示的姿态，w=1.0表示无旋转
ocm.absolute_x_axis_tolerance = 0.1;	// 姿态X轴方向的弧度绝对容差
ocm.absolute_y_axis_tolerance = 0.1;	// 姿态Y轴方向的弧度绝对容差
ocm.absolute_z_axis_tolerance = 0.1;	// 姿态Z轴方向的弧度绝对容差
ocm.weight = 1.0;	// 约束的权重(1.0表示最高优先级)

// Now, set it as the path constraint for the group.
moveit_msgs::msg::Constraints test_constraints;
test_constraints.orientation_constraints.push_back(ocm);	//将约束添加到约束集合
move_group.setPathConstraints(test_constraints);	// 为move_group设置路径约束
// 这样设置以后的规划都是在这样的约束上，知道解除约束。
```

解除约束

````c++
// 解除路径上的约束
move_group.clearPathConstraints();
````

使用以笛卡尔路径的规划。

```c++
// 单个点时可以如下
auto plan_cartesian = 
    [&move_group](geometry_msgs::msg::Pose pose) -> std::pair<bool, moveit_msgs::msg::RobotTrajectory> {
    	moveit_msgs::msg::RobotTrajectory trajectory;
    	std::vector<geometry_msgs::msg::Pose> poses;
    	poses.push_back(pose);
    	bool success = (move_group.computeCartesianPath(poses, 0.1, trajectory) == moveit::core::MoveItErrorCode::SUCCESS);
    	return make_pair(success,trajectory);
};

auto [s, j] = plan_cartesian(target_pose1);

// 多个点的规划
std::vector<geometry_msgs::msg::Pose> waypoints;
waypoints.push_back(start_pose2);

geometry_msgs::msg::Pose target_pose3 = start_pose2;

target_pose3.position.z -= 0.2;
waypoints.push_back(target_pose3);  // down

target_pose3.position.y -= 0.2;
waypoints.push_back(target_pose3);  // right

target_pose3.position.z += 0.2;
target_pose3.position.y += 0.2;
target_pose3.position.x -= 0.2;
waypoints.push_back(target_pose3);  // up and left

// We want the Cartesian path to be interpolated at a resolution of 1 cm,
// which is why we will specify 0.01 as the max step in Cartesian translation.
const double eef_step = 0.01;	// 每一步可能的最大路长
moveit_msgs::msg::RobotTrajectory trajectory;
double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);	// 返回值为规划的路径完成度百分数

```

设置规划的起始位置

````c++
// 先设置为初始位置
moveit::core::RobotState start_state(*move_group.getCurrentState());
geometry_msgs::msg::Pose start_pose2;
start_pose2.orientation.w = 1.0;
start_pose2.position.x = 0.55;
start_pose2.position.y = -0.05;
start_pose2.position.z = 0.8;
start_state.setFromIK(joint_model_group, start_pose2);	// 以初始位置找到离指定姿态的关节空间
move_group.setStartState(start_state);
````



### 场地

创建一个场景对象

```c++
moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
```

在场地中添加一个物体对象

```c++
// Now, let's define a collision object ROS message for the robot to avoid.
moveit_msgs::msg::CollisionObject collision_object;
collision_object.header.frame_id = move_group.getPlanningFrame();

// The id of the object is used to identify it.
collision_object.id = "box1";

// Define a box to add to the world.
shape_msgs::msg::SolidPrimitive primitive;
primitive.type = primitive.BOX;
primitive.dimensions.resize(3);
primitive.dimensions[primitive.BOX_X] = 0.1;
primitive.dimensions[primitive.BOX_Y] = 1.5;
primitive.dimensions[primitive.BOX_Z] = 0.5;

// Define a pose for the box (specified relative to frame_id).
geometry_msgs::msg::Pose box_pose;
box_pose.orientation.w = 1.0;
box_pose.position.x = 0.48;
box_pose.position.y = 0.0;
box_pose.position.z = 0.25;

collision_object.primitives.push_back(primitive);
collision_object.primitive_poses.push_back(box_pose);
collision_object.operation = collision_object.ADD;

std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
collision_objects.push_back(collision_object);

// Now, let's add the collision object into the world
// (using a vector that could contain additional objects)
RCLCPP_INFO(LOGGER, "Add an object into the world");
planning_scene_interface.addCollisionObjects(collision_objects);	// 添加了一个长方体
```

删除物体

````c++
RCLCPP_INFO(LOGGER, "Remove the objects from the world");
std::vector<std::string> object_ids;
object_ids.push_back(collision_object.id);
planning_scene_interface.removeCollisionObjects(object_ids);
````



给末端添加一个拾取物体

```c++
moveit_msgs::msg::CollisionObject object_to_attach;
object_to_attach.id = "cylinder1";

shape_msgs::msg::SolidPrimitive cylinder_primitive;
cylinder_primitive.type = primitive.CYLINDER;
cylinder_primitive.dimensions.resize(2);
cylinder_primitive.dimensions[primitive.CYLINDER_HEIGHT] = 0.20;
cylinder_primitive.dimensions[primitive.CYLINDER_RADIUS] = 0.04;

// We define the frame/pose for this cylinder so that it appears in the gripper.
object_to_attach.header.frame_id = move_group.getEndEffectorLink();	// 坐标系选择末端坐标系
geometry_msgs::msg::Pose grab_pose;
grab_pose.orientation.w = 1.0;
grab_pose.position.z = 0.2;

// First, we add the object to the world (without using a vector).
object_to_attach.primitives.push_back(cylinder_primitive);
object_to_attach.primitive_poses.push_back(grab_pose);
object_to_attach.operation = object_to_attach.ADD;
planning_scene_interface.applyCollisionObject(object_to_attach);

// Then, we "attach" the object to the robot. It uses the frame_id to determine which robot link it is attached to.
// We also need to tell MoveIt that the object is allowed to be in collision with the finger links of the gripper.
// You could also use applyAttachedCollisionObject to attach an object to the robot directly.
RCLCPP_INFO(LOGGER, "Attach the object to the robot");
std::vector<std::string> touch_links;
touch_links.push_back("panda_rightfinger");
touch_links.push_back("panda_leftfinger");
move_group.attachObject(object_to_attach.id, "panda_hand", touch_links);	// 末端添加的物体可以和末端Link发生碰撞
```

删除拾取物体

````c++
// 从末端取下物体
RCLCPP_INFO(LOGGER, "Detach the object from the robot");
move_group.detachObject(object_to_attach.id);

// 从世界中删除物体
RCLCPP_INFO(LOGGER, "Remove the objects from the world");
std::vector<std::string> object_ids;
object_ids.push_back(object_to_attach.id);
planning_scene_interface.removeCollisionObjects(object_ids);
````



### 可视化

创建rviz可视化tool

```c++
namespace rvt = rviz_visual_tools;
moveit_visual_tools::MoveItVisualTools visual_tools(move_group_node, "panda_link0", "move_group_tutorial", move_group.getRobotModel());

visual_tools.deleteAllMarkers();
visual_tools.loadRemoteControl();
```

`rviz`可视化显示文字

```c++
auto const draw_title = [&visual_tools](auto text) {
	auto const text_pose = [] {
  		auto msg = Eigen::Isometry3d::Identity();
  		msg.translation().z() = 1.0;
  		return msg;
	}();
	visual_tools.publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
};
draw_title("")	//便可以可视化显示文字
```

`bash`中提示文字，按压`next`继续。

```c++
auto const prompt = [&visual_tools](auto text) {
	visual_tools.prompt(text);
};
prompt("")
```

`rviz`中显示轨迹

```c++
// 规划一条轨迹
auto const [success, plan] = [&move_group]{
    moveit::planning_interface::MoveGroupInterface::Plan msg;
    auto const ok = static_cast<bool>(move_group.plan(msg));
    return std::make_pair(ok, msg);
}();
// 显示这条轨迹
auto const draw_trajectory_tool_path = 
	[&visual_tools,
	jmg = move_group.getRobotModel()->getJointModelGroup("panda_arm")]
	(auto const trajectory) {
		visual_tools.publishTrajectoryLine(trajectory, jmg);
};

draw_trajectory_tool_path(plan.trajectory);
```

发送到`rviz`中

```c++
visual_tools.trigger();
```

## 大致信息流

如果直接使用信息流的话，就是将反馈信息发给`/joint_state`话题，然后`moveit`将会以其从`/joint_state`获得的数据作为规划的`start_state`，但是如果自己手动设置过`start_state`的话，那么下一次规划会以这次手动设置过的`start_state`作为起点。



## 初步的Ros2_Control

最小配置如下（官网说的）：

1. 一个yaml配置文件，告诉Moveit哪些控制器可用，哪些关节和控制器的关联，以及Moveit控制器接口类型。
2. 一个启动文件Launch文件，这个启动文件必须加载`moveit_controllers`yaml文件并指定`moveit_simple_controller_manager/MoveItSimpleControllerManager`。加载这些yaml文件后，他们将作为参数传递到MoveGroup节点。
3. 启动相应的 `ros2_control` JointTrajectoryControllers。这与 MoveIt 2 生态系统是分开的。[ 启动 ros2_control 示例 ](https://github.com/ros-controls/ros2_control_demos)。每个 JointTrajectoryController 都提供了一个 action 接口。给定上面的 yaml 文件，MoveIt 会自动连接到这个动作接口。







