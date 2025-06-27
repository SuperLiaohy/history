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

