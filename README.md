# ROS2 Packages for Coroco Mobile Base
```
                                                                                                 $$$$$$\  
                                                                                                $$  __$$\ 
 $$$$$$$\  $$$$$$\   $$$$$$\   $$$$$$\   $$$$$$$\  $$$$$$\         $$$$$$\   $$$$$$\   $$$$$$$\ \__/  $$ |
$$  _____|$$  __$$\ $$  __$$\ $$  __$$\ $$  _____|$$  __$$\       $$  __$$\ $$  __$$\ $$  _____| $$$$$$  |
$$ /      $$ /  $$ |$$ |  \__|$$ /  $$ |$$ /      $$ /  $$ |      $$ |  \__|$$ /  $$ |\$$$$$$\  $$  ____/ 
$$ |      $$ |  $$ |$$ |      $$ |  $$ |$$ |      $$ |  $$ |      $$ |      $$ |  $$ | \____$$\ $$ |      
\$$$$$$$\ \$$$$$$  |$$ |      \$$$$$$  |\$$$$$$$\ \$$$$$$  |      $$ |      \$$$$$$  |$$$$$$$  |$$$$$$$$\ 
 \_______| \______/ \__|       \______/  \_______| \______/       \__|       \______/ \_______/ \________|
```
[English Version](./READNE-en.md)

## 软件环境
- 架构：x86_64 / arm64
- 操作系统：Ubuntu 20.04
- ROS 版本：Foxy / Humble

该项目也应该可以在其他环境中工作，但只有上面列出的环境做过测试。

## 功能包
- `coroco_chassis` : coroco 底盘控制功能包，主要接受和发布底层CAN信息。并发布自身里程计到 `/odom`话题下。 
- `coroco_msgs` : coroco 自定义消息类型。
- `coroco_control` : coroco 控制功能包，实现一些控制算法。将控制命令发布到`/cmd_vel`话题下。
- `coroco_startup` : 一些 launch file 和其他相关启动配置。


## 依赖及前置配置
- 请确保在运行项目之前安装上 `ttzn_sdk`底层驱动。若没安装，请参考该 [README](https://github.com/ttzntech/ttzn_sdk/#安装底盘驱动) 进行该驱动的安装。
- 请确保初始化了CAN通讯设备，具体请参考 [CAN 设备初始化](https://github.com/ttzntech/ttzn_sdk/#can-设备初始化) 进行初始化。

## 发布接受话题及参数
### 发布话题
以下具体消息类型请查阅 `coroco_msgs` 功能包。各参数含义请查阅 coroco 用户使用手册。
- `/coroco/sys_status` : 发布CAN底层反馈的系统状态。
- `/coroco/move_ctrl_fb` : 发布CAN底层反馈的速度、角速度。
- `/coroco/re_move_ctrl_fb` : 发布由遥控器控制的速度、角速度。
- `/coroco/motor_info_fb` : 发布CAN底层返回的各电机状态。
- `/coroco/warn_fb` : 发布CAN底层返回的警告信息。
- `/coroco/BMS_fb` : 发布CAN底层返回的电池管理系统信息。
- `/coroco/odom_fb` : 发布CAN底层反馈的里程计。\
*注意 : 该odom是四车轮平均行程，不为常规 odom。* 
- `/odom` : 根据速度等信息和自身模型，计算出 odom 并发布。

### 接受话题
- `/cmd_vel` : 接受控制信息，并通过CAN下发底盘。\
*注意 : 上述的 `/cmd_vel` 接收的消息类型为自定义的 `coroco_msgs::msg::MoveCtrl`。* 
- `coroco/mode_ctrl` : 接受控制模式信息，并通过CAN下发底盘。
- `coroco/light_ctrl` : 接受车灯控制信息，并通过CAN下发底盘。

### 参数
- `/coroco/coroco_chassis_node/pub_tf` : 是否发布 tf 变换。
- `/coroco/coroco_chassis_node/base_frame` : tf 变换 base frame 名称，默认 `map`。
- `/coroco/coroco_chassis_node/odom_frame` : tf 变换 odom frame 名称，默认 `odom`。
- `/coroco/coroco_chassis_node/dev_type` : CAN dev 的类型，默认 `0`。\
*注：'0 -> usbttlcan'、 '1 -> canable'、 '2 -> origin'*
- `/coroco/coroco_chassis_node/dev_path` : CAN dev 的路径名称， 默认 `/dev/ttyUSB0`。
- `/coroco/coroco_chassis_node/ctrl_mode` : 控制模式， 默认 `1`。\
*注：'0 -> remote'、'1 -> CAN'*

## 基础使用方法
### 编译该项目
```bash
cd <your_colcon_ws>/src
git clone https://github.com/ttzntech/coroco_ros2.git
cd ..
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```
### 示例
- 启动 coroco_chassis 节点
```bash
# 启动模版 注：<...> 需要替换为自定义内容，若留空则使用默认值
ros2 launch coroco_chassis coroco_chassis.launch.py dev_type:=<dev_type> dev_path:=<dev_path> ctrl_mode:=<ctrl_mode>

# 默认参数启动
ros2 launch coroco_chassis coroco_chassis.launch.py

# 启动 CANAble 模式、设备为 can0
ros2 launch coroco_chassis coroco_chassis.launch.py dev_type:=1 dev_path:=can0

# 启动原生模式、设备为 can0
ros2 launch coroco_chassis coroco_chassis.launch.py dev_type:=2 dev_path:=can0

# 启动遥控器控制模式
ros2 launch coroco_chassis coroco_chassis.launch.py ctrl_mode:=0
```
- 启动 coroco_rviz 和 teleop 键盘控制节点
```bash
# 启动模版 注：<...> 需要替换为自定义内容，若留空则使用默认值
ros2 launch coroco_startup coroco_rviz.launch.py dev_type:=<dev_type> dev_path:=<dev_path> ctrl_mode:=<ctrl_mode>

# 另起一终端
ros2 run coroco_control teleop.py
```
-  在 rviz 中查看 coroco 模型
```bash
ros2 launch coroco_description display_coroco.launch.py 
```

- 动态切换控制模式
```bash
# 切换为 CAN 控制模式
ros2 topic pub --once /coroco/mode_ctrl coroco_msgs/msg/ModeCtrl "{mode: 1}"
# 切换为 遥控器 控制模式
ros2 topic pub --once /coroco/mode_ctrl coroco_msgs/msg/ModeCtrl "{mode: 0}"
```

- 记录遥控器控制数据并回放
```bash
# 启动节点，详细看上文
...

# 记录数据包 Ctrl+C 停止记录
ros2 bag record /coroco/re_move_ctrl_fb -o <file_name>.bag

# 回放数据包
ros2 bag play <file_name>.bag --remap /coroco/re_move_ctrl_fb:=/cmd_vel
```
---
Copyright &copy; 2023 [威海天特智能科技有限公司](http://ttzntech.com/)
