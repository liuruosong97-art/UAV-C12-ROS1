# C12 ROS1 云台相机靶标任务包

本仓库提供一个独立 ROS1 包 `c12_ros1`，用于 C12 云台相机在无人机上的靶标搜索、识别、位姿转换和目标点发布。

代码位置：

```text
src/c12_ros1
```

本包只迁移和新增云台相机、靶标识别、位姿转换、任务状态机相关代码，不修改 LIO、PX4、MAVROS 或现有规划器代码。

## 实现功能

无人机启动并悬停后，C12 云台相机自动开始巡视搜索靶标。检测到靶标后，系统会：

1. 从 C12 RTSP 视频流中检测 AprilTag、ArUco 或 QR 靶标。
2. 根据相机内参和靶标实际尺寸，计算靶标在相机光学坐标系下的位姿。
3. 结合云台 yaw、pitch、roll 角度、云台 URDF、相机安装外参和 LIO 里程计 TF，将靶标位姿转换到 `world` 坐标系。
4. 对靶标世界坐标做窗口滤波和稳定锁定。
5. 以锁定靶标为目标，在其上方生成悬停目标点。
6. 将目标点发布到现有规划器默认目标话题 `/remote/goal`。

## 与现有 LIO/规划器接口

默认接口已经按现有 LIO README 适配：

| 接口 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/lio/robo/odom` | `nav_msgs/Odometry` | 订阅 | LIO 输出的无人机位姿 |
| `/remote/goal` | `geometry_msgs/PoseStamped` | 发布 | 发送给规划器的靶标上方悬停目标点 |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | 本包不直接使用 | 仍由现有规划器读取 |

## 使用步骤

先按原流程启动无人机基础系统：

1. 启动 `roscore`
2. 启动 Livox 雷达驱动
3. 启动 LIO
4. 启动 MAVROS
5. 启动现有规划器和 PX4 控制节点
6. 确认无人机已起飞并处于悬停状态

然后启动完整 C12 任务链：

```bash
roslaunch c12_ros1 c12_hardware_pipeline.launch
```

该 launch 默认 `auto_start: true`，启动后任务状态机会自动进入 `SEARCH`，不需要再手动调用启动服务。

## 完整任务链启动内容

`c12_hardware_pipeline.launch` 会启动：

| 节点 | 作用 |
|---|---|
| `c12_gimbal` | C12 云台 UDP 控制，发布云台角度和 joint state |
| `c12_odom_tf_bridge` | 将 `/lio/robo/odom` 转成 `world -> base_link` TF |
| `robot_state_publisher` | 根据 URDF 和云台 joint state 发布云台 TF |
| `c12_mount_tf` | 发布 `base_link -> c12_mount` 静态安装外参 |
| `c12_tag_pose` | 打开 RTSP，检测靶标，发布相机坐标系下的靶标位姿，并控制云台搜索/跟踪 |
| `c12_tag_localization` | 将靶标位姿转换到 `world`，滤波并锁定稳定靶标 |
| `c12_tag_mission` | 任务状态机，发布靶标上方悬停目标点 |
| `rviz` | 显示相机调试图、TF、靶标位姿、机体位姿和目标点 |

## 配置文件

主要配置文件：

```text
src/c12_ros1/config/c12_pipeline.yaml
```

常用配置项：

| 参数 | 默认值 | 说明 |
|---|---|---|
| `c12_gimbal.camera_ip` | `192.168.1.91` | C12 云台相机 IP |
| `c12_tag_pose.rtsp_url` | `rtsp://192.168.1.91:554/stream=1` | 可见光相机 RTSP 地址 |
| `c12_tag_pose.detector_type` | `apriltag` | 靶标类型，可选 `apriltag`、`aruco`、`qrcode` |
| `c12_tag_pose.target_id` | `0` | 目标 tag ID，设为 `-1` 表示接受任意 ID |
| `c12_tag_pose.tag_size_m` | `0.20` | 靶标实际边长，单位 m |
| `c12_tag_pose.enable_gimbal_search` | `true` | 是否启用云台自动巡视搜索 |
| `c12_tag_pose.enable_gimbal_tracking` | `true` | 检测到靶标后是否启用云台视觉跟踪 |
| `c12_tag_mission.odom_topic` | `/lio/robo/odom` | LIO 里程计输入 |
| `c12_tag_mission.planner_goal_topic` | `/remote/goal` | 规划器目标点输出 |
| `c12_tag_mission.hover_height_m` | `2.0` | 靶标上方悬停高度 |
| `c12_tag_mission.auto_start` | `true` | launch 后是否自动进入搜索 |

## 相机内参

当前默认使用已标定的 1280x720 内参：

```text
src/c12_ros1/config/camera_1280x720.yaml
```

这份内参来自：

```text
c12_tag_mission_ws/src/c12_tag_pose/config/camera_1280x720.yaml
```

如果实际 RTSP 分辨率、相机焦距或变焦倍率变化，需要重新标定或替换该文件。`camera_1920x1080.yaml` 保留为占位示例，不作为默认启动使用。

## 云台安装外参

机体中心到云台安装座的静态外参在：

```text
src/c12_ros1/launch/c12_hardware_pipeline.launch
```

对应节点：

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="c12_mount_tf"
      args="0.10 0.0 0.0 -1.578 0.0 0.0 base_link c12_mount"/>
```

ROS1 `static_transform_publisher` 在此处的参数顺序为：

```text
x y z yaw pitch roll parent_frame child_frame
```

需要根据实机测量结果修改 `base_link -> c12_mount` 的平移和旋转。

## TF 坐标链

期望 TF 链路：

```text
world
  -> base_link
    -> c12_mount
      -> c12_yaw_link
        -> c12_pitch_link
          -> c12_roll_link
            -> c12_visible_optical_frame
```

来源说明：

| TF | 来源 |
|---|---|
| `world -> base_link` | `c12_odom_tf_bridge` 根据 `/lio/robo/odom` 发布 |
| `base_link -> c12_mount` | `c12_mount_tf` 静态外参 |
| `c12_mount -> c12_visible_optical_frame` | C12 URDF + `/c12/gimbal/joint_states` |

靶标定位链路：

```text
/c12/tag/pose_camera
  + TF(world <- c12_visible_optical_frame)
  -> /c12/tag/pose_world
  -> /c12/tag/pose_world_filtered
  -> /c12/tag/locked_pose_world
  -> /remote/goal
```

## 话题接口

### 云台控制

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/gimbal/cmd_angle_deg` | `geometry_msgs/Vector3` | 订阅 | 绝对角度控制，`x=yaw_deg`，`y=pitch_deg`，`z=speed_deg_s` |
| `/c12/gimbal/cmd_speed_deg_s` | `geometry_msgs/Vector3` | 订阅 | 速度控制，`x=yaw_speed`，`y=pitch_speed`，`z` 未使用 |
| `/c12/gimbal/cmd_ptz` | `std_msgs/String` | 订阅 | 文本控制：`left/right/up/down/stop/center` |
| `/c12/gimbal/cmd_center` | `std_msgs/Empty` | 订阅 | 云台回中 |
| `/c12/gimbal/angles_deg` | `geometry_msgs/Vector3Stamped` | 发布 | 云台当前 `yaw/pitch/roll`，单位 degree |
| `/c12/gimbal/joint_states` | `sensor_msgs/JointState` | 发布 | 给 `robot_state_publisher` 使用 |
| `/c12/gimbal/raw_rx` | `std_msgs/String` | 发布 | 原始 UDP 回包，调试用 |

### 相机和靶标检测

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/tag/visible` | `std_msgs/Bool` | 发布 | 当前是否检测到靶标 |
| `/c12/tag/id` | `std_msgs/Int32` | 发布 | 检测到的 tag ID |
| `/c12/tag/code` | `std_msgs/String` | 发布 | 靶标编码字符串 |
| `/c12/tag/center_pixel` | `geometry_msgs/PointStamped` | 发布 | 靶标中心像素坐标，`z` 存检测分数 |
| `/c12/tag/pose_camera` | `geometry_msgs/PoseStamped` | 发布 | 靶标在 `c12_visible_optical_frame` 下的位姿 |
| `/c12/tag/pose_error` | `std_msgs/Float32` | 发布 | 位姿误差或重投影误差 |
| `/c12/tag/debug/compressed` | `sensor_msgs/CompressedImage` | 发布 | 带靶标框和中心点的调试图像 |

### 靶标定位和任务

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/c12/tag/pose_world` | `geometry_msgs/PoseStamped` | 发布 | 靶标在 `world` 下的原始位姿 |
| `/c12/tag/pose_world_filtered` | `geometry_msgs/PoseStamped` | 发布 | 滤波后的靶标世界位姿 |
| `/c12/tag/locked_pose_world` | `geometry_msgs/PoseStamped` | 发布，latched | 稳定锁定后的靶标世界位姿 |
| `/c12/tag/locked` | `std_msgs/Bool` | 发布，latched | 是否已经锁定靶标 |
| `/c12/tag/position_std_m` | `std_msgs/Float32` | 发布 | 锁定窗口内位置标准差 |
| `/tag_mission/state` | `std_msgs/String` | 发布，latched | 当前任务状态 |
| `/tag_mission/arrived` | `std_msgs/Bool` | 发布，latched | 是否到达靶标上方并稳定悬停 |
| `/remote/goal` | `geometry_msgs/PoseStamped` | 发布 | 发送给现有规划器的目标点 |

任务状态包括：

```text
IDLE, SEARCH, ACQUIRE, APPROACH, FINE_ALIGN, ARRIVED, TARGET_LOST, ABORTED
```

## 服务接口

| 服务 | 类型 | 说明 |
|---|---|---|
| `/tag_mission/start` | `std_srvs/Trigger` | 手动进入 `SEARCH`，默认不需要 |
| `/tag_mission/reset` | `std_srvs/Trigger` | 重置任务到 `IDLE` |
| `/tag_mission/abort` | `std_srvs/Trigger` | 发布当前位置保持目标并进入 `ABORTED` |
| `/c12/tag/reset_lock` | `std_srvs/Trigger` | 清除当前锁定靶标，用于重新搜索新目标 |

## RViz 显示

完整任务链会自动打开：

```text
src/c12_ros1/config/c12_task.rviz
```

默认显示：

- `/c12/tag/debug/compressed`：相机画面和靶标检测框
- TF 坐标系
- C12 云台模型
- `/lio/robo/odom`：无人机 LIO 位姿
- `/c12/tag/pose_world_filtered`：滤波靶标位姿
- `/c12/tag/locked_pose_world`：锁定靶标位姿
- `/remote/goal`：发给规划器的悬停目标点

如果 LIO 自己已经打开 RViz，C12 的 RViz 仍然可以用于观察相机画面和靶标任务链。若机载电脑算力紧张，可以在 `c12_hardware_pipeline.launch` 中注释 `rviz` 节点。

## 常用检查命令

```bash
rostopic hz /c12/gimbal/angles_deg
rostopic hz /c12/tag/debug/compressed
rostopic echo /c12/tag/visible
rostopic echo /c12/tag/pose_camera
rostopic echo /c12/tag/locked
rostopic echo /remote/goal
```

TF 检查：

```bash
rosrun tf view_frames
rosrun tf tf_echo world c12_visible_optical_frame
```

## 常见问题

没有图像：

- 检查 `c12_tag_pose.rtsp_url`
- 检查电脑和 C12 相机是否在同一网段
- 检查 `ffmpeg` 和 `ffprobe` 是否可用

有图像但没有靶标：

- 检查 `detector_type`
- 检查 `target_id`
- 检查靶标尺寸 `tag_size_m`
- 检查光照、距离和画面清晰度

有 `/c12/tag/pose_camera` 但没有 `/c12/tag/pose_world`：

- 检查 TF 链是否完整
- 检查 `/lio/robo/odom` 是否正常发布
- 检查 `base_link -> c12_mount` 外参是否发布
- 检查 `/c12/gimbal/joint_states` 是否正常发布

目标点尺度不对：

- 检查相机分辨率是否为 1280x720
- 检查是否使用了正确的 `camera_1280x720.yaml`
- 检查 `tag_size_m` 是否等于真实靶标边长

无人机不到达或一直不到达：

- 检查 `/remote/goal` 是否被现有规划器订阅
- 检查 `/lio/robo/odom` 坐标系是否与规划器一致
- 检查 `hover_height_m`
- 检查 `horizontal_arrival_m`、`vertical_arrival_m`、`pixel_arrival_px`
