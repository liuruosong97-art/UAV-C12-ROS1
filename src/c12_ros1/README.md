# c12_ros1

ROS1 package for the C12 gimbal camera target mission pipeline.

This package only adds the gimbal, camera, target pose, localization, and mission nodes. It does not modify LIO or the existing planner.

## Function

After the UAV is already flying and hovering, this package drives the C12 gimbal camera to search for a target tag. When the tag is detected, the node estimates the tag pose in the camera optical frame, uses TF plus the gimbal joint angles and mount extrinsics to transform it into the world frame, locks a stable target pose, then publishes a hover goal above the target to the existing planner.

Default integration with the existing LIO/planner stack:

- LIO odometry input: `/lio/robo/odom`
- Planner goal output: `/remote/goal`
- LIO map input is still handled by the existing planner: `/lio/cloud_world`

## Start

Start the original UAV stack first:

1. `roscore`
2. Livox driver
3. LIO
4. MAVROS
5. Existing planner / PX4 control nodes

Then start the full C12 mission pipeline with one command:

```bash
roslaunch c12_ros1 c12_hardware_pipeline.launch
```

The mission starts automatically in `SEARCH`; no extra `/tag_mission/start` call is required.

## What The Launch Starts

`launch/c12_hardware_pipeline.launch` starts:

- `c12_gimbal`: UDP control for C12 gimbal angles and speed.
- `c12_odom_tf_bridge`: publishes `world -> base_link` TF from `/lio/robo/odom`.
- `robot_state_publisher`: publishes the C12 gimbal TF chain from joint states.
- `c12_mount_tf`: static body-to-gimbal mount transform, `base_link -> c12_mount`.
- `c12_tag_pose`: opens RTSP, detects AprilTag/ArUco/QR target, publishes target pose in camera frame, controls gimbal search/tracking.
- `c12_tag_localization`: transforms camera-frame target pose into `world`, filters and locks a stable target pose.
- `c12_tag_mission`: mission state machine; publishes hover goal above locked target.
- `rviz`: opens `config/c12_task.rviz` for camera/debug/pose visualization.

`launch/c12_driver.launch` is only for low-level debugging. It starts the gimbal driver and image bridge, but not target localization or mission planning.

## Configuration

All normal edits are made inside package files, not through launch arguments.

Main config:

```text
config/c12_pipeline.yaml
```

Important fields:

- `c12_gimbal.camera_ip`: C12 camera/gimbal IP.
- `c12_tag_pose.rtsp_url`: visible camera RTSP URL.
- `c12_tag_pose.detector_type`: `apriltag`, `aruco`, or `qrcode`.
- `c12_tag_pose.target_id`: target tag ID; use `-1` to accept any ID.
- `c12_tag_pose.tag_size_m`: physical tag side length in meters.
- `c12_tag_pose.enable_gimbal_search`: enables automatic search scan.
- `c12_tag_pose.enable_gimbal_tracking`: enables visual servo tracking after detection.
- `c12_tag_mission.odom_topic`: default `/lio/robo/odom`.
- `c12_tag_mission.planner_goal_topic`: default `/remote/goal`.
- `c12_tag_mission.hover_height_m`: hover height above the target.
- `c12_tag_mission.auto_start`: default `true`.

Camera calibration:

```text
config/camera_1280x720.yaml
```

The default calibration is copied from:

```text
c12_tag_mission_ws/src/c12_tag_pose/config/camera_1280x720.yaml
```

It is the calibrated 1280x720 C12 visible camera file. If the camera stream resolution or zoom changes, recalibrate or replace this file.

Body-to-gimbal mount transform is in `launch/c12_hardware_pipeline.launch`:

```xml
<node pkg="tf2_ros" type="static_transform_publisher" name="c12_mount_tf"
      args="0.10 0.0 0.0 -1.578 0.0 0.0 base_link c12_mount"/>
```

ROS1 `static_transform_publisher` argument order here is:

```text
x y z yaw pitch roll parent_frame child_frame
```

Measure and set the transform from UAV body center `base_link` to the C12 mount frame `c12_mount`.

## TF Chain

Expected TF chain:

```text
world
  -> base_link
    -> c12_mount
      -> c12_yaw_link
        -> c12_pitch_link
          -> c12_roll_link
            -> c12_visible_optical_frame
```

Sources:

- `world -> base_link`: `odom_tf_bridge_node.py`, from `/lio/robo/odom`.
- `base_link -> c12_mount`: static transform in launch.
- `c12_mount -> c12_visible_optical_frame`: URDF plus `/c12/gimbal/joint_states`.

Target pose conversion uses:

```text
/c12/tag/pose_camera + TF(world <- c12_visible_optical_frame)
```

and publishes:

```text
/c12/tag/pose_world
/c12/tag/pose_world_filtered
/c12/tag/locked_pose_world
```

## Topic Interfaces

### LIO / Planner

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/lio/robo/odom` | `nav_msgs/Odometry` | subscribe | UAV pose from LIO. |
| `/remote/goal` | `geometry_msgs/PoseStamped` | publish | Hover goal above locked target, consumed by existing planner. |
| `/lio/cloud_world` | `sensor_msgs/PointCloud2` | not used directly | Existing planner reads this from LIO. |

### Gimbal

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/c12/gimbal/cmd_angle_deg` | `geometry_msgs/Vector3` | subscribe | Absolute command: `x=yaw_deg`, `y=pitch_deg`, `z=speed_deg_s`. |
| `/c12/gimbal/cmd_speed_deg_s` | `geometry_msgs/Vector3` | subscribe | Speed command: `x=yaw_speed`, `y=pitch_speed`, `z` unused. |
| `/c12/gimbal/cmd_ptz` | `std_msgs/String` | subscribe | Text command: `left/right/up/down/stop/center`. |
| `/c12/gimbal/cmd_center` | `std_msgs/Empty` | subscribe | Center/home command. |
| `/c12/gimbal/angles_deg` | `geometry_msgs/Vector3Stamped` | publish | Current `yaw/pitch/roll` in degrees when firmware reports full GAC. |
| `/c12/gimbal/joint_states` | `sensor_msgs/JointState` | publish | Joint states for `robot_state_publisher`. |
| `/c12/gimbal/raw_rx` | `std_msgs/String` | publish | Raw UDP receive packets for debugging. |

### Camera / Detection

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/c12/tag/visible` | `std_msgs/Bool` | publish | Whether target is currently visible. |
| `/c12/tag/id` | `std_msgs/Int32` | publish | Detected tag ID. |
| `/c12/tag/code` | `std_msgs/String` | publish | Detected target code or ID string. |
| `/c12/tag/center_pixel` | `geometry_msgs/PointStamped` | publish | Pixel center; `z` stores detector score. |
| `/c12/tag/pose_camera` | `geometry_msgs/PoseStamped` | publish | Target pose in `c12_visible_optical_frame`. |
| `/c12/tag/pose_error` | `std_msgs/Float32` | publish | Detector pose/reprojection error. |
| `/c12/tag/debug/compressed` | `sensor_msgs/CompressedImage` | publish | Debug image with target outline and center marker. |

### Localization / Mission

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/c12/tag/pose_world` | `geometry_msgs/PoseStamped` | publish | Raw target pose in world frame. |
| `/c12/tag/pose_world_filtered` | `geometry_msgs/PoseStamped` | publish | Median-filtered target pose. |
| `/c12/tag/locked_pose_world` | `geometry_msgs/PoseStamped` | publish, latched | Stable locked target pose. |
| `/c12/tag/locked` | `std_msgs/Bool` | publish, latched | Whether target lock is active. |
| `/c12/tag/position_std_m` | `std_msgs/Float32` | publish | Max position standard deviation in lock window. |
| `/tag_mission/state` | `std_msgs/String` | publish, latched | Mission state. |
| `/tag_mission/arrived` | `std_msgs/Bool` | publish, latched | True after arrival criteria stay stable. |

Mission states:

```text
IDLE, SEARCH, ACQUIRE, APPROACH, FINE_ALIGN, ARRIVED, TARGET_LOST, ABORTED
```

## Service Interfaces

| Service | Type | Purpose |
|---|---|---|
| `/tag_mission/start` | `std_srvs/Trigger` | Manually enter `SEARCH`. Usually not needed because `auto_start` is true. |
| `/tag_mission/reset` | `std_srvs/Trigger` | Reset mission state to `IDLE`. |
| `/tag_mission/abort` | `std_srvs/Trigger` | Publish hold goal and enter `ABORTED`. |
| `/c12/tag/reset_lock` | `std_srvs/Trigger` | Clear locked target pose before searching for a new target. |

## RViz

The full launch opens RViz with:

```text
config/c12_task.rviz
```

It shows:

- `/c12/tag/debug/compressed`: camera image with detection overlay.
- TF frames.
- C12 gimbal model.
- `/lio/robo/odom`: UAV odometry.
- `/c12/tag/pose_world_filtered`: filtered target pose.
- `/c12/tag/locked_pose_world`: locked target pose.
- `/remote/goal`: target hover goal sent to planner.

If LIO already opens its own RViz, this C12 RViz is still useful for the camera and tag pipeline. If onboard compute is tight, remove or comment the `rviz` node in `c12_hardware_pipeline.launch`.

## Quick Checks

After launch, check these topics:

```bash
rostopic hz /c12/gimbal/angles_deg
rostopic hz /c12/tag/debug/compressed
rostopic echo /c12/tag/visible
rostopic echo /c12/tag/pose_camera
rostopic echo /c12/tag/locked
rostopic echo /remote/goal
```

Common issues:

- No image: check `c12_tag_pose.rtsp_url`, network route to camera IP, and `ffmpeg/ffprobe`.
- No world target pose: check TF chain from `world` to `c12_visible_optical_frame`.
- Target pose scale is wrong: check `tag_size_m` and camera calibration resolution.
- Arrival never becomes true: check image size, pixel threshold, gimbal home pitch/yaw, and `/lio/robo/odom`.
