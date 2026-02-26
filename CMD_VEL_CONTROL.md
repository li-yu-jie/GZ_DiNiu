# /cmd_vel 控制说明

## 目标
使用 ROS2 标准话题 `/cmd_vel` 控制本项目：
- `linear.x`：驱动轮速度目标
- `angular.z`：转向角速度目标（经 `motor_control_node` 转成 `steer_target_rate_deg_s`）

## 1. 环境
```bash
cd /root/GZ_DiNiu
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 启动转向角速度闭环
```bash
ros2 run steer_closed_loop steer_closed_loop_node
```

## 3. 启动 /cmd_vel 转发与驱动节点
```bash
ros2 run motor_control_py motor_control_node
```

如果只测试转向，不让驱动轮转动：
```bash
ros2 run motor_control_py motor_control_node --ros-args -p cmd_vel_scale:=0.0
```

## 一键启动（推荐）
```bash
ros2 launch motor_control_py cmd_vel_full.launch.py
```

常用参数：
```bash
# 使用 sudo -n 启动 pigpiod
ros2 launch motor_control_py cmd_vel_full.launch.py pigpiod_use_sudo:=true

# 不启动转向闭环
ros2 launch motor_control_py cmd_vel_full.launch.py enable_steer:=false
```

## 4. 发布 /cmd_vel
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}"
```

## 5. 观察反馈
驱动速度反馈：
```bash
ros2 topic echo /linear_velocity
```

转向角速度反馈：
```bash
ros2 topic echo /steer_rate_deg_s
```

转向指令（由 `motor_control_node` 发布）：
```bash
ros2 topic echo /steer_target_rate_deg_s
```

## 6. 停车/停转向
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 7. 常用参数
`motor_control_node`：
- `cmd_vel_scale`：`linear` 到驱动速度的缩放，设为 `0.0` 可仅测转向
- `steer_rate_scale_deg_per_rad_s`：`angular.z` 到 `deg/s` 的缩放
- `steer_topic`：默认 `steer_target_rate_deg_s`

`steer_closed_loop_node`：
- `cmd_topic`：默认 `steer_target_rate_deg_s`
- `feedback_topic`：默认 `steer_rate_deg_s`
- `kp` `ki` `kd` `i_max`：角速度闭环参数
- `limit_active_level`：限位触发电平（`1` 或 `0`）
- `positive_dir_hits_left_limit`：正方向对应哪侧限位
