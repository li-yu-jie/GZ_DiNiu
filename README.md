# /cmd_vel 控制说明

详细文档：[`steer_closed_loop_node 使用说明`](docs/steer_closed_loop_node.md)

## 目标
使用 ROS2 标准话题 `/cmd_vel` 控制本项目：
- `linear.x`：驱动轮速度目标
- `angular.z`：车体角速度目标（经 `motor_control_node` 按 Ackermann 换算成 `target_steer`）

## 1. 环境
```bash
cd /root/GZ_DiNiu
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 2. 启动转向位置闭环（PID）
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

# 启用 Ackermann 转向换算（示例参数）
ros2 run motor_control_py motor_control_node --ros-args \
  -p steer_mode:=ackermann \
  -p ackermann_wheelbase_m:=1.18 \
  -p ackermann_max_steer_angle_deg:=45.0
```

## 4. 发布 /cmd_vel
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.4}}"
```

说明：
- 角速度控制与直线控制都建议持续发布（例如 `-r 20`）。
- `motor_control_node` 默认开启直线超时停止（`cmd_timeout_s=0.3`），超过该时间未收到新 `/cmd_vel` 会自动把驱动目标速度置 0。

## 5. 观察反馈
驱动速度反馈：
```bash
ros2 topic echo /linear_velocity
```

转向位置反馈：
```bash
ros2 topic echo /steer_position
```

转向指令（由 `motor_control_node` 发布）：
```bash
ros2 topic echo /target_steer
```

## 6. 停车/停转向
```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 7. 常用参数
`motor_control_node`：
- `cmd_vel_scale`：`linear` 到驱动速度的缩放，设为 `0.0` 可仅测转向
- `cmd_timeout_s`：直线速度命令超时（秒），默认 `0.3`；设为 `0.0` 关闭超时
- `steer_mode`：`ackermann`（默认）或 `yaw_rate_scale`
- `steer_rate_scale_deg_per_rad_s`：`yaw_rate_scale` 模式下，`angular.z` 转角度目标的缩放
- `steer_topic`：默认 `target_steer`
- `ackermann_wheelbase_m`：Ackermann 轴距（m）
- `ackermann_min_speed_m_s`：最小等效速度阈值，用于低速/静止时防止 `omega/v` 发散
- `ackermann_max_steer_angle_deg`：目标前轮转角限幅（deg）
- `steer_cmd_timeout_s`：转向命令超时回零（秒），默认 `0.0`（关闭）

`steer_closed_loop_node`：
- `cmd_topic`：默认 `target_steer`
- `feedback_topic`：默认 `steer_position`
- `kp` `ki` `kd` `i_max`：位置环 PID 参数
- `min_effective_pwm_percent`：最小有效 PWM，占空比低于该值时按该值输出（提升起步速度）
- `brake_gpio`：刹车控制引脚，默认 `12`（BCM 编码）
- `brake_active_level`：刹车有效电平，默认 `0`（输出 GND = 刹车）
- `startup_auto_home`：默认 `true`，启动自动找左限位并回 0°
- `left_limit_deg` / `right_limit_deg`：默认 `45.0 / -45.0`
- `home_seek_pwm_percent`：找左限位时 PWM 百分比
- `home_zero_tolerance_deg`：回 0° 判定误差阈值

## 8. 快速测试
1) 一键启动：
```bash
ros2 launch motor_control_py cmd_vel_full.launch.py
```

2) 发布 5 秒运动命令：
```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}"
```

3) 停止发布后，驱动轮应在约 `0.3s` 内自动停止（默认超时保护生效）。

## 9. 开机自启动（pigpiod + ROS2）
目标：
- 开机自动启动 `pigpiod`
- 开机自动启动 `ros2 launch motor_control_py cmd_vel_full.launch.py`

已配置的 systemd 服务：
- `/etc/systemd/system/pigpiod.service`
- `/etc/systemd/system/ros2-motor-control.service`

`ros2-motor-control.service` 的启动命令：
```bash
source /opt/ros/humble/setup.bash
source /root/GZ_DiNiu/install/setup.bash
ros2 launch motor_control_py cmd_vel_full.launch.py start_pigpiod:=false
```

说明：`start_pigpiod:=false` 用于避免 launch 文件重复拉起 `pigpiod`。

启用与启动：
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pigpiod.service
sudo systemctl enable --now ros2-motor-control.service
```

常用管理命令：
# 编译指令
colcon build
sudo systemctl restart ros2-motor-control.service
```bash
# 查看状态
systemctl status pigpiod.service --no-pager -l
systemctl status ros2-motor-control.service --no-pager -l

# 重启
sudo systemctl restart pigpiod.service
sudo systemctl restart ros2-motor-control.service

# 停止
sudo systemctl stop ros2-motor-control.service
sudo systemctl stop pigpiod.service
```

日志查看：
```bash
journalctl -u pigpiod.service -f
journalctl -u ros2-motor-control.service -f
```

开机自检建议（重启后）：
```bash
systemctl is-active pigpiod.service
systemctl is-active ros2-motor-control.service
```
期望输出均为 `active`。

网页监控
source /opt/ros/humble/setup.bash
source /root/GZ_DiNiu/install/setup.bash
pkill -f speed_web_node || true
ros2 run motor_control_py speed_web_node


# 单次发 20°
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: 20.0}"

# 单次发 -30°
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: -30.0}"

# 回中 0°
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: 0.0}"
