# GZ_DiNiu ROS2 Nodes

本仓库包含多个 ROS2 功能包：
- **encoder_vel**：使用 libgpiod 读取编码器并发布线速度（std_msgs/Float64）。
- **motor_control_py**：使用 pigpio 守护进程输出硬件 PWM，订阅 `/cmd_vel` 与反馈速度做 PID 闭环控制。
- **imu_bno08x**：通过 I2C 读取 BNO08x IMU，发布 Imu 与 MagneticField。
- **steer_closed_loop**：转向电机角速度闭环控制（霍尔三相编码器计数 + 限位保护）。

## 依赖
- ROS2 Humble
- libgpiod (C++ 读取 GPIO)
- pigpio / pigpiod (Python 输出硬件 PWM)
- sysfs PWM（/sys/class/pwm/*）

## 构建
```bash
cd /home/y/GZ_DiNiu
colcon build
source install/setup.bash
```

## 运行电机控制（Python）
启动 pigpiod：
```bash
sudo pigpiod
## 一键启动（Launch）
```bash
ros2 launch motor_control_py bringup.launch.py start_pigpiod:=true
```

可选参数：
```bash
# 启动 pigpiod（默认 true）
ros2 launch motor_control_py bringup.launch.py start_pigpiod:=true

# 使用 sudo 启动 pigpiod（需要免密 sudo）
ros2 launch motor_control_py bringup.launch.py pigpiod_use_sudo:=true

# 不启动 IMU
ros2 launch motor_control_py bringup.launch.py enable_imu:=false

# 不启动转向闭环
ros2 launch motor_control_py bringup.launch.py enable_steer:=false
```

1) 启动 pigpiod：
```bash
sudo pigpiod
```
2) 先运行编码器速度发布（提供反馈）：
```bash
ros2 run encoder_vel encoder_vel_node
```
3) 运行电机控制节点：
```bash
ros2 run motor_control_py motor_control_node
```
4) 发布 `/cmd_vel`（geometry_msgs/Twist）：
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```
说明：默认使用 `linear.x` 作为行走目标速度；`angular.z` 作为转向角速度指令，直接发布到 `steer_target_rate_deg_s`。

5) 查看反馈线速度（由编码器节点发布）：
```bash
ros2 topic echo /linear_velocity
```

### motor_control_py 参数
- `pwm_gpio`：PWM 引脚（默认 18，BCM18/物理脚 12）
- `dir_gpio`：方向引脚（默认 26）
- `pwm_freq_hz`：PWM 频率（默认 100000）
- `invert_dir`：方向反转（默认 true）
- `cmd_topic`：速度命令话题名（默认 `/cmd_vel`）
- `cmd_vel_axis`：从 `cmd_vel.linear` 取哪一轴做速度控制（`x`/`y`/`z`，默认 `x`）
- `cmd_vel_scale`：命令缩放系数（默认 `1.0`）
- `feedback_topic`：反馈速度话题名（默认 linear_velocity）
- `enable_steer_cmd`：是否把 `cmd_vel.angular.z` 转成转向角速度发布（默认 `true`）
- `steer_topic`：转向角速度话题（默认 `steer_target_rate_deg_s`）
- `steer_rate_scale_deg_per_rad_s`：角速度缩放系数，`steer_rate_deg_s = angular.z * scale`（默认 `30.0`）
- `kp`/`ki`/`kd`：PID 参数（默认 kp=90, ki=20, kd=0）
- `control_hz`：控制频率（默认 50）
- `filter_alpha`：反馈低通滤波系数（默认 0.05）
- `deadband`：误差死区（默认 0.03）
- `max_pwm_step`：每周期最大 PWM 变化（默认 1.5%）

示例（反转方向）：
```bash
ros2 run motor_control_py motor_control_node --ros-args -p invert_dir:=true
```

## 运行编码器速度发布（C++）
```bash
ros2 run encoder_vel encoder_vel_node
```

## 运行 IMU（Python）
```bash
ros2 run imu_bno08x imu_bno08x_node
```

### 查看 IMU 数据
```bash
ros2 topic echo /imu/data
ros2 topic echo /imu/mag
```

### imu_bno08x 参数
- `i2c_address`：I2C 地址（默认 0x4B）
- `frame_id`：坐标系（默认 imu_link）
- `imu_topic`：IMU 话题（默认 imu/data）
- `mag_topic`：磁力计话题（默认 imu/mag）
- `rate_hz`：IMU 数据频率（默认 50）
- `mag_rate_hz`：磁力计频率（默认 20）
- `use_reset`：启动时软复位（默认 true）

说明：motor_control_py 的 PID 需要反馈速度（/linear_velocity），如果未启动编码器节点，
控制器会保持不输出 PWM。

### encoder_vel 参数
- `chip`：gpiochip 路径（默认 /dev/gpiochip0）
- `gpio_a`：A 相 GPIO（默认 21）
- `gpio_b`：B 相 GPIO（默认 20）
- `counts_per_rev`：每圈脉冲数（默认 600）
- `wheel_radius_m`：轮半径（默认 0.165）
- `publish_hz`：发布频率（默认 50）
- `topic`：发布话题名（默认 linear_velocity）

## 运行转向闭环（C++）
```bash
ros2 run steer_closed_loop steer_closed_loop_node
```

### 发布目标角速度（单位：deg/s）
```bash
ros2 topic pub /steer_target_rate_deg_s std_msgs/Float64 "{data: 20.0}"
```

### 查看角速度反馈
```bash
ros2 topic echo /steer_rate_deg_s
```

### steer_closed_loop 常用参数
- `limit_active_level`：限位触发电平（默认 `1`；若你的限位低电平触发，设为 `0`）
- `limit_left_gpio` / `limit_right_gpio`：左右限位 GPIO（默认 `24` / `25`）
- `cmd_topic`：角速度命令话题（默认 `steer_target_rate_deg_s`）
- `feedback_topic`：角速度反馈话题（默认 `steer_rate_deg_s`）
- `positive_dir_hits_left_limit`：正方向对应左限位（默认 `true`）

## 常见问题
- **PWM 没输出**：
  - 确认 `pigpiod` 已启动：`pgrep -a pigpiod`
  - 用 `pigs hp 18 100000 500000` 验证硬件 PWM 是否有波形
  - 确认测量的是 **BCM18（物理脚 12）**
- **方向反了**：
  - 运行时加 `-p invert_dir:=true`
- **编码器无计数**：
  - 检查 A/B 相 GPIO 号和接线
  - 确认 A 相有上升沿波形
- **转向回零失败**：
  - 检查左右限位 GPIO 引脚号与接线
  - 确认限位触发电平与代码配置一致
