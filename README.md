# GZ_DiNiu ROS2 阿克曼底盘控制

这是一个 ROS2 Humble 底盘控制工作区，用标准 `/cmd_vel` 控制阿克曼结构小车。主链路是：

```text
/cmd_vel -> motor_control_node -> 驱动 PWM + /target_steer
                              -> steer_closed_loop_node -> /steer_angle
/linear_velocity + /steer_angle -> ackermann_odom_node -> /wheel_odom
```

## 文档入口

- [阿克曼底盘控制说明](src/docs/阿克曼底盘控制说明.md)：`/cmd_vel` 控制链路、启动、测试、话题和参数
- [转向闭环节点使用说明](src/docs/转向闭环节点使用说明.md)：转向 PID、霍尔编码器、限位、启动回零保护
- [阿克曼里程计节点说明](src/docs/阿克曼里程计节点说明.md)：Ackermann/Bicycle 里程计模型
- [运行维护说明](src/docs/运行维护说明.md)：编译、systemd、日志、网页监控、IO 控制

## 功能包

- `motor_control_py`：订阅 `/cmd_vel`，输出驱动 PWM，并按 Ackermann 模型发布 `/target_steer`
- `steer_closed_loop`：转向位置闭环，发布 `/steer_position`、`/steer_angle`、`/steer_encoder_count`
- `encoder_vel`：驱动轮编码器测速，发布 `/linear_velocity`
- `ackermann_odom`：订阅速度和转向角，发布 `/wheel_odom`
- `io_control_py`：Joy/Bool 话题控制 GPIO 输出
- `imu_bno08x`：BNO08x IMU 采集节点，默认不随一键启动启用
- `betop_teleop`：手柄遥控相关节点

## 启动前注意

- 确认车辆处在安全测试区域，首次测试建议车轮悬空。
- 确认 `pigpiod`、GPIO、PWM、限位开关和刹车电平与实车接线一致。
- 转向闭环默认启动自动找左限位并回 0°；保护触发后会进入 `failed` 并抱闸。

## 快速启动

```bash
cd /root/GZ_DiNiu
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch motor_control_py cmd_vel_full.launch.py
```

使用 systemd 服务：

```bash
sudo systemctl restart ros2-motor-control.service
systemctl status ros2-motor-control.service --no-pager -l
```

## 基本控制

前进并右转：

```bash
ros2 topic pub -r 20 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.3}}"
```

停车：

```bash
ros2 topic pub -1 /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## 常用观察

```bash
ros2 topic echo /linear_velocity
ros2 topic echo /target_steer
ros2 topic echo /steer_angle
ros2 topic echo /wheel_odom
```

## 当前默认保护

- `/cmd_vel` 和驱动闭环日志默认按 1 秒节流。
- 速度反馈超过 `0.5s` 未更新时，驱动 PWM 会强制置 0。
- 停车目标默认直接关闭驱动 PWM，不使用反向 PWM 主动刹车，避免停稳后后退。
- 低速/静止时 `angular.z` 默认用于转向预摆；可将 `ackermann_low_speed_policy` 设为 `ignore` 改为忽略。

## 常用维护

```bash
colcon build
source install/setup.bash
journalctl -u ros2-motor-control.service -f
```
