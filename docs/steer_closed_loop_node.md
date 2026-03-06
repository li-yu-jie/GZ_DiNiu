# steer_closed_loop_node 使用说明（位置环 PID）

本文档说明 `src/steer_closed_loop/src/steer_closed_loop_node.cpp` 当前实现的转向位置闭环逻辑。

## 1. 节点职责

`steer_closed_loop_node` 负责：

- 订阅目标转向角：`/target_steer`（`std_msgs/msg/Float64`，单位：度）
- 读取霍尔编码器并估算当前转向角
- 运行位置环 PID，输出方向 GPIO + PWM
- 控制刹车 GPIO（当前默认：输出 GND 为抱闸）
- 发布转向角反馈：`/steer_position`（`std_msgs/msg/Float64`，单位：度）

## 2. 启动状态机

上电后默认按以下顺序执行：

1. `kSeekingLeftLimit`：找左限位
2. `kReturningToZero`：左限位标定后回到 0 度
3. `kDone`：进入正常闭环控制

说明：

- 左限位触发后，会把该位置标定为 `left_limit_deg`（默认 `+45.0`）
- 右限位对应 `right_limit_deg`（默认 `-45.0`）
- 回零完成前会忽略外部 `/target_steer` 命令

## 3. 话题

- 输入命令：`/target_steer`（Float64，单位 deg）
- 输出反馈：`/steer_position`（Float64，单位 deg）

发送角度示例：

```bash
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: 20.0}"
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: -30.0}"
ros2 topic pub -1 /target_steer std_msgs/msg/Float64 "{data: 0.0}"
```

查看反馈：

```bash
ros2 topic echo /steer_position
```

## 4. 关键参数（当前默认值）

### 4.1 硬件与引脚

- `chip_path=/dev/gpiochip0`
- `hu_gpio=5`
- `hv_gpio=6`
- `hw_gpio=13`
- `dir_gpio=16`
- `brake_gpio=12`
- `brake_active_level=0`（低电平有效，输出 GND=刹车）
- `limit_left_gpio=24`
- `limit_right_gpio=25`
- `limit_active_level=1`
- `positive_dir_hits_left_limit=true`

### 4.2 PWM 与编码器

- `pwm_chip_path=/sys/class/pwm/pwmchip0/`
- `pwm_channel=1`
- `pwm_period_ns=100000`
- `encoder_counts_per_rev=9200`
- `invert_encoder=true`
- `event_debounce_us=100`

### 4.3 位置环控制

- `control_hz=50.0`
- `kp=9.5`
- `ki=0.0`
- `kd=0.0`
- `i_max=50.0`
- `position_deadband_deg=0.5`
- `max_pwm_percent=90.0`
- `max_pwm_step=12.0`
- `min_effective_pwm_percent=18.0`
- `cmd_timeout_s=0.0`（关闭超时回中）
- `debug_hz=2.0`

### 4.4 启动回零

- `startup_auto_home=true`
- `left_limit_deg=45.0`
- `right_limit_deg=-45.0`
- `home_seek_pwm_percent=65.0`
- `home_zero_tolerance_deg=1.0`
- `zero_offset_deg=0.0`

## 5. 启动命令

```bash
cd /root/GZ_DiNiu
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run steer_closed_loop steer_closed_loop_node
```

## 6. 调试要点

节点日志包含：

- `state`：当前状态（`seeking_left` / `returning_zero` / `ready`）
- `pos`：当前角度
- `target`：目标角度
- `err`：角度误差
- `out%`：输出 PWM 百分比
- `L` / `R`：左右限位状态
- `B`：刹车状态（1=抱闸，0=松闸）

## 7. 常见问题

### 7.1 发了角度不动

- 可能仍在启动回零阶段（`state != ready`）
- 检查限位是否常态误触发
- 检查刹车电平定义是否与硬件一致（`brake_active_level`）

### 7.2 角度偏差

- 比例误差：优先校准 `encoder_counts_per_rev`
- 零点偏差：调 `zero_offset_deg`

