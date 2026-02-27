# 开机自启动说明（pigpiod + ROS2）

## 目标
- 开机自动启动 `pigpiod`
- 开机自动启动：
  `ros2 launch motor_control_py cmd_vel_full.launch.py`

## 已配置的 systemd 服务
- `/etc/systemd/system/pigpiod.service`
- `/etc/systemd/system/ros2-motor-control.service`

`ros2-motor-control.service` 的启动命令为：
```bash
source /opt/ros/humble/setup.bash
source /root/GZ_DiNiu/install/setup.bash
ros2 launch motor_control_py cmd_vel_full.launch.py start_pigpiod:=false
```

说明：`start_pigpiod:=false` 用于避免 launch 文件重复拉起 `pigpiod`。

## 启用与启动
```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pigpiod.service
sudo systemctl enable --now ros2-motor-control.service
```

## 常用管理命令
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

## 日志查看
```bash
journalctl -u pigpiod.service -f
journalctl -u ros2-motor-control.service -f
```

## 开机自检建议
重启后执行：
```bash
systemctl is-active pigpiod.service
systemctl is-active ros2-motor-control.service
```
期望输出均为 `active`。
