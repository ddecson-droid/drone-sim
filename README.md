# 四旋翼无人机仿真平台

PX4 SITL + Gazebo Harmonic + ROS 2 Jazzy + RViz2 完整仿真系统。

## 系统架构

```
PX4 SITL (gz_x500)            Micro XRCE-DDS Agent          ROS 2 Jazzy
├── 飞行控制栈                   (UDP :8888)                  ├── px4_tf_broadcaster
├── uXRCE-DDS Client ────────── Agent ──── /fmu/out/*        ├── px4_status_monitor
├── Gazebo 物理引擎                                         ├── robot_state_publisher
└── 传感器仿真 → ros_gz_bridge → /clock, /camera, /lidar     ├── ros_gz_bridge
                                                             └── RViz2 (3D 可视化)
```

## 环境要求

| 组件 | 版本 | 说明 |
|------|------|------|
| 操作系统 | Ubuntu 24.04 LTS / 24.10 | |
| ROS 2 | Jazzy Jalisco | `ros-jazzy-desktop` |
| Gazebo | Harmonic 8.x | `gz-harmonic` |
| PX4 | v1.15+ (main 分支) | 必须 `--recursive` 克隆 |
| 内存 | 8GB+ | 编译阶段需要大内存 |

## 快速开始

### 1. 环境准备

```bash
# 安装 ROS 2 Jazzy
sudo apt update
sudo apt install -y ros-jazzy-desktop python3-colcon-common-extensions

# 安装 Gazebo Harmonic
sudo apt install -y gz-harmonic

# 安装构建工具
sudo apt install -y python3-rosdep cmake build-essential git
```

### 2. 克隆本项目

```bash
git clone --recursive https://github.com/<your-username>/drone-sim.git ~/drone_ws
cd ~/drone_ws
chmod +x setup.sh
./setup.sh
```

> 如果 GitHub 连不上，先配代理：`git config --global url."https://ghfast.top/https://github.com/".insteadOf "https://github.com/"`

### 3. 克隆 PX4（含全部子模块）

```bash
cd ~
git clone --recursive --depth 1 https://github.com/PX4/PX4-Autopilot.git
```

> **注意：** 必须 `--recursive`，否则编译时缺子模块。

### 4. 安装 PX4 依赖

```bash
cd ~/PX4-Autopilot
bash Tools/setup/ubuntu.sh --no-sim-tools
```

### 5. 构建 PX4 SITL

```bash
cd ~/PX4-Autopilot
git init && git add -A && git commit -m "init" && git tag v1.15.0
export GZ_VERSION=harmonic

# 内存不足时用 -j1 限制单核编译
make px4_sitl gz_x500 -j2
```

首次编译约 15-20 分钟。**如果闪退**，说明内存不够：
```bash
# 方法1：单核编译
make px4_sitl gz_x500 -j1

# 方法2：增加 swap
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile && sudo mkswap /swapfile && sudo swapon /swapfile
```

### 6. 构建 ROS 2 工作空间

```bash
cd ~/drone_ws
git clone --depth 1 https://github.com/PX4/px4_msgs.git src/px4_msgs
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> 如果 `drone_utils/drone_control` 报 `libexec directory does not exist`，创建软链接：
> ```bash
> for pkg in drone_utils drone_control; do
>   mkdir -p ~/drone_ws/install/$pkg/lib/$pkg
>   cd ~/drone_ws/install/$pkg/lib/$pkg
>   ln -sf ../../bin/* ./
> done
> ```

## 启动仿真

### 设置环境变量

添加到 `~/.bashrc`：

```bash
source /opt/ros/jazzy/setup.bash
source ~/drone_ws/install/setup.bash

export PX4_AUTOPILOT=$HOME/PX4-Autopilot
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$PX4_AUTOPILOT/Tools/simulation/gz/models
export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:$PX4_AUTOPILOT/build/px4_sitl_default/src/modules/simulation/gz_plugins
```

### 终端 1 — PX4 SITL + Gazebo

```bash
source ~/.bashrc
cd ~/PX4-Autopilot
export GZ_VERSION=harmonic
make px4_sitl gz_x500
```

等 `pxh>` 出现后：

```
param set COM_RCL_EXCEPT 4
```

### 终端 2 — Micro XRCE-DDS Agent

```bash
/snap/bin/MicroXRCEAgent udp4 -p 8888
```

看到 `Client connected` 即可。

### 终端 3 — ROS 2 仿真栈

```bash
source ~/.bashrc
ros2 launch drone_bringup simulation.launch.py
```

RViz2 窗口打开后，**Global Options → Fixed Frame 设为 `odom`**（如果下拉没有就手打）。

### 终端 4（可选）— 自动起飞

```bash
source ~/.bashrc
ros2 launch drone_control offboard.launch.py
```

## PX4 常用命令

在 `pxh>` 终端输入：

| 命令 | 说明 |
|------|------|
| `commander arm -f` | 强制解锁 |
| `commander takeoff` | 起飞（默认 2.5 米） |
| `commander land` | 降落 |
| `commander disarm` | 上锁 |
| `commander mode offboard` | 切换到外部控制模式 |
| `param set COM_RCL_EXCEPT 4` | 绕过 GCS 检查 |
| `commander status` | 查看飞行状态 |

## 启动参数

```bash
# 不启动 RViz2
ros2 launch drone_bringup simulation.launch.py use_rviz:=false

# 启用相机话题桥接
ros2 launch drone_bringup simulation.launch.py use_camera:=true

# 自定义起飞高度和悬停时间
ros2 launch drone_control offboard.launch.py takeoff_height:=-10.0 hover_time:=30.0
```

## 键盘遥控

```bash
ros2 launch drone_control teleop.launch.py
```

| 按键 | 功能 |
|------|------|
| `w`/`s` | 前进/后退 |
| `a`/`d` | 左移/右移 |
| `r`/`f` | 上升/下降 |
| `q`/`e` | 偏航 |
| `t` | 解锁+切换到 Offboard |
| `l` | 降落 |
| `空格` | 悬停 |
| `Esc` | 退出 |

## 验证仿真

```bash
# 查看 PX4 话题
ros2 topic list | grep fmu

# 查看里程计
ros2 topic echo /fmu/out/vehicle_odometry --once

# 查看 TF 树
ros2 run tf2_tools view_frames

# 查看话题频率
ros2 topic hz /fmu/out/vehicle_odometry
```

## 常见问题

### Gazebo 窗口无响应/不显示

```bash
# 软件渲染模式
LIBGL_ALWAYS_SOFTWARE=1 gz sim -g &

# 或增强 VirtualBox 3D 加速：设置 → 显示 → 显存 128MB + 启用 3D 加速
```

### 编译时闪退 (OOM)

- 用 `-j1` 单核编译
- 增加 swap：`sudo fallocate -l 8G /swapfile`
- VirtualBox 给虚拟机分配 8GB+ 内存

### GitHub 连不上

```bash
git config --global url."https://ghfast.top/https://github.com/".insteadOf "https://github.com/"
```

### libexec directory does not exist

```bash
for pkg in drone_utils drone_control; do
  mkdir -p ~/drone_ws/install/$pkg/lib/$pkg
  cd ~/drone_ws/install/$pkg/lib/$pkg
  ln -sf ../../bin/* ./
done
```

### 无法解锁 (Preflight Fail: No connection to GCS)

```
pxh> param set COM_RCL_EXCEPT 4
```

### Micro XRCE-DDS Agent 未找到

```bash
sudo snap install micro-xrce-dds-agent --edge
# 路径在 /snap/bin/MicroXRCEAgent
```

### Gazebo 窗口没出来

Gazebo 可能以无头模式运行。单独打开 GUI：

```bash
gz sim -g &
```

## 项目结构

```
drone_ws/
├── setup.sh                    # 依赖安装
├── run_simulation.sh           # tmux 一键启动
├── README.md
└── src/
    ├── drone_description/      # X500 URDF 模型
    │   ├── urdf/x500.urdf
    │   └── launch/description.launch.py
    ├── drone_bringup/          # 启动编排
    │   ├── launch/
    │   │   ├── simulation.launch.py   # 顶层启动
    │   │   ├── gz_bridge.launch.py
    │   │   └── rviz.launch.py
    │   └── config/
    │       ├── gz_bridge.yaml
    │       └── rviz_config.rviz
    ├── drone_utils/            # 工具节点
    │   └── drone_utils/
    │       ├── px4_tf_broadcaster.py   # NED→ENU 坐标转换
    │       └── px4_status_monitor.py
    └── drone_control/          # 外部控制
        └── drone_control/
            ├── offboard_control.py    # 自动起飞
            └── keyboard_teleop.py     # 键盘遥控
```

## 坐标系统

| 坐标系 | 说明 |
|--------|------|
| PX4 (NED) | x=北, y=东, z=下 (FRD 机体系) |
| ROS (ENU) | x=东, y=北, z=上 (FLU 机体系) |
| TF: odom → base_link | `px4_tf_broadcaster` 自动转换 |

## 参考资料

- [PX4 ROS 2 用户指南](https://docs.px4.io/main/en/ros2/)
- [PX4 Gazebo 仿真](https://docs.px4.io/main/en/sim_gazebo_gz/)
- [ROS 2 Jazzy 文档](https://docs.ros.org/en/jazzy/)
- [Gazebo Harmonic 文档](https://gazebosim.org/docs/harmonic)

## License

MIT
