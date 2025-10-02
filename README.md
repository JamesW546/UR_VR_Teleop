# UR_VR_TELEOP

## English README

### Overview
This repository contains a ROS2 Python package `ur3_controller` that enables teleoperation of Universal Robots (UR) using VR devices such as Quest. The package maps controller poses to velocity commands sent to UR robots via RTDE.

### System Requirements
- Ubuntu 22.04 LTS
- ROS2
- Python 3.10+

**Note**: This project was developed and tested on Ubuntu 22 Humble + ROS2. Using the same system versions is recommended for successful operation.

### Package Structure
- `src/ur3_controller/`: main ROS2 package containing node scripts and package configuration
  - `ur3_quest_teleop.py`: main node script — reads controller poses and sends speed commands to UR robots using RTDE
  - `package.xml`, `setup.py`: package metadata and installation setup

### Quick Start
1) Install Python dependencies in your ROS2 environment:

```bash
pip install rtde_control oculus_reader numpy
```

2) Create a workspace at the repository root and build:

```bash
# at repository root
mkdir -p ws/src
cp -r src ws/src/
cd ws
colcon build --symlink-install
```

3) Source and run the node:

```bash
source install/setup.bash
ros2 run ur3_controller ur3_quest_teleop
```

### Important Notes
- Replace placeholder IP addresses in the code before running (e.g. `192.168.XXX.XXX`), or pass them via ROS parameters
- Test in a safe environment before running on real hardware and ensure E-stop and speed limits are configured

### License
This project is licensed under the MIT License. See the `LICENSE` file for details.

## 中文说明

### 概述
本仓库包含一个 ROS2 Python 包 `ur3_controller`，用于通过 VR 设备（Quest）遥操作 Universal Robots（UR）机械臂，实现基于控制器位姿的速度控制（teleoperation）。

### 系统要求
- Ubuntu 22.04 LTS
- ROS2
- Python 3.10+

**注意**：本项目在 Ubuntu 22 Humble + ROS2 环境下开发和测试，建议使用相同系统版本以保证成功运行。

### 包结构
- `src/ur3_controller/`：主要的 ROS2 包，包含节点脚本和包配置
  - `ur3_quest_teleop.py`：主节点脚本，读取 Quest 控制器位姿并通过 RTDE 向 UR 机械臂发送速度命令
  - `package.xml`, `setup.py`：包元数据与安装脚本

### 快速开始
1) 在 ROS2 的 Python 环境中安装依赖：

```bash
pip install rtde_control oculus_reader numpy
```

2) 在仓库根目录创建工作区并构建：

```bash
# 在仓库根目录执行
mkdir -p ws/src
cp -r src ws/src/
cd ws
colcon build --symlink-install
```

3) source 并运行节点：

```bash
source install/setup.bash
ros2 run ur3_controller ur3_quest_teleop
```

### 重要提示
- 请在运行前将代码中的占位 IP（例如 `192.168.XXX.XXX`）替换为实际的 UR 机械臂 IP 与 Quest 设备 IP，或将其改为通过 ROS 参数传入
- 在真实硬件上运行前务必先在仿真或安全环境中充分测试，并确保急停（E-stop）与速度限制等安全机制已配置

### 许可证
本项目采用 MIT 许可证，详见 `LICENSE` 文件。
