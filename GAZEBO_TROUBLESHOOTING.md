# Gazebo 仿真问题排查与解决方案

本文档记录了解决 Gazebo 仿真环境常见问题的步骤。

## 已解决的问题

### 1. 无法找到 sun 和 ground_plane 模型 ✅

**问题**: `Unable to find uri[model://sun]` 和 `model://ground_plane`

**原因**: GAZEBO_MODEL_PATH 没有正确设置，无法找到系统模型

**解决方案**: 
- 在 `simulation.launch.py` 中添加了 `SetEnvironmentVariable` 来设置正确的模型路径
- 系统模型位于 `/usr/share/gazebo-11/models/`

### 2. 网络/SSL 连接错误 ✅

**问题**: `libcurl: (35) unexpected eof while reading` 和模型库连接失败

**原因**: 离线环境或网络/代理问题导致无法连接在线模型库

**解决方案**:
- 在 launch 文件中设置 `GAZEBO_MODEL_DATABASE_URI=""` 关闭在线模型库

### 3. 插件类型错误 ✅

**问题**: `incorrect plugin type` for `gazebo_ros_init` 和 `gazebo_ros_factory`

**原因**: `<plugin>` 标签可能放在了错误的作用域

**解决方案**:
- 确认 world 文件中的插件标签直接放在 `<world>` 内，而不是 `<model>` 内
- 当前 `table_scene.world` 的插件位置已正确

### 4. GAZEBO_MODEL_PATH 包含错误目录 ✅

**问题**: `Missing model.config for model` 错误

**原因**: GAZEBO_MODEL_PATH 指向了 install 或顶层目录

**解决方案**:
- 使用 `SetEnvironmentVariable` 而非 `AppendEnvironmentVariable` 来明确设置路径
- 只包含真正的模型目录

---

## 运行仿真的步骤

### 方法 1: 直接使用 launch 文件（推荐）

```bash
# 进入工作空间
cd ~/projects/gazebo_ws

# Source ROS 和工作空间
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行仿真
ros2 launch robot_gazebo_sim simulation.launch.py
```

### 方法 2: 手动设置环境变量后运行

```bash
# 进入工作空间
cd ~/projects/gazebo_ws

# Source 环境设置脚本
source setup_gazebo_env.sh

# Source ROS 和工作空间
source /opt/ros/humble/setup.bash
source install/setup.bash

# 运行仿真
ros2 launch robot_gazebo_sim simulation.launch.py
```

---

## 仍可能遇到的警告（可忽略）

1. `Desired controller update period (0.01 s) is slower than the gazebo simulation period (0.001 s)`
   - 仅性能警告，不影响仿真

2. `base_link has an inertia ...`
   - URDF 根惯量警告，可通过添加虚拟 dummy link 规避，但不影响仿真启动

---

## 文件结构

```
gazebo_ws/
├── setup_gazebo_env.sh           # 环境变量设置脚本
├── src/
│   └── robot_gazebo_sim/
│       ├── launch/
│       │   └── simulation.launch.py   # 主启动文件
│       ├── worlds/
│       │   └── table_scene.world      # 仿真世界文件
│       ├── urdf/
│       │   └── robot_system.urdf.xacro
│       └── config/
│           └── display.rviz
└── install/                      # 编译后的安装目录
```

---

## 重要修改说明

### simulation.launch.py 的修改

1. 添加了 `SetEnvironmentVariable` import
2. 设置 `GAZEBO_MODEL_DATABASE_URI=""` 禁用在线模型库
3. 设置 `GAZEBO_MODEL_PATH` 包含系统模型路径 `/usr/share/gazebo-11/models`
4. 使用 `AppendEnvironmentVariable` 追加 ROS 包路径

### table_scene.world 的修改

1. 取消了 `model://sun` 和 `model://ground_plane` 的注释
2. 现在场景有正确的光照和地面

---

## 如果问题仍然存在

1. 检查 Gazebo 是否正确安装：
   ```bash
   dpkg -l | grep gazebo
   ```

2. 验证系统模型存在：
   ```bash
   ls /usr/share/gazebo-11/models/
   ```

3. 检查 ROS 包是否正确编译：
   ```bash
   cd ~/projects/gazebo_ws
   colcon build --packages-select robot_gazebo_sim
   ```

4. 查看详细日志：
   ```bash
   ros2 launch robot_gazebo_sim simulation.launch.py --log-level debug
   ```
