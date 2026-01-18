#!/bin/bash
# Gazebo 环境变量设置脚本
# 用于解决模型加载和网络连接问题

# 1. 关闭在线模型库（防止 SSL 错误和网络超时）
export GAZEBO_MODEL_DATABASE_URI=""

# 2. 设置正确的模型路径
# - 首先添加系统 Gazebo 模型目录（包含 ground_plane 和 sun）
# - 然后添加本地模型目录（如果有的话）
export GAZEBO_MODEL_PATH="/usr/share/gazebo-11/models:$HOME/.gazebo/models"

# 3. 设置资源路径（包含 Gazebo shader 与本地资源）
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_RESOURCE_PATH="$SCRIPT_DIR/src/robot_moveit_config"
GAZEBO_RESOURCE_PATH="/usr/share/gazebo-11:/usr/share/gazebo:${PKG_RESOURCE_PATH}:${GAZEBO_RESOURCE_PATH}"

echo "Gazebo 环境变量已设置:"
echo "  GAZEBO_MODEL_DATABASE_URI=$GAZEBO_MODEL_DATABASE_URI"
echo "  GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "  GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH"
