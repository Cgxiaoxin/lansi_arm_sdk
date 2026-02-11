# Lansi Arm SDK

<div align="center">

![License](https://img.shields.io/badge/License-MIT-blue.svg)
![Python](https://img.shields.io/badge/Python-3.9%2B-yellowgreen)
![C++](https://img.shields.io/badge/C%2B%2B-17-orange)
![Version](https://img.shields.io/badge/Version-1.0.0-blueviolet)

**Lansi 7-DOF 机械臂 SDK - 专业级机器人开发工具包**

[English](README_EN.md) | 简体中文

</div>

## 📖 概述

Lansi Arm SDK 是一个专业级的 7 自由度机械臂软件开发工具包，支持 Linux 平台（Windows 后续支持），提供 Python 和 C++ 两种语言接口。

### ✨ 核心特性

- 🔄 **7-DOF 双臂支持** - 左臂(51-57) + 右臂(61-67) 独立或协同控制
- 📡 **多协议通信** - CAN总线 + Modbus/TCP
- 🧮 **完整运动学** - 正解/逆解/雅可比矩阵
- 🎯 **高级规划** - 关节空间/笛卡尔空间轨迹规划
- 🤖 **视觉集成** - 手眼标定/深度相机/碰撞检测
- 📚 **完善文档** - MkDocs + Material 主题

### 🚀 快速开始

```python
from lansi_arm import ArmController

# 创建控制器
arm = ArmController(
    motor_ids=[51, 52, 53, 54, 55, 56, 57],
    can_channel="can0"
)

# 连接并初始化
arm.connect()
arm.initialize()

# 获取关节位置
positions = arm.get_joint_positions()
print(f"当前关节角度: {positions}")

# 设置目标位置
arm.set_joint_positions([0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0])

# 断开连接
arm.disconnect()
```

### 📦 安装

```bash
# Python (需要 Python 3.9+)
pip install lansi-arm-sdk

# C++ (需要 C++17)
git clone https://github.com/lansi-robotics/lansi_arm_sdk.git
cd lansi_arm_sdk/cpp
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

### 📚 文档

| 文档 | 说明 |
|-----|-----|
| [用户指南](docs/user_guide/README.md) | 快速入门和使用教程 |
| [API 参考](docs/api/README.md) | 完整的 API 文档 |
| [示例代码](examples/) | 各种使用场景示例 |
| [故障排除](docs/troubleshooting.md) | 常见问题和解决方案 |

### 🏗️ SDK 架构

```
lansi_arm_sdk/
├── 🐍 python/                 # Python SDK
│   ├── lansi_arm/            # 主包
│   │   ├── core/             # 核心通信层
│   │   ├── controller/       # 控制器层
│   │   ├── kinematics/       # 运动学层
│   │   ├── urdf/             # URDF 模型
│   │   ├── vision/           # 视觉集成
│   │   └── utils/            # 工具类
│   ├── examples/             # 示例
│   └── tests/                # 测试
│
├── 🇨🇺 cpp/                   # C++ SDK
│   ├── include/lansi_arm/    # 头文件
│   ├── src/                  # 实现
│   └── examples/              # 示例
│
└── 📄 docs/                   # 文档
```

### 🛠️ 技术栈

| 层级 | 技术选型 |
|-----|---------|
| 通信协议 | CAN Bus (SocketCAN), Modbus/TCP |
| Python | 3.9+, pytest, mypy, black |
| C++ | 17, CMake, vcpkg, googletest |
| 文档 | MkDocs + Material |
| 测试 | pytest (Python), googletest (C++) |

### 📋 支持的功能

#### 基础控制
- [x] 单关节控制
- [x] 多关节同步控制
- [x] 速度/加速度配置
- [x] 紧急停止
- [x] 零点标定

#### 运动学
- [x] 正向运动学 (FK)
- [x] 逆向运动学 (IK)
- [x] 雅可比矩阵计算
- [x] 奇异性检测

#### 轨迹规划
- [x] 关节空间插值
- [x] 笛卡尔空间插值
- [x] 速度/加速度规划
- [x] 轨迹平滑处理

#### 高级功能
- [x] 双臂协同控制
- [x] 碰撞检测
- [x] 手眼标定
- [x] 深度相机集成

### 🤝 贡献

欢迎贡献代码！请查看 [贡献指南](CONTRIBUTING.md)。

### 📄 许可证

本项目采用 MIT 许可证 - 查看 [LICENSE](LICENSE) 文件。

### 📞 联系

- 作者: [alex]
- GitHub: [@lansi-robotics](https://github.com/lansi-robotics)
- 邮箱: 178176916cc@gmail.com

---

<div align="center">

**Built with ❤️ by Lansi Robotics**

</div>
