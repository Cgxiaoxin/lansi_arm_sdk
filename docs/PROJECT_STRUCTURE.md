# Lansi Arm SDK 项目结构

**版本**: 1.0.0  
**日期**: 2025-02-11  
**作者**: Lansi Robotics

---

## 目录

1. [整体结构](#1-整体结构)
2. [Python SDK 结构](#2-python-sdk-结构)
3. [C++ SDK 结构](#3-c-sdk-结构)
4. [文档结构](#4-文档结构)
5. [文件规范](#5-文件规范)

---

## 1. 整体结构

```
lansi_arm_sdk/
│
├── 📄 README.md                    # 项目自述文件
├── 📄 LICENSE                      # MIT 许可证
├── 📄 .gitignore                   # Git 忽略文件
├── 📄 requirements.txt            # Python 依赖 (pip)
├── 📄 setup.py                     # Python 包配置
├── 📄 pyproject.toml               # Python 项目配置 (Poetry)
│
├── 📄 CMakeLists.txt               # C++ 根 CMake 配置
├── 📄 vcpkg.json                   # C++ 依赖配置
│
├── 📄 Makefile                     # Makefile 快捷命令
├── 📄 docker-compose.yml           # Docker 开发环境
├── 📄 Dockerfile                   # Docker 构建环境
│
├── 🐍 python/                      # Python SDK
│   ├── 📄 setup.py
│   ├── 📄 pyproject.toml
│   ├── 📄 requirements.txt
│   │
│   ├── 📄 lansi_arm/               # 主包
│   │   ├── 📄 __init__.py
│   │   ├── 📄 __version__.py
│   │   ├── 📄 exceptions.py
│   │   ├── 📄 constants.py
│   │   │
│   │   ├── 📂 core/               # 核心通信层
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 can_bus.py
│   │   │   ├── 📄 motor.py
│   │   │   ├── 📄 protocol.py
│   │   │   ├── 📄 data_converter.py
│   │   │   └── 📂 backend/        # CAN 后端
│   │   │       ├── 📄 __init__.py
│   │   │       ├── 📄 socketcan.py
│   │   │       └── 📄 peak_can.py
│   │   │
│   │   ├── 📂 controller/         # 控制器层
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 arm_controller.py
│   │   │   ├── 📄 group_controller.py
│   │   │   ├── 📄 trajectory.py
│   │   │   └── 📄 motion_planner.py
│   │   │
│   │   ├── 📂 kinematics/        # 运动学层
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 kinematics_base.py
│   │   │   ├── 📄 forward_kinematics.py
│   │   │   ├── 📄 inverse_kinematics.py
│   │   │   ├── 📄 jacobian.py
│   │   │   ├── 📄 dh_parameters.py
│   │   │   └── 📄 trajectory_interpolation.py
│   │   │
│   │   ├── 📂 urdf/               # URDF 模型
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 lansi_arm.urdf
│   │   │   ├── 📄 joint_limits.yaml
│   │   │   ├── 📄 meshes/         # 3D 模型文件
│   │   │   │   ├── 📄 base.stl
│   │   │   │   ├── 📄 link1.stl
│   │   │   │   └── 📄 ...
│   │   │   └── 📄 generator.py    # URDF 生成器
│   │   │
│   │   ├── 📂 vision/            # 视觉集成
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 vision_base.py
│   │   │   ├── 📄 hand_eye_calibration.py
│   │   │   ├── 📄 depth_camera.py
│   │   │   └── 📄 collision_detection.py
│   │   │
│   │   ├── 📂 utils/             # 工具类
│   │   │   ├── 📄 __init__.py
│   │   │   ├── 📄 logger.py
│   │   │   ├── 📄 timer.py
│   │   │   ├── 📄 validation.py
│   │   │   ├── 📄 calibration.py
│   │   │   └── 📄 serialization.py
│   │   │
│   │   └── 📂 config/            # 配置
│   │       ├── 📄 __init__.py
│   │       ├── 📄 default_config.py
│   │       └── 📄 schema.py
│   │
│   ├── 📂 examples/              # 示例代码
│   │   ├── 📄 __init__.py
│   │   ├── 📄 01_basic_movement.py
│   │   ├── 📄 02_joint_control.py
│   │   ├── 📄 03_pose_control.py
│   │   ├── 📄 04_trajectory_playback.py
│   │   ├── 📄 05_kinematics_demo.py
│   │   ├── 📄 06_dual_arm_control.py
│   │   ├── 📄 07_advanced_motion.py
│   │   └── 📄 README.md
│   │
│   └── 📂 tests/                 # 测试
│       ├── 📄 __init__.py
│       ├── 📂 unit/               # 单元测试
│       │   ├── 📄 test_kinematics.py
│       │   ├── 📄 test_motor.py
│       │   ├── 📄 test_can_bus.py
│       │   └── 📄 test_data_converter.py
│       │
│       ├── 📂 integration/        # 集成测试
│       │   ├── 📄 test_arm_controller.py
│       │   └── 📄 test_trajectory.py
│       │
│       ├── 📂 fixtures/           # 测试数据
│       │   ├── 📄 trajectories/
│       │   └── 📄 urdf/
│       │
│       └── 📄 conftest.py         # pytest 配置
│
├── 🇨🇺 cpp/                        # C++ SDK
│   ├── 📄 CMakeLists.txt
│   ├── 📄 vcpkg.json
│   │
│   ├── 📂 include/lansi_arm/      # 头文件
│   │   ├── 📄 version.hpp
│   │   ├── 📄 types.hpp
│   │   ├── 📄 exceptions.hpp
│   │   │
│   │   ├── 📂 core/              # 核心通信层
│   │   │   ├── 📄 can_bus.hpp
│   │   │   ├── 📄 motor.hpp
│   │   │   ├── 📄 protocol.hpp
│   │   │   └── 📄 data_converter.hpp
│   │   │
│   │   ├── 📂 controller/        # 控制器层
│   │   │   ├── 📄 arm_controller.hpp
│   │   │   ├── 📄 group_controller.hpp
│   │   │   └── 📄 trajectory.hpp
│   │   │
│   │   ├── 📂 kinematics/       # 运动学层
│   │   │   ├── 📄 kinematics.hpp
│   │   │   ├── 📄 forward_kinematics.hpp
│   │   │   ├── 📄 inverse_kinematics.hpp
│   │   │   └── 📄 jacobian.hpp
│   │   │
│   │   └── 📂 utils/             # 工具类
│   │       ├── 📄 logger.hpp
│   │       └── 📄 math_utils.hpp
│   │
│   ├── 📂 src/                   # 源文件
│   │   ├── 📄 version.cpp
│   │   │
│   │   ├── 📂 core/
│   │   │   ├── 📄 can_bus.cpp
│   │   │   ├── 📄 motor.cpp
│   │   │   ├── 📄 protocol.cpp
│   │   │   └── 📄 data_converter.cpp
│   │   │
│   │   ├── 📂 controller/
│   │   │   ├── 📄 arm_controller.cpp
│   │   │   ├── 📄 group_controller.cpp
│   │   │   └── 📄 trajectory.cpp
│   │   │
│   │   └── 📂 kinematics/
│   │       ├── 📄 kinematics.cpp
│   │       ├── 📄 forward_kinematics.cpp
│   │       ├── 📄 inverse_kinematics.cpp
│   │       └── 📄 jacobian.cpp
│   │
│   └── 📂 examples/              # 示例
│       ├── 📄 basic_example.cpp
│       └── 📄 trajectory_example.cpp
│
├── 📄 docs/                       # 文档
│   ├── 📄 index.md
│   ├── 📄 ARCHITECTURE.md         # 架构设计
│   ├── 📄 API_DESIGN.md           # API 设计
│   ├── 📄 PROJECT_STRUCTURE.md    # 项目结构
│   ├── 📄 ROADMAP.md              # 开发路线图
│   │
│   ├── 📂 user_guide/            # 用户指南
│   │   ├── 📄 01_installation.md
│   │   ├── 📄 02_quick_start.md
│   │   ├── 📄 03_basic_control.md
│   │   ├── 📄 04_advanced_control.md
│   │   ├── 📄 05_trajectory.md
│   │   └── 📄 06_troubleshooting.md
│   │
│   ├── 📂 api_reference/         # API 参考
│   │   ├── 📄 can_bus.md
│   │   ├── 📄 motor.md
│   │   ├── 📄 arm_controller.md
│   │   ├── 📄 kinematics.md
│   │   └── 📄 trajectory.md
│   │
│   └── 📂 development/           # 开发指南
│       ├── 📄 01_contributing.md
│       ├── 📄 02_testing.md
│       ├── 📄 03_documentation.md
│       └── 📄 04_releasing.md
│
├── 📂 scripts/                    # 工具脚本
│   ├── 📄 setup_dev_env.sh        # 开发环境设置
│   ├── 📄 run_tests.sh           # 运行测试
│   ├── 📄 build_docs.sh           # 构建文档
│   ├── 📄 generate_urdf.py        # URDF 生成
│   └── 📄 calibrate_arm.py       # 标定工具
│
└── 📂 .github/                    # GitHub 配置
    ├── 📂 workflows/
    │   ├── 📄 python-package.yml
    │   ├── 📄 cpp-package.yml
    │   └── 📄 docs.yml
    │
    └── 📂 issue_templates/
        ├── 📄 bug_report.md
        └── 📄 feature_request.md
```

---

## 2. Python SDK 结构

### 2.1 包结构

```
lansi_arm/
├── __init__.py                    # 包初始化，导出公共 API
├── __version__.py                 # 版本信息
├── exceptions.py                  # 异常类定义
├── constants.py                   # 常量定义
│
├── core/                          # 核心通信层
│   ├── __init__.py
│   │   # 导出: CanBus, Motor, Protocol, DataConverter
│   │
│   ├── can_bus.py                 # CAN 总线封装
│   │   class CanBus:
│   │       def __init__(channel, bitrate, timeout)
│   │       def connect() -> bool
│   │       def disconnect()
│   │       def send(msg_id, data, timeout) -> bool
│   │       def recv(timeout) -> Optional[Message]
│   │       def register_callback(msg_id, callback)
│   │
│   ├── motor.py                   # 电机控制
│   │   class Motor:
│   │       def enable(enabled) -> bool
│   │       def set_position(pos) -> bool
│   │       def set_velocity(vel) -> bool
│   │       def set_mode(mode) -> bool
│   │       def get_status() -> MotorStatus
│   │       def clear_alarm() -> bool
│   │
│   ├── protocol.py                # 协议定义
│   │   class CommandCode: ...
│   │   class MotorMode: ...
│   │
│   ├── data_converter.py          # 数据转换
│   │   class DataConverter:
│   │       def float_to_bytes(value) -> bytes
│   │       def bytes_to_float(data) -> float
│   │
│   └── backend/                   # CAN 后端实现
│       ├── __init__.py
│       ├── socketcan.py           # Linux SocketCAN
│       └── peak_can.py            # Peak CAN (Windows)
│
├── controller/                    # 控制器层
│   ├── __init__.py
│   │   # 导出: ArmController, GroupController
│   │
│   ├── arm_controller.py          # 单臂控制器
│   │   class ArmController:
│   │       def __init__(motor_ids, can_channel, joint_limits)
│   │       def connect() -> bool
│   │       def disconnect()
│   │       def initialize(mode) -> bool
│   │       def enable(enabled) -> bool
│   │       def get_joint_positions() -> List[float]
│   │       def set_joint_positions(positions, speed, blocking) -> bool
│   │       def get_ee_pose() -> Pose
│   │       def move_to_pose(pose, speed, blocking) -> bool
│   │       def emergency_stop()
│   │       def get_state() -> Dict
│   │
│   ├── group_controller.py        # 群控器
│   │   class GroupController:
│   │       def connect_left(channel) -> bool
│   │       def connect_right(channel) -> bool
│   │       def disconnect_all()
│   │       def move_both(left_pose, right_pose, sync) -> bool
│   │       def play_trajectory(trajectory, sync) -> bool
│   │       def emergency_stop()
│   │
│   ├── trajectory.py              # 轨迹管理
│   │   class Trajectory:
│   │       def __init__(name, points, loop)
│   │       def add_point(point)
│   │       def remove_point(index)
│   │       def save(filepath)
│   │       @staticmethod def load(filepath) -> Trajectory
│   │
│   └── motion_planner.py          # 运动规划
│       class MotionPlanner:
│           def plan_joint_space(start, goal, constraints) -> Trajectory
│           def plan_cartesian_space(start_pose, goal_pose, constraints) -> Trajectory
│           def interpolate(trajectory, num_points) -> Trajectory
│
├── kinematics/                    # 运动学层
│   ├── __init__.py
│   │   # 导出: Kinematics, ForwardKinematics, InverseKinematics
│   │
│   ├── kinematics_base.py         # 运动学基类
│   │   class KinematicsBase:
│   │       @abstractmethod def forward_kinematics(joints) -> Pose
│   │       @abstractmethod def inverse_kinematics(pose, seed) -> Optional[List[float]]
│   │
│   ├── forward_kinematics.py      # 正向运动学
│   │   class ForwardKinematics(KinematicsBase):
│   │       def __init__(dh_parameters)
│   │       def set_dh_parameters(params)
│   │       def forward_kinematics(joints) -> Pose
│   │
│   ├── inverse_kinematics.py      # 逆向运动学
│   │   class InverseKinematics(KinematicsBase):
│   │       def __init__(dh_parameters)
│   │       def inverse_kinematics(pose, seed, constraints) -> List[float]
│   │       def set_constraints(constraints)
│   │
│   ├── jacobian.py                # 雅可比矩阵
│   │   class Jacobian:
│   │       def compute(joints) -> np.ndarray
│   │       def compute_geometric(joints) -> np.ndarray
│   │       def get_singular_values(joints) -> np.ndarray
│   │
│   ├── dh_parameters.py           # DH 参数
│   │   class DHParameters:
│   │       a: List[float]      # 连杆长度
│   │       alpha: List[float]   # 连杆扭角
│   │       d: List[float]       # 连杆偏移
│   │       theta: List[float]   # 关节角度
│   │
│   └── trajectory_interpolation.py # 轨迹插值
│       class TrajectoryInterpolator:
│           def linear_interpolate(p1, p2, t) -> List[float]
│           def cubic_interpolate(p1, p2, v1, v2, t) -> List[float]
│           def quintic_interpolate(p1, p2, v1, v2, a1, a2, t) -> List[float]
│
├── urdf/                          # URDF 模型
│   ├── __init__.py
│   │   # 导出: LansiArmURDF
│   │
│   ├── lansi_arm.urdf            # URDF 文件 (模板)
│   ├── joint_limits.yaml         # 关节限制配置
│   │
│   ├── generator.py              # URDF 生成器
│   │   class URDFGenerator:
│   │       def __init__(config)
│   │       def generate() -> str
│   │       def save(filepath)
│   │
│   └── meshes/                    # 3D 模型文件
│       ├── base.stl
│       ├── link1.stl
│       ├── link2.stl
│       ├── ...
│       └── end_effector.stl
│
├── vision/                        # 视觉集成
│   ├── __init__.py
│   │   # 导出: VisionInterface, HandEyeCalibration
│   │
│   ├── vision_base.py             # 视觉接口基类
│   │   class VisionInterface:
│   │       @abstractmethod def get_point_cloud() -> np.ndarray
│   │       @abstractmethod def get_rgb_image() -> np.ndarray
│   │       @abstractmethod def detect_objects() -> List[DetectedObject]
│   │
│   ├── hand_eye_calibration.py   # 手眼标定
│   │   class HandEyeCalibration:
│   │       def add_calibration_point(robot_pose, vision_pose)
│   │       def calibrate() -> Transformation
│   │       def get_calibration_matrix() -> np.ndarray
│   │
│   ├── depth_camera.py            # 深度相机
│   │   class DepthCamera:
│   │       def __init__(device_id)
│   │       def connect()
│   │       def get_depth_image() -> np.ndarray
│   │       def get_point_cloud() -> np.ndarray
│   │
│   └── collision_detection.py    # 碰撞检测
│       class CollisionDetector:
│           def __init__(urdf_path)
│           def check_collision(robot_state) -> bool
│           def get_collision_pairs(robot_state) -> List[Tuple]
│
├── utils/                         # 工具类
│   ├── __init__.py
│   │
│   ├── logger.py                  # 日志
│   │   class Logger:
│   │       def __init__(name, level)
│   │       def info(msg)
│   │       def debug(msg)
│   │       def warning(msg)
│   │       def error(msg)
│   │
│   ├── timer.py                   # 计时器
│   │   class Timer:
│   │       def __init__(name)
│   │       def start()
│   │       def stop() -> float
│   │       def reset()
│   │
│   ├── validation.py              # 参数验证
│   │   def validate_joint_positions(positions) -> bool
│   │   def validate_pose(pose) -> bool
│   │   def validate_motor_id(motor_id) -> bool
│   │
│   ├── calibration.py             # 标定工具
│   │   class Calibration:
│   │       def __init__(arm_controller)
│   │       def calibrate_joint(joint_index) -> float
│   │       def calibrate_all_joints() -> Dict[int, float]
│   │       def save_calibration(filepath)
│   │       def load_calibration(filepath)
│   │
│   └── serialization.py           # 序列化
│       class Serializer:
│           @staticmethod def to_json(obj) -> str
│           @staticmethod def from_json(json_str) -> Any
│           @staticmethod def save_yaml(obj, filepath)
│           @staticmethod def load_yaml(filepath) -> Any
│
└── config/                        # 配置
    ├── __init__.py
    ├── default_config.py          # 默认配置
    │   class DefaultConfig:
    │       CAN_CHANNEL = "can0"
    │       CAN_BITRATE = 1000000
    │       DEFAULT_VELOCITY = 0.5
    │       DEFAULT_ACCELERATION = 0.5
    │       JOINT_LIMITS = {...}
    │
    └── schema.py                  # 配置模式
        class ConfigSchema:
            @staticmethod def validate(config) -> bool
```

### 2.2 导出规范

```python
# lansi_arm/__init__.py

"""
Lansi Arm SDK - 7-DOF 机械臂控制库
"""

from .__version__ import __version__

# 异常
from .exceptions import (
    LansiArmError,
    CommunicationError,
    CANError,
    ControlError,
    KinematicsError,
)

# 常量
from .constants import MotorMode, ArmState, ControlTarget

# 核心
from .core import CanBus, Motor, DataConverter

# 控制器
from .controller import (
    ArmController,
    GroupController,
    Trajectory,
    MotionPlanner,
)

# 运动学
from .kinematics import (
    Kinematics,
    ForwardKinematics,
    InverseKinematics,
    DHParameters,
    Pose,
)

# URDF
from .urdf import LansiArmURDF

# 工具
from .utils import Logger, Timer, Calibration

__all__ = [
    "__version__",
    # 异常
    "LansiArmError",
    "CommunicationError",
    "CANError",
    "ControlError",
    "KinematicsError",
    # 常量
    "MotorMode",
    "ArmState",
    "ControlTarget",
    # 核心
    "CanBus",
    "Motor",
    "DataConverter",
    # 控制器
    "ArmController",
    "GroupController",
    "Trajectory",
    "MotionPlanner",
    # 运动学
    "Kinematics",
    "ForwardKinematics",
    "InverseKinematics",
    "DHParameters",
    "Pose",
    # URDF
    "LansiArmURDF",
    # 工具
    "Logger",
    "Timer",
    "Calibration",
]
```

---

## 3. C++ SDK 结构

### 3.1 头文件结构

```
include/lansi_arm/
├── version.hpp                    # 版本信息
├── types.hpp                      # 类型定义
├── exceptions.hpp                 # 异常类
│
├── core/                          # 核心通信层
│   ├── can_bus.hpp                # CAN 总线
│   │   class CanBus {
│   │    public:
│   │        CanBus(const std::string& channel, uint32_t bitrate);
│   │        ~CanBus();
│   │        bool connect();
│   │        void disconnect();
│   │        bool send(uint32_t id, const std::vector<uint8_t>& data);
│   │        std::optional<Message> recv(double timeout);
│   │   };
│   │
│   ├── motor.hpp                  # 电机
│   │   class Motor {
│   │    public:
│   │        Motor(uint8_t id, CanBus* bus);
│   │        bool enable(bool enabled);
│   │        bool setPosition(float position);
│   │        bool setVelocity(float velocity);
│   │        MotorStatus getStatus() const;
│   │   };
│   │
│   ├── protocol.hpp               # 协议
│   │   enum class MotorMode { ... };
│   │   struct CommandCode { ... };
│   │
│   └── data_converter.hpp         # 数据转换
│       class DataConverter {
│        public:
│           static std::vector<uint8_t> floatToBytes(float value);
│           static float bytesToFloat(const std::vector<uint8_t>& data);
│       };
│
├── controller/                    # 控制器层
│   ├── arm_controller.hpp         # 单臂控制器
│   │   class ArmController {
│   │    public:
│   │        ArmController(const std::vector<uint8_t>& motorIds,
│   │                        const std::string& channel);
│   │        bool connect();
│   │        void disconnect();
│   │        bool initialize(MotorMode mode);
│   │        std::vector<double> getJointPositions() const;
│   │        bool setJointPositions(const std::vector<double>& positions);
│   │        Pose getEEPose() const;
│   │        bool moveToPose(const Pose& pose);
│   │        void emergencyStop();
│   │   };
│   │
│   ├── group_controller.hpp       # 群控器
│   │   class GroupController { ... };
│   │
│   └── trajectory.hpp             # 轨迹
│       class Trajectory { ... };
│
├── kinematics/                    # 运动学层
│   ├── kinematics.hpp             # 运动学基类
│   │   class Kinematics {
│   │    public:
│   │        virtual ~Kinematics() = default;
│   │        virtual Pose forwardKinematics(
│   │            const std::vector<double>& joints) = 0;
│   │        virtual std::optional<std::vector<double>> inverseKinematics(
│   │            const Pose& pose,
│   │            const std::vector<double>& seed) = 0;
│   │   };
│   │
│   ├── forward_kinematics.hpp     # 正向运动学
│   │   class ForwardKinematics : public Kinematics { ... };
│   │
│   ├── inverse_kinematics.hpp     # 逆向运动学
│   │   class InverseKinematics : public Kinematics { ... };
│   │
│   └── jacobian.hpp               # 雅可比矩阵
│       class Jacobian { ... };
│
└── utils/                         # 工具类
    ├── logger.hpp                 # 日志
    │   class Logger { ... };
    │
    └── math_utils.hpp             # 数学工具
        namespace MathUtils {
            Eigen::Matrix4d createTransformMatrix(...);
            Eigen::Vector3d eulerToQuaternion(...);
        }
```

---

## 4. 文档结构

```
docs/
├── index.md                        # 文档首页
├── ARCHITECTURE.md                 # 架构设计
├── API_DESIGN.md                   # API 设计
├── PROJECT_STRUCTURE.md            # 项目结构
├── ROADMAP.md                      # 开发路线图
│
├── user_guide/                     # 用户指南
│   ├── 01_installation.md          # 安装指南
│   │   # 章节内容:
│   │   # - 环境要求
│   │   # - Python 安装
│   │   # - C++ 安装
│   │   # - Docker 安装
│   │   # - 常见问题
│   │
│   ├── 02_quick_start.md           # 快速开始
│   │   # 章节内容:
│   │   # - Hello World 示例
│   │   # - 基本概念
│   │   # - 第一个程序
│   │
│   ├── 03_basic_control.md         # 基础控制
│   │   # 章节内容:
│   │   # - 连接机械臂
│   │   # - 关节控制
│   │   # - 位置查询
│   │   # - 示教操作
│   │
│   ├── 04_advanced_control.md       # 高级控制
│   │   # 章节内容:
│   │   # - 笛卡尔空间控制
│   │   # - 轨迹规划
│   │   # - 双臂协同
│   │   # - 参数配置
│   │
│   ├── 05_trajectory.md             # 轨迹操作
│   │   # 章节内容:
│   │   # - 轨迹录制
│   │   # - 轨迹编辑
│   │   # - 轨迹播放
│   │   # - 轨迹导入导出
│   │
│   └── 06_troubleshooting.md        # 故障排除
│       # 章节内容:
│       # - 常见错误
│       # - 诊断方法
│       # - 日志分析
│       # - 联系支持
│
├── api_reference/                  # API 参考
│   ├── can_bus.md                  # CAN 总线 API
│   │   # 自动生成 from docstrings
│   │
│   ├── motor.md                    # 电机 API
│   │
│   ├── arm_controller.md           # 控制器 API
│   │
│   ├── kinematics.md               # 运动学 API
│   │
│   └── trajectory.md               # 轨迹 API
│
└── development/                    # 开发指南
    ├── 01_contributing.md          # 贡献指南
    │   # - 如何贡献
    │   # - 代码风格
    │   # - 提交规范
    │   # - Pull Request 流程
    │
    ├── 02_testing.md               # 测试指南
    │   # - 运行测试
    │   # - 编写测试
    │   # - 测试覆盖率
    │
    ├── 03_documentation.md         # 文档指南
    │   # - 文档结构
    │   # - 编写规范
    │   # - 构建文档
    │
    └── 04_releasing.md             # 发布指南
        # - 版本号规则
        # - 发布流程
        # - 更新日志
```

---

## 5. 文件规范

### 5.1 Python 文件规范

```python
#!/usr/bin/env python3
"""
模块描述

详细描述模块的功能、用途和主要类/函数。
"""

from __future__ import annotations

from typing import List, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum
import threading
import time

# 导入相对路径
from .constants import MotorMode
from .exceptions import LansiArmError
from .data_converter import DataConverter


__all__ = ["ClassName", "function_name"]


@dataclass
class ClassName:
    """类描述"""
    attribute: type

    def method(self, param: type) -> return_type:
        """
        方法描述

        Args:
            param: 参数说明

        Returns:
            返回值说明

        Raises:
            Exception: 异常说明
        """
        pass


def function_name(param: type) -> return_type:
    """
    函数描述

    Args:
        param: 参数说明

    Returns:
        返回值说明
    """
    pass
```

### 5.2 C++ 文件规范

```cpp
/**
 * @file filename.hpp
 * @brief 文件描述
 *
 * 详细描述文件的功能、类和函数。
 *
 * @author Lansi Robotics
 * @date 2025-02-11
 */

#ifndef LANSI_ARM_FILENAME_HPP_
#define LANSI_ARM_FILENAME_HPP_

#include <string>
#include <vector>
#include <memory>

namespace lansi_arm {

/**
 * @brief 类描述
 */
class ClassName {
 public:
    /**
     * @brief 构造函数
     * @param param 参数描述
     */
    explicit ClassName(Type param);

    /**
     * @brief 析构函数
     */
    ~ClassName() = default;

    /**
     * @brief 方法描述
     * @param param 参数描述
     * @return 返回值描述
     */
    ReturnType method(Type param);

    /**
     * @brief 静态方法描述
     */
    static ReturnType staticMethod(Type param);

 private:
    Type member_;  ///< 成员变量描述
};

}  // namespace lansi_arm

#endif  // LANSI_ARM_FILENAME_HPP_
```

### 5.3 配置文件规范

```yaml
# config/default_config.py

"""
默认配置参数
"""

from typing import Dict, List


class DefaultConfig:
    """默认配置"""

    # CAN 配置
    CAN_CHANNEL = "can0"
    CAN_BITRATE = 1000000
    CAN_TIMEOUT = 0.1

    # 电机配置
    LEFT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]
    RIGHT_MOTOR_IDS = [61, 62, 63, 64, 65, 66, 67]

    # 运动参数
    DEFAULT_VELOCITY = 0.5
    DEFAULT_ACCELERATION = 0.5
    DEFAULT_DECELERATION = 0.5

    # 关节限制 (弧度)
    JOINT_LIMITS: Dict[int, Dict[str, float]] = {
        0: {"min": -3.14, "max": 3.14},
        1: {"min": -2.5, "max": 2.5},
        2: {"min": -3.0, "max": 3.0},
        3: {"min": -3.0, "max": 3.0},
        4: {"min": -3.0, "max": 3.0},
        5: {"min": -2.5, "max": 2.5},
        6: {"min": -3.14, "max": 3.14},
    }

    # URDF 路径
    URDF_PATH = "lansi_arm/urdf/lansi_arm.urdf"

    # 日志配置
    LOG_LEVEL = "INFO"
    LOG_FORMAT = "%(asctime)s - %(name)s - %(levelname)s - %(message)s"

    # 轨迹配置
    TRAJECTORY_DIR = "trajectories"
    DEFAULT_INTERPOLATION = "quintic"
```

---

## 附录

### A. 版本历史

| 版本 | 日期 | 修改内容 |
|-----|-----|---------|
| 1.0.0 | 2025-02-11 | 初始版本 |

### B. 参考资料

1. [Python  Packaging Authority](https://packaging.python.org/)
2. [CMake Documentation](https://cmake.org/documentation/)
3. [Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)
4. [MkDocs Documentation](https://www.mkdocs.org/)

---

**文档版本**: 1.0.0  
**最后更新**: 2025-02-11
