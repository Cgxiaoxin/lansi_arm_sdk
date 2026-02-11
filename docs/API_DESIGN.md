# Lansi Arm SDK API 设计规范

**版本**: 1.0.0  
**日期**: 2025-02-11  
**作者**: Lansi Robotics

---

## 目录

1. [设计原则](#1-设计原则)
2. [命名规范](#2-命名规范)
3. [类设计](#3-类设计)
4. [接口定义](#4-接口定义)
5. [数据类型](#5-数据类型)
6. [错误处理](#6-错误处理)
7. [使用示例](#7-使用示例)

---

## 1. 设计原则

### 1.1 面向对象原则

SDK 遵循 SOLID 设计原则：

| 原则 | 说明 | 应用 |
|-----|-----|-----|
| **单一职责 (SRP)** | 每个类只有一个职责 | `CanBus` 只管通信，`Kinematics` 只管运动学 |
| **开放封闭 (OCP)** | 对扩展开放，对修改封闭 | 通过接口支持不同的 CAN 后端 |
| **里氏替换 (LSP)** | 子类可以替换父类 | 不同 IK 算法可以替换 |
| **接口分离 (ISP)** | 多个小接口优于一个大接口 | `ReadableStatus`/`ControllableMotor` |
| **依赖反转 (DIP)** | 依赖抽象而非实现 | 依赖 `CanBusBackend` 而非具体实现 |

### 1.2 API 设计原则

```python
# ✅ 好：清晰的职责分离
class Motor:
    def enable(self) -> bool: ...
    def set_position(self, pos: float) -> bool: ...
    def get_status(self) -> MotorStatus: ...

class ArmController:
    def __init__(self, motor_ids: List[int], can_channel: str): ...
    def set_joint_positions(self, positions: List[float]) -> bool: ...

# ❌ 不好：职责混乱
class Arm:
    def connect_can(self) -> bool: ...
    def calculate_fk(self) -> np.ndarray: ...
    def save_to_database(self) -> None: ...
```

---

## 2. 命名规范

### 2.1 类名 (PascalCase)

```python
class ArmController:           # 机械臂控制器
class GroupController:         # 群控控制器
class TrajectoryPlanner:        # 轨迹规划器
class ForwardKinematics:        # 正向运动学
class InverseKinematics:        # 逆向运动学
class CollisionDetector:       # 碰撞检测器
class CanBus:                   # CAN总线
class Motor:                   # 电机
class Pose:                    # 位姿
class JointLimits:             # 关节限制
```

### 2.2 方法名 (snake_case)

```python
# 获取类方法
def get_joint_positions(self) -> List[float]: ...
def get_ee_pose(self) -> Pose: ...
def get_state(self) -> ArmState: ...

# 设置类方法
def set_joint_positions(self, positions: List[float]) -> bool: ...
def set_velocity_limit(self, limit: float) -> None: ...
def set_mode(self, mode: MotorMode) -> bool: ...

# 操作类方法
def enable(self, enabled: bool) -> bool: ...
def connect(self) -> bool: ...
def disconnect(self) -> None: ...
def initialize(self) -> bool: ...
def calibrate_zero(self) -> bool: ...
def emergency_stop(self) -> None: ...

# 布尔判断方法 (使用动词/形容词)
def is_connected(self) -> bool: ...
def is_enabled(self) -> bool: ...
def is_ready(self) -> bool: ...
def has_error(self) -> bool: ...
```

### 2.3 常量和枚举 (SCREAMING_SNAKE_CASE)

```python
class MotorMode(Enum):
    TORQUE = 0
    MIT = 1
    VELOCITY = 2
    PROFILE_VELOCITY = 3
    POSITION = 4
    PROFILE_POSITION = 5


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    INITIALIZED = "initialized"
    ENABLED = "enabled"
    ERROR = "error"


class TrajectoryInterpolation(Enum):
    LINEAR = "linear"
    QUINTIC = "quintic"      # 五次多项式
    CUBIC = "cubic"          # 三次多项式
    B_SPLINE = "b_spline"    # B样条
```

### 2.4 参数命名

```python
# 标准参数名
def set_joint_positions(
    self,
    positions: List[float],      # 关节角度列表
    speed: float = 0.5,          # 速度系数 (0.0-1.0)
    acceleration: float = 0.5,  # 加速度系数
    blocking: bool = True,      # 是否阻塞等待完成
    timeout: Optional[float] = None  # 超时时间
) -> bool: ...

# 位姿参数
def move_to_pose(
    self,
    pose: Pose,                         # 目标位姿
    frame: str = "base",                 # 坐标系
    velocity: float = 0.2,              # 速度
    acceleration: float = 0.5,         # 加速度
    path_constraints: Optional[PathConstraints] = None  # 路径约束
) -> bool: ...

# 索引参数 (明确类型)
def set_joint_position(
    self,
    joint_index: int,       # 关节索引 (0-6)
    position: float,        # 目标位置
    speed: float = 0.5
) -> bool: ...
```

---

## 3. 类设计

### 3.1 核心类图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                           核心类图                                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                          CanBus                                  │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ - channel: str                                                   │   │
│  │ - bitrate: int                                                   │   │
│  │ - timeout: float                                                 │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ + connect(): bool                                                │   │
│  │ + disconnect(): void                                             │   │
│  │ + send(msg_id, data): bool                                       │   │
│  │ + recv(msg_id): Optional[CanMessage]                            │   │
│  │ + register_callback(msg_id, callback): void                     │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    ▲                                     │
│                                    │                                     │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                           Motor                                  │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ - motor_id: int                                                  │   │
│  │ - status: MotorStatus                                            │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ + enable(enabled: bool): bool                                    │   │
│  │ + set_position(pos: float): bool                                │   │
│  │ + set_velocity(vel: float): bool                                │   │
│  │ + set_mode(mode: MotorMode): bool                                │   │
│  │ + get_status(): MotorStatus                                      │   │
│  │ + clear_alarm(): bool                                            │   │
│  │ + calibrate_zero(): bool                                        │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                    ▲                                     │
│                                    │                                     │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                       ArmController                             │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ - motor_ids: List[int]                                           │   │
│  │ - motors: Dict[int, Motor]                                       │   │
│  │ - can_bus: CanBus                                                │   │
│  │ - kinematics: Kinematics                                         │   │
│  │ - state: ArmState                                                │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ + connect(): bool                                               │   │
│  │ + disconnect(): void                                            │   │
│  │ + initialize(mode): bool                                        │   │
│  │ + enable_all(enabled: bool): bool                                │   │
│  │ + get_joint_positions(): List[float]                            │   │
│  │ + set_joint_positions(positions): bool                          │   │
│  │ + get_ee_pose(): Pose                                            │   │
│  │ + move_to_pose(pose): bool                                       │   │
│  │ + emergency_stop(): void                                         │   │
│  │ + calibrate_zero(): bool                                        │   │
│  │ + get_state(): Dict                                             │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                       Kinematics                                  │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ - dh_parameters: DHParameters                                     │   │
│  │ - n_joints: int                                                  │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ + forward_kinematics(joints): Pose                               │   │
│  │ + inverse_kinematics(pose, seed): Optional[List[float]]          │   │
│  │ + compute_jacobian(joints): np.ndarray                            │   │
│  │ + check_singularity(joints): Tuple[bool, int]                    │   │
│  │ + set_dh_parameters(params): void                                │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
│  ┌─────────────────────────────────────────────────────────────────┐   │
│  │                     GroupController                               │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ - left_arm: Optional[ArmController]                               │   │
│  │ - right_arm: Optional[ArmController]                             │   │
│  │ - target: ControlTarget                                          │   │
│  ├─────────────────────────────────────────────────────────────────┤   │
│  │ + connect_left(channel): bool                                    │   │
│  │ + connect_right(channel): bool                                   │   │
│  │ + disconnect_all(): void                                         │   │
│  │ + move_both(poses): bool                                          │   │
│  │ + play_trajectory(traj): bool                                     │   │
│  │ + emergency_stop(): void                                          │   │
│  └─────────────────────────────────────────────────────────────────┘   │
│                                                                         │
└─────────────────────────────────────────────────────────────────────────┘
```

### 3.2 数据类设计

```python
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any
from enum import Enum
import time


@dataclass
class MotorStatus:
    """电机状态"""
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    voltage: float = 0.0
    temperature: float = 0.0
    enabled: bool = False
    mode: int = 0
    error_code: int = 0


@dataclass
class JointLimits:
    """关节限制"""
    position_min: List[float] = field(default_factory=lambda: [-3.14] * 7)
    position_max: List[float] = field(default_factory=lambda: [3.14] * 7)
    velocity_max: List[float] = field(default_factory=lambda: [2.0] * 7)
    acceleration_max: List[float] = field(default_factory=lambda: [5.0] * 7)


@dataclass
class MotionParameters:
    """运动参数"""
    velocity: float = 1.0
    acceleration: float = 1.0
    deceleration: float = 1.0
    jerk: float = 1.0  # 加加速度


@dataclass
class Pose:
    """末端位姿"""
    position: List[float]  # [x, y, z] (米)
    orientation: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])  # [roll, pitch, yaw] (弧度)

    def __post_init__(self):
        if len(self.position) != 3:
            raise ValueError("position必须是3维向量 [x, y, z]")
        if len(self.orientation) != 3:
            raise ValueError("orientation必须是3维向量 [roll, pitch, yaw]")


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    joint_positions: List[float]
    time_from_start: float = 0.0
    velocity: Optional[List[float]] = None
    acceleration: Optional[List[float]] = None


@dataclass
class Trajectory:
    """轨迹"""
    name: str
    points: List[TrajectoryPoint]
    loop: bool = False
    description: str = ""

    def __post_init__(self):
        if not self.points:
            raise ValueError("轨迹必须包含至少一个点")


@dataclass
class IKResult:
    """逆运动学求解结果"""
    success: bool
    joint_positions: Optional[List[float]]
    iterations: int = 0
    error: float = 0.0
    singularities: List[int] = field(default_factory=list)
```

---

## 4. 接口定义

### 4.1 CanBus 接口

```python
from abc import ABC, abstractmethod
from typing import Callable, Optional, Protocol, runtime_checkable
import can


@runtime_checkable
class CanBusBackend(Protocol):
    """CAN总线后端协议"""

    def connect(self, channel: str, bitrate: int) -> bool:
        """连接CAN设备"""
        ...

    def disconnect(self) -> None:
        """断开CAN设备"""
        ...

    def send(self, msg_id: int, data: bytes, timeout: float = 0.1) -> bool:
        """发送CAN消息"""
        ...

    def recv(self, timeout: float = 0.1) -> Optional[can.Message]:
        """接收CAN消息"""
        ...

    def register_callback(
        self,
        msg_id: int,
        callback: Callable[[can.Message], None]
    ) -> None:
        """注册消息回调"""
        ...


class CanBus:
    """CAN总线封装类"""

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 1000000,
        timeout: float = 0.1,
        backend: Optional[CanBusBackend] = None
    ):
        """
        Args:
            channel: CAN通道名称
            bitrate: 波特率
            timeout: 默认超时时间
            backend: CAN后端实现 (可选，默认使用SocketCAN)
        """
        self.channel = channel
        self.bitrate = bitrate
        self.timeout = timeout
        self._backend = backend
        self._bus: Optional[can.interface.Bus] = None
        self._running = False

    def connect(self) -> bool:
        """连接CAN总线"""
        if self._backend:
            return self._backend.connect(self.channel, self.bitrate)

        # 使用默认 SocketCAN 后端
        try:
            self._bus = can.interface.Bus(
                channel=self.channel,
                interface="socketcan",
                bitrate=self.bitrate
            )
            return True
        except CANError:
            return False

    def disconnect(self) -> None:
        """断开CAN总线"""
        if self._backend:
            self._backend.disconnect()
        elif self._bus:
            self._bus.shutdown()
            self._bus = None

        self._running = False

    def send(
        self,
        msg_id: int,
        data: bytes,
        timeout: Optional[float] = None
    ) -> bool:
        """
        发送CAN消息

        Args:
            msg_id: CAN消息ID (电机ID)
            data: 数据载荷 (1-8字节)
            timeout: 超时时间

        Returns:
            bool: 是否发送成功
        """
        if self._backend:
            return self._backend.send(msg_id, data, timeout or self.timeout)

        if not self._bus:
            raise CANNotConnectedError("CAN总线未连接")

        try:
            message = can.Message(
                arbitration_id=msg_id,
                data=data[:8],
                is_extended_id=False
            )
            self._bus.send(message, timeout=timeout or self.timeout)
            return True
        except CANError:
            return False

    def recv(self, timeout: float = 0.1) -> Optional[can.Message]:
        """
        接收CAN消息

        Args:
            timeout: 超时时间

        Returns:
            can.Message: 接收的消息，超时返回None
        """
        if self._backend:
            return self._backend.recv(timeout)

        if not self._bus:
            raise CANNotConnectedError("CAN总线未连接")

        try:
            return self._bus.recv(timeout)
        except CANError:
            return None

    def register_callback(
        self,
        msg_id: int,
        callback: Callable[[can.Message], None]
    ) -> None:
        """注册消息回调"""
        if self._backend:
            self._backend.register_callback(msg_id, callback)
```

### 4.2 Motor 接口

```python
from enum import Enum
from dataclasses import dataclass
from typing import Protocol, runtime_checkable


class MotorMode(Enum):
    TORQUE = 0
    MIT = 1
    VELOCITY = 2
    PROFILE_VELOCITY = 3
    POSITION = 4
    PROFILE_POSITION = 5


@dataclass
class MotorStatus:
    position: float = 0.0
    velocity: float = 0.0
    current: float = 0.0
    voltage: float = 0.0
    temperature: float = 0.0
    enabled: bool = False
    mode: MotorMode = MotorMode.POSITION
    error_code: int = 0


@runtime_checkable
class MotorProtocol(Protocol):
    """电机控制协议"""

    @property
    def motor_id(self) -> int:
        """电机ID"""
        ...

    @property
    def status(self) -> MotorStatus:
        """电机状态"""
        ...

    def enable(self, enabled: bool) -> bool:
        """使能/禁用电机"""
        ...

    def set_position(self, position: float) -> bool:
        """设置目标位置"""
        ...

    def set_velocity(self, velocity: float) -> bool:
        """设置目标速度"""
        ...

    def set_mode(self, mode: MotorMode) -> bool:
        """设置控制模式"""
        ...

    def get_status(self) -> MotorStatus:
        """获取电机状态"""
        ...

    def clear_alarm(self) -> bool:
        """清除报警"""
        ...


class Motor:
    """电机控制类"""

    # 命令码定义
    CMD_READ_POSITION = 0x06
    CMD_READ_VELOCITY = 0x05
    CMD_READ_CURRENT = 0x04
    CMD_SET_POSITION = 0x0A
    CMD_SET_VELOCITY = 0x09
    CMD_SET_CURRENT = 0x08
    CMD_SET_MODE = 0x07
    CMD_ENABLE = 0x2A
    CMD_CLEAR_ALARM = 0xFE

    def __init__(
        self,
        motor_id: int,
        can_bus: CanBus,
        data_converter: Optional["DataConverter"] = None
    ):
        """
        Args:
            motor_id: 电机ID (51-67)
            can_bus: CAN总线实例
            data_converter: 数据转换器
        """
        if not (51 <= motor_id <= 67 and motor_id not in [58, 59, 60]):
            raise ValueError(f"无效的电机ID: {motor_id}")

        self.motor_id = motor_id
        self.can_bus = can_bus
        self.converter = data_converter or DataConverter()
        self._status = MotorStatus()

    @property
    def status(self) -> MotorStatus:
        """获取电机状态"""
        return self._status

    def enable(self, enabled: bool = True) -> bool:
        """使能/禁用电机"""
        data = bytes([1 if enabled else 0])
        return self.can_bus.send(
            self.motor_id,
            self.CMD_ENABLE,
            data
        )

    def set_position(self, position: float) -> bool:
        """设置目标位置 (弧度)"""
        data = self.converter.float_to_bytes(position)
        return self.can_bus.send(
            self.motor_id,
            self.CMD_SET_POSITION,
            data
        )

    def set_velocity(self, velocity: float) -> bool:
        """设置目标速度 (rad/s)"""
        data = self.converter.float_to_bytes(velocity)
        return self.can_bus.send(
            self.motor_id,
            self.CMD_SET_VELOCITY,
            data
        )

    def set_mode(self, mode: MotorMode) -> bool:
        """设置控制模式"""
        return self.can_bus.send(
            self.motor_id,
            self.CMD_SET_MODE,
            bytes([mode.value])
        )

    def get_status(self) -> MotorStatus:
        """获取电机状态"""
        # 发送读取命令
        self.can_bus.send(
            self.motor_id,
            self.CMD_READ_POSITION,
            bytes([0])
        )

        # 等待响应
        import time
        time.sleep(0.02)

        return self._status

    def clear_alarm(self) -> bool:
        """清除报警"""
        return self.can_bus.send(
            self.motor_id,
            self.CMD_CLEAR_ALARM,
            bytes([])
        )
```

### 4.3 ArmController 接口

```python
from typing import List, Optional, Dict, Any, Protocol, runtime_checkable


class ArmControlProtocol(Protocol):
    """机械臂控制协议"""

    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        ...

    @property
    def is_enabled(self) -> bool:
        """是否已使能"""
        ...

    @property
    def joint_positions(self) -> List[float]:
        """当前关节位置"""
        ...

    def connect(self) -> bool:
        """连接机械臂"""
        ...

    def disconnect(self) -> None:
        """断开机械臂"""
        ...

    def initialize(self, mode: MotorMode = MotorMode.PROFILE_POSITION) -> bool:
        """初始化机械臂"""
        ...

    def enable(self, enabled: bool) -> bool:
        """使能/禁用"""
        ...

    def get_joint_positions(self) -> List[float]:
        """获取关节位置"""
        ...

    def set_joint_positions(
        self,
        positions: List[float],
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """设置关节位置"""
        ...

    def get_ee_pose(self) -> Pose:
        """获取末端位姿"""
        ...

    def move_to_pose(
        self,
        pose: Pose,
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """移动到目标位姿"""
        ...

    def emergency_stop(self) -> None:
        """紧急停止"""
        ...


class ArmController:
    """7-DOF 机械臂控制器"""

    def __init__(
        self,
        motor_ids: List[int],
        can_channel: str = "can0",
        joint_limits: Optional[JointLimits] = None,
        kinematics: Optional[Kinematics] = None
    ):
        """
        Args:
            motor_ids: 电机ID列表 [51, 52, 53, 54, 55, 56, 57]
            can_channel: CAN通道名称
            joint_limits: 关节限制
            kinematics: 运动学求解器
        """
        self.motor_ids = motor_ids
        self.can_channel = can_channel
        self.joint_limits = joint_limits or JointLimits()

        # 初始化组件
        self.can_bus = CanBus(channel=can_channel)
        self.kinematics = kinematics or Kinematics()

        # 初始化电机
        self.motors: Dict[int, Motor] = {
            mid: Motor(mid, self.can_bus) for mid in motor_ids
        }

        # 状态
        self._state = ArmState.DISCONNECTED

    @property
    def is_connected(self) -> bool:
        """是否已连接"""
        return self._state not in [ArmState.DISCONNECTED, ArmState.ERROR]

    @property
    def is_enabled(self) -> bool:
        """是否已使能"""
        return self._state == ArmState.ENABLED

    @property
    def joint_positions(self) -> List[float]:
        """当前关节位置"""
        return self.get_joint_positions()

    def connect(self) -> bool:
        """连接机械臂"""
        if not self.can_bus.connect():
            self._state = ArmState.ERROR
            return False

        self._state = ArmState.CONNECTED
        return True

    def disconnect(self) -> None:
        """断开机械臂"""
        self.enable(False)
        self.can_bus.disconnect()
        self._state = ArmState.DISCONNECTED

    def initialize(self, mode: MotorMode = MotorMode.PROFILE_POSITION) -> bool:
        """初始化机械臂"""
        try:
            # 清除报警
            for motor in self.motors.values():
                motor.clear_alarm()

            import time
            time.sleep(0.3)

            # 设置控制模式
            for motor in self.motors.values():
                motor.set_mode(mode)

            time.sleep(0.2)

            self._state = ArmState.INITIALIZED
            return True

        except Exception:
            self._state = ArmState.ERROR
            return False

    def enable(self, enabled: bool = True) -> bool:
        """使能/禁用所有电机"""
        success = True
        for motor in self.motors.values():
            if not motor.enable(enabled):
                success = False

        self._state = ArmState.ENABLED if enabled else ArmState.INITIALIZED
        return success

    def get_joint_positions(self) -> List[float]:
        """获取所有关节位置"""
        import time
        positions = []

        for mid in self.motor_ids:
            self.can_bus.send(mid, Motor.CMD_READ_POSITION, bytes([0]))
            time.sleep(0.01)
            positions.append(self.motors[mid].status.position)

        return positions

    def set_joint_positions(
        self,
        positions: List[float],
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """设置所有关节目标位置"""
        if len(positions) != len(self.motor_ids):
            raise ValueError(f"需要{len(self.motor_ids)}个关节位置")

        import time

        for mid, pos in zip(self.motor_ids, positions):
            if not self.motors[mid].set_position(pos):
                return False

        if blocking:
            time.sleep(0.1)

        return True

    def get_ee_pose(self) -> Pose:
        """获取末端位姿"""
        positions = self.get_joint_positions()
        return self.kinematics.forward_kinematics(positions)

    def move_to_pose(
        self,
        pose: Pose,
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """移动到目标位姿"""
        # 逆运动学求解
        joint_positions = self.kinematics.inverse_kinematics(pose)

        if joint_positions is None:
            raise IKError("逆运动学求解失败")

        return self.set_joint_positions(joint_positions, speed, blocking)

    def emergency_stop(self) -> None:
        """紧急停止"""
        self.enable(False)

    def get_state(self) -> Dict[str, Any]:
        """获取机械臂状态"""
        return {
            "state": self._state.value,
            "joint_positions": self.get_joint_positions(),
            "ee_pose": self.get_ee_pose().__dict__,
            "connected_motors": len(self.motors)
        }
```

---

## 5. 数据类型

### 5.1 基本类型映射

| Python 类型 | C++ 类型 | 说明 |
|------------|---------|-----|
| `float` | `double` | 双精度浮点 |
| `int` | `int32_t` | 32位整数 |
| `bool` | `bool` | 布尔值 |
| `str` | `std::string` | 字符串 |
| `List[float]` | `std::vector<double>` | 浮点数组 |
| `List[int]` | `std::vector<int32_t>` | 整数数组 |
| `Optional[T]` | `std::optional<T>` | 可空类型 |
| `Dict[str, T]` | `std::unordered_map<std::string, T>` | 字典 |

### 5.2 枚举类型

```python
class MotorMode(Enum):
    TORQUE = 0              # 力矩模式
    MIT = 1                 # MIT模式
    VELOCITY = 2            # 速度模式
    PROFILE_VELOCITY = 3    # 轨迹速度模式
    POSITION = 4            # 位置模式
    PROFILE_POSITION = 5    # 轨迹位置模式


class ArmState(Enum):
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    INITIALIZED = "initialized"
    ENABLED = "enabled"
    ERROR = "error"


class ControlTarget(Enum):
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


class InterpolationType(Enum):
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"
    B_SPLINE = "b_spline"
```

### 5.3 单位约定

| 参数 | 单位 | 说明 |
|-----|-----|-----|
| 位置 | rad (弧度) | 关节角度 |
| 位置 | m (米) | 笛卡尔空间位置 |
| 速度 | rad/s | 关节速度 |
| 速度 | m/s | 笛卡尔速度 |
| 加速度 | rad/s² | 关节加速度 |
| 加速度 | m/s² | 笛卡尔加速度 |
| 力矩 | N·m | 关节力矩 |
| 温度 | °C | 摄氏度 |
| 电压 | V | 伏特 |
| 电流 | A | 安培 |

---

## 6. 错误处理

### 6.1 异常层次

```python
class LansiArmError(Exception):
    """SDK基础异常"""
    def __init__(self, message: str, code: Optional[int] = None):
        super().__init__(message)
        self.message = message
        self.code = code
        self.timestamp = time.time()


class CommunicationError(LansiArmError):
    """通信异常"""
    pass


class CANError(CommunicationError):
    """CAN通信异常"""
    pass


class CANNotConnectedError(CANError):
    """CAN未连接"""
    pass


class ControlError(LansiArmError):
    """控制异常"""
    pass


class JointLimitError(ControlError):
    """关节限制异常"""
    pass


class MotorError(ControlError):
    """电机异常"""
    pass


class KinematicsError(LansiArmError):
    """运动学异常"""
    pass


class IKError(KinematicsError):
    """逆运动学异常"""
    pass
```

### 6.2 错误码定义

```python
class ErrorCode:
    """错误码定义"""
    SUCCESS = 0

    # 通信错误 (1xx)
    ERR_CAN_NOT_CONNECTED = 100
    ERR_CAN_SEND_FAILED = 101
    ERR_CAN_RECV_TIMEOUT = 102
    ERR_CAN_INVALID_ID = 103

    # 控制错误 (2xx)
    ERR_NOT_INITIALIZED = 200
    ERR_NOT_ENABLED = 201
    ERR_JOINT_LIMIT = 202
    ERR_MOTOR_ALARM = 203
    ERR_EMERGENCY_STOP = 204

    # 运动学错误 (3xx)
    ERR_IK_NO_SOLUTION = 300
    ERR_IK_MAX_ITERATIONS = 301
    ERR_SINGULARITY = 302
    ERR_INVALID_POSE = 303

    # 参数错误 (4xx)
    ERR_INVALID_PARAMETER = 400
    ERR_INVALID_JOINT_INDEX = 401
    ERR_INVALID_POSITION = 402
```

### 6.3 异常处理示例

```python
from contextlib import contextmanager


@contextmanager
def error_handler(operation: str):
    """通用错误处理上下文"""
    try:
        yield
    except JointLimitError as e:
        print(f"关节限制错误: {e.message}")
        raise
    except MotorAlarmError as e:
        print(f"电机报警: motor={e.details['motor_id']}, code={e.details['error_code']}")
        raise
    except CANError as e:
        print(f"CAN通信错误: {e.message}")
        raise
    except LansiArmError as e:
        print(f"SDK错误: {e.message}")
        raise


# 使用示例
with error_handler("移动机械臂"):
    arm.set_joint_positions([0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0])
```

---

## 7. 使用示例

### 7.1 基础使用

```python
from lansi_arm import ArmController

# 创建控制器
arm = ArmController(
    motor_ids=[51, 52, 53, 54, 55, 56, 57],
    can_channel="can0"
)

try:
    # 连接
    arm.connect()
    print("连接成功")

    # 初始化
    arm.initialize()
    print("初始化成功")

    # 使能
    arm.enable(True)
    print("使能成功")

    # 获取当前位置
    positions = arm.get_joint_positions()
    print(f"当前关节位置: {positions}")

    # 移动到新位置
    arm.set_joint_positions([0.0, -0.5, 0.0, 1.0, 0.0, 0.5, 0.0])
    print("移动完成")

    # 获取末端位姿
    pose = arm.get_ee_pose()
    print(f"末端位置: {pose.position}")
    print(f"末端姿态: {pose.orientation}")

finally:
    # 断开连接
    arm.disconnect()
```

### 7.2 轨迹播放

```python
from lansi_arm import Trajectory, TrajectoryPlanner

# 创建轨迹
trajectory = Trajectory(
    name="pick_and_place",
    points=[
        TrajectoryPoint(joint_positions=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
        TrajectoryPoint(joint_positions=[0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]),
        TrajectoryPoint(joint_positions=[0.2, -0.3, 0.1, 1.0, -0.5, 0.3, 0.0]),
    ],
    loop=False
)

# 创建规划器
planner = TrajectoryPlanner(arm_controller=arm)

# 播放轨迹
planner.play(trajectory, speed=1.0, blocking=True)
```

### 7.3 双手协同

```python
from lansi_arm import GroupController

# 创建群控器
group = GroupController()

# 连接双臂
group.connect_left(can_channel="can0")
group.connect_right(can_channel="can1")

# 初始化双臂
group.initialize_all()

# 双手同时移动
from lansi_arm import Pose
left_pose = Pose(position=[0.3, 0.2, 0.4], orientation=[0, 0, 0])
right_pose = Pose(position=[0.3, -0.2, 0.4], orientation=[0, 0, 0])

group.move_both(left_pose, right_pose, sync=True)

# 断开连接
group.disconnect_all()
```

---

## 附录

### A. 版本历史

| 版本 | 日期 | 修改内容 |
|-----|-----|---------|
| 1.0.0 | 2025-02-11 | 初始版本 |

### B. 参考资料

1. [Python PEP 8 风格指南](https://pep8.org/)
2. [Google Python 风格指南](https://google.github.io/styleguide/pyguide.html)
3. [ROS 设计指南](https://docs.ros.org/en/humble/Concepts/About-Moving-Forward/Significant-Design.html)

---

**文档版本**: 1.0.0  
**最后更新**: 2025-02-11
