# Lansi Arm SDK 架构设计文档

**版本**: 1.0.0
**日期**: 2025-02-11
**作者**: Lansi Robotics

---

## 目录

1. [概述](#1-概述)
2. [系统架构](#2-系统架构)
3. [分层设计](#3-分层设计)
4. [核心模块详解](#4-核心模块详解)
5. [数据流设计](#5-数据流设计)
6. [异常处理](#6-异常处理)
7. [线程安全](#7-线程安全)
8. [扩展性设计](#8-扩展性设计)

---

## 1. 概述

### 1.1 文档目的

本文档描述 Lansi Arm SDK 的整体架构设计，为开发人员提供系统架构的详细说明，确保 SDK 的高质量、可维护性和可扩展性。

### 1.2 设计原则

| 原则 | 说明 |
|-----|-----|
| **分层架构** | 将系统分为通信层、控制器层、运动学层和应用层 |
| **高内聚低耦合** | 每个模块职责单一，模块间通过明确定义的接口通信 |
| **面向对象** | 使用类封装数据和行为，提供清晰的 API |
| **类型安全** | Python 使用类型提示，C++ 使用强类型 |
| **可测试性** | 每个模块可独立测试，通过依赖注入实现 |
| **可扩展性** | 预留扩展点，支持新功能添加 |

### 1.3 技术选型

| 层级 | 技术选型 | 理由 |
|-----|---------|-----|
| 通信协议 | CAN Bus (SocketCAN), Modbus/TCP | 工业标准、实时性好 |
| Python 版本 | 3.9+ | 类型提示完善、生态丰富 |
| C++ 标准 | C++17 | 现代特性、编译稳定 |
| 构建工具 | CMake | 跨平台、成熟可靠 |
| 包管理 | pip (Python), vcpkg (C++) | 社区主流 |
| 测试框架 | pytest (Python), googletest (C++) | 功能强大 |
| 文档工具 | MkDocs + Material | 美观易用 |

---

## 2. 系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          Lansi Arm SDK                                  │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      应用层 (Application Layer)                  │    │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────────────┐  │    │
│  │  │ 示教编程     │  │ 轨迹复现     │  │ 视觉引导控制            │  │    │
│  │  │ Teaching     │  │ Trajectory  │  │ Vision Guided          │  │    │
│  │  └─────────────┘  └─────────────┘  └─────────────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                    │
│                                    │                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      控制层 (Control Layer)                       │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    │
│  │  │ 单臂控制器       │  │ 群控控制器       │  │ 轨迹规划器       │  │    │
│  │  │ ArmController   │  │ GroupController │  │ TrajectoryPlanner│  │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                    │
│                                    │                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      运动学层 (Kinematics Layer)                 │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    │
│  │  │ 正向运动学       │  │ 逆向运动学       │  │ 碰撞检测         │  │    │
│  │  │ ForwardKinematics│ │ InverseKinematics│ │ CollisionDetector│  │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                    │
│                                    │                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      通信层 (Communication Layer)                │    │
│  │  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐  │    │
│  │  │ CAN 通信模块     │  │ Modbus 客户端    │  │ 协议解析器       │  │    │
│  │  │ CanBus          │  │ ModbusClient     │  │ ProtocolParser   │  │    │
│  │  └─────────────────┘  └─────────────────┘  └─────────────────┘  │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                    │
│                                    │                                    │
│  ┌─────────────────────────────────────────────────────────────────┐    │
│  │                      硬件抽象层 (Hardware Abstraction)            │    │
│  │                                                                      │    │
│  │     USB-CAN 适配器          电机驱动器            传感器            │    │
│  │   (Peak CAN/Kvaser)       (51-67 ID)           (编码器等)          │    │
│  │                                                                      │    │
│  └─────────────────────────────────────────────────────────────────┘    │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

### 2.2 模块依赖关系

```
┌──────────────────────────────────────────────────────────────────────────┐
│                           模块依赖图                                      │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│   user_app                                                               │
│      │                                                                   │
│      ▼                                                                   │
│ ┌─────────────────────────────────────────────────────────────────────┐  │
│ │                         GroupController                              │  │
│ │    (双臂协同控制、轨迹播放、状态管理)                                   │  │
│ └─────────────────────────────────────────────────────────────────────┘  │
│      │                                                                   │
│      ├──────────────────┬────────────────────┬─────────────────────────┐ │
│      │                  │                    │                         │ │
│      ▼                  ▼                    ▼                         ▼ │
│ ┌─────────────┐  ┌─────────────────┐  ┌─────────────────┐  ┌──────────┐│ │
│ │ArmController│  │ TrajectoryPlanner│  │ CollisionDetector│ │ Vision  ││ │
│ │ (单臂控制)   │  │  (轨迹规划)       │  │  (碰撞检测)       │  │(视觉集成)││ │
│ └─────────────┘  └─────────────────┘  └─────────────────┘  └──────────┘│ │
│      │                  │                    │                         │ │
│      ├──────────┬───────┴─────┬────────────┴──────────┬──────────────┘ │
│      │          │             │                          │                 │
│      ▼          ▼             ▼                          ▼                 │
│ ┌─────────────────────────────────────────────────────────────────────┐  │
│ │                      Kinematics (运动学核心)                          │  │
│ │   正解(FK) / 逆解(IK) / 雅可比矩阵 / 奇异性分析 / 轨迹插值              │  │
│ └─────────────────────────────────────────────────────────────────────┘  │
│      │                                                                    │
│      │          ┌─────────────────────────────────────────────┐          │
│      │          │              CanBus (CAN通信)                │          │
│      │          │   Motor / Protocol / DataConverter / Parser │          │
│      │          └─────────────────────────────────────────────┘          │
│      │                                                                    │
│      ▼                                                                    │
│ ┌─────────────────────────────────────────────────────────────────────┐  │
│ │                         Hardware (硬件抽象)                            │  │
│ │              USB-CAN适配器 / 电机驱动器 / 编码器 / 传感器               │  │
│ └─────────────────────────────────────────────────────────────────────┘  │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 3. 分层设计

### 3.1 分层原则

SDK 采用 **四层架构** 设计，每层职责明确，层间通过接口通信：

| 层级 | 名称 | 职责 | 示例 |
|-----|-----|-----|-----|
| L4 | 应用层 | 高级应用功能 | 示教编程、轨迹复现、视觉引导 |
| L3 | 控制层 | 运动控制和协调 | 单臂/群控、轨迹规划、状态管理 |
| L2 | 运动学层 | 数学计算和算法 | 正逆解、碰撞检测、轨迹插值 |
| L1 | 通信层 | 硬件通信和协议 | CAN通信、Modbus、协议解析 |
| L0 | 硬件抽象层 | 硬件访问 | USB-CAN适配器、电机驱动器 |

### 3.2 层间接口规范

#### L4 → L3 接口

```python
# 高层应用接口示例
class ArmApplication:
    """应用层接口"""

    def teach_and_replay(self, trajectory_name: str) -> bool:
        """示教并复现轨迹"""
        # 获取当前姿态
        current_pose = self.controller.get_ee_pose()
        # 记录轨迹点
        self.trajectory_recorder.record_point(current_pose, name="start")
        # 移动到目标
        self.controller.move_to_pose(target_pose)
        # 记录结束点
        self.trajectory_recorder.record_point(target_pose, name="end")
        # 复现轨迹
        return self.trajectory_player.play(trajectory_name)
```

#### L3 → L2 接口

```python
# 控制层 → 运动学层接口
class KinematicsInterface:
    """运动学层接口"""

    def forward_kinematics(self, joint_positions: List[float]) -> Pose:
        """
        正向运动学：关节角度 → 末端位姿

        Args:
            joint_positions: 7个关节角度 (弧度)

        Returns:
            Pose: 末端位姿 (位置 + 姿态)
        """
        # 参数校验
        self._validate_joint_positions(joint_positions)

        # 计算正运动学
        return self.fk_solver.compute(joint_positions)

    def inverse_kinematics(
        self,
        target_pose: Pose,
        seed: Optional[List[float]] = None,
        constraints: Optional[IKConstraints] = None
    ) -> List[float]:
        """
        逆向运动学：末端位姿 → 关节角度

        Args:
            target_pose: 目标末端位姿
            seed: 初始猜测关节角度
            constraints: 约束条件（关节限制、奇异性避免等）

        Returns:
            List[float]: 7个关节角度
        """
        # 校验位姿
        self._validate_pose(target_pose)

        # 求解逆运动学
        return self.ik_solver.solve(target_pose, seed, constraints)
```

#### L2 → L1 接口

```python
# 运动学层 → 通信层接口
class CanBusInterface:
    """CAN通信层接口"""

    def send_command(
        self,
        motor_id: int,
        command_code: int,
        data: bytes,
        timeout: float = 0.1
    ) -> bool:
        """
        发送CAN命令

        Args:
            motor_id: 电机ID (51-67)
            command_code: 命令码 (0x00-0xFF)
            data: 数据载荷 (最多7字节)
            timeout: 超时时间(秒)

        Returns:
            bool: 是否发送成功
        """
        # 构建CAN消息
        can_message = CanMessage(
            arbitration_id=motor_id,
            data=self.protocol.encode(command_code, data)
        )

        # 发送消息
        return self.bus.send(can_message, timeout)

    def read_response(
        self,
        motor_id: int,
        expected_command: int,
        timeout: float = 0.1
    ) -> Optional[bytes]:
        """
        读取CAN响应

        Args:
            motor_id: 电机ID
            expected_command: 期望的命令码
            timeout: 超时时间

        Returns:
            bytes: 响应数据，失败返回None
        """
        # 接收消息
        response = self.bus.recv(motor_id, timeout)

        if response is None:
            return None

        # 解析响应
        return self.protocol.decode(response)
```

#### L1 → L0 接口

```python
# 通信层 → 硬件抽象层接口
class HardwareAbstraction:
    """硬件抽象层接口"""

    def open_device(self, device_path: str, bitrate: int) -> bool:
        """
        打开CAN设备

        Args:
            device_path: 设备路径 (如 /dev/can0)
            bitrate: 波特率 (如 1000000)

        Returns:
            bool: 是否打开成功
        """
        raise NotImplementedError

    def close_device(self) -> None:
        """关闭CAN设备"""
        raise NotImplementedError

    def send_frame(self, frame: CanFrame) -> int:
        """
        发送CAN帧

        Args:
            frame: CAN帧

        Returns:
            int: 发送的字节数
        """
        raise NotImplementedError

    def receive_frame(self, timeout: float) -> Optional[CanFrame]:
        """
        接收CAN帧

        Args:
            timeout: 超时时间

        Returns:
            CanFrame: 接收的帧，超时返回None
        """
        raise NotImplementedError
```

---

## 4. 核心模块详解

### 4.1 通信层 (Communication Layer)

#### 4.1.1 CAN通信模块

```python
class CanBus:
    """
    CAN总线通信模块

    职责：
    - CAN设备初始化和配置
    - CAN消息发送和接收
    - 消息过滤和缓存
    - 错误检测和恢复
    """

    def __init__(
        self,
        channel: str = "can0",
        bitrate: int = 1000000,
        timeout: float = 0.1
    ):
        """
        Args:
            channel: CAN通道名称 (Linux: can0, can1)
            bitrate: CAN总线波特率 (默认1Mbps)
            timeout: 默认超时时间(秒)
        """
        self.channel = channel
        self.bitrate = bitrate
        self.timeout = timeout
        self._bus: Optional[can.interface.Bus] = None
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._callbacks: Dict[int, List[Callable]] = defaultdict(list)

    def connect(self) -> bool:
        """连接CAN总线"""
        try:
            # 创建CAN总线接口 (使用python-can)
            self._bus = can.interface.Bus(
                channel=self.channel,
                interface="socketcan",
                bitrate=self.bitrate
            )

            # 启动接收线程
            self._running = True
            self._recv_thread = threading.Thread(
                target=self._receive_loop,
                daemon=True
            )
            self._recv_thread.start()

            return True

        except CANError as e:
            self._handle_error(f"CAN连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开CAN总线"""
        self._running = False

        if self._recv_thread and self._recv_thread.is_alive():
            self._recv_thread.join(timeout=1.0)

        if self._bus:
            self._bus.shutdown()
            self._bus = None

    def send(
        self,
        message_id: int,
        data: bytes,
        timeout: Optional[float] = None
    ) -> bool:
        """
        发送CAN消息

        Args:
            message_id: CAN消息ID (电机ID)
            data: 数据载荷 (1-8字节)
            timeout: 超时时间

        Returns:
            bool: 是否发送成功
        """
        if not self._bus:
            raise CANNotConnectedError("CAN总线未连接")

        try:
            # 构建CAN消息
            can_message = can.Message(
                arbitration_id=message_id,
                data=data[:8],  # 最多8字节
                is_extended_id=False
            )

            # 发送消息
            self._bus.send(can_message, timeout=timeout or self.timeout)
            return True

        except CANError as e:
            self._handle_error(f"CAN发送失败: {e}")
            return False

    def _receive_loop(self) -> None:
        """接收消息循环"""
        while self._running:
            try:
                # 接收消息 (阻塞式，带超时)
                message = self._bus.recv(timeout=0.1)

                if message:
                    # 触发回调
                    self._dispatch_message(message)

            except CANError as e:
                self._handle_error(f"CAN接收错误: {e}")

    def _dispatch_message(self, message: can.Message) -> None:
        """分发消息到回调函数"""
        message_id = message.arbitration_id

        for callback in self._callbacks.get(message_id, []):
            try:
                callback(message)
            except Exception as e:
                self._handle_error(f"回调执行错误: {e}")

    def register_callback(
        self,
        message_id: int,
        callback: Callable[[can.Message], None]
    ) -> None:
        """注册消息回调"""
        self._callbacks[message_id].append(callback)

    def unregister_callback(
        self,
        message_id: int,
        callback: Callable[[can.Message], None]
    ) -> None:
        """取消注册消息回调"""
        if callback in self._callbacks[message_id]:
            self._callbacks[message_id].remove(callback)
```

#### 4.1.2 电机控制模块

```python
from enum import Enum
from dataclasses import dataclass
from typing import Optional
from .data_converter import DataConverter


class MotorMode(Enum):
    """电机控制模式"""
    TORQUE = 0          # 力矩模式
    MIT = 1             # MIT模式
    VELOCITY = 2        # 速度模式
    PROFILE_VELOCITY = 3  # 轨迹速度模式
    POSITION = 4        # 位置模式
    PROFILE_POSITION = 5  # 轨迹位置模式


@dataclass
class MotorStatus:
    """电机状态"""
    position: float = 0.0          # 当前位置 (rad)
    velocity: float = 0.0           # 当前速度 (rad/s)
    current: float = 0.0           # 当前电流 (A)
    voltage: float = 0.0           # 母线电压 (V)
    temperature: float = 0.0       # 电机温度 (°C)
    enabled: bool = False          # 使能状态
    mode: MotorMode = MotorMode.POSITION  # 控制模式
    error_code: int = 0           # 错误码


class Motor:
    """
    单个电机控制类

    封装电机的所有操作：
    - 位置/速度/力矩控制
    - 参数读写
    - 状态监控
    - 报警处理
    """

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
        data_converter: Optional[DataConverter] = None
    ):
        """
        Args:
            motor_id: 电机ID (51-57 或 61-67)
            can_bus: CAN总线实例
            data_converter: 数据转换器实例
        """
        if not (51 <= motor_id <= 67 and motor_id != 58 and motor_id != 59 and motor_id != 60):
            raise ValueError(f"无效的电机ID: {motor_id}")

        self.motor_id = motor_id
        self.can_bus = can_bus
        self.converter = data_converter or DataConverter()
        self._status = MotorStatus()
        self._lock = threading.RLock()

    def enable(self, enabled: bool = True) -> bool:
        """
        使能/禁用电机

        Args:
            enabled: True=使能, False=禁用

        Returns:
            bool: 操作是否成功
        """
        data = [1 if enabled else 0]
        return self.can_bus.send(
            self.motor_id,
            self.CMD_ENABLE,
            bytes(data)
        )

    def set_position(self, position: float, blocking: bool = True) -> bool:
        """
        设置目标位置

        Args:
            position: 目标位置 (弧度)
            blocking: 是否等待设置完成

        Returns:
            bool: 操作是否成功
        """
        # 转换浮点数到字节
        data = self.converter.float_to_bytes(position)

        # 发送命令
        success = self.can_bus.send(
            self.motor_id,
            self.CMD_SET_POSITION,
            data
        )

        if success and blocking:
            # 等待位置更新
            time.sleep(0.05)

        return success

    def set_velocity(self, velocity: float) -> bool:
        """设置目标速度 (rad/s)"""
        data = self.converter.float_to_bytes(velocity)
        return self.can_bus.send(
            self.motor_id,
            self.CMD_SET_VELOCITY,
            data
        )

    def set_mode(self, mode: MotorMode) -> bool:
        """
        设置控制模式

        Args:
            mode: 电机控制模式

        Returns:
            bool: 操作是否成功
        """
        return self.can_bus.send(
            self.motor_id,
            self.CMD_SET_MODE,
            bytes([mode.value])
        )

    def get_status(self) -> MotorStatus:
        """
        获取电机状态

        Returns:
            MotorStatus: 当前电机状态
        """
        with self._lock:
            # 发送读取位置命令
            self.can_bus.send(
                self.motor_id,
                self.CMD_READ_POSITION,
                bytes([0])
            )

            # 等待响应
            time.sleep(0.02)

            return copy.deepcopy(self._status)

    def clear_alarm(self) -> bool:
        """清除报警"""
        return self.can_bus.send(
            self.motor_id,
            self.CMD_CLEAR_ALARM,
            bytes([])
        )

    def calibrate_zero(self) -> bool:
        """标定零点"""
        # 发送标定命令
        return self.can_bus.send(
            self.motor_id,
            0x87,  # SET_HOME
            bytes([])
        )
```

### 4.2 控制层 (Control Layer)

#### 4.2.1 单臂控制器

```python
from typing import List, Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import time


class ArmState(Enum):
    """机械臂状态"""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    INITIALIZED = "initialized"
    ENABLED = "enabled"
    ERROR = "error"


@dataclass
class JointLimits:
    """关节限制"""
    position_min: List[float] = field(default_factory=list)
    position_max: List[float] = field(default_factory=list)
    velocity_max: List[float] = field(default_factory=list)
    acceleration_max: List[float] = field(default_factory=list)


class ArmController:
    """
    7-DOF 机械臂控制器

    职责：
    - 电机协调控制
    - 关节空间运动
    - 速度/加速度配置
    - 状态监控
    - 零点标定
    """

    def __init__(
        self,
        motor_ids: List[int],
        can_channel: str = "can0",
        joint_limits: Optional[JointLimits] = None
    ):
        """
        Args:
            motor_ids: 电机ID列表 [51, 52, 53, 54, 55, 56, 57]
            can_channel: CAN通道名称
            joint_limits: 关节限制
        """
        self.motor_ids = motor_ids
        self.can_channel = can_channel
        self.joint_limits = joint_limits or self._get_default_limits()

        # 初始化通信
        self.can_bus = CanBus(channel=can_channel)

        # 初始化电机
        self.motors: Dict[int, Motor] = {}
        for mid in motor_ids:
            self.motors[mid] = Motor(mid, self.can_bus)

        # 运动学实例
        self.kinematics = Kinematics()

        # 状态
        self._state = ArmState.DISCONNECTED
        self._lock = threading.RLock()

    def _get_default_limits(self) -> JointLimits:
        """获取默认关节限制"""
        return JointLimits(
            position_min=[-3.14] * 7,
            position_max=[3.14] * 7,
            velocity_max=[2.0] * 7,
            acceleration_max=[5.0] * 7
        )

    def connect(self) -> bool:
        """连接机械臂"""
        with self._lock:
            # 连接CAN总线
            if not self.can_bus.connect():
                self._state = ArmState.ERROR
                return False

            # 更新电机状态
            for motor in self.motors.values():
                motor.status.enabled = False

            self._state = ArmState.CONNECTED
            return True

    def disconnect(self) -> None:
        """断开机械臂"""
        with self._lock:
            # 禁用电机
            self.enable_all(False)

            # 断开CAN总线
            self.can_bus.disconnect()

            self._state = ArmState.DISCONNECTED

    def initialize(self, mode: MotorMode = MotorMode.PROFILE_POSITION) -> bool:
        """
        初始化机械臂

        Args:
            mode: 控制模式

        Returns:
            bool: 初始化是否成功
        """
        with self._lock:
            try:
                # 清除报警
                self._clear_all_alarms()
                time.sleep(0.3)

                # 设置控制模式
                for motor in self.motors.values():
                    motor.set_mode(mode)
                time.sleep(0.2)

                # 配置速度参数
                self._configure_motion_parameters()

                self._state = ArmState.INITIALIZED
                return True

            except Exception as e:
                self._state = ArmState.ERROR
                raise ArmInitializationError(f"初始化失败: {e}")

    def enable_all(self, enabled: bool = True) -> bool:
        """
        使能/禁用所有电机

        Args:
            enabled: True=使能, False=禁用

        Returns:
            bool: 操作是否成功
        """
        success = True
        for motor in self.motors.values():
            if not motor.enable(enabled):
                success = False

        with self._lock:
            if success:
                self._state = ArmState.ENABLED if enabled else ArmState.INITIALIZED

        return success

    def get_joint_positions(self) -> List[float]:
        """
        获取所有关节位置

        Returns:
            List[float]: 7个关节位置 (弧度)
        """
        positions = []
        for mid in self.motor_ids:
            # 发送读取命令
            self.can_bus.send(mid, Motor.CMD_READ_POSITION, bytes([0]))
            time.sleep(0.01)

            # 获取缓存的位置
            positions.append(self.motors[mid].status.position)

        return positions

    def set_joint_positions(
        self,
        positions: List[float],
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """
        设置所有关节目标位置

        Args:
            positions: 7个目标位置 (弧度)
            speed: 速度系数 (0.0-1.0)
            blocking: 是否等待移动完成

        Returns:
            bool: 操作是否成功
        """
        # 校验参数
        if len(positions) != 7:
            raise ValueError("必须提供7个关节位置")

        # 校验关节范围
        self._check_joint_limits(positions)

        # 发送位置命令
        for mid, pos in zip(self.motor_ids, positions):
            if not self.motors[mid].set_position(pos, blocking=False):
                return False

        if blocking:
            time.sleep(0.1)

        return True

    def set_joint_position(
        self,
        joint_index: int,
        position: float,
        speed: float = 0.5
    ) -> bool:
        """
        设置单个关节位置

        Args:
            joint_index: 关节索引 (0-6)
            position: 目标位置 (弧度)
            speed: 速度系数

        Returns:
            bool: 操作是否成功
        """
        if not 0 <= joint_index < 7:
            raise ValueError(f"无效的关节索引: {joint_index}")

        motor_id = self.motor_ids[joint_index]
        return self.motors[motor_id].set_position(position)

    def get_state(self) -> Dict[str, Any]:
        """
        获取机械臂状态

        Returns:
            Dict: 包含位置、速度、状态等信息的字典
        """
        with self._lock:
            return {
                "state": self._state.value,
                "joint_positions": self.get_joint_positions(),
                "joint_velocities": self._get_joint_velocities(),
                "pose": self.get_ee_pose().__dict__,
                "connected_motors": len(self.motors)
            }

    def get_ee_pose(self) -> Pose:
        """
        获取末端执行器位姿

        Returns:
            Pose: 末端位姿
        """
        positions = self.get_joint_positions()
        return self.kinematics.forward_kinematics(positions)

    def move_to_pose(
        self,
        pose: Pose,
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """
        移动到指定末端位姿

        Args:
            pose: 目标末端位姿
            speed: 速度系数
            blocking: 是否等待移动完成

        Returns:
            bool: 操作是否成功
        """
        # 逆运动学求解
        joint_positions = self.kinematics.inverse_kinematics(pose)

        if joint_positions is None:
            raise IKError("逆运动学求解失败")

        # 移动到目标位置
        return self.set_joint_positions(joint_positions, speed, blocking)

    def go_to_zero(self) -> bool:
        """回到零位"""
        return self.set_joint_positions([0.0] * 7)

    def emergency_stop(self) -> None:
        """紧急停止"""
        self.enable_all(False)

    def calibrate_zero(self) -> bool:
        """标定零位"""
        success = True
        for motor in self.motors.values():
            if not motor.calibrate_zero():
                success = False
        return success

    def _clear_all_alarms(self) -> None:
        """清除所有报警"""
        for motor in self.motors.values():
            motor.clear_alarm()
            time.sleep(0.02)

    def _configure_motion_parameters(
        self,
        velocity: float = 1.0,
        acceleration: float = 1.0,
        deceleration: float = 1.0
    ) -> None:
        """配置运动参数"""
        for motor in self.motors.values():
            # 设置轨迹速度
            motor.can_bus.send(
                motor.motor_id,
                0x1F,  # SET_PT_V
                DataConverter().float_to_bytes(velocity)
            )

            # 设置轨迹加速度
            motor.can_bus.send(
                motor.motor_id,
                0x20,  # SET_PT_A
                DataConverter().float_to_bytes(acceleration)
            )

            # 设置轨迹减速度
            motor.can_bus.send(
                motor.motor_id,
                0x21,  # SET_PT_D
                DataConverter().float_to_bytes(deceleration)
            )

            time.sleep(0.02)

    def _check_joint_limits(self, positions: List[float]) -> None:
        """检查关节限制"""
        for i, pos in enumerate(positions):
            if pos < self.joint_limits.position_min[i]:
                raise JointLimitError(
                    f"关节{i}位置 {pos} 低于下限 {self.joint_limits.position_min[i]}"
                )
            if pos > self.joint_limits.position_max[i]:
                raise JointLimitError(
                    f"关节{i}位置 {pos} 超过上限 {self.joint_limits.position_max[i]}"
                )

    def _get_joint_velocities(self) -> List[float]:
        """获取所有关节速度"""
        velocities = []
        for mid in self.motor_ids:
            self.can_bus.send(mid, Motor.CMD_READ_VELOCITY, bytes([0]))
            time.sleep(0.01)
            velocities.append(self.motors[mid].status.velocity)
        return velocities
```

### 4.3 运动学层 (Kinematics Layer)

#### 4.3.1 运动学核心类

```python
import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class DHConvention(Enum):
    """DH参数约定"""
    STANDARD = "standard"    # 标准DH
    MODIFIED = "modified"     # 改进DH


@dataclass
class DHParameters:
    """DH参数"""
    a: List[float]      # 连杆长度
    alpha: List[float]  # 连杆扭角
    d: List[float]      # 连杆偏移
    theta: List[float]  # 关节角度


@dataclass
class Pose:
    """末端位姿"""
    position: List[float]      # [x, y, z] (米)
    orientation: List[float]  # [roll, pitch, yaw] (弧度)

    def to_matrix(self) -> np.ndarray:
        """转换为4x4变换矩阵"""
        return create_transform_matrix(
            self.position,
            euler_to_quaternion(self.orientation)
        )

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Pose":
        """从4x4变换矩阵创建"""
        position = matrix[:3, 3].tolist()
        orientation = quaternion_to_euler(matrix_to_quaternion(matrix))
        return cls(position=position, orientation=orientation)


class Kinematics:
    """
    运动学求解器

    支持：
    - 标准DH参数模型
    - 正向运动学
    - 逆向运动学 (迭代法)
    - 雅可比矩阵计算
    - 奇异性检测
    """

    def __init__(
        self,
        dh_parameters: Optional[DHParameters] = None,
        convention: DHConvention = DHConvention.STANDARD
    ):
        """
        Args:
            dh_parameters: DH参数 (如为None则使用模板参数)
            convention: DH参数约定
        """
        self.convention = convention

        # 使用参数化DH模板 (需要根据实际机械臂更新)
        self.dh = dh_parameters or self._create_default_dh()

        # 关节数量
        self.n_joints = len(self.dh.a)

        # 缓存变换矩阵
        self._cache_enabled = True
        self._transform_cache: List[np.ndarray] = []

    def _create_default_dh(self) -> DHParameters:
        """
        创建默认DH参数模板

        注意：此参数需要根据实际机械臂测量后更新

        7轴机械臂DH参数模板：
        - 基座到关节1
        - 关节1到关节2
        - ...
        - 关节6到关节7 (末端)
        """
        # TODO: 根据实际机械臂更新DH参数
        return DHParameters(
            a=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # 连杆长度 (米)
            alpha=[0.0] * 7,                         # 连杆扭角 (弧度)
            d=[0.0] * 7,                             # 连杆偏移 (米)
            theta=[0.0] * 7                          # 初始角度 (弧度)
        )

    def set_dh_parameters(self, dh: DHParameters) -> None:
        """设置DH参数"""
        self.dh = dh
        self.n_joints = len(dh.a)
        self._clear_cache()

    def forward_kinematics(
        self,
        joint_positions: List[float],
        use_cache: bool = True
    ) -> Pose:
        """
        正向运动学：关节角度 → 末端位姿

        Args:
            joint_positions: 7个关节角度 (弧度)
            use_cache: 是否使用缓存

        Returns:
            Pose: 末端位姿
        """
        # 计算每个关节的变换矩阵
        T = np.eye(4)

        for i in range(self.n_joints):
            # 计算单关节DH变换
            T_i = self._dh_transform(
                self.dh.a[i],
                self.dh.alpha[i],
                self.dh.d[i],
                joint_positions[i] + self.dh.theta[i]
            )

            # 累积变换
            T = T @ T_i

        return Pose.from_matrix(T)

    def inverse_kinematics(
        self,
        target_pose: Pose,
        seed: Optional[List[float]] = None,
        max_iterations: int = 1000,
        tolerance: float = 1e-4,
        constraints: Optional["IKConstraints"] = None
    ) -> Optional[List[float]]:
        """
        逆向运动学：末端位姿 → 关节角度

        使用循环坐标下降法 (CCD) 或 雅可比转置法

        Args:
            target_pose: 目标末端位姿
            seed: 初始猜测 (默认使用当前关节角度)
            max_iterations: 最大迭代次数
            tolerance: 收敛容差 (米/弧度)
            constraints: 关节约束

        Returns:
            List[float]: 关节角度，失败返回None
        """
        # 初始猜测
        if seed is None:
            seed = [0.0] * self.n_joints

        q = np.array(seed, dtype=np.float64)

        # 目标变换矩阵
        T_target = target_pose.to_matrix()

        for iteration in range(max_iterations):
            # 计算当前末端位姿
            T_current = self._compute_fk_matrix(q)
            current_pose = Pose.from_matrix(T_current)

            # 计算位置误差
            position_error = np.array(target_pose.position) - current_pose.position

            # 计算姿态误差 (简化处理)
            orientation_error = np.array(target_pose.orientation) - current_pose.orientation

            # 总误差
            total_error = np.linalg.norm(position_error)

            # 检查收敛
            if total_error < tolerance:
                return q.tolist()

            # 计算雅可比矩阵
            J = self.compute_jacobian(q)

            # 阻尼最小二乘法 (Levenberg-Marquardt)
            damping = 0.01
            try:
                # J @ dq = error
                # (J^T J + λI) dq = J^T error
                J_T = J.T
                dq = np.linalg.solve(
                    J_T @ J + damping * np.eye(self.n_joints),
                    J_T @ np.hstack([position_error, orientation_error[:3]])
                )

                # 更新关节角度
                q = q + dq

            except np.linalg.LinAlgError:
                return None

            # 应用约束
            if constraints:
                q = self._apply_constraints(q, constraints)

        # 未收敛
        return None

    def compute_jacobian(
        self,
        joint_positions: List[float],
        reference_frame: str = "end_effector"
    ) -> np.ndarray:
        """
        计算雅可比矩阵

        J = [Jv; Jw] - 3xN 线速度雅可比 + 3xN 角速度雅可比

        Args:
            joint_positions: 关节角度
            reference_frame: 参考坐标系 ("base" 或 "end_effector")

        Returns:
            np.ndarray: 6xN 雅可比矩阵
        """
        n = self.n_joints
        J = np.zeros((6, n))

        # 计算末端位置和姿态
        T_total = np.eye(4)
        joint_positions = np.array(joint_positions)

        # 末端位置（在基坐标系下）
        T_ee = self.forward_kinematics(joint_positions.tolist())
        p_ee = np.array(T_ee.position)

        # 计算每个关节的雅可比列
        for i in range(n):
            # 计算当前关节位置（在基坐标系下）
            T_i = np.eye(4)
            for j in range(i + 1):
                T_j = self._dh_transform(
                    self.dh.a[j],
                    self.dh.alpha[j],
                    self.dh.d[j],
                    joint_positions[j] + self.dh.theta[j]
                )
                T_i = T_i @ T_j

            p_i = T_i[:3, 3]

            # 关节轴线方向（旋转轴）
            if i == 0:
                z_i = np.array([0, 0, 1])  # 基座Z轴
            else:
                z_i = T_i[:3, 2]

            # 线速度雅可比列: z_i × (p_ee - p_i)
            J[:3, i] = np.cross(z_i, p_ee - p_i)

            # 角速度雅可比列: z_i
            J[3:, i] = z_i

        return J

    def check_singularity(
        self,
        joint_positions: List[float],
        threshold: float = 1e-3
    ) -> Tuple[bool, int]:
        """
        检测奇异性

        Args:
            joint_positions: 关节角度
            threshold: 奇异阈值

        Returns:
            Tuple[bool, int]: (是否奇异, 奇异关节索引)
        """
        J = self.compute_jacobian(joint_positions)

        # 计算雅可比矩阵的最小奇异值
        try:
            singular_values = np.linalg.svd(J, compute_uv=False)
            min_sv = singular_values[-1]  # 最小奇异值

            if min_sv < threshold:
                # 找到导致奇异性的关节
                J_reduced = J[:3, :]  # 只考虑线速度
                sv_reduced = np.linalg.svd(J_reduced, compute_uv=False)

                for i, sv in enumerate(sv_reduced):
                    if sv < threshold:
                        return True, i

            return False, -1

        except np.linalg.LinAlgError:
            return True, 0

    def _dh_transform(
        self,
        a: float,
        alpha: float,
        d: float,
        theta: float
    ) -> np.ndarray:
        """
        计算DH变换矩阵

        标准DH变换：
        T = Tz(d) * Tz(theta) * Tx(a) * Rx(alpha)

        Args:
            a: 连杆长度
            alpha: 连杆扭角
            d: 连杆偏移
            theta: 关节角度

        Returns:
            np.ndarray: 4x4变换矩阵
        """
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        T = np.array([
            [ct, -st * ca,  st * ca, a * ct],
            [st,  ct * ca, -ct * ca, a * st],
            [0,      sa,      ca,     d],
            [0,       0,       0,      1]
        ])

        return T

    def _compute_fk_matrix(self, joint_positions: np.ndarray) -> np.ndarray:
        """计算正向运动学变换矩阵"""
        T = np.eye(4)
        for i in range(self.n_joints):
            T_i = self._dh_transform(
                self.dh.a[i],
                self.dh.alpha[i],
                self.dh.d[i],
                joint_positions[i] + self.dh.theta[i]
            )
            T = T @ T_i
        return T

    def _apply_constraints(
        self,
        q: np.ndarray,
        constraints: "IKConstraints"
    ) -> np.ndarray:
        """应用关节约束"""
        # 位置约束
        if constraints.position_limits is not None:
            q = np.clip(
                q,
                constraints.position_limits.min,
                constraints.position_limits.max
            )

        # 速度约束
        if constraints.velocity_limits is not None:
            q = np.clip(
                q,
                -constraints.velocity_limits.max,
                constraints.velocity_limits.max
            )

        return q

    def _clear_cache(self) -> None:
        """清除缓存"""
        self._transform_cache = []
```

---

## 5. 数据流设计

### 5.1 命令发送数据流

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          命令发送数据流                                    │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  用户代码                                                                 │
│     │                                                                   │
│     ▼                                                                   │
│  ArmController.set_joint_positions([...])                                │
│     │                                                                   │
│     ├───────────────────────────────────────────────────────────────┐    │
│     │                                                               │    │
│     ▼                                                               ▼    │
│  参数校验                                                    约束检查     │
│  (关节范围)                                               (位置/速度)    │
│     │                                                               │    │
│     └───────────────────────────────────────────────────────────────┘    │
│                                    │                                    │
│                                    ▼                                    │
│  运动学转换 (如需要)                                                   │
│  关节空间 → 笛卡尔空间 → 关节空间                                         │
│                                    │                                    │
│                                    ▼                                    │
│  Motor.set_position()                                                   │
│     │                                                                   │
│     ▼                                                                   │
│  数据编码                                                               │
│  float → bytes (IEEE 754 小端序)                                         │
│     │                                                                   │
│     ▼                                                                   │
│  CanBus.send(motor_id, cmd, data)                                       │
│     │                                                                   │
│     ▼                                                                   │
│  SocketCAN 驱动                                                         │
│     │                                                                   │
│     ▼                                                                   │
│  USB-CAN 适配器硬件                                                     │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 5.2 状态读取数据流

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          状态读取数据流                                    │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  用户代码                                                                 │
│     │                                                                   │
│     ▼                                                                   │
│  ArmController.get_joint_positions()                                     │
│     │                                                                   │
│     ▼                                                                   │
│  接收线程 (后台运行)                                                      │
│     │                                                                   │
│     ▼                                                                   │
│  CanBus._receive_loop()                                                  │
│     │                                                                   │
│     ▼                                                                   │
│  SocketCAN 接收中断                                                      │
│     │                                                                   │
│     ▼                                                                   │
│  USB-CAN 适配器 → CAN帧                                                  │
│     │                                                                   │
│     ▼                                                                   │
│  协议解析                                                               │
│  bytes → float (状态数据)                                                │
│     │                                                                   │
│     ▼                                                                   │
│  Motor.status 更新                                                       │
│     │                                                                   │
│     ▼                                                                   │
│  返回缓存的状态数据                                                      │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 5.3 轨迹播放数据流

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          轨迹播放数据流                                    │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  轨迹文件 (JSON)                                                          │
│     │                                                                   │
│     ▼                                                                   │
│  Trajectory.load()                                                       │
│     │                                                                   │
│     ▼                                                                   │
│  轨迹插值                                                                │
│  (关节空间插值 / 笛卡尔空间插值)                                           │
│     │                                                                   │
│     ▼                                                                   │
│  速度/加速度规划                                                         │
│     │                                                                   │
│     ▼                                                                   │
│  逐点发送                                                               │
│     │                                                                   │
│     ├───────────────────────────────────────────────────────────────┐    │
│     │                                                               │    │
│     ▼                                                               ▼    │
│  单臂控制                                                         双臂同步  │
│     │                                                               │    │
│     │                                                               ▼    │
│     │                                                         线程同步    │
│     │                                                               │    │
│     └───────────────────────────────────────────────────────────────┘    │
│                                    │                                    │
│                                    ▼                                    │
│  电机执行                                                                │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

---

## 6. 异常处理

### 6.1 异常类层次结构

```python
class LansiArmError(Exception):
    """SDK基础异常类"""
    def __init__(self, message: str, details: Optional[Dict] = None):
        super().__init__(message)
        self.message = message
        self.details = details or {}
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


class CANSendError(CANError):
    """CAN发送失败"""
    pass


class CANReceiveError(CANError):
    """CAN接收失败"""
    pass


class ModbusError(CommunicationError):
    """Modbus通信异常"""
    pass


class ControlError(LansiArmError):
    """控制异常"""
    pass


class ArmInitializationError(ControlError):
    """机械臂初始化异常"""
    pass


class JointLimitError(ControlError):
    """关节限制异常"""
    pass


class MotorError(ControlError):
    """电机异常"""
    pass


class MotorNotEnabledError(MotorError):
    """电机未使能"""
    pass


class MotorAlarmError(MotorError):
    """电机报警"""
    pass


class KinematicsError(LansiArmError):
    """运动学异常"""
    pass


class IKError(KinematicsError):
    """逆运动学求解异常"""
    pass


class FKError(KinematicsError):
    """正运动学计算异常"""
    pass


class SingularityError(KinematicsError):
    """奇异性异常"""
    pass


class ConfigurationError(LansiArmError):
    """配置异常"""
    pass


class InvalidParameterError(ConfigurationError):
    """无效参数异常"""
    pass
```

### 6.2 异常处理策略

```python
def example_with_error_handling():
    """异常处理示例"""
    arm = ArmController(motor_ids=[51, 52, 53, 54, 55, 56, 57])

    try:
        # 连接
        arm.connect()

        # 初始化
        arm.initialize()

        # 移动
        arm.set_joint_positions([0.1] * 7)

    except JointLimitError as e:
        # 关节超出限制
        print(f"关节限制错误: {e.message}")
        print(f"有效范围: {e.details['valid_range']}")

    except MotorAlarmError as e:
        # 电机报警
        print(f"电机报警: {e.details['motor_id']}")
        print(f"错误码: {e.details['error_code']}")
        arm.clear_all_alarms()

    except CANError as e:
        # CAN通信错误
        print(f"CAN错误: {e.message}")
        # 尝试重连
        arm.disconnect()
        time.sleep(1)
        arm.connect()

    except LansiArmError as e:
        # 其他SDK错误
        print(f"SDK错误: {e.message}")
        raise

    finally:
        # 确保断开连接
        arm.disconnect()
```

---

## 7. 线程安全

### 7.1 线程模型

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          线程模型                                         │
├──────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │                         主线程                                     │    │
│  │   - 用户API调用                                                    │    │
│  │   - 状态查询                                                       │    │
│  │   - 阻塞操作                                                       │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│                                    │                                    │
│                                    │ 共享数据                           │
│                                    ▼                                    │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │                       共享数据区                                    │    │
│  │   - Motor状态                                                      │    │
│  │   - Arm状态                                                       │    │
│  │   - 轨迹缓存                                                      │    │
│  │                                                                  │    │
│  │   使用 threading.RLock 保护                                        │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│                                    ▲                                    │
│                                    │                                    │
│  ┌──────────────────────────────────────────────────────────────────┐    │
│  │                       接收线程                                     │    │
│  │   - 持续监听CAN消息                                               │    │
│  │   - 解析并更新状态                                                │    │
│  │   - 触发回调函数                                                  │    │
│  └──────────────────────────────────────────────────────────────────┘    │
│                                                                          │
└──────────────────────────────────────────────────────────────────────────┘
```

### 7.2 线程安全设计原则

| 场景 | 解决方案 |
|-----|---------|
| **多读单写** | 使用 `RLock` 或 `Lock` |
| **状态更新** | 只在接收线程中更新 |
| **状态读取** | 主线程读取（Python GIL保证原子性） |
| **批量操作** | 使用 `with self._lock:` 包围 |
| **回调调用** | 在锁外调用，避免死锁 |

### 7.3 线程安全示例

```python
class ThreadSafeMotor(Motor):
    """线程安全的电机类"""

    def __init__(self, motor_id: int, can_bus: CanBus):
        super().__init__(motor_id, can_bus)
        self._lock = threading.RLock()
        self._status = MotorStatus()
        self._status_changed = threading.Event()

    def get_status(self) -> MotorStatus:
        """线程安全的获取状态"""
        with self._lock:
            return copy.deepcopy(self._status)

    def update_status(self, new_status: MotorStatus) -> None:
        """线程安全的更新状态 (仅接收线程调用)"""
        with self._lock:
            self._status = new_status
            self._status_changed.set()
```

---

## 8. 扩展性设计

### 8.1 扩展点

SDK 设计了以下扩展点：

| 扩展点 | 接口 | 用途 |
|-------|-----|-----|
| 通信后端 | `CanBusBackend` | 支持不同的CAN硬件 |
| 运动学求解器 | `KinematicsSolver` | 不同的IK算法 |
| 轨迹插值器 | `TrajectoryInterpolator` | 不同的插值方法 |
| 碰撞检测器 | `CollisionDetector` | 不同的碰撞检测方法 |
| 视觉前端 | `VisionInterface` | 不同的视觉系统 |

### 8.2 扩展示例

```python
# 扩展：使用不同的CAN硬件
class PeakCanBackend(CanBusBackend):
    """Peak CAN 适配器后端"""

    def __init__(self, device_type: str = "PCAN_USB"):
        self.device_type = device_type

    def connect(self, channel: str, bitrate: int) -> bool:
        # 使用Peak CAN SDK
        from pcan import PCAN
        self.handle = PCAN(self.device_type)
        return self.handle.initialize(bitrate)

    def send(self, msg_id: int, data: bytes) -> bool:
        # 发送CAN消息
        return self.handle.write(msg_id, data)


# 使用自定义后端
arm = ArmController(motor_ids=[51, 52, 53, 54, 55, 56, 57])
arm.can_bus.set_backend(PeakCanBackend())
```

---

## 附录

### A. 术语表

| 术语 | 定义 |
|-----|-----|
| DH参数 | Denavit-Hartenberg参数，用于描述机械臂运动学 |
| FK | Forward Kinematics，正向运动学 |
| IK | Inverse Kinematics，逆向运动学 |
| 雅可比矩阵 | 描述关节速度与末端速度关系的矩阵 |
| 奇异位形 | 雅可比矩阵秩亏的位形 |
| 轨迹插值 | 在轨迹点之间生成平滑过渡 |
| CAN | Controller Area Network，现场总线协议 |
| Modbus | 工业通信协议 |

### B. 参考资料

1. 《机器人学：建模、规划与控制》 - Siciliano
2. ROS MoveIt! 文档
3. python-can 官方文档
4. 蓝思电机通信协议 (蓝思电机文档.pdf)

---

**文档版本**: 1.0.0
**最后更新**: 2025-02-11
