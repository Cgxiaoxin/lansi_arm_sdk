"""
单臂控制器

提供7-DOF机械臂的统一控制接口。
"""


import time
from dataclasses import dataclass, field
from typing import List, Optional, Dict, Any, Callable
from enum import Enum

from ..core.can_bus import CanBus, CANConfig
from ..core.motor import Motor, MotorData, MotorLimits
from ..core.data_converter import DataConverter
from ..constants import MotorMode, ArmState, CommandCode
from ..exceptions import (
    ArmInitializationError,
    JointLimitError,
    CANError,
)


class TrajectoryInterpolation(Enum):
    """轨迹插值类型"""
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"


@dataclass
class JointLimits:
    """关节限制"""
    position_min: List[float] = field(default_factory=lambda: [-3.14] * 7)
    position_max: List[float] = field(default_factory=lambda: [3.14] * 7)
    velocity_max: List[float] = field(default_factory=lambda: [2.0] * 7)
    acceleration_max: List[float] = field(default_factory=lambda: [5.0] * 7)

    def __post_init__(self):
        if len(self.position_min) != 7:
            self.position_min = [-3.14] * 7
        if len(self.position_max) != 7:
            self.position_max = [3.14] * 7
        if len(self.velocity_max) != 7:
            self.velocity_max = [2.0] * 7
        if len(self.acceleration_max) != 7:
            self.acceleration_max = [5.0] * 7


@dataclass
class MotionParameters:
    """运动参数"""
    velocity: float = 1.0
    acceleration: float = 1.0
    deceleration: float = 1.0
    interpolation: TrajectoryInterpolation = TrajectoryInterpolation.LINEAR


class ArmController:
    """
    7-DOF 机械臂控制器

    提供机械臂的完整控制功能：
    - 连接/断开
    - 初始化/使能
    - 关节位置/速度控制
    - 状态监控
    - 零点标定
    """

    # 默认电机ID (左臂)
    DEFAULT_MOTOR_IDS = [51, 52, 53, 54, 55, 56, 57]

    def __init__(
        self,
        motor_ids: Optional[List[int]] = None,
        can_channel: str = "can0",
        joint_limits: Optional[JointLimits] = None,
        can_config: Optional[CANConfig] = None
    ):
        """
        Args:
            motor_ids: 电机ID列表 (默认: [51, 52, 53, 54, 55, 56, 57])
            can_channel: CAN通道名称
            joint_limits: 关节限制
            can_config: CAN配置
        """
        self.motor_ids = motor_ids or self.DEFAULT_MOTOR_IDS.copy()
        self.can_channel = can_channel
        self.joint_limits = joint_limits or JointLimits()

        # 初始化组件
        self.can_bus = CanBus(config=can_config)
        self.converter = DataConverter()

        # 初始化电机
        self._motors: Dict[int, Motor] = {}
        for mid in self.motor_ids:
            self._motors[mid] = Motor(mid, self.can_bus, self.converter)

        # 状态
        self._state = ArmState.DISCONNECTED
        self._current_mode = MotorMode.PROFILE_POSITION

        # 回调函数
        self._on_state_update: Optional[Callable] = None
        self._on_position_update: Optional[Callable] = None

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._state not in [ArmState.DISCONNECTED, ArmState.ERROR]

    @property
    def is_enabled(self) -> bool:
        """检查是否已使能"""
        return self._state == ArmState.ENABLED

    @property
    def is_initialized(self) -> bool:
        """检查是否已初始化"""
        return self._state in [ArmState.INITIALIZED, ArmState.ENABLED]

    @property
    def state(self) -> ArmState:
        """获取当前状态"""
        return self._state

    @property
    def motor_ids(self) -> List[int]:
        """获取电机ID列表"""
        return self._motor_ids

    @motor_ids.setter
    def motor_ids(self, value: List[int]):
        """设置电机ID列表"""
        self._motor_ids = value

    @property
    def motors(self) -> Dict[int, Motor]:
        """获取电机字典"""
        return self._motors

    def connect(self) -> bool:
        """
        连接机械臂

        Returns:
            bool: 是否连接成功
        """
        try:
            # 连接CAN总线
            self.can_bus.connect()

            # 更新状态
            self._state = ArmState.CONNECTED

            return True

        except CANError as e:
            self._state = ArmState.ERROR
            raise ArmInitializationError(f"连接失败: {e}")

    def disconnect(self) -> None:
        """断开机械臂"""
        # 禁用电机
        self.enable(False)

        # 断开CAN总线
        self.can_bus.disconnect()

        self._state = ArmState.DISCONNECTED

    def initialize(
        self,
        mode: MotorMode = MotorMode.PROFILE_POSITION,
        velocity: float = 1.0,
        acceleration: float = 1.0,
        deceleration: float = 1.0
    ) -> bool:
        """
        初始化机械臂

        Args:
            mode: 控制模式
            velocity: 轨迹速度系数
            acceleration: 轨迹加速度系数
            deceleration: 轨迹减速度系数

        Returns:
            bool: 初始化是否成功
        """
        if not self.is_connected:
            raise ArmInitializationError("机械臂未连接")

        try:
            # 清除报警
            self._clear_all_alarms()
            time.sleep(0.3)

            # 设置控制模式
            for motor in self._motors.values():
                motor.set_mode(mode)
            time.sleep(0.2)

            # 配置运动参数
            self._configure_motion_parameters(velocity, acceleration, deceleration)

            # 读取电机限制参数
            for motor in self._motors.values():
                try:
                    motor.read_limits()
                except Exception:
                    pass

            self._current_mode = mode
            self._state = ArmState.INITIALIZED

            return True

        except Exception as e:
            self._state = ArmState.ERROR
            raise ArmInitializationError(f"初始化失败: {e}")

    def enable(self, enabled: bool = True) -> bool:
        """
        使能/禁用所有电机

        Args:
            enabled: True=使能, False=禁用

        Returns:
            bool: 操作是否成功
        """
        success = True
        for motor in self._motors.values():
            if not motor.enable(enabled):
                success = False

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
            try:
                pos = self._motors[mid].read_position()
                positions.append(pos)
            except Exception:
                positions.append(self._motors[mid].position)

        return positions

    def get_joint_velocities(self) -> List[float]:
        """
        获取所有关节速度

        Returns:
            List[float]: 7个关节速度 (rad/s)
        """
        velocities = []

        for mid in self.motor_ids:
            try:
                vel = self._motors[mid].read_velocity()
                velocities.append(vel)
            except Exception:
                velocities.append(self._motors[mid].velocity)

        return velocities

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
            blocking: 是否阻塞等待移动完成

        Returns:
            bool: 操作是否成功

        Raises:
            ValueError: 参数数量错误
            JointLimitError: 位置超出关节限制
        """
        # 校验参数数量
        if len(positions) != len(self.motor_ids):
            raise ValueError(
                f"位置数量不匹配: 需要{len(self.motor_ids)}个，实际{len(positions)}个"
            )

        # 校验关节限制
        self._check_joint_limits(positions)

        # 发送位置命令
        for mid, pos in zip(self.motor_ids, positions):
            if not self._motors[mid].set_position(pos, blocking=False):
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
        if not 0 <= joint_index < len(self.motor_ids):
            raise ValueError(f"无效的关节索引: {joint_index}")

        # 校验关节限制
        if position < self.joint_limits.position_min[joint_index]:
            raise JointLimitError(
                f"关节{joint_index}位置 {position} 低于下限 "
                f"{self.joint_limits.position_min[joint_index]}"
            )
        if position > self.joint_limits.position_max[joint_index]:
            raise JointLimitError(
                f"关节{joint_index}位置 {position} 超过上限 "
                f"{self.joint_limits.position_max[joint_index]}"
            )

        motor_id = self.motor_ids[joint_index]
        return self._motors[motor_id].set_position(position)

    def set_joint_velocities(
        self,
        velocities: List[float],
        blocking: bool = True
    ) -> bool:
        """
        设置所有关节目标速度

        Args:
            velocities: 7个目标速度 (rad/s)
            blocking: 是否阻塞

        Returns:
            bool: 操作是否成功
        """
        if len(velocities) != len(self.motor_ids):
            raise ValueError(
                f"速度数量不匹配: 需要{len(self.motor_ids)}个，实际{len(velocities)}个"
            )

        for mid, vel in zip(self.motor_ids, velocities):
            if not self._motors[mid].set_velocity(vel):
                return False

        if blocking:
            time.sleep(0.05)

        return True

    def go_to_zero(self) -> bool:
        """
        回到零位

        Returns:
            bool: 操作是否成功
        """
        zero_positions = [0.0] * len(self.motor_ids)
        return self.set_joint_positions(zero_positions)

    def go_to_home(self) -> bool:
        """
        回到初始位置

        Returns:
            bool: 操作是否成功
        """
        return self.go_to_zero()

    def emergency_stop(self) -> None:
        """紧急停止"""
        self.enable(False)

    def stop(self) -> None:
        """停止移动"""
        for motor in self._motors.values():
            try:
                motor.set_velocity(0.0)
            except Exception:
                pass

    def calibrate_zero(self) -> bool:
        """
        标定零位

        Returns:
            bool: 操作是否成功
        """
        success = True
        for motor in self._motors.values():
            if not motor.calibrate_zero():
                success = False
            time.sleep(0.05)

        return success

    def clear_all_alarms(self) -> bool:
        """
        清除所有报警

        Returns:
            bool: 操作是否成功
        """
        return self._clear_all_alarms()

    def _clear_all_alarms(self) -> bool:
        """清除所有报警"""
        success = True
        for motor in self._motors.values():
            if not motor.clear_alarm():
                success = False
            time.sleep(0.02)

        return success

    def _configure_motion_parameters(
        self,
        velocity: float,
        acceleration: float,
        deceleration: float
    ) -> None:
        """配置运动参数"""
        for motor in self._motors.values():
            # 设置轨迹速度
            try:
                self.can_bus.send_command(
                    motor.motor_id,
                    CommandCode.SET_PT_V,
                    self.converter.float_to_bytes(velocity)
                )
            except CANError:
                pass

            # 设置轨迹加速度
            try:
                self.can_bus.send_command(
                    motor.motor_id,
                    CommandCode.SET_PT_A,
                    self.converter.float_to_bytes(acceleration)
                )
            except CANError:
                pass

            # 设置轨迹减速度
            try:
                self.can_bus.send_command(
                    motor.motor_id,
                    CommandCode.SET_PT_D,
                    self.converter.float_to_bytes(deceleration)
                )
            except CANError:
                pass

            time.sleep(0.02)

    def _check_joint_limits(self, positions: List[float]) -> None:
        """检查关节限制"""
        for i, pos in enumerate(positions):
            if pos < self.joint_limits.position_min[i]:
                raise JointLimitError(
                    f"关节{i}位置 {pos} 低于下限 "
                    f"{self.joint_limits.position_min[i]}"
                )
            if pos > self.joint_limits.position_max[i]:
                raise JointLimitError(
                    f"关节{i}位置 {pos} 超过上限 "
                    f"{self.joint_limits.position_max[i]}"
                )

    def set_motion_parameters(
        self,
        velocity: float,
        acceleration: float,
        deceleration: float
    ) -> None:
        """
        设置运动参数

        Args:
            velocity: 速度系数
            acceleration: 加速度系数
            deceleration: 减速度系数
        """
        self._configure_motion_parameters(velocity, acceleration, deceleration)

    def get_state(self) -> Dict[str, Any]:
        """
        获取机械臂状态

        Returns:
            dict: 状态信息
        """
        positions = self.get_joint_positions()
        velocities = self.get_joint_velocities()

        motors_status = {}
        for mid, motor in self._motors.items():
            motors_status[mid] = {
                "position": motor.position,
                "velocity": motor.velocity,
                "enabled": motor.is_enabled,
                "temperature": motor.data.temperature,
                "error_code": motor.error_code
            }

        return {
            "state": self._state.value,
            "mode": self._current_mode.value,
            "positions": positions,
            "velocities": velocities,
            "can_channel": self.can_channel,
            "motor_count": len(self._motors),
            "motors": motors_status,
            "joint_limits": {
                "position_min": self.joint_limits.position_min,
                "position_max": self.joint_limits.position_max
            }
        }

    def set_callbacks(
        self,
        on_state_update: Optional[Callable] = None,
        on_position_update: Optional[Callable] = None
    ) -> None:
        """
        设置回调函数

        Args:
            on_state_update: 状态更新回调
            on_position_update: 位置更新回调
        """
        self._on_state_update = on_state_update
        self._on_position_update = on_position_update

    def read_all_limits(self) -> Dict[int, MotorLimits]:
        """
        读取所有电机的限制参数

        Returns:
            dict: 电机ID -> 限制参数
        """
        limits_map = {}
        for motor in self._motors.values():
            try:
                limits_map[motor.motor_id] = motor.read_limits()
            except Exception:
                limits_map[motor.motor_id] = motor.limits

        return limits_map

    def read_all_parameters(self) -> Dict[int, Dict]:
        """
        读取所有电机参数

        Returns:
            dict: 电机ID -> 参数字典
        """
        params_map = {}
        for motor in self._motors.values():
            params = {
                "gear_ratio": motor.read_gear_ratio(),
                "torque_coeff": motor.read_torque_coeff()
            }
            params_map[motor.motor_id] = params

        return params_map

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False

    def __repr__(self) -> str:
        return (
            f"ArmController(motors={len(self._motors)}, "
            f"state={self._state.value}, "
            f"channel={self.can_channel})"
        )
