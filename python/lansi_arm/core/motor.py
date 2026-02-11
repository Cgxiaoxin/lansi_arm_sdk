"""
电机控制模块

提供单个电机的控制接口，包括：
- 使能/禁用
- 位置/速度/力矩控制
- 参数读写
- 状态监控
- 报警处理
"""


import time
from dataclasses import dataclass, field
from typing import Optional, Dict, Any
from enum import Enum

from .can_bus import CanBus
from .data_converter import DataConverter
from ..constants import CommandCode, MotorMode
from ..exceptions import (
    MotorError,
    MotorNotEnabledError,
    MotorAlarmError,
    CANError,
)


class MotorStatus(Enum):
    """电机状态"""
    DISABLED = 0
    ENABLED = 1
    ERROR = 2


@dataclass
class MotorData:
    """电机运行时数据"""
    position: float = 0.0          # 位置 (rad)
    velocity: float = 0.0          # 速度 (rad/s)
    current: float = 0.0           # 电流 (A)
    voltage: float = 0.0           # 母线电压 (V)
    temperature: float = 0.0       # 电机温度 (°C)
    inverter_temperature: float = 0.0  # 逆变器温度 (°C)
    core_temperature: float = 0.0   # 核心温度 (°C)


@dataclass
class MotorLimits:
    """电机限制参数"""
    position_min: float = -3.14    # 位置下限 (rad)
    position_max: float = 3.14     # 位置上限 (rad)
    velocity_max: float = 2.0      # 最大速度 (rad/s)
    current_max: float = 10.0      # 最大电流 (A)


@dataclass
class MotorParameters:
    """电机参数"""
    motor_id: int
    gear_ratio: float = 1.0        # 减速比
    torque_coeff: float = 1.0      # 转矩系数
    motor_resistance: float = 0.0  # 电机电阻
    motor_inductance: float = 0.0  # 电机电感


class Motor:
    """
    电机控制类

    封装单个电机的所有操作，支持多种控制模式。
    """

    def __init__(
        self,
        motor_id: int,
        can_bus: CanBus,
        data_converter: Optional[DataConverter] = None
    ):
        """
        Args:
            motor_id: 电机ID (51-67)
            can_bus: CAN总线实例
            data_converter: 数据转换器实例

        Raises:
            ValueError: 无效的电机ID
        """
        # 验证电机ID
        if not (51 <= motor_id <= 67):
            raise ValueError(f"无效的电机ID: {motor_id}，有效范围: 51-67")

        self.motor_id = motor_id
        self.can_bus = can_bus
        self.converter = data_converter or DataConverter()

        # 运行时数据
        self._data = MotorData()

        # 状态
        self._enabled = False
        self._mode = MotorMode.POSITION
        self._error_code = 0

        # 参数
        self._parameters = MotorParameters(motor_id=motor_id)
        self._limits = MotorLimits()

        # 线程锁
        self._lock = time

    @property
    def motor_id(self) -> int:
        """获取电机ID"""
        return self._parameters.motor_id

    @property
    def data(self) -> MotorData:
        """获取电机运行时数据"""
        return self._data

    @property
    def limits(self) -> MotorLimits:
        """获取电机限制参数"""
        return self._limits

    @property
    def parameters(self) -> MotorParameters:
        """获取电机参数"""
        return self._parameters

    @property
    def is_enabled(self) -> bool:
        """检查是否已使能"""
        return self._enabled

    @property
    def mode(self) -> MotorMode:
        """获取当前控制模式"""
        return self._mode

    @property
    def error_code(self) -> int:
        """获取错误码"""
        return self._error_code

    @property
    def position(self) -> float:
        """获取当前位置 (rad)"""
        return self._data.position

    @property
    def velocity(self) -> float:
        """获取当前速度 (rad/s)"""
        return self._data.velocity

    @property
    def current(self) -> float:
        """获取当前电流 (A)"""
        return self._data.current

    def enable(self, enabled: bool = True) -> bool:
        """
        使能/禁用电机

        Args:
            enabled: True=使能, False=禁用

        Returns:
            bool: 操作是否成功
        """
        try:
            # 发送使能命令
            result = self.can_bus.send_command(
                self.motor_id,
                CommandCode.S_ON_OFF,
                bytes([1 if enabled else 0])
            )

            if result:
                self._enabled = enabled

            return result

        except CANError as e:
            raise MotorError(f"电机使能失败: {e}")

    def set_mode(self, mode: MotorMode) -> bool:
        """
        设置控制模式

        Args:
            mode: 控制模式

        Returns:
            bool: 操作是否成功
        """
        try:
            result = self.can_bus.send_command(
                self.motor_id,
                CommandCode.SET_MODE,
                bytes([mode.value])
            )

            if result:
                self._mode = mode

            return result

        except CANError as e:
            raise MotorError(f"设置控制模式失败: {e}")

    def set_position(self, position: float, blocking: bool = True) -> bool:
        """
        设置目标位置

        Args:
            position: 目标位置 (弧度)
            blocking: 是否等待设置完成

        Returns:
            bool: 操作是否成功

        Raises:
            MotorNotEnabledError: 电机未使能
            ValueError: 位置超出限制
        """
        if not self._enabled:
            raise MotorNotEnabledError(f"电机 {self.motor_id} 未使能")

        # 检查位置限制
        if position < self._limits.position_min or position > self._limits.position_max:
            raise ValueError(
                f"位置 {position} 超出范围 [{self._limits.position_min}, {self._limits.position_max}]"
            )

        try:
            # 发送位置命令
            data = self.converter.position_to_bytes(position)
            result = self.can_bus.send_command(
                self.motor_id,
                CommandCode.SET_POSITION,
                data
            )

            if result and blocking:
                time.sleep(0.02)

            return result

        except CANError as e:
            raise MotorError(f"设置位置失败: {e}")

    def set_velocity(self, velocity: float) -> bool:
        """
        设置目标速度

        Args:
            velocity: 目标速度 (rad/s)

        Returns:
            bool: 操作是否成功
        """
        if not self._enabled:
            raise MotorNotEnabledError(f"电机 {self.motor_id} 未使能")

        # 检查速度限制
        if abs(velocity) > self._limits.velocity_max:
            raise ValueError(
                f"速度 {velocity} 超出限制 {self._limits.velocity_max}"
            )

        try:
            data = self.converter.velocity_to_bytes(velocity)
            return self.can_bus.send_command(
                self.motor_id,
                CommandCode.SET_VELOCITY,
                data
            )

        except CANError as e:
            raise MotorError(f"设置速度失败: {e}")

    def set_current(self, current: float) -> bool:
        """
        设置目标电流 (力矩控制)

        Args:
            current: 目标电流 (A)

        Returns:
            bool: 操作是否成功
        """
        if not self._enabled:
            raise MotorNotEnabledError(f"电机 {self.motor_id} 未使能")

        try:
            data = self.converter.current_to_bytes(current)
            return self.can_bus.send_command(
                self.motor_id,
                CommandCode.SET_CURRENT,
                data
            )

        except CANError as e:
            raise MotorError(f"设置电流失败: {e}")

    def read_position(self) -> float:
        """
        读取当前位置

        Returns:
            float: 当前位置 (rad)
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_POSITION,
                timeout=0.1
            )

            if response:
                position = self.converter.bytes_to_position(response)
                self._data.position = position
                return position

            return self._data.position

        except CANError:
            return self._data.position

    def read_velocity(self) -> float:
        """
        读取当前速度

        Returns:
            float: 当前速度 (rad/s)
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_VELOCITY,
                timeout=0.1
            )

            if response:
                velocity = self.converter.bytes_to_velocity(response)
                self._data.velocity = velocity
                return velocity

            return self._data.velocity

        except CANError:
            return self._data.velocity

    def read_current(self) -> float:
        """
        读取当前电流

        Returns:
            float: 当前电流 (A)
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_CURRENT,
                timeout=0.1
            )

            if response:
                current = self.converter.bytes_to_current(response)
                self._data.current = current
                return current

            return self._data.current

        except CANError:
            return self._data.current

    def read_status(self) -> MotorData:
        """
        读取完整电机状态

        Returns:
            MotorData: 电机状态数据
        """
        self.read_position()
        self.read_velocity()
        self.read_current()

        # 读取电压
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_VOLTAGE,
                timeout=0.1
            )
            if response:
                self._data.voltage = self.converter.bytes_to_float(response)
        except CANError:
            pass

        # 读取温度
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_MOTOR_TEMP,
                timeout=0.1
            )
            if response:
                self._data.temperature = self.converter.bytes_to_float(response)
        except CANError:
            pass

        return self._data

    def clear_alarm(self) -> bool:
        """
        清除报警

        Returns:
            bool: 操作是否成功
        """
        try:
            result = self.can_bus.send_command(
                self.motor_id,
                CommandCode.CLEAR_ALARM,
                bytes([])
            )

            if result:
                self._error_code = 0

            return result

        except CANError as e:
            raise MotorError(f"清除报警失败: {e}")

    def read_alarm(self) -> int:
        """
        读取报警信息

        Returns:
            int: 报警码
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_ALARM,
                timeout=0.1
            )

            if response:
                alarm_code = self.converter.bytes_to_int(response, 4)
                self._error_code = alarm_code
                return alarm_code

            return self._error_code

        except CANError:
            return self._error_code

    def read_limits(self) -> MotorLimits:
        """
        读取关节限制参数

        使用命令 0x85 (R_P_MAX_L) 和 0x86 (R_P_MIN_L)

        Returns:
            MotorLimits: 关节限制
        """
        # 读取位置上限
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_P_MAX_L,
                timeout=0.1
            )
            if response:
                self._limits.position_max = self.converter.bytes_to_float(response)
        except CANError:
            pass

        # 读取位置下限
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_P_MIN_L,
                timeout=0.1
            )
            if response:
                self._limits.position_min = self.converter.bytes_to_float(response)
        except CANError:
            pass

        return self._limits

    def read_gear_ratio(self) -> float:
        """
        读取减速比

        使用命令 0xC5 (R_GEAR_RATIO)

        Returns:
            float: 减速比
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_GEAR_RATIO,
                timeout=0.1
            )

            if response:
                self._parameters.gear_ratio = self.converter.bytes_to_float(response)
                return self._parameters.gear_ratio

        except CANError:
            pass

        return self._parameters.gear_ratio

    def read_torque_coeff(self) -> float:
        """
        读取转矩系数

        使用命令 0xCB (R_TORQUE_COEFF)

        Returns:
            float: 转矩系数
        """
        try:
            response = self.can_bus.recv_response(
                self.motor_id,
                CommandCode.R_TORQUE_COEFF,
                timeout=0.1
            )

            if response:
                self._parameters.torque_coeff = self.converter.bytes_to_float(response)
                return self._parameters.torque_coeff

        except CANError:
            pass

        return self._parameters.torque_coeff

    def calibrate_zero(self) -> bool:
        """
        标定零点

        使用命令 0x87 (SET_HOME)

        Returns:
            bool: 操作是否成功
        """
        try:
            return self.can_bus.send_command(
                self.motor_id,
                CommandCode.SET_HOME,
                bytes([])
            )

        except CANError as e:
            raise MotorError(f"零点标定失败: {e}")

    def get_state(self) -> Dict[str, Any]:
        """
        获取电机状态

        Returns:
            dict: 状态信息
        """
        return {
            "motor_id": self.motor_id,
            "enabled": self._enabled,
            "mode": self._mode.value,
            "position": self._data.position,
            "velocity": self._data.velocity,
            "current": self._data.current,
            "voltage": self._data.voltage,
            "temperature": self._data.temperature,
            "error_code": self._error_code,
            "limits": {
                "position_min": self._limits.position_min,
                "position_max": self._limits.position_max,
                "velocity_max": self._limits.velocity_max
            },
            "parameters": {
                "gear_ratio": self._parameters.gear_ratio,
                "torque_coeff": self._parameters.torque_coeff
            }
        }

    def __repr__(self) -> str:
        return (
            f"Motor(motor_id={self.motor_id}, "
            f"enabled={self._enabled}, "
            f"mode={self._mode.name}, "
            f"position={self._data.position:.4f})"
        )
