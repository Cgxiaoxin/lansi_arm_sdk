"""
核心通信模块

提供CAN总线通信和电机控制的基础功能。
"""

from .can_bus import CanBus, CANConfig, CANMessageData
from .motor import Motor, MotorData, MotorLimits, MotorParameters
from .data_converter import DataConverter

__all__ = [
    "CanBus",
    "CANConfig",
    "CANMessageData",
    "Motor",
    "MotorData",
    "MotorLimits",
    "MotorParameters",
    "DataConverter",
]
