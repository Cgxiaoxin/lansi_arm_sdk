"""
配置模块

提供机械臂配置参数
"""

from .arm_config import (
    ArmType,
    JointConfig,
    ArmConfig,
    LEFT_ARM,
    RIGHT_ARM,
    LEFT_ARM_JOINTS,
    RIGHT_ARM_JOINTS,
    get_arm_config,
    get_joint_limits,
    get_motor_id_map,
    DEFAULT_JOINT_LIMITS,
)

__all__ = [
    "ArmType",
    "JointConfig",
    "ArmConfig",
    "LEFT_ARM",
    "RIGHT_ARM",
    "LEFT_ARM_JOINTS",
    "RIGHT_ARM_JOINTS",
    "get_arm_config",
    "get_joint_limits",
    "get_motor_id_map",
    "DEFAULT_JOINT_LIMITS",
]
