"""
Lansi Arm URDF配置

基于 lens_dual_arm_description.urdf 的参数配置
"""


from dataclasses import dataclass, field
from typing import List, Dict, Tuple, Optional
from enum import Enum


class ArmType(Enum):
    """机械臂类型"""
    LEFT = "left"
    RIGHT = "right"


@dataclass
class JointConfig:
    """单个关节配置"""
    name: str                    # 关节名称
    motor_id: int               # 电机ID
    axis: Tuple[float, float, float]  # 旋转轴 (xyz)
    origin: Tuple[float, float, float]  # 关节位置偏移 (xyz)
    lower_limit: float          # 角度下限 (rad)
    upper_limit: float          # 角度上限 (rad)
    velocity_limit: float        # 速度限制 (rad/s)
    effort_limit: float         # 力矩限制 (Nm)


@dataclass
class ArmConfig:
    """单臂配置"""
    arm_type: ArmType
    joint_configs: List[JointConfig]
    base_height: float = 1.217  # 基座高度 (m)

    @property
    def motor_ids(self) -> List[int]:
        """获取所有电机ID"""
        return [jc.motor_id for jc in self.joint_configs]

    @property
    def n_joints(self) -> int:
        """获取关节数量"""
        return len(self.joint_configs)


# 左臂关节配置 (基于URDF)
LEFT_ARM_JOINTS = [
    JointConfig(
        name="Shoulder_Pitch",
        motor_id=51,
        axis=(0, 1, 0),           # Y轴
        origin=(0, 0.096, 0),
        lower_limit=-2.97,        # -170°
        upper_limit=1.05,          # 60°
        velocity_limit=10.0,
        effort_limit=80.0
    ),
    JointConfig(
        name="Shoulder_Roll",
        motor_id=52,
        axis=(1, 0, 0),           # X轴
        origin=(0.0328, 0.075, 0),
        lower_limit=-0.35,        # -20°
        upper_limit=3.49,         # 200°
        velocity_limit=10.0,
        effort_limit=80.0
    ),
    JointConfig(
        name="Shoulder_Yaw",
        motor_id=53,
        axis=(0, 0, 1),           # Z轴
        origin=(-0.0328, 0, -0.12),
        lower_limit=-2.79,         # -160°
        upper_limit=2.79,          # 160°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Elbow_Pitch",
        motor_id=54,
        axis=(0, 1, 0),           # Y轴
        origin=(0.0095, -0.0325, -0.15),
        lower_limit=-2.355,        # -135°
        upper_limit=0.0,           # 0°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Yaw",
        motor_id=55,
        axis=(0, 0, 1),           # Z轴
        origin=(-0.0095, 0.0325, -0.11),
        lower_limit=-2.79,         # -160°
        upper_limit=2.79,          # 160°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Pitch",
        motor_id=56,
        axis=(0, 1, 0),           # Y轴
        origin=(0, -0.022, -0.06),
        lower_limit=-1.57,         # -90°
        upper_limit=1.57,          # 90°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Roll",
        motor_id=57,
        axis=(1, 0, 0),           # X轴
        origin=(0.0225, 0.022, -0.064),
        lower_limit=-1.57,         # -90°
        upper_limit=1.57,          # 90°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
]


# 右臂关节配置 (基于URDF)
RIGHT_ARM_JOINTS = [
    JointConfig(
        name="Shoulder_Pitch",
        motor_id=61,
        axis=(0, 1, 0),           # Y轴
        origin=(0, -0.096, 0),
        lower_limit=-2.97,        # -170°
        upper_limit=1.05,          # 60°
        velocity_limit=10.0,
        effort_limit=80.0
    ),
    JointConfig(
        name="Shoulder_Roll",
        motor_id=62,
        axis=(1, 0, 0),           # X轴
        origin=(0.0328, -0.075, 0),
        lower_limit=-3.49,         # -200°
        upper_limit=0.35,          # 20°
        velocity_limit=10.0,
        effort_limit=80.0
    ),
    JointConfig(
        name="Shoulder_Yaw",
        motor_id=63,
        axis=(0, 0, 1),           # Z轴
        origin=(-0.0328, 0, -0.12),
        lower_limit=-2.79,         # -160°
        upper_limit=2.79,          # 160°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Elbow_Pitch",
        motor_id=64,
        axis=(0, 1, 0),           # Y轴
        origin=(0.0095, -0.0325, -0.15),
        lower_limit=-2.355,        # -135°
        upper_limit=0.0,           # 0°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Yaw",
        motor_id=65,
        axis=(0, 0, 1),           # Z轴
        origin=(-0.0095, 0.0325, -0.11),
        lower_limit=-2.79,         # -160°
        upper_limit=2.79,          # 160°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Pitch",
        motor_id=66,
        axis=(0, 1, 0),           # Y轴
        origin=(0, 0.022, -0.06),
        lower_limit=-1.57,         # -90°
        upper_limit=1.57,          # 90°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
    JointConfig(
        name="Wrist_Roll",
        motor_id=67,
        axis=(1, 0, 0),           # X轴
        origin=(0.0225, -0.022, -0.064),
        lower_limit=-1.57,         # -90°
        upper_limit=1.57,          # 90°
        velocity_limit=10.0,
        effort_limit=36.0
    ),
]


# 创建双臂配置
LEFT_ARM = ArmConfig(
    arm_type=ArmType.LEFT,
    joint_configs=LEFT_ARM_JOINTS,
    base_height=1.217
)

RIGHT_ARM = ArmConfig(
    arm_type=ArmType.RIGHT,
    joint_configs=RIGHT_ARM_JOINTS,
    base_height=1.217
)


def get_arm_config(arm_type: ArmType) -> ArmConfig:
    """
    获取指定类型的机械臂配置

    Args:
        arm_type: "left" 或 "right"

    Returns:
        ArmConfig: 机械臂配置
    """
    if arm_type == ArmType.LEFT:
        return LEFT_ARM
    elif arm_type == ArmType.RIGHT:
        return RIGHT_ARM
    else:
        raise ValueError(f"未知的机械臂类型: {arm_type}")


def get_joint_limits(arm_type: ArmType) -> Dict[str, Dict[str, float]]:
    """
    获取关节限制

    Args:
        arm_type: "left" 或 "right"

    Returns:
        Dict: 关节名称 -> {lower, upper, velocity}
    """
    config = get_arm_config(ArmType(arm_type))
    limits = {}

    for jc in config.joint_configs:
        limits[jc.name] = {
            "lower": jc.lower_limit,
            "upper": jc.upper_limit,
            "velocity": jc.velocity_limit,
            "effort": jc.effort_limit
        }

    return limits


def get_motor_id_map(arm_type: ArmType) -> Dict[int, str]:
    """
    获取电机ID到关节名称的映射

    Args:
        arm_type: "left" 或 "right"

    Returns:
        Dict: 电机ID -> 关节名称
    """
    config = get_arm_config(arm_type)
    return {jc.motor_id: jc.name for jc in config.joint_configs}


# 默认关节限制 (用于控制器)
DEFAULT_JOINT_LIMITS = {
    "position_min": [
        -2.97,   # Shoulder_Pitch
        -3.49,   # Shoulder_Roll (右臂), -0.35 (左臂)
        -2.79,   # Shoulder_Yaw
        -2.355,  # Elbow_Pitch
        -2.79,   # Wrist_Yaw
        -1.57,   # Wrist_Pitch
        -1.57,   # Wrist_Roll
    ],
    "position_max": [
        1.05,    # Shoulder_Pitch
        0.35,    # Shoulder_Roll (右臂), 3.49 (左臂)
        2.79,    # Shoulder_Yaw
        0.0,     # Elbow_Pitch
        2.79,    # Wrist_Yaw
        1.57,    # Wrist_Pitch
        1.57,    # Wrist_Roll
    ],
    "velocity_max": [10.0] * 7,
}
