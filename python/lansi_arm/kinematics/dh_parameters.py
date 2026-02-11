"""
DH参数配置

基于URDF提取的DH参数，用于运动学计算。
"""


from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import math


@dataclass
class DHParameter:
    """单个DH参数"""
    a: float      # 连杆长度 (沿X轴)
    alpha: float  # 连杆扭角 (绕X轴)
    d: float      # 连杆偏移 (沿Z轴)
    theta: float  # 关节角度 (绕Z轴)


@dataclass
class DHParameters:
    """DH参数集"""
    # 7个关节的DH参数
    # 注意：这是标准DH参数，需要根据实际机械臂调整
    params: List[DHParameter] = field(default_factory=list)

    def __post_init__(self):
        if not self.params:
            # 使用默认参数 (需要根据实际测量调整)
            self.params = [
                DHParameter(a=0, alpha=-math.pi/2, d=0, theta=0),      # Joint 0
                DHParameter(a=0, alpha=math.pi/2, d=0.096, theta=0),  # Joint 1
                DHParameter(a=0, alpha=-math.pi/2, d=0.075, theta=0),  # Joint 2
                DHParameter(a=0, alpha=math.pi/2, d=-0.12, theta=0),   # Joint 3
                DHParameter(a=0, alpha=-math.pi/2, d=-0.15, theta=0),  # Joint 4
                DHParameter(a=0, alpha=math.pi/2, d=-0.11, theta=0),   # Joint 5
                DHParameter(a=0, alpha=-math.pi/2, d=-0.06, theta=0),  # Joint 6
            ]

    @property
    def n_joints(self) -> int:
        """获取关节数量"""
        return len(self.params)


# 左臂DH参数 (需要根据实际机械臂测量调整)
LEFT_ARM_DH = DHParameters(
    params=[
        # Joint 0: Shoulder Pitch (绕Y轴)
        DHParameter(a=0, alpha=-math.pi/2, d=0.096 + 1.217, theta=0),
        # Joint 1: Shoulder Roll (绕X轴)
        DHParameter(a=0, alpha=math.pi/2, d=0.075, theta=0),
        # Joint 2: Shoulder Yaw (绕Z轴)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.12, theta=0),
        # Joint 3: Elbow Pitch (绕Y轴)
        DHParameter(a=0, alpha=math.pi/2, d=-0.15, theta=0),
        # Joint 4: Wrist Yaw (绕Z轴)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.11, theta=0),
        # Joint 5: Wrist Pitch (绕Y轴)
        DHParameter(a=0, alpha=math.pi/2, d=-0.06, theta=0),
        # Joint 6: Wrist Roll (绕X轴)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.064, theta=0),
    ]
)


# 右臂DH参数 (镜像)
RIGHT_ARM_DH = DHParameters(
    params=[
        # Joint 0: Shoulder Pitch (绕Y轴，反向)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.096 + 1.217, theta=0),
        # Joint 1: Shoulder Roll (绕X轴，反向)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.075, theta=0),
        # Joint 2: Shoulder Yaw (绕Z轴)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.12, theta=0),
        # Joint 3: Elbow Pitch (绕Y轴，反向)
        DHParameter(a=0, alpha=math.pi/2, d=-0.15, theta=0),
        # Joint 4: Wrist Yaw (绕Z轴)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.11, theta=0),
        # Joint 5: Wrist Pitch (绕Y轴，反向)
        DHParameter(a=0, alpha=math.pi/2, d=-0.06, theta=0),
        # Joint 6: Wrist Roll (绕X轴，反向)
        DHParameter(a=0, alpha=-math.pi/2, d=-0.064, theta=0),
    ]
)


def get_dh_parameters(arm_type: str) -> DHParameters:
    """
    获取指定机械臂的DH参数

    Args:
        arm_type: "left" 或 "right"

    Returns:
        DHParameters: DH参数集
    """
    if arm_type.lower() == "left":
        return LEFT_ARM_DH
    elif arm_type.lower() == "right":
        return RIGHT_ARM_DH
    else:
        raise ValueError(f"未知的机械臂类型: {arm_type}")
