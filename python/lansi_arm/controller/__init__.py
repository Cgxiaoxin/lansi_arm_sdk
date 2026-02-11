"""
控制器模块

提供机械臂的运动控制接口。
"""

from .arm_controller import ArmController, JointLimits, MotionParameters
from .group_controller import GroupController, Trajectory, TrajectoryPoint, PlaybackState
from .trajectory import (
    TrajectoryInterpolator,
    TrajectoryRecorder,
    TrajectoryPlayer,
    TrajectoryPlanner,
    TrajectoryManager,
    InterpolationType,
    TrajectoryConfig,
    Waypoint,
)

__all__ = [
    "ArmController",
    "JointLimits",
    "MotionParameters",
    "GroupController",
    "Trajectory",
    "TrajectoryPoint",
    "PlaybackState",
    # 轨迹模块
    "TrajectoryInterpolator",
    "TrajectoryRecorder",
    "TrajectoryPlayer",
    "TrajectoryPlanner",
    "TrajectoryManager",
    "InterpolationType",
    "TrajectoryConfig",
    "Waypoint",
]
