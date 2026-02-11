"""
Lansi Arm SDK

专业级7-DOF机械臂控制开发工具包。

主要功能:
- CAN总线通信
- 电机控制
- 单臂/群臂控制
- 运动学计算
- 轨迹规划
- 碰撞检测
- 视觉集成
"""


from .__version__ import __version__
from .exceptions import (
    LansiArmError,
    CommunicationError,
    CANError,
    ControlError,
    KinematicsError,
    JointLimitError,
    MotorError,
    ArmInitializationError,
)

from .constants import (
    MotorMode,
    ArmState,
    ControlTarget,
    CommandCode,
)

from .core import (
    CanBus,
    Motor,
    MotorData,
    MotorLimits,
    DataConverter,
)

from .controller import (
    ArmController,
    JointLimits as ControllerJointLimits,
    MotionParameters,
    GroupController,
    Trajectory,
    TrajectoryPoint,
    PlaybackState,
    # 轨迹模块
    TrajectoryInterpolator,
    TrajectoryRecorder,
    TrajectoryPlayer,
    TrajectoryPlanner,
    TrajectoryManager,
    InterpolationType,
    TrajectoryConfig,
    Waypoint,
)

from .config import (
    ArmType,
    ArmConfig,
    JointConfig,
    LEFT_ARM,
    RIGHT_ARM,
    get_arm_config,
    get_joint_limits,
    get_motor_id_map,
    DEFAULT_JOINT_LIMITS,
)

from .kinematics import (
    Pose,
    ForwardKinematics,
    InverseKinematics,
    Kinematics,
    Jacobian,
    DHParameters,
    get_dh_parameters,
)

from .urdf import (
    CollisionDetector,
    CollisionResult,
    CollisionConfig,
    CollisionType,
    BoundingBox,
    LinkBoundingBoxes,
)

from .vision import (
    VisionType,
    CalibrationType,
    CameraIntrinsics,
    Transform3D,
    CalibrationResult,
    DetectedObject,
    VisionInterface,
    HandEyeCalibration,
    VisionGuidedController,
    DepthCamera,
    RealsenseCamera,
    OpenCVCamera,
)

__version__ = "1.0.0"
__author__ = "Lansi Robotics"

__all__ = [
    # 版本
    "__version__",
    "__author__",
    # 异常
    "LansiArmError",
    "CommunicationError",
    "CANError",
    "ControlError",
    "KinematicsError",
    "JointLimitError",
    "MotorError",
    "ArmInitializationError",
    # 常量
    "MotorMode",
    "ArmState",
    "ControlTarget",
    "CommandCode",
    # 核心
    "CanBus",
    "Motor",
    "MotorData",
    "MotorLimits",
    "DataConverter",
    # 控制器
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
    # 配置
    "ArmType",
    "ArmConfig",
    "JointConfig",
    "LEFT_ARM",
    "RIGHT_ARM",
    "get_arm_config",
    "get_joint_limits",
    "get_motor_id_map",
    "DEFAULT_JOINT_LIMITS",
    # 运动学
    "Pose",
    "ForwardKinematics",
    "InverseKinematics",
    "Kinematics",
    "Jacobian",
    "DHParameters",
    "get_dh_parameters",
    # 碰撞检测
    "CollisionDetector",
    "CollisionResult",
    "CollisionConfig",
    "CollisionType",
    "BoundingBox",
    "LinkBoundingBoxes",
    # 视觉集成
    "VisionType",
    "CalibrationType",
    "CameraIntrinsics",
    "Transform3D",
    "CalibrationResult",
    "DetectedObject",
    "VisionInterface",
    "HandEyeCalibration",
    "VisionGuidedController",
    "DepthCamera",
    "RealsenseCamera",
    "OpenCVCamera",
]
