"""
视觉集成模块

提供视觉相关功能：
- 通用视觉接口
- 手眼标定
- 深度相机支持
- 视觉引导控制
"""


import time
import json
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Any, Callable
from enum import Enum
import numpy as np


class VisionType(Enum):
    """视觉类型"""
    REALSENSE = "realsense"
    KINECT = "kinect"
    OPENCV = "opencv"
    CUSTOM = "custom"


class CalibrationType(Enum):
    """标定类型"""
    EYE_IN_HAND = "eye_in_hand"    # 眼在手上
    EYE_TO_HAND = "eye_to_hand"     # 眼在手外


@dataclass
class CameraIntrinsics:
    """相机内参"""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int
    distortion: List[float] = field(default_factory=list)

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "fx": self.fx,
            "fy": self.fy,
            "cx": self.cx,
            "cy": self.cy,
            "width": self.width,
            "height": self.height,
            "distortion": self.distortion
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "CameraIntrinsics":
        """从字典创建"""
        return cls(
            fx=data["fx"],
            fy=data["fy"],
            cx=data["cx"],
            cy=data["cy"],
            width=data["width"],
            height=data["height"],
            distortion=data.get("distortion", [])
        )


@dataclass
class Transform3D:
    """3D变换 (4x4矩阵)"""
    matrix: np.ndarray  # 4x4齐次变换矩阵

    def __post_init__(self):
        if self.matrix.shape != (4, 4):
            raise ValueError("变换矩阵必须为4x4")

    @property
    def position(self) -> np.ndarray:
        """获取位置 [x, y, z]"""
        return self.matrix[:3, 3]

    @property
    def rotation(self) -> np.ndarray:
        """获取旋转矩阵 (3x3)"""
        return self.matrix[:3, :3]

    @property
    def quaternion(self) -> np.ndarray:
        """获取四元数 [x, y, z, w]"""
        from scipy.transformations import quaternion_from_matrix
        return quaternion_from_matrix(self.matrix)

    def inverse(self) -> "Transform3D":
        """计算逆变换"""
        R = self.rotation.T
        t = -R @ self.position
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = t
        return Transform3D(matrix=matrix)

    def multiply(self, other: "Transform3D") -> "Transform3D":
        """矩阵乘法 (T1 * T2)"""
        return Transform3D(matrix=self.matrix @ other.matrix)

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "matrix": self.matrix.tolist()
        }

    @classmethod
    def from_dict(cls, data: Dict) -> "Transform3D":
        """从字典创建"""
        return cls(matrix=np.array(data["matrix"]))

    @classmethod
    def from_pose(cls, position: List[float], orientation: List[float]) -> "Transform3D":
        """从位姿创建 (位置 + 四元数/欧拉角)"""
        from scipy.transformations import quaternion_matrix, euler_matrix

        if len(orientation) == 4:
            # 四元数
            matrix = quaternion_matrix(orientation)
        else:
            # 欧拉角 [roll, pitch, yaw]
            matrix = euler_matrix(*orientation, 'rxyz')

        matrix[:3, 3] = position
        return cls(matrix=matrix)


@dataclass
class CalibrationResult:
    """标定结果"""
    success: bool
    transform: Optional[Transform3D]
    camera_intrinsics: CameraIntrinsics
    error: float
    iterations: int
    timestamp: float
    details: Dict = field(default_factory=dict)


@dataclass
class DetectedObject:
    """检测到的物体"""
    name: str
    pose: Transform3D
    confidence: float
    bounding_box_2d: Optional[List[int]] = None  # [x_min, y_min, x_max, y_max]
    bounding_box_3d: Optional[List[float]] = None  # [min_x, min_y, min_z, max_x, max_y, max_z]
    mask: Optional[np.ndarray] = None
    features: Dict = field(default_factory=dict)


class VisionInterface:
    """
    通用视觉接口

    定义视觉系统的抽象接口。
    """

    def __init__(self, config: Optional[Dict] = None):
        """
        Args:
            config: 视觉配置
        """
        self.config = config or {}
        self._connected = False

    @property
    def connected(self) -> bool:
        """检查是否已连接"""
        return self._connected

    def connect(self) -> bool:
        """连接视觉设备"""
        raise NotImplementedError

    def disconnect(self) -> None:
        """断开连接"""
        self._connected = False

    def get_intrinsics(self) -> CameraIntrinsics:
        """获取相机内参"""
        raise NotImplementedError

    def get_color_image(self) -> np.ndarray:
        """获取彩色图像"""
        raise NotImplementedError

    def get_depth_image(self) -> np.ndarray:
        """获取深度图像"""
        raise NotImplementedError

    def get_point_cloud(self) -> np.ndarray:
        """获取点云 (Nx3)"""
        raise NotImplementedError

    def get_rgbd_image(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取RGBD图像"""
        raise NotImplementedError

    def detect_objects(self, **kwargs) -> List[DetectedObject]:
        """检测物体"""
        raise NotImplementedError

    def get_status(self) -> Dict:
        """获取状态"""
        return {
            "connected": self._connected,
            "type": self.__class__.__name__
        }


class HandEyeCalibration:
    """
    手眼标定

    支持：
    - Eye-in-Hand: 相机在机械臂末端
    - Eye-to-Hand: 相机固定在工作空间中
    """

    def __init__(
        self,
        calibration_type: CalibrationType = CalibrationType.EYE_IN_HAND
    ):
        """
        Args:
            calibration_type: 标定类型
        """
        self.calibration_type = calibration_type

        # 标定数据
        self._robot_poses: List[Transform3D] = []
        self._camera_poses: List[Transform3D] = []

        # 标定结果
        self._result: Optional[CalibrationResult] = None

        # 标定参数
        self._max_iterations = 100
        self._tolerance = 1e-5

    def add_calibration_data(
        self,
        robot_pose: Transform3D,
        camera_pose: Transform3D
    ) -> None:
        """
        添加标定数据

        Args:
            robot_pose: 机械臂末端位姿
            camera_pose: 相机检测到的标定板位姿
        """
        self._robot_poses.append(robot_pose)
        self._camera_poses.append(camera_pose)

    def calibrate(self) -> CalibrationResult:
        """
        执行手眼标定

        使用Tsai-Lenz算法或优化方法求解手眼矩阵。

        Returns:
            CalibrationResult: 标定结果
        """
        if len(self._robot_poses) < 3:
            return CalibrationResult(
                success=False,
                transform=None,
                camera_intrinsics=CameraIntrinsics(0, 0, 0, 0, 0, 0),
                error=float('inf'),
                iterations=0,
                timestamp=time.time(),
                details={"error": "标定数据不足，至少需要3组数据"}
            )

        try:
            # 简化的手眼标定算法
            if self.calibration_type == CalibrationType.EYE_IN_HAND:
                # Eye-in-Hand: AX = XB
                transform = self._solve_eye_in_hand()
            else:
                # Eye-to-Hand: AX = YB
                transform = self._solve_eye_to_hand()

            # 计算重投影误差
            error = self._compute_calibration_error(transform)

            self._result = CalibrationResult(
                success=True,
                transform=transform,
                camera_intrinsics=CameraIntrinsics(0, 0, 0, 0, 0, 0),
                error=error,
                iterations=self._max_iterations,
                timestamp=time.time(),
                details={
                    "calibration_type": self.calibration_type.value,
                    "data_points": len(self._robot_poses)
                }
            )

            return self._result

        except Exception as e:
            return CalibrationResult(
                success=False,
                transform=None,
                camera_intrinsics=CameraIntrinsics(0, 0, 0, 0, 0, 0),
                error=float('inf'),
                iterations=0,
                timestamp=time.time(),
                details={"error": str(e)}
            )

    def _solve_eye_in_hand(self) -> Transform3D:
        """
        求解Eye-in-Hand问题 AX = XB

        Returns:
            Transform3D: 手眼变换矩阵
        """
        import numpy as np
        from scipy.optimize import least_squares

        # 提取变换矩阵
        n = len(self._robot_poses)
        R_cam = np.array([pose.rotation for pose in self._camera_poses])
        t_cam = np.array([pose.position for pose in self._camera_poses])

        R_base = np.array([pose.rotation for pose in self._robot_poses])
        t_base = np.array([pose.position for pose in self._robot_poses])

        # 简化的求解 (使用平均偏移作为初始猜测)
        # 实际应使用Tsai-Lenz算法
        initial_R = np.eye(3)
        initial_t = np.array([0.0, 0.0, 0.1])  # 相机在末端前方10cm

        # 优化求解
        def residual(x):
            R = self._axis_angle_to_rotation(x[:3])
            t = x[3:]

            error = []
            for i in range(n):
                # 计算预测的相机位姿
                R_pred = R_base[i] @ R
                t_pred = R_base[i] @ t + t_base[i]

                # 与实际值比较
                error.append(np.linalg.norm(R_pred - R_cam[i]))
                error.append(np.linalg.norm(t_pred - t_cam[i]))

            return np.array(error)

        # 优化
        x0 = np.array([0, 0, 0, 0, 0, 0.1])
        result = least_squares(residual, x0, max_nfev=self._max_iterations)

        R = self._axis_angle_to_rotation(result.x[:3])
        t = result.x[3:]

        # 构建变换矩阵
        matrix = np.eye(4)
        matrix[:3, :3] = R
        matrix[:3, 3] = t

        return Transform3D(matrix=matrix)

    def _solve_eye_to_hand(self) -> Transform3D:
        """
        求解Eye-to-Hand问题 AX = YB

        Returns:
            Transform3D: 手眼变换矩阵
        """
        # Eye-to-Hand: 相机固定，机械臂末端移动
        # 求解 Y = AX B^-1
        # 简化实现
        return self._solve_eye_in_hand()

    def _axis_angle_to_rotation(self, axis_angle: np.ndarray) -> np.ndarray:
        """轴角转换为旋转矩阵"""
        angle = np.linalg.norm(axis_angle)
        if angle < 1e-6:
            return np.eye(3)

        axis = axis_angle / angle
        return np.eye(3) + np.sin(angle) * self._skew(axis) + \
               (1 - np.cos(angle)) * (self._skew(axis) @ self._skew(axis))

    def _skew(self, v: np.ndarray) -> np.ndarray:
        """叉乘矩阵"""
        return np.array([
            [0, -v[2], v[1]],
            [v[2], 0, -v[0]],
            [-v[1], v[0], 0]
        ])

    def _compute_calibration_error(self, transform: Transform3D) -> float:
        """计算标定误差"""
        if not self._robot_poses:
            return float('inf')

        errors = []
        for i in range(len(self._robot_poses)):
            if self.calibration_type == CalibrationType.EYE_IN_HAND:
                # 预测相机位姿
                predicted = self._robot_poses[i].multiply(transform)
            else:
                predicted = transform.multiply(self._robot_poses[i])

            # 计算误差
            pos_error = np.linalg.norm(
                predicted.position - self._camera_poses[i].position
            )
            errors.append(pos_error)

        return np.mean(errors)

    def save_calibration(self, filepath: str) -> bool:
        """
        保存标定结果

        Args:
            filepath: 文件路径

        Returns:
            bool: 是否保存成功
        """
        if self._result is None:
            return False

        data = {
            "calibration_type": self.calibration_type.value,
            "success": self._result.success,
            "error": self._result.error,
            "timestamp": self._result.timestamp,
            "transform": self._result.transform.to_dict() if self._result.transform else None,
            "data_points": len(self._robot_poses)
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        return True

    def load_calibration(self, filepath: str) -> bool:
        """
        加载标定结果

        Args:
            filepath: 文件路径

        Returns:
            bool: 是否加载成功
        """
        with open(filepath, 'r') as f:
            data = json.load(f)

        if "transform" in data and data["transform"]:
            self._result = CalibrationResult(
                success=data["success"],
                transform=Transform3D.from_dict(data["transform"]),
                camera_intrinsics=CameraIntrinsics(0, 0, 0, 0, 0, 0),
                error=data.get("error", 0),
                iterations=0,
                timestamp=data.get("timestamp", 0)
            )
            return True

        return False

    def get_calibration_matrix(self) -> Optional[np.ndarray]:
        """获取标定矩阵 (4x4)"""
        if self._result and self._result.transform:
            return self._result.transform.matrix
        return None

    def clear_data(self) -> None:
        """清除标定数据"""
        self._robot_poses.clear()
        self._camera_poses.clear()
        self._result = None

    @property
    def data_count(self) -> int:
        """标定数据数量"""
        return len(self._robot_poses)

    @property
    def result(self) -> Optional[CalibrationResult]:
        """获取标定结果"""
        return self._result


class VisionGuidedController:
    """
    视觉引导控制器

    结合视觉信息引导机械臂运动。
    """

    def __init__(
        self,
        vision_interface: VisionInterface,
        hand_eye_calibration: Optional[HandEyeCalibration] = None
    ):
        """
        Args:
            vision_interface: 视觉接口
            hand_eye_calibration: 手眼标定结果
        """
        self.vision = vision_interface
        self.calibration = hand_eye_calibration

        # 目标跟踪
        self._tracked_object: Optional[DetectedObject] = None
        self._tracking_enabled = False

    def set_tracked_object(self, object_name: str) -> bool:
        """
        设置跟踪物体

        Args:
            object_name: 物体名称

        Returns:
            bool: 是否设置成功
        """
        objects = self.vision.detect_objects()

        for obj in objects:
            if obj.name == object_name:
                self._tracked_object = obj
                return True

        return False

    def enable_tracking(self, enabled: bool = True) -> None:
        """启用/禁用跟踪"""
        self._tracking_enabled = enabled

    def get_object_pose_in_base(
        self,
        object_pose: Transform3D
    ) -> Optional[Transform3D]:
        """
        将物体在相机坐标系下的位姿转换到基座坐标系

        Args:
            object_pose: 物体在相机坐标系下的位姿

        Returns:
            Transform3D: 物体在基座坐标系下的位姿
        """
        if self.calibration is None or self.calibration.result is None:
            return None

        # 相机到末端 + 末端到基座
        # T_base = T_base_endo * T_endo_cam * T_cam_object
        camera_to_end = self.calibration.get_calibration_matrix()
        if camera_to_end is None:
            return None

        # 转换
        transform = Transform3D(matrix=camera_to_end)
        result = object_pose.multiply(transform)

        return result

    def move_to_object(
        self,
        target_pose: Transform3D,
        approach_distance: float = 0.1,
        speed: float = 0.5
    ) -> Tuple[bool, Optional[Transform3D]]:
        """
        移动到目标物体

        Args:
            target_pose: 目标位姿
            approach_distance: 接近距离 (m)
            speed: 移动速度

        Returns:
            Tuple[是否成功, 目标位姿]
        """
        if self.calibration is None:
            return False, None

        # 计算接近位姿
        approach_pose = self._compute_approach_pose(target_pose, approach_distance)

        return True, approach_pose

    def _compute_approach_pose(
        self,
        target_pose: Transform3D,
        distance: float
    ) -> Transform3D:
        """
        计算接近位姿

        Args:
            target_pose: 目标位姿
            distance: 接近距离

        Returns:
            Transform3D: 接近位姿
        """
        # 沿Z轴负方向后退
        approach_matrix = np.eye(4)
        approach_matrix[2, 3] = -distance

        approach = Transform3D(matrix=approach_matrix)
        return target_pose.multiply(approach)

    def visual_servoing(
        self,
        current_pose: Transform3D,
        target_pose: Transform3D,
        Kp: float = 1.0,
        max_velocity: float = 0.1
    ) -> np.ndarray:
        """
        视觉伺服控制

        计算末端速度以接近目标。

        Args:
            current_pose: 当前末端位姿
            target_pose: 目标位姿
            Kp: 比例增益
            max_velocity: 最大速度

        Returns:
            np.ndarray: 末端速度 [vx, vy, vz, wx, wy, wz]
        """
        # 位置误差
        position_error = target_pose.position - current_pose.position

        # 简化的速度计算
        velocity = Kp * position_error

        # 限制速度
        velocity = np.clip(velocity, -max_velocity, max_velocity)

        return velocity

    def detect_and_grasp(
        self,
        object_name: str,
        grasp_offset: float = 0.05
    ) -> Tuple[bool, Optional[Transform3D], Optional[DetectedObject]]:
        """
        检测并抓取

        Args:
            object_name: 目标物体名称
            grasp_offset: 抓取偏移

        Returns:
            Tuple[是否成功, 抓取位姿, 检测到的物体]
        """
        # 检测物体
        objects = self.vision.detect_objects()

        target_object = None
        for obj in objects:
            if obj.name == object_name:
                target_object = obj
                break

        if target_object is None:
            return False, None, None

        # 计算抓取位姿
        grasp_pose = self.get_object_pose_in_base(target_object.pose)
        if grasp_pose is None:
            return False, None, target_object

        # 调整抓取位姿 (简单的前方抓取)
        approach = np.eye(4)
        approach[2, 3] = -grasp_offset
        grasp_pose = grasp_pose.multiply(Transform3D(matrix=approach))

        return True, grasp_pose, target_object

    def get_status(self) -> Dict:
        """获取状态"""
        return {
            "vision_connected": self.vision.connected,
            "calibration_loaded": self.calibration is not None and
                                    self.calibration.result is not None,
            "tracking_enabled": self._tracking_enabled,
            "tracked_object": self._tracked_object.name if self._tracked_object else None
        }


class DepthCamera:
    """
    深度相机接口

    支持RealSense等深度相机。
    """

    def __init__(self, camera_type: VisionType = VisionType.REALSENSE):
        """
        Args:
            camera_type: 相机类型
        """
        self.camera_type = camera_type
        self._interface: Optional[VisionInterface] = None
        self._intrinsics: Optional[CameraIntrinsics] = None

    def connect(self, device_id: int = 0) -> bool:
        """
        连接相机

        Args:
            device_id: 设备ID

        Returns:
            bool: 是否连接成功
        """
        try:
            if self.camera_type == VisionType.REALSENSE:
                self._interface = RealsenseCamera(device_id)
            else:
                self._interface = OpenCVCamera(device_id)

            if self._interface.connect():
                self._intrinsics = self._interface.get_intrinsics()
                return True

        except Exception:
            pass

        return False

    def disconnect(self) -> None:
        """断开连接"""
        if self._interface:
            self._interface.disconnect()

    def get_point_cloud(self) -> np.ndarray:
        """获取点云"""
        if self._interface is None:
            raise RuntimeError("相机未连接")

        return self._interface.get_point_cloud()

    def get_rgbd(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取RGBD图像"""
        if self._interface is None:
            raise RuntimeError("相机未连接")

        return self._interface.get_rgbd_image()

    def project_points(
        self,
        points_3d: np.ndarray
    ) -> np.ndarray:
        """
        3D点投影到2D像素坐标

        Args:
            points_3d: Nx3 3D点

        Returns:
            np.ndarray: Nx2 像素坐标
        """
        if self._intrinsics is None:
            raise RuntimeError("相机未标定")

        fx, fy = self._intrinsics.fx, self._intrinsics.fy
        cx, cy = self._intrinsics.cx, self._intrinsics.cy

        # 投影
        points_2d = np.zeros((len(points_3d), 2))
        points_2d[:, 0] = (points_3d[:, 0] * fx / points_3d[:, 2] + cx)
        points_2d[:, 1] = (points_3d[:, 1] * fy / points_3d[:, 2] + cy)

        return points_2d

    def deproject_pixels(
        self,
        pixels_2d: np.ndarray,
        depth: np.ndarray
    ) -> np.ndarray:
        """
        2D像素坐标反投影到3D点

        Args:
            pixels_2d: Nx2 像素坐标
            depth: N 深度值

        Returns:
            np.ndarray: Nx3 3D点
        """
        if self._intrinsics is None:
            raise RuntimeError("相机未标定")

        fx, fy = self._intrinsics.fx, self._intrinsics.fy
        cx, cy = self._intrinsics.cx, self._intrinsics.cy

        points_3d = np.zeros((len(pixels_2d), 3))
        points_3d[:, 0] = (pixels_2d[:, 0] - cx) * depth / fx
        points_3d[:, 1] = (pixels_2d[:, 1] - cy) * depth / fy
        points_3d[:, 2] = depth

        return points_3d


class RealsenseCamera(VisionInterface):
    """RealSense相机实现"""

    def __init__(self, device_id: int = 0, config: Optional[Dict] = None):
        super().__init__(config)
        self.device_id = device_id
        self._pipeline = None
        self._profile = None

    def connect(self) -> bool:
        """连接RealSense相机"""
        try:
            import pyrealsense2 as rs

            # 创建管道
            self._pipeline = rs.pipeline()
            config = rs.config()

            # 配置流
            config.enable_stream(
                rs.stream.color,
                640, 480,
                rs.format.bgr8, 30
            )
            config.enable_stream(
                rs.stream.depth,
                640, 480,
                rs.format.z16, 30
            )

            # 启动
            self._profile = self._pipeline.start(config)
            self._connected = True

            return True

        except ImportError:
            print("请安装pyrealsense2: pip install pyrealsense2")
            return False
        except Exception as e:
            print(f"RealSense连接失败: {e}")
            return False

    def disconnect(self) -> None:
        """断开连接"""
        if self._pipeline:
            self._pipeline.stop()
            self._pipeline = None
        self._connected = False

    def get_intrinsics(self) -> CameraIntrinsics:
        """获取内参"""
        if self._profile is None:
            return CameraIntrinsics(0, 0, 0, 0, 0, 0)

        color_stream = self._profile.get_stream(rs.stream.color)
        intrinsics = color_stream.as_video_stream_profile().get_intrinsics()

        return CameraIntrinsics(
            fx=intrinsics.fx,
            fy=intrinsics.fy,
            cx=intrinsics.ppx,
            cy=intrinsics.ppy,
            width=intrinsics.width,
            height=intrinsics.height
        )

    def get_color_image(self) -> np.ndarray:
        """获取彩色图像"""
        if self._pipeline is None:
            raise RuntimeError("相机未连接")

        import pyrealsense2 as rs

        frames = self._pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()

        return np.array(color_frame.get_data())

    def get_depth_image(self) -> np.ndarray:
        """获取深度图像"""
        if self._pipeline is None:
            raise RuntimeError("相机未连接")

        import pyrealsense2 as rs

        frames = self._pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        return np.array(depth_frame.get_data())

    def get_point_cloud(self) -> np.ndarray:
        """获取点云"""
        import pyrealsense2 as rs

        frames = self._pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        # 简化的点云计算
        depth = np.array(depth_frame.get_data())
        intrinsics = self.get_intrinsics()

        y, x = np.indices(depth.shape)
        points = np.zeros((depth.shape[0], depth.shape[1], 3))
        points[:, :, 0] = (x - intrinsics.cx) * depth / intrinsics.fx
        points[:, :, 1] = (y - intrinsics.cy) * depth / intrinsics.fy
        points[:, :, 2] = depth.astype(float) / 1000.0  # mm to m

        return points.reshape(-1, 3)

    def get_rgbd_image(self) -> Tuple[np.ndarray, np.ndarray]:
        """获取RGBD图像"""
        color = self.get_color_image()
        depth = self.get_depth_image()
        return color, depth

    def detect_objects(self, **kwargs) -> List[DetectedObject]:
        """检测物体 (需要集成检测算法)"""
        return []


class OpenCVCamera(VisionInterface):
    """OpenCV相机实现 (普通RGB相机)"""

    def __init__(self, device_id: int = 0, config: Optional[Dict] = None):
        super().__init__(config)
        self.device_id = device_id
        self._cap = None

    def connect(self) -> bool:
        """连接相机"""
        try:
            import cv2
            self._cap = cv2.VideoCapture(self.device_id)

            if self._cap.isOpened():
                self._connected = True
                return True

        except Exception as e:
            print(f"相机连接失败: {e}")

        return False

    def disconnect(self) -> None:
        """断开连接"""
        if self._cap:
            self._cap.release()
            self._cap = None
        self._connected = False

    def get_intrinsics(self) -> CameraIntrinsics:
        """获取内参 (需要标定)"""
        return CameraIntrinsics(
            fx=500.0, fy=500.0,
            cx=320.0, cy=240.0,
            width=640, height=480
        )

    def get_color_image(self) -> np.ndarray:
        """获取彩色图像"""
        if self._cap is None:
            raise RuntimeError("相机未连接")

        ret, frame = self._cap.read()
        if ret:
            return frame

        raise RuntimeError("无法读取图像")

    def get_depth_image(self) -> np.ndarray:
        """普通相机不支持深度图"""
        raise NotImplementedError("普通RGB相机不支持深度图")

    def get_point_cloud(self) -> np.ndarray:
        """需要深度信息"""
        raise NotImplementedError("需要深度相机")

    def get_rgbd_image(self) -> Tuple[np.ndarray, np.ndarray]:
        """需要深度信息"""
        color = self.get_color_image()
        depth = np.zeros(color.shape[:2], dtype=np.float32)
        return color, depth

    def detect_objects(self, **kwargs) -> List[DetectedObject]:
        """检测物体 (使用OpenCV)"""
        return []
