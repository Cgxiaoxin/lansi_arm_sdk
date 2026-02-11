"""
碰撞检测模块

提供基于URDF的碰撞检测功能：
- 自碰撞检测
- 环境碰撞检测
- 包围盒检测
- 碰撞可视化
"""


import time
from dataclasses import dataclass
from typing import List, Dict, Optional, Tuple, Set, Any
from enum import Enum
import numpy as np
from collections import defaultdict


class CollisionType(Enum):
    """碰撞类型"""
    NO_COLLISION = "no_collision"
    SELF_COLLISION = "self_collision"
    ENVIRONMENT_COLLISION = "environment_collision"
    BOUNDING_BOX = "bounding_box"


@dataclass
class CollisionResult:
    """碰撞检测结果"""
    collision: bool
    collision_type: CollisionType
    colliding_links: List[Tuple[str, str]]
    distance: float
    penetration_depth: float
    timestamp: float


@dataclass
class CollisionConfig:
    """碰撞检测配置"""
    enable_self_collision: bool = True
    enable_environment_collision: bool = True
    enable_bounding_box: bool = True
    padding: float = 0.01  # 碰撞padding (m)
    threshold: float = 0.001  # 碰撞检测阈值 (m)
    collision_groups: Dict[str, Set[str]] = None

    def __post_init__(self):
        if self.collision_groups is None:
            self.collision_groups = {
                "left_arm": {"left_shoulder", "left_elbow", "left_wrist", "left_ee"},
                "right_arm": {"right_shoulder", "right_elbow", "right_wrist", "right_ee"},
                "environment": {"table", "obstacle", "wall"}
            }


class BoundingBox:
    """轴对齐包围盒 (AABB)"""

    def __init__(
        self,
        min_corner: Optional[List[float]] = None,
        max_corner: Optional[List[float]] = None,
        center: Optional[List[float]] = None,
        size: Optional[List[float]] = None
    ):
        """
        Args:
            min_corner: 最小角坐标
            max_corner: 最大角坐标
            center: 包围盒中心
            size: 包围盒尺寸
        """
        if min_corner is not None and max_corner is not None:
            self.min = np.array(min_corner)
            self.max = np.array(max_corner)
        elif center is not None and size is not None:
            half_size = np.array(size) / 2
            self.min = np.array(center) - half_size
            self.max = np.array(center) + half_size
        else:
            self.min = np.array([0.0, 0.0, 0.0])
            self.max = np.array([0.0, 0.0, 0.0])

    @property
    def center(self) -> np.ndarray:
        """获取中心"""
        return (self.min + self.max) / 2

    @property
    def size(self) -> np.ndarray:
        """获取尺寸"""
        return self.max - self.min

    @property
    def volume(self) -> float:
        """获取体积"""
        return np.prod(self.size)

    def expand(self, padding: float) -> "BoundingBox":
        """扩展包围盒"""
        return BoundingBox(
            min_corner=(self.min - padding).tolist(),
            max_corner=(self.max + padding).tolist()
        )

    def contains(self, point: np.ndarray) -> bool:
        """检查点是否在包围盒内"""
        return np.all(point >= self.min) and np.all(point <= self.max)

    def intersect(self, other: "BoundingBox") -> bool:
        """检查是否与另一个包围盒相交"""
        return np.all(self.min <= other.max) and np.all(self.max >= other.min)

    def distance_to(self, other: "BoundingBox") -> float:
        """计算到另一个包围盒的距离"""
        # 如果相交，距离为0
        if self.intersect(other):
            return 0.0

        # 计算各轴距离
        distance = 0.0
        for i in range(3):
            if self.max[i] < other.min[i]:
                distance += (other.min[i] - self.max[i]) ** 2
            elif other.max[i] < self.min[i]:
                distance += (self.min[i] - other.max[i]) ** 2

        return np.sqrt(distance)

    def is_inside(self, other: "BoundingBox") -> bool:
        """检查是否完全在另一个包围盒内"""
        return np.all(other.min <= self.min) and np.all(self.max <= other.max)

    def merge(self, other: "BoundingBox") -> "BoundingBox":
        """合并两个包围盒"""
        return BoundingBox(
            min_corner=np.minimum(self.min, other.min).tolist(),
            max_corner=np.maximum(self.max, other.max).tolist()
        )

    def to_dict(self) -> Dict:
        """转换为字典"""
        return {
            "min": self.min.tolist(),
            "max": self.max.tolist(),
            "center": self.center.tolist(),
            "size": self.size.tolist()
        }


class LinkBoundingBoxes:
    """连杆包围盒管理器"""

    def __init__(self):
        """初始化"""
        self._boxes: Dict[str, BoundingBox] = {}
        self._transforms: Dict[str, np.ndarray] = {}
        self._link_sizes: Dict[str, List[float]] = {}

        # 基于URDF的默认尺寸 (单位: 米)
        self._default_sizes = {
            "base": [0.3, 0.3, 1.217],  # 基座
            "shoulder": [0.15, 0.15, 0.2],  # 肩部
            "upper_arm": [0.12, 0.12, 0.35],  # 大臂
            "forearm": [0.10, 0.10, 0.30],  # 小臂
            "wrist": [0.08, 0.08, 0.15],  # 腕部
            "end_effector": [0.05, 0.05, 0.10],  # 末端执行器
        }

    def set_link_size(self, link_name: str, size: List[float]) -> None:
        """
        设置连杆尺寸

        Args:
            link_name: 连杆名称
            size: [x, y, z] 尺寸
        """
        self._link_sizes[link_name] = size

    def update_link_transform(self, link_name: str, transform: np.ndarray) -> None:
        """
        更新连杆变换矩阵

        Args:
            link_name: 连杆名称
            transform: 4x4 变换矩阵
        """
        self._transforms[link_name] = transform

        # 更新包围盒
        if link_name in self._link_sizes:
            size = np.array(self._link_sizes[link_name])
            center = transform[:3, 3]

            # 计算包围盒 (考虑旋转)
            half_size = size / 2
            corners = np.array([
                [-half_size[0], -half_size[1], -half_size[2], 1],
                [half_size[0], -half_size[1], -half_size[2], 1],
                [-half_size[0], half_size[1], -half_size[2], 1],
                [half_size[0], half_size[1], -half_size[2], 1],
                [-half_size[0], -half_size[1], half_size[2], 1],
                [half_size[0], -half_size[1], half_size[2], 1],
                [-half_size[0], half_size[1], half_size[2], 1],
                [half_size[0], half_size[1], half_size[2], 1],
            ])

            # 变换角点
            transformed_corners = (transform @ corners.T).T[:, :3]

            # 更新包围盒
            self._boxes[link_name] = BoundingBox(
                min_corner=np.min(transformed_corners, axis=0).tolist(),
                max_corner=np.max(transformed_corners, axis=0).tolist()
            )

    def get_bounding_box(self, link_name: str) -> Optional[BoundingBox]:
        """获取连杆包围盒"""
        return self._boxes.get(link_name)

    def check_collision(
        self,
        link1: str,
        link2: str,
        padding: float = 0.01
    ) -> Tuple[bool, float]:
        """
        检查两个连杆是否碰撞

        Args:
            link1: 连杆1名称
            link2: 连杆2名称
            padding: 碰撞padding

        Returns:
            Tuple[是否碰撞, 距离]
        """
        box1 = self._boxes.get(link1)
        box2 = self._boxes.get(link2)

        if box1 is None or box2 is None:
            return False, float('inf')

        # 扩展包围盒
        box1_expanded = box1.expand(padding)
        box2_expanded = box2.expand(padding)

        # 检查相交
        if box1_expanded.intersect(box2_expanded):
            return True, 0.0

        # 计算距离
        distance = box1_expanded.distance_to(box2_expanded)
        return False, distance

    def get_all_bounding_boxes(self) -> Dict[str, BoundingBox]:
        """获取所有包围盒"""
        return self._boxes.copy()


class CollisionDetector:
    """
    碰撞检测器

    基于包围盒的碰撞检测实现。
    """

    def __init__(self, config: Optional[CollisionConfig] = None):
        """
        Args:
            config: 碰撞检测配置
        """
        self.config = config or CollisionConfig()
        self.bounding_boxes = LinkBoundingBoxes()

        # 碰撞对列表 (避免重复检测)
        self._collision_pairs: Set[Tuple[str, str]] = set()

        # 环境障碍物包围盒
        self._obstacles: Dict[str, BoundingBox] = {}

        # 碰撞历史
        self._collision_history: List[CollisionResult] = []
        self._max_history = 100

        # 初始化连杆包围盒
        self._init_link_bounding_boxes()

    def _init_link_bounding_boxes(self) -> None:
        """初始化连杆包围盒尺寸"""
        # 左臂
        self.bounding_boxes.set_link_size("Left_Shoulder_Pitch", [0.15, 0.15, 0.20])
        self.bounding_boxes.set_link_size("Left_Shoulder_Roll", [0.12, 0.12, 0.15])
        self.bounding_boxes.set_link_size("Left_Shoulder_Yaw", [0.10, 0.10, 0.12])
        self.bounding_boxes.set_link_size("Left_Elbow_Pitch", [0.10, 0.10, 0.30])
        self.bounding_boxes.set_link_size("Left_Wrist_Yaw", [0.08, 0.08, 0.10])
        self.bounding_boxes.set_link_size("Left_Wrist_Pitch", [0.08, 0.08, 0.10])
        self.bounding_boxes.set_link_size("Left_Wrist_Roll", [0.06, 0.06, 0.08])

        # 右臂
        self.bounding_boxes.set_link_size("Right_Shoulder_Pitch", [0.15, 0.15, 0.20])
        self.bounding_boxes.set_link_size("Right_Shoulder_Roll", [0.12, 0.12, 0.15])
        self.bounding_boxes.set_link_size("Right_Shoulder_Yaw", [0.10, 0.10, 0.12])
        self.bounding_boxes.set_link_size("Right_Elbow_Pitch", [0.10, 0.10, 0.30])
        self.bounding_boxes.set_link_size("Right_Wrist_Yaw", [0.08, 0.08, 0.10])
        self.bounding_boxes.set_link_size("Right_Wrist_Pitch", [0.08, 0.08, 0.10])
        self.bounding_boxes.set_link_size("Right_Wrist_Roll", [0.06, 0.06, 0.08])

        # 基座
        self.bounding_boxes.set_link_size("Base", [0.3, 0.3, 1.217])

        # 初始化碰撞对 (自碰撞)
        self._init_collision_pairs()

    def _init_collision_pairs(self) -> None:
        """初始化碰撞对"""
        left_links = [
            "Left_Shoulder_Pitch", "Left_Shoulder_Roll", "Left_Shoulder_Yaw",
            "Left_Elbow_Pitch", "Left_Wrist_Yaw", "Left_Wrist_Pitch", "Left_Wrist_Roll"
        ]
        right_links = [
            "Right_Shoulder_Pitch", "Right_Shoulder_Roll", "Right_Shoulder_Yaw",
            "Right_Elbow_Pitch", "Right_Wrist_Yaw", "Right_Wrist_Pitch", "Right_Wrist_Roll"
        ]

        # 左臂自碰撞对
        for i in range(len(left_links)):
            for j in range(i + 1, len(left_links)):
                self._collision_pairs.add((left_links[i], left_links[j]))

        # 右臂自碰撞对
        for i in range(len(right_links)):
            for j in range(i + 1, len(right_links)):
                self._collision_pairs.add((right_links[i], right_links[j]))

        # 双臂间碰撞对 (近距离连杆)
        adjacent_pairs = [
            ("Left_Shoulder_Pitch", "Right_Shoulder_Pitch"),
            ("Left_Shoulder_Roll", "Right_Shoulder_Roll"),
            ("Left_Elbow_Pitch", "Right_Elbow_Pitch"),
            ("Left_Wrist_Yaw", "Right_Wrist_Yaw"),
        ]
        self._collision_pairs.update(adjacent_pairs)

    def add_obstacle(
        self,
        name: str,
        bounding_box: BoundingBox,
        padding: float = 0.01
    ) -> None:
        """
        添加环境障碍物

        Args:
            name: 障碍物名称
            bounding_box: 障碍物包围盒
            padding: 安全padding
        """
        self._obstacles[name] = bounding_box.expand(padding)

    def remove_obstacle(self, name: str) -> None:
        """移除环境障碍物"""
        if name in self._obstacles:
            del self._obstacles[name]

    def clear_obstacles(self) -> None:
        """清除所有障碍物"""
        self._obstacles.clear()

    def update_link_transforms(self, transforms: Dict[str, np.ndarray]) -> None:
        """
        更新所有连杆变换矩阵

        Args:
            transforms: {连杆名称: 4x4变换矩阵}
        """
        for link_name, transform in transforms.items():
            self.bounding_boxes.update_link_transform(link_name, transform)

    def check_self_collision(self) -> Tuple[bool, List[Tuple[str, str]], float]:
        """
        检查自碰撞

        Returns:
            Tuple[是否对列表, 碰撞, 碰撞最小距离]
        """
        colliding_pairs = []
        min_distance = float('inf')

        for link1, link2 in self._collision_pairs:
            is_collision, distance = self.bounding_boxes.check_collision(
                link1, link2, self.config.padding
            )

            if is_collision:
                colliding_pairs.append((link1, link2))

            if distance < min_distance:
                min_distance = distance

        return len(colliding_pairs) > 0, colliding_pairs, min_distance

    def check_environment_collision(
        self,
        link_name: str
    ) -> Tuple[bool, List[str], float]:
        """
        检查与环境障碍物的碰撞

        Args:
            link_name: 连杆名称

        Returns:
            Tuple[是否碰撞, 碰撞障碍物列表, 最小距离]
        """
        colliding_obstacles = []
        min_distance = float('inf')

        link_box = self.bounding_boxes.get_bounding_box(link_name)

        if link_box is None:
            return False, [], float('inf')

        for obstacle_name, obstacle_box in self._obstacles.items():
            if link_box.intersect(obstacle_box):
                colliding_obstacles.append(obstacle_name)

            distance = link_box.distance_to(obstacle_box)
            if distance < min_distance:
                min_distance = distance

        return len(colliding_obstacles) > 0, colliding_obstacles, min_distance

    def check_collision(
        self,
        joint_positions: Dict[str, List[float]],
        transforms: Optional[Dict[str, np.ndarray]] = None
    ) -> CollisionResult:
        """
        综合碰撞检测

        Args:
            joint_positions: 关节角度 {"left": [...], "right": [...]}
            transforms: 预先计算的变换矩阵 (可选)

        Returns:
            CollisionResult: 碰撞检测结果
        """
        # 如果没有提供变换矩阵，尝试计算
        if transforms is None:
            transforms = self._compute_transforms(joint_positions)

        # 更新连杆变换
        self.update_link_transforms(transforms)

        # 检查自碰撞
        self_collision, self_pairs, self_distance = self.check_self_collision()

        # 检查环境碰撞
        env_collision = False
        env_pairs = []
        env_distance = float('inf')

        if self.config.enable_environment_collision:
            all_links = list(self.bounding_boxes._boxes.keys())

            for link in all_links:
                collision, obstacles, distance = self.check_environment_collision(link)

                if collision:
                    env_collision = True
                    for obs in obstacles:
                        env_pairs.append((link, obs))

                if distance < env_distance:
                    env_distance = distance

        # 综合结果
        collision = self_collision or env_collision

        if self_collision:
            collision_type = CollisionType.SELF_COLLISION
            colliding_links = self_pairs
        elif env_collision:
            collision_type = CollisionType.ENVIRONMENT_COLLISION
            colliding_links = env_pairs
        else:
            collision_type = CollisionType.NO_COLLISION
            colliding_links = []

        # 计算穿透深度
        penetration_depth = 0.0
        if collision:
            penetration_depth = min(self_distance, env_distance)

        # 保存历史
        result = CollisionResult(
            collision=collision,
            collision_type=collision_type,
            colliding_links=colliding_links,
            distance=min(self_distance, env_distance),
            penetration_depth=penetration_depth,
            timestamp=time.time()
        )

        self._collision_history.append(result)
        if len(self._collision_history) > self._max_history:
            self._collision_history.pop(0)

        return result

    def check_point_collision(
        self,
        point: List[float],
        padding: float = 0.01
    ) -> Tuple[bool, List[str], float]:
        """
        检查点是否与环境碰撞

        Args:
            point: [x, y, z] 坐标
            padding: 安全距离

        Returns:
            Tuple[是否碰撞, 碰撞障碍物列表, 最小距离]
        """
        point_array = np.array(point)
        colliding_obstacles = []
        min_distance = float('inf')

        for obstacle_name, obstacle_box in self._obstacles.items():
            # 扩展障碍物包围盒
            expanded = obstacle_box.expand(padding)

            if expanded.contains(point_array):
                colliding_obstacles.append(obstacle_name)
                distance = 0.0
            else:
                # 计算到各面的距离
                distance = 0.0
                for i in range(3):
                    if point_array[i] < expanded.min[i]:
                        distance += (expanded.min[i] - point_array[i]) ** 2
                    elif point_array[i] > expanded.max[i]:
                        distance += (point_array[i] - expanded.max[i]) ** 2

                distance = np.sqrt(distance)

            if distance < min_distance:
                min_distance = distance

        return len(colliding_obstacles) > 0, colliding_obstacles, min_distance

    def get_safety_distance(
        self,
        joint_positions: Dict[str, List[float]],
        transforms: Optional[Dict[str, np.ndarray]] = None
    ) -> float:
        """
        获取到最近碰撞的距离

        Args:
            joint_positions: 关节角度
            transforms: 变换矩阵

        Returns:
            float: 到最近碰撞的距离
        """
        result = self.check_collision(joint_positions, transforms)
        return result.distance

    def is_safe(
        self,
        joint_positions: Dict[str, List[float]],
        transforms: Optional[Dict[str, np.ndarray]] = None,
        safety_margin: float = 0.05
    ) -> bool:
        """
        检查是否安全 (无碰撞风险)

        Args:
            joint_positions: 关节角度
            transforms: 变换矩阵
            safety_margin: 安全距离

        Returns:
            bool: 是否安全
        """
        distance = self.get_safety_distance(joint_positions, transforms)
        return distance > safety_margin

    def get_collision_history(self) -> List[CollisionResult]:
        """获取碰撞历史"""
        return self._collision_history.copy()

    def clear_history(self) -> None:
        """清除碰撞历史"""
        self._collision_history.clear()

    def _compute_transforms(
        self,
        joint_positions: Dict[str, List[float]]
    ) -> Dict[str, np.ndarray]:
        """
        计算连杆变换矩阵 (简化版)

        Args:
            joint_positions: 关节角度

        Returns:
            Dict: {连杆名称: 4x4变换矩阵}
        """
        transforms = {}

        # 基座变换
        transforms["Base"] = np.eye(4)

        # 左臂变换 (简化计算)
        left_positions = joint_positions.get("left", [0.0] * 7)
        current_transform = np.eye(4)

        # 基座到肩部
        current_transform[:3, 3] = [0, 0, 1.217]  # 基座高度
        transforms["Left_Shoulder_Pitch"] = current_transform.copy()

        # 简化的正向运动学
        link_names = [
            "Left_Shoulder_Roll", "Left_Shoulder_Yaw", "Left_Elbow_Pitch",
            "Left_Wrist_Yaw", "Left_Wrist_Pitch", "Left_Wrist_Roll"
        ]

        for i, link_name in enumerate(link_names):
            if i < len(left_positions):
                angle = left_positions[i]

                # 简化的DH变换
                tz = 0.1  # 简化的连杆偏移
                rot_z = np.array([
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, tz],
                    [0, 0, 0, 1]
                ])

                current_transform = current_transform @ rot_z
                transforms[link_name] = current_transform.copy()

        # 右臂变换 (类似左臂)
        right_positions = joint_positions.get("right", [0.0] * 7)
        current_transform = np.eye(4)
        current_transform[:3, 3] = [0, 0, 1.217]
        transforms["Right_Shoulder_Pitch"] = current_transform.copy()

        link_names = [
            "Right_Shoulder_Roll", "Right_Shoulder_Yaw", "Right_Elbow_Pitch",
            "Right_Wrist_Yaw", "Right_Wrist_Pitch", "Right_Wrist_Roll"
        ]

        for i, link_name in enumerate(link_names):
            if i < len(right_positions):
                angle = right_positions[i]
                tz = 0.1
                rot_z = np.array([
                    [np.cos(angle), -np.sin(angle), 0, 0],
                    [np.sin(angle), np.cos(angle), 0, 0],
                    [0, 0, 1, tz],
                    [0, 0, 0, 1]
                ])
                current_transform = current_transform @ rot_z
                transforms[link_name] = current_transform.copy()

        return transforms

    def get_status(self) -> Dict[str, Any]:
        """获取碰撞检测器状态"""
        return {
            "config": {
                "enable_self_collision": self.config.enable_self_collision,
                "enable_environment_collision": self.config.enable_environment_collision,
                "padding": self.config.padding,
                "threshold": self.config.threshold
            },
            "obstacles_count": len(self._obstacles),
            "collision_pairs_count": len(self._collision_pairs),
            "links_count": len(self.bounding_boxes._boxes),
            "history_count": len(self._collision_history)
        }
