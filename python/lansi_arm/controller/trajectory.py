"""
轨迹规划模块

提供轨迹录制、插值和播放功能：
- 三次多项式插值
- 五次多项式插值
- 示教录制
- 轨迹平滑
"""


import time
import json
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Optional, Callable, Tuple, Any
from enum import Enum
from pathlib import Path
import numpy as np

from .group_controller import GroupController, Trajectory, TrajectoryPoint


class InterpolationType(Enum):
    """插值类型"""
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"
    B_SPLINE = "b_spline"


@dataclass
class TrajectoryConfig:
    """轨迹配置"""
    interpolation: InterpolationType = InterpolationType.QUINTIC
    velocity_limit: float = 1.0
    acceleration_limit: float = 1.0
    time_scaling: float = 1.0
    smoothing: bool = True
    smoothing_factor: float = 0.1


@dataclass
class Waypoint:
    """路径点"""
    positions: List[float]
    velocities: Optional[List[float]] = None
    accelerations: Optional[List[float]] = None
    time_from_start: float = 0.0
    name: str = ""


class TrajectoryInterpolator:
    """轨迹插值器"""

    @staticmethod
    def cubic_interpolate(
        p0: float, p1: float,
        v0: float, v1: float,
        t: float, T: float
    ) -> float:
        """
        三次多项式插值

        p(t) = a0 + a1*t + a2*t^2 + a3*t^3

        Args:
            p0, p1: 起点和终点位置
            v0, v1: 起点和终点速度
            t: 当前时间
            T: 总时间

        Returns:
            float: 插值位置
        """
        T = max(T, 1e-6)  # 避免除零
        t_normalized = t / T

        # 三次多项式系数
        a0 = p0
        a1 = v0 * T
        a2 = -3 * p0 + 3 * p1 - 2 * v0 * T - v1 * T
        a3 = 2 * p0 - 2 * p1 + v0 * T + v1 * T

        return a0 + a1 * t_normalized + a2 * t_normalized**2 + a3 * t_normalized**3

    @staticmethod
    def quintic_interpolate(
        p0: float, p1: float,
        v0: float, v1: float,
        a0: float, a1: float,
        t: float, T: float
    ) -> float:
        """
        五次多项式插值 (位置+速度+加速度连续)

        p(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

        Args:
            p0, p1: 起点和终点位置
            v0, v1: 起点和终点速度
            a0, a1: 起点和终点加速度
            t: 当前时间
            T: 总时间

        Returns:
            float: 插值位置
        """
        T = max(T, 1e-6)
        t_normalized = t / T

        # 五次多项式系数
        a0 = p0
        a1 = v0 * T
        a2 = 0.5 * a0 * T**2
        a3 = -10 * p0 + 10 * p1 - 6 * v0 * T - 4 * v1 * T - 0.5 * a0 * T**2 + 0.5 * a1 * T**2
        a4 = 15 * p0 - 15 * p1 + 8 * v0 * T + 7 * v1 * T + 1.5 * a0 * T**2 - a1 * T**2
        a5 = -6 * p0 + 6 * p1 - 4 * v0 * T - 3 * v1 * T - a0 * T**2 + 0.5 * a1 * T**2

        return (a0 + a1 * t_normalized +
                a2 * t_normalized**2 +
                a3 * t_normalized**3 +
                a4 * t_normalized**4 +
                a5 * t_normalized**5)

    @staticmethod
    def linear_interpolate(p0: float, p1: float, t: float, T: float) -> float:
        """线性插值"""
        T = max(T, 1e-6)
        return p0 + (p1 - p0) * (t / T)

    @classmethod
    def interpolate_trajectory(
        cls,
        waypoints: List[Waypoint],
        num_points: int,
        interpolation: InterpolationType = InterpolationType.QUINTIC
    ) -> List[Waypoint]:
        """
        对轨迹进行插值

        Args:
            waypoints: 关键路径点
            num_points: 输出点数
            interpolation: 插值类型

        Returns:
            List[Waypoint]: 插值后的路径点
        """
        if len(waypoints) < 2:
            return waypoints

        # 计算总时间
        total_time = waypoints[-1].time_from_start
        if total_time <= 0:
            total_time = len(waypoints) - 1  # 默认1秒每段

        # 生成时间点
        time_points = np.linspace(0, total_time, num_points)

        # 插值每个关节
        n_joints = len(waypoints[0].positions)
        result = []

        for t in time_points:
            # 找到当前时间段
            for i in range(len(waypoints) - 1):
                if (waypoints[i].time_from_start <= t <=
                    waypoints[i + 1].time_from_start):
                    # 时间段内的相对时间
                    t0 = waypoints[i].time_from_start
                    t1 = waypoints[i + 1].time_from_start
                    dt = t - t0
                    T = t1 - t0

                    if T < 1e-6:
                        segment_points = waypoints[i].positions
                        velocities = waypoints[i].velocities or [0.0] * n_joints
                        accelerations = waypoints[i].accelerations or [0.0] * n_joints
                    else:
                        # 插值每个关节
                        segment_points = []
                        velocities = []
                        accelerations = []

                        for j in range(n_joints):
                            p0 = waypoints[i].positions[j]
                            p1 = waypoints[i + 1].positions[j]

                            v0 = (waypoints[i].velocities[j]
                                  if waypoints[i].velocities else 0.0)
                            v1 = (waypoints[i + 1].velocities[j]
                                  if waypoints[i + 1].velocities else 0.0)

                            a0 = (waypoints[i].accelerations[j]
                                  if waypoints[i].accelerations else 0.0)
                            a1 = (waypoints[i + 1].accelerations[j]
                                  if waypoints[i + 1].accelerations else 0.0)

                            if interpolation == InterpolationType.QUINTIC:
                                p = cls.quintic_interpolate(p0, p1, v0, v1, a0, a1, dt, T)
                            elif interpolation == InterpolationType.CUBIC:
                                p = cls.cubic_interpolate(p0, p1, v0, v1, dt, T)
                            else:
                                p = cls.linear_interpolate(p0, p1, dt, T)

                            segment_points.append(p)
                            velocities.append(v0)
                            accelerations.append(a0)

                    result.append(Waypoint(
                        positions=segment_points,
                        velocities=velocities,
                        accelerations=accelerations,
                        time_from_start=t
                    ))
                    break

        return result


class TrajectoryRecorder:
    """
    轨迹录制器

    用于手动示教录制轨迹。
    """

    def __init__(self, group_controller: GroupController):
        """
        Args:
            group_controller: 群控器实例
        """
        self.group = group_controller
        self._recorded_points: List[Dict] = []
        self._is_recording = False
        self._start_time = 0.0

    def start_recording(self) -> None:
        """开始录制"""
        self._recorded_points = []
        self._is_recording = True
        self._start_time = time.time()

    def record_point(self, name: str = "") -> Dict:
        """
        录制当前点

        Args:
            name: 点名称

        Returns:
            Dict: 录制的数据
        """
        if not self._is_recording:
            raise RuntimeError("未开始录制")

        # 获取当前位置
        positions = self.group.get_joint_positions()

        point = {
            "name": name or f"point_{len(self._recorded_points)}",
            "timestamp": time.time() - self._start_time,
            "positions": positions
        }

        self._recorded_points.append(point)
        return point

    def stop_recording(self) -> Trajectory:
        """
        停止录制并生成轨迹

        Returns:
            Trajectory: 录制生成的轨迹
        """
        self._is_recording = False

        # 转换为TrajectoryPoint
        points = []
        for i, pt in enumerate(self._recorded_points):
            # 计算时间间隔
            if i == 0:
                delay = 1.0
            else:
                delay = (pt["timestamp"] -
                        self._recorded_points[i - 1]["timestamp"])

            points.append(TrajectoryPoint(
                name=pt["name"],
                timestamp=pt["timestamp"],
                delay=delay,
                positions={"left": pt["positions"]} if "left" in str(self.group.target) else {},
                right_positions={"right": pt["positions"]} if "right" in str(self.group.target) else {}
            ))

        return Trajectory(
            name=f"recorded_{int(time.time())}",
            points=points,
            loop=False
        )

    def clear(self) -> None:
        """清空录制数据"""
        self._recorded_points = []
        self._is_recording = False

    @property
    def is_recording(self) -> bool:
        """是否正在录制"""
        return self._is_recording

    @property
    def point_count(self) -> int:
        """已录制点数"""
        return len(self._recorded_points)


class TrajectoryPlayer:
    """
    轨迹播放器

    用于播放录制或生成的轨迹。
    """

    def __init__(self, group_controller: GroupController):
        """
        Args:
            group_controller: 群控器实例
        """
        self.group = group_controller
        self._is_playing = False
        self._current_trajectory: Optional[Trajectory] = None
        self._current_point_index = 0
        self._speed = 1.0
        self._loop = False
        self._on_progress: Optional[Callable] = None
        self._stop_requested = False

    def play(
        self,
        trajectory: Trajectory,
        speed: float = 1.0,
        loop: bool = False,
        on_progress: Optional[Callable] = None
    ) -> None:
        """
        播放轨迹

        Args:
            trajectory: 要播放的轨迹
            speed: 播放速度
            loop: 是否循环
            on_progress: 进度回调
        """
        self._current_trajectory = trajectory
        self._speed = speed
        self._loop = loop
        self._on_progress = on_progress
        self._current_point_index = 0
        self._stop_requested = False
        self._is_playing = True

        try:
            # 遍历轨迹点
            while self._is_playing and self._current_point_index < len(trajectory.points):
                # 检查停止请求
                if self._stop_requested:
                    break

                point = trajectory.points[self._current_point_index]

                # 移动到该点
                self._execute_point(point)

                # 回调进度
                if self._on_progress:
                    progress = {
                        "current": self._current_point_index + 1,
                        "total": len(trajectory.points),
                        "point_name": point.name
                    }
                    self._on_progress(progress)

                # 等待延时
                adjusted_delay = point.delay / speed
                if adjusted_delay > 0:
                    # 分段等待，支持停止
                    sleep_step = 0.05
                    for _ in np.arange(0, adjusted_delay, sleep_step):
                        if self._stop_requested:
                            break
                        time.sleep(sleep_step)

                # 下一帧
                self._current_point_index += 1

                # 检查循环
                if (self._current_point_index >= len(trajectory.points) and
                    self._loop and not self._stop_requested):
                    self._current_point_index = 0

        finally:
            if not self._loop:
                self._is_playing = False

    def stop(self) -> None:
        """停止播放"""
        self._stop_requested = True

    def pause(self) -> None:
        """暂停播放"""
        self._is_playing = False

    def resume(self, speed: float = 1.0) -> None:
        """继续播放"""
        if self._current_trajectory and self._current_point_index < len(self._current_trajectory.points):
            self._is_playing = True
            self._speed = speed
            self._stop_requested = False
            self.play(self._current_trajectory, speed, self._loop, self._on_progress)

    def _execute_point(self, point: TrajectoryPoint) -> None:
        """执行单个轨迹点"""
        positions = {}

        if hasattr(point, 'positions') and point.positions:
            if "left" in point.positions:
                positions["left"] = point.positions["left"]
            if "right" in point.positions:
                positions["right"] = point.positions["right"]

        if positions:
            self.group.set_joint_positions(positions, speed=self._speed)


class TrajectoryPlanner:
    """
    轨迹规划器

    提供高级轨迹规划功能：
    - 点到点规划
    - 直线规划
    - 圆弧规划
    """

    def __init__(self, group_controller: Optional[GroupController] = None):
        """
        Args:
            group_controller: 群控器实例
        """
        self.group = group_controller
        self.interpolator = TrajectoryInterpolator()

    def plan_point_to_point(
        self,
        start_positions: List[float],
        end_positions: List[float],
        duration: float,
        num_points: int = 100,
        interpolation: InterpolationType = InterpolationType.QUINTIC
    ) -> List[Waypoint]:
        """
        点到点轨迹规划

        Args:
            start_positions: 起始位置
            end_positions: 目标位置
            duration: 运动时间 (秒)
            num_points: 轨迹点数
            interpolation: 插值类型

        Returns:
            List[Waypoint]: 规划轨迹
        """
        waypoints = [
            Waypoint(positions=start_positions, time_from_start=0.0),
            Waypoint(positions=end_positions, time_from_start=duration)
        ]

        return self.interpolator.interpolate_trajectory(
            waypoints, num_points, interpolation
        )

    def plan_linear(
        self,
        start_pose: Dict[str, List[float]],
        end_pose: Dict[str, List[float]],
        duration: float,
        num_points: int = 100
    ) -> Trajectory:
        """
        直线轨迹规划

        Args:
            start_pose: {"left": [...], "right": [...]}
            end_pose: {"left": [...], "right": [...]}
            duration: 运动时间
            num_points: 轨迹点数

        Returns:
            Trajectory: 规划轨迹
        """
        # 为每臂分别规划
        left_trajectory = None
        right_trajectory = None

        if "left" in start_pose and "left" in end_pose:
            waypoints = [
                Waypoint(positions=start_pose["left"], time_from_start=0.0),
                Waypoint(positions=end_pose["left"], time_from_start=duration)
            ]
            points = self.interpolator.interpolate_trajectory(
                waypoints, num_points, InterpolationType.QUINTIC
            )
            left_trajectory = Trajectory(name="left_linear", points=points)

        if "right" in start_pose and "right" in end_pose:
            waypoints = [
                Waypoint(positions=start_pose["right"], time_from_start=0.0),
                Waypoint(positions=end_pose["right"], time_from_start=duration)
            ]
            points = self.interpolator.interpolate_trajectory(
                waypoints, num_points, InterpolationType.QUINTIC
            )
            right_trajectory = Trajectory(name="right_linear", points=points)

        # 合并轨迹
        trajectory = Trajectory(
            name="linear",
            points=[],
            loop=False
        )

        # 同步左右臂轨迹点
        max_points = max(
            len(left_trajectory.points) if left_trajectory else 0,
            len(right_trajectory.points) if right_trajectory else 0
        )

        for i in range(max_points):
            positions = {}

            if left_trajectory and i < len(left_trajectory.points):
                positions["left"] = left_trajectory.points[i].positions

            if right_trajectory and i < len(right_trajectory.points):
                positions["right"] = right_trajectory.points[i].positions

            trajectory.points.append(TrajectoryPoint(
                positions=positions,
                time_from_start=i * duration / max_points,
                delay=duration / max_points
            ))

        return trajectory

    def smooth_trajectory(
        self,
        trajectory: Trajectory,
        smoothing_factor: float = 0.1,
        iterations: int = 3
    ) -> Trajectory:
        """
        轨迹平滑

        Args:
            trajectory: 原始轨迹
            smoothing_factor: 平滑因子 (0.0-1.0)
            iterations: 迭代次数

        Returns:
            Trajectory: 平滑后的轨迹
        """
        if len(trajectory.points) < 3:
            return trajectory

        # 转换为Waypoint列表
        waypoints = []
        for i, pt in enumerate(trajectory.points):
            positions = (pt.positions.get("left") or
                        pt.positions.get("right") or
                        pt.positions.get("", []))
            waypoints.append(Waypoint(
                positions=positions,
                time_from_start=pt.timestamp or i * pt.delay
            ))

        # 平滑处理
        for _ in range(iterations):
            for i in range(1, len(waypoints) - 1):
                # 移动平均
                for j in range(len(waypoints[i].positions)):
                    waypoints[i].positions[j] = (
                        smoothing_factor * waypoints[i - 1].positions[j] +
                        (1 - 2 * smoothing_factor) * waypoints[i].positions[j] +
                        smoothing_factor * waypoints[i + 1].positions[j]
                    )

        # 转换回Trajectory
        points = []
        for i, wp in enumerate(waypoints):
            positions = {"left": wp.positions} if "left" in str(trajectory) else {}
            points.append(TrajectoryPoint(
                name=wp.name or f"point_{i}",
                positions=positions,
                delay=waypoints[i + 1].time_from_start - wp.time_from_start if i < len(waypoints) - 1 else 1.0,
                timestamp=wp.time_from_start
            ))

        return Trajectory(
            name=f"{trajectory.name}_smoothed",
            points=points,
            loop=trajectory.loop,
            description=trajectory.description
        )

    def execute_trajectory(
        self,
        trajectory: Trajectory,
        speed: float = 1.0,
        loop: bool = False,
        on_progress: Optional[Callable] = None
    ) -> None:
        """
        执行轨迹 (需要群控器)

        Args:
            trajectory: 要执行的轨迹
            speed: 速度
            loop: 是否循环
            on_progress: 进度回调
        """
        if self.group is None:
            raise ValueError("未提供群控器实例")

        player = TrajectoryPlayer(self.group)
        player.play(trajectory, speed, loop, on_progress)


class TrajectoryManager:
    """
    轨迹管理器

    提供轨迹的保存、加载和管理功能。
    """

    def __init__(self, save_dir: str = "./trajectories"):
        """
        Args:
            save_dir: 保存目录
        """
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)

    def save(self, trajectory: Trajectory, filename: Optional[str] = None) -> str:
        """
        保存轨迹到文件

        Args:
            trajectory: 要保存的轨迹
            filename: 文件名 (可选)

        Returns:
            str: 保存的文件路径
        """
        filename = filename or f"{trajectory.name}.json"
        filepath = self.save_dir / filename

        # 转换为字典
        data = {
            "name": trajectory.name,
            "description": trajectory.description,
            "loop": trajectory.loop,
            "speed_multiplier": trajectory.speed_multiplier,
            "points": [
                {
                    "name": pt.name,
                    "timestamp": pt.timestamp,
                    "delay": pt.delay,
                    "positions": pt.positions
                }
                for pt in trajectory.points
            ]
        }

        with open(filepath, 'w', encoding='utf-8') as f:
            json.dump(data, f, ensure_ascii=False, indent=2)

        return str(filepath)

    def load(self, filename: str) -> Optional[Trajectory]:
        """
        从文件加载轨迹

        Args:
            filename: 文件名或路径

        Returns:
            Trajectory: 加载的轨迹，失败返回None
        """
        filepath = Path(filename)
        if not filepath.exists():
            filepath = self.save_dir / filename

        if not filepath.exists():
            return None

        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                data = json.load(f)

            points = []
            for pt_data in data.get("points", []):
                points.append(TrajectoryPoint(
                    name=pt_data.get("name", ""),
                    timestamp=pt_data.get("timestamp", 0.0),
                    delay=pt_data.get("delay", 1.0),
                    positions=pt_data.get("positions", {})
                ))

            return Trajectory(
                name=data.get("name", filepath.stem),
                description=data.get("description", ""),
                points=points,
                loop=data.get("loop", False),
                speed_multiplier=data.get("speed_multiplier", 1.0)
            )

        except Exception:
            return None

    def list(self) -> List[Dict]:
        """
        列出所有保存的轨迹

        Returns:
            List[Dict]: 轨迹信息列表
        """
        trajectories = []

        for filepath in self.save_dir.glob("*.json"):
            trajectory = self.load(filepath.name)
            if trajectory:
                trajectories.append({
                    "filename": filepath.name,
                    "name": trajectory.name,
                    "points_count": len(trajectory.points),
                    "loop": trajectory.loop
                })

        return trajectories

    def delete(self, filename: str) -> bool:
        """
        删除轨迹文件

        Args:
            filename: 文件名

        Returns:
            bool: 是否删除成功
        """
        filepath = Path(filename)
        if not filepath.exists():
            filepath = self.save_dir / filename

        if filepath.exists():
            filepath.unlink()
            return True
        return False
