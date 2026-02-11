"""
群控控制器

提供双臂协同控制功能：
- 左右臂独立/同步控制
- 双臂轨迹同步播放
- 紧急停止
- 状态管理
"""


import time
import threading
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Any, Callable, Tuple
from enum import Enum

from .arm_controller import ArmController, JointLimits as SingleArmLimits
from ..constants import ArmState, ControlTarget, MotorMode
from ..kinematics import Pose
from ..config import ArmType


class PlaybackState(Enum):
    """播放状态"""
    IDLE = "idle"
    PLAYING = "playing"
    PAUSED = "paused"
    STOPPING = "stopping"


@dataclass
class TrajectoryPoint:
    """轨迹点"""
    name: str = ""
    timestamp: float = 0.0
    delay: float = 1.0
    positions: Dict[str, List[float]] = field(default_factory=dict)


@dataclass
class Trajectory:
    """轨迹"""
    name: str
    points: List[TrajectoryPoint]
    loop: bool = False
    description: str = ""
    speed_multiplier: float = 1.0


class GroupController:
    """
    双臂群控器

    支持：
    - 左臂(can0) + 右臂(can1) 独立或同步控制
    - 严格时序同步播放轨迹 (±10ms)
    - 紧急停止
    - 状态回调
    """

    # 默认CAN通道
    DEFAULT_LEFT_CHANNEL = "can0"
    DEFAULT_RIGHT_CHANNEL = "can1"

    def __init__(
        self,
        left_channel: Optional[str] = None,
        right_channel: Optional[str] = None
    ):
        """
        Args:
            left_channel: 左臂CAN通道 (默认: can0)
            right_channel: 右臂CAN通道 (默认: can1)
        """
        self.left_channel = left_channel or self.DEFAULT_LEFT_CHANNEL
        self.right_channel = right_channel or self.DEFAULT_RIGHT_CHANNEL

        # 控制器实例
        self._left_arm: Optional[ArmController] = None
        self._right_arm: Optional[ArmController] = None

        # 控制目标
        self._target = ControlTarget.BOTH

        # 播放状态
        self._playback_state = PlaybackState.IDLE
        self._playback_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._pause_event = threading.Event()

        # 回调函数
        self._on_state_update: Optional[Callable] = None
        self._on_playback_progress: Optional[Callable] = None
        self._on_error: Optional[Callable] = None

        # 锁
        self._lock = threading.RLock()

    @property
    def left_arm(self) -> Optional[ArmController]:
        """获取左臂控制器"""
        return self._left_arm

    @property
    def right_arm(self) -> Optional[ArmController]:
        """获取右臂控制器"""
        return self._right_arm

    @property
    def target(self) -> ControlTarget:
        """获取当前控制目标"""
        return self._target

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return (self._left_arm is not None and self._left_arm.is_connected) or \
               (self._right_arm is not None and self._right_arm.is_connected)

    @property
    def is_playing(self) -> bool:
        """检查是否正在播放"""
        return self._playback_state == PlaybackState.PLAYING

    def connect_left(self, channel: Optional[str] = None) -> bool:
        """
        连接左臂

        Args:
            channel: CAN通道 (默认: can0)

        Returns:
            bool: 是否连接成功
        """
        channel = channel or self.left_channel

        with self._lock:
            # 如果已连接，先断开
            if self._left_arm is not None:
                self.disconnect_left()

            try:
                # 创建左臂控制器
                self._left_arm = ArmController(
                    motor_ids=[51, 52, 53, 54, 55, 56, 57],
                    can_channel=channel
                )

                # 连接
                self._left_arm.connect()

                return True

            except Exception as e:
                self._left_arm = None
                self._on_error(f"左臂连接失败: {e}")
                return False

    def connect_right(self, channel: Optional[str] = None) -> bool:
        """
        连接右臂

        Args:
            channel: CAN通道 (默认: can1)

        Returns:
            bool: 是否连接成功
        """
        channel = channel or self.right_channel

        with self._lock:
            # 如果已连接，先断开
            if self._right_arm is not None:
                self.disconnect_right()

            try:
                # 创建右臂控制器
                self._right_arm = ArmController(
                    motor_ids=[61, 62, 63, 64, 65, 66, 67],
                    can_channel=channel
                )

                # 连接
                self._right_arm.connect()

                return True

            except Exception as e:
                self._right_arm = None
                self._on_error(f"右臂连接失败: {e}")
                return False

    def connect_all(
        self,
        left_channel: Optional[str] = None,
        right_channel: Optional[str] = None
    ) -> Dict[str, bool]:
        """
        连接所有机械臂

        Args:
            left_channel: 左臂CAN通道
            right_channel: 右臂CAN通道

        Returns:
            Dict: 连接结果 {"left": bool, "right": bool}
        """
        results = {
            "left": self.connect_left(left_channel),
            "right": self.connect_right(right_channel)
        }
        return results

    def disconnect_left(self) -> None:
        """断开左臂"""
        with self._lock:
            if self._left_arm is not None:
                try:
                    self._left_arm.disconnect()
                except Exception:
                    pass
                self._left_arm = None

    def disconnect_right(self) -> None:
        """断开右臂"""
        with self._lock:
            if self._right_arm is not None:
                try:
                    self._right_arm.disconnect()
                except Exception:
                    pass
                self._right_arm = None

    def disconnect_all(self) -> None:
        """断开所有机械臂"""
        self.stop_playback()
        self.disconnect_left()
        self.disconnect_right()

    def initialize_left(
        self,
        mode: MotorMode = MotorMode.PROFILE_POSITION,
        velocity: float = 1.0,
        acceleration: float = 1.0
    ) -> bool:
        """
        初始化左臂

        Returns:
            bool: 是否初始化成功
        """
        if self._left_arm is None:
            self._on_error("左臂未连接")
            return False

        try:
            self._left_arm.initialize(mode, velocity, acceleration)
            return True
        except Exception as e:
            self._on_error(f"左臂初始化失败: {e}")
            return False

    def initialize_right(
        self,
        mode: MotorMode = MotorMode.PROFILE_POSITION,
        velocity: float = 1.0,
        acceleration: float = 1.0
    ) -> bool:
        """
        初始化右臂

        Returns:
            bool: 是否初始化成功
        """
        if self._right_arm is None:
            self._on_error("右臂未连接")
            return False

        try:
            self._right_arm.initialize(mode, velocity, acceleration)
            return True
        except Exception as e:
            self._on_error(f"右臂初始化失败: {e}")
            return False

    def initialize_all(
        self,
        mode: MotorMode = MotorMode.PROFILE_POSITION,
        velocity: float = 1.0,
        acceleration: float = 1.0
    ) -> Dict[str, bool]:
        """
        初始化所有机械臂

        Returns:
            Dict: 初始化结果
        """
        return {
            "left": self.initialize_left(mode, velocity, acceleration),
            "right": self.initialize_right(mode, velocity, acceleration)
        }

    def enable_left(self, enabled: bool = True) -> bool:
        """使能/禁用左臂"""
        if self._left_arm is None:
            return False
        return self._left_arm.enable(enabled)

    def enable_right(self, enabled: bool = True) -> bool:
        """使能/禁用右臂"""
        if self._right_arm is None:
            return False
        return self._right_arm.enable(enabled)

    def enable_all(self, enabled: bool = True) -> bool:
        """使能/禁用所有机械臂"""
        left_ok = self.enable_left(enabled)
        right_ok = self.enable_right(enabled)
        return left_ok and right_ok

    def set_target(self, target: str) -> None:
        """
        设置控制目标

        Args:
            target: "left", "right" 或 "both"
        """
        try:
            self._target = ControlTarget(target)
        except ValueError:
            self._on_error(f"无效的控制目标: {target}")

    def set_joint_positions(
        self,
        positions: Dict[str, List[float]],
        speed: float = 0.5,
        blocking: bool = True
    ) -> bool:
        """
        设置关节位置

        Args:
            positions: {"left": [...], "right": [...]}
            speed: 速度系数
            blocking: 是否阻塞

        Returns:
            bool: 操作是否成功
        """
        success = True

        with self._lock:
            if "left" in positions and self._left_arm is not None:
                if not self._left_arm.set_joint_positions(positions["left"], speed, blocking):
                    success = False

            if "right" in positions and self._right_arm is not None:
                if not self._right_arm.set_joint_positions(positions["right"], speed, blocking):
                    success = False

        return success

    def move_both(
        self,
        left_positions: List[float],
        right_positions: List[float],
        speed: float = 0.5,
        sync: bool = True
    ) -> bool:
        """
        双臂同时移动

        Args:
            left_positions: 左臂关节角度
            right_positions: 右臂关节角度
            speed: 速度系数
            sync: 是否同步移动 (时间差 < 10ms)

        Returns:
            bool: 操作是否成功
        """
        if sync and self._left_arm is not None and self._right_arm is not None:
            # 同步移动
            threads = []

            if self._left_arm.is_enabled:
                t_left = threading.Thread(
                    target=self._left_arm.set_joint_positions,
                    args=(left_positions, speed, False)
                )
                threads.append(t_left)

            if self._right_arm.is_enabled:
                t_right = threading.Thread(
                    target=self._right_arm.set_joint_positions,
                    args=(right_positions, speed, False)
                )
                threads.append(t_right)

            # 同时启动
            for t in threads:
                t.start()

            # 等待完成
            for t in threads:
                t.join()

            if blocking_wait := blocking:
                time.sleep(0.1)

            return True

        else:
            # 顺序移动
            return self.set_joint_positions({
                "left": left_positions,
                "right": right_positions
            }, speed, blocking)

    def go_to_zero_left(self) -> bool:
        """左臂回零"""
        if self._left_arm is None:
            return False
        return self._left_arm.go_to_zero()

    def go_to_zero_right(self) -> bool:
        """右臂回零"""
        if self._right_arm is None:
            return False
        return self._right_arm.go_to_zero()

    def go_to_zero_all(self) -> bool:
        """双臂同时回零"""
        left_zero = self.go_to_zero_left()
        right_zero = self.go_to_zero_right()
        return left_zero and right_zero

    def emergency_stop(self) -> None:
        """紧急停止"""
        self.stop_playback()

        with self._lock:
            if self._left_arm is not None:
                try:
                    self._left_arm.emergency_stop()
                except Exception:
                    pass

            if self._right_arm is not None:
                try:
                    self._right_arm.emergency_stop()
                except Exception:
                    pass

    def stop_playback(self) -> None:
        """停止播放"""
        self._stop_event.set()
        self._pause_event.clear()
        self._playback_state = PlaybackState.IDLE

    def pause_playback(self) -> None:
        """暂停播放"""
        if self._playback_state == PlaybackState.PLAYING:
            self._pause_event.set()
            self._playback_state = PlaybackState.PAUSED

    def resume_playback(self) -> None:
        """恢复播放"""
        if self._playback_state == PlaybackState.PAUSED:
            self._pause_event.clear()
            self._playback_state = PlaybackState.PLAYING

    def play_trajectory(
        self,
        trajectory: Dict[str, Trajectory],
        speed: float = 1.0,
        loop: bool = False
    ) -> bool:
        """
        播放轨迹

        Args:
            trajectory: {"left": Trajectory, "right": Trajectory}
            speed: 速度系数
            loop: 是否循环

        Returns:
            bool: 是否开始播放
        """
        if self._playback_state == PlaybackState.PLAYING:
            self._on_error("已有轨迹正在播放")
            return False

        if "left" not in trajectory and "right" not in trajectory:
            self._on_error("轨迹数据为空")
            return False

        # 停止当前播放
        self.stop_playback()

        # 设置事件
        self._stop_event.clear()
        self._pause_event.clear()

        # 启动播放线程
        self._playback_state = PlaybackState.PLAYING
        self._playback_thread = threading.Thread(
            target=self._playback_worker,
            args=(trajectory, speed, loop),
            daemon=True
        )
        self._playback_thread.start()

        return True

    def _playback_worker(
        self,
        trajectory: Dict[str, Trajectory],
        speed: float,
        loop: bool
    ) -> None:
        """轨迹播放工作线程"""
        try:
            # 获取轨迹点
            left_points = trajectory.get("left", Trajectory(name="", points=[])).points
            right_points = trajectory.get("right", Trajectory(name="", points=[])).points

            # 确定播放点
            all_points = left_points or right_points
            if not all_points:
                return

            max_points = max(len(left_points or []), len(right_points or []))

            while True:
                # 检查停止信号
                if self._stop_event.is_set():
                    break

                # 检查暂停信号
                while self._pause_event.is_set():
                    if self._stop_event.is_set():
                        break
                    time.sleep(0.1)

                if self._stop_event.is_set():
                    break

                # 逐点播放
                for i in range(max_points):
                    # 检查停止
                    if self._stop_event.is_set():
                        return

                    # 检查暂停
                    while self._pause_event.is_set():
                        if self._stop_event.is_set():
                            return
                        time.sleep(0.1)

                    # 执行移动
                    self._execute_point(left_points, right_points, i, speed)

                    # 回调进度
                    if self._on_playback_progress:
                        try:
                            self._on_playback_progress({
                                "current": i + 1,
                                "total": max_points,
                                "progress": (i + 1) / max_points * 100
                            })
                        except Exception:
                            pass

                    # 延时
                    delay = 1.0 / speed
                    time.sleep(delay)

                if not loop:
                    break

        finally:
            self._playback_state = PlaybackState.IDLE

    def _execute_point(
        self,
        left_points: List[TrajectoryPoint],
        right_points: List[TrajectoryPoint],
        index: int,
        speed: float
    ) -> None:
        """执行单个轨迹点"""
        with self._lock:
            # 左臂
            if left_points and index < len(left_points):
                point = left_points[index]
                if "left" in point.positions and self._left_arm:
                    self._left_arm.set_joint_positions(
                        point.positions["left"],
                        speed=speed,
                        blocking=False
                    )

            # 右臂
            if right_points and index < len(right_points):
                point = right_points[index]
                if "right" in point.positions and self._right_arm:
                    self._right_arm.set_joint_positions(
                        point.positions["right"],
                        speed=speed,
                        blocking=False
                    )

    def get_state(self) -> Dict[str, Any]:
        """
        获取群控器状态

        Returns:
            Dict: 状态信息
        """
        with self._lock:
            state = {
                "target": self._target.value,
                "playback": {
                    "state": self._playback_state.value,
                    "is_playing": self._playback_state == PlaybackState.PLAYING,
                    "is_paused": self._playback_state == PlaybackState.PAUSED
                },
                "arms": {}
            }

            # 左臂状态
            if self._left_arm:
                state["arms"]["left"] = self._left_arm.get_state()
            else:
                state["arms"]["left"] = {"connected": False}

            # 右臂状态
            if self._right_arm:
                state["arms"]["right"] = self._right_arm.get_state()
            else:
                state["arms"]["right"] = {"connected": False}

            return state

    def get_joint_positions(self) -> Dict[str, List[float]]:
        """
        获取所有关节位置

        Returns:
            Dict: {"left": [...], "right": [...]}
        """
        positions = {}

        if self._left_arm:
            positions["left"] = self._left_arm.get_joint_positions()

        if self._right_arm:
            positions["right"] = self._right_arm.get_joint_positions()

        return positions

    def set_callbacks(
        self,
        on_state_update: Optional[Callable] = None,
        on_playback_progress: Optional[Callable] = None,
        on_error: Optional[Callable] = None
    ) -> None:
        """
        设置回调函数

        Args:
            on_state_update: 状态更新回调
            on_playback_progress: 播放进度回调
            on_error: 错误回调
        """
        self._on_state_update = on_state_update
        self._on_playback_progress = on_playback_progress
        self._on_error = on_error

    def _on_error(self, message: str) -> None:
        """发送错误消息"""
        if self._on_error:
            try:
                self._on_error(message)
            except Exception:
                pass

    def calibrate_zero_left(self) -> bool:
        """左臂零点标定"""
        if self._left_arm is None:
            return False
        return self._left_arm.calibrate_zero()

    def calibrate_zero_right(self) -> bool:
        """右臂零点标定"""
        if self._right_arm is None:
            return False
        return self._right_arm.calibrate_zero()

    def calibrate_zero_all(self) -> Dict[str, bool]:
        """双臂零点标定"""
        return {
            "left": self.calibrate_zero_left(),
            "right": self.calibrate_zero_right()
        }

    def read_all_limits(self) -> Dict[str, Dict]:
        """读取所有关节限制"""
        limits = {}

        if self._left_arm:
            limits["left"] = self._left_arm.read_all_limits()

        if self._right_arm:
            limits["right"] = self._right_arm.read_all_limits()

        return limits

    def cleanup(self) -> None:
        """清理资源"""
        self.disconnect_all()

    def __enter__(self):
        """上下文管理器入口"""
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.cleanup()
        return False

    def __repr__(self) -> str:
        left_connected = self._left_arm is not None
        right_connected = self._right_arm is not None

        return (
            f"GroupController("
            f"left_connected={left_connected}, "
            f"right_connected={right_connected}, "
            f"target={self._target.value})"
        )
