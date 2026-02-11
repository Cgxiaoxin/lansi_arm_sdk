"""
测试轨迹规划模块
"""
import pytest
import numpy as np
from lansi_arm.controller import (
    Trajectory,
    TrajectoryPoint,
    TrajectoryConfig,
    Waypoint,
    InterpolationType,
)


class TestTrajectoryPoint:
    """轨迹点测试类"""

    def test_trajectory_point_creation(self):
        """测试轨迹点创建"""
        point = TrajectoryPoint(
            positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            velocities=[0.0] * 7,
            accelerations=[0.0] * 7,
            time=1.0
        )
        assert len(point.positions) == 7
        assert len(point.velocities) == 7
        assert point.time == 1.0

    def test_trajectory_point_default_values(self):
        """测试轨迹点默认值"""
        point = TrajectoryPoint(
            positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7]
        )
        assert point.velocities == [0.0] * 7
        assert point.accelerations == [0.0] * 7
        assert point.time == 0.0


class TestTrajectory:
    """轨迹测试类"""

    def test_trajectory_creation(self):
        """测试轨迹创建"""
        trajectory = Trajectory()
        assert len(trajectory.points) == 0

    def test_trajectory_add_point(self):
        """测试添加轨迹点"""
        trajectory = Trajectory()
        point = TrajectoryPoint(
            positions=[0.1] * 7,
            time=0.0
        )
        trajectory.add_point(point)
        assert len(trajectory.points) == 1

    def test_trajectory_add_waypoints(self):
        """测试添加路点"""
        trajectory = Trajectory()
        waypoints = [
            Waypoint(positions=[0.0] * 7, time=0.0),
            Waypoint(positions=[0.1] * 7, time=1.0),
            Waypoint(positions=[0.2] * 7, time=2.0),
        ]
        trajectory.add_waypoints(waypoints)
        assert len(trajectory.points) == 3

    def test_trajectory_duration(self):
        """测试轨迹持续时间"""
        trajectory = Trajectory()
        trajectory.add_point(TrajectoryPoint([0.0] * 7, time=0.0))
        trajectory.add_point(TrajectoryPoint([0.1] * 7, time=2.0))
        trajectory.add_point(TrajectoryPoint([0.2] * 7, time=5.0))
        assert abs(trajectory.duration - 5.0) < 1e-6

    def test_trajectory_empty_duration(self):
        """测试空轨迹持续时间"""
        trajectory = Trajectory()
        assert trajectory.duration == 0.0


class TestWaypoint:
    """路点测试类"""

    def test_waypoint_creation(self):
        """测试路点创建"""
        waypoint = Waypoint(
            positions=[0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
            time=1.0
        )
        assert len(waypoint.positions) == 7
        assert waypoint.time == 1.0

    def test_waypoint_default_time(self):
        """测试路点默认时间"""
        waypoint = Waypoint(positions=[0.1] * 7)
        assert waypoint.time == 0.0


class TestTrajectoryConfig:
    """轨迹配置测试类"""

    def test_default_config(self):
        """测试默认配置"""
        config = TrajectoryConfig()
        assert config.interpolation_type == InterpolationType.CUBIC
        assert config.velocity == 0.5
        assert config.acceleration == 0.5
        assert config.smoothing == 0.0

    def test_custom_config(self):
        """测试自定义配置"""
        config = TrajectoryConfig(
            interpolation_type=InterpolationType.QUINTIC,
            velocity=1.0,
            acceleration=0.8,
            smoothing=0.1
        )
        assert config.interpolation_type == InterpolationType.QUINTIC
        assert config.velocity == 1.0


class TestInterpolationType:
    """插值类型测试类"""

    def test_interpolation_type_values(self):
        """测试插值类型值"""
        assert InterpolationType.LINEAR.value == "linear"
        assert InterpolationType.CUBIC.value == "cubic"
        assert InterpolationType.QUINTIC.value == "quintic"
        assert InterpolationType.B_SPLINE.value == "b_spline"

    def test_interpolation_type_count(self):
        """测试插值类型数量"""
        assert len(InterpolationType) == 4


class TestTrajectoryInterpolator:
    """轨迹插值器测试类"""

    def test_linear_interpolation(self):
        """测试线性插值"""
        from lansi_arm.controller import TrajectoryInterpolator

        interpolator = TrajectoryInterpolator(
            interpolation_type=InterpolationType.LINEAR
        )

        start = [0.0, 0.0, 0.0]
        end = [1.0, 1.0, 1.0]
        t = 0.5

        result = interpolator.interpolate(start, end, t)

        assert len(result) == 3
        assert abs(result[0] - 0.5) < 1e-6
        assert abs(result[1] - 0.5) < 1e-6
        assert abs(result[2] - 0.5) < 1e-6

    def test_cubic_interpolation(self):
        """测试三次插值"""
        from lansi_arm.controller import TrajectoryInterpolator

        interpolator = TrajectoryInterpolator(
            interpolation_type=InterpolationType.CUBIC
        )

        start = [0.0, 0.0, 0.0]
        end = [1.0, 1.0, 1.0]
        t = 0.5

        result = interpolator.interpolate(start, end, t)

        assert len(result) == 3
        # 三次插值在中间点应该接近0.5
        assert 0.0 <= result[0] <= 1.0

    def test_interpolation_t_bounds(self):
        """测试插值参数范围"""
        from lansi_arm.controller import TrajectoryInterpolator

        interpolator = TrajectoryInterpolator()

        start = [0.0]
        end = [1.0]

        # t = 0
        assert interpolator.interpolate(start, end, 0.0)[0] == 0.0

        # t = 1
        assert interpolator.interpolate(start, end, 1.0)[0] == 1.0


class TestTrajectoryPlanner:
    """轨迹规划器测试类"""

    def test_planner_creation(self):
        """测试规划器创建"""
        from lansi_arm.controller import TrajectoryPlanner

        planner = TrajectoryPlanner()
        assert planner is not None

    def test_plan_joint_trajectory(self):
        """测试关节轨迹规划"""
        from lansi_arm.controller import TrajectoryPlanner

        planner = TrajectoryPlanner()

        waypoints = [
            Waypoint(positions=[0.0] * 7, time=0.0),
            Waypoint(positions=[0.1] * 7, time=1.0),
            Waypoint(positions=[0.2] * 7, time=2.0),
        ]

        trajectory = planner.plan(waypoints)

        assert isinstance(trajectory, Trajectory)
        assert len(trajectory.points) > 0

    def test_plan_with_config(self):
        """测试带配置的轨迹规划"""
        from lansi_arm.controller import TrajectoryPlanner

        config = TrajectoryConfig(
            interpolation_type=InterpolationType.QUINTIC,
            velocity=0.8,
            acceleration=0.6
        )

        planner = TrajectoryPlanner(config=config)

        waypoints = [
            Waypoint(positions=[0.0] * 7, time=0.0),
            Waypoint(positions=[0.1] * 7, time=1.0),
        ]

        trajectory = planner.plan(waypoints)

        assert isinstance(trajectory, Trajectory)
        assert len(trajectory.points) > 0


class TestTrajectoryRecorder:
    """轨迹录制器测试类"""

    def test_recorder_creation(self):
        """测试录制器创建"""
        from lansi_arm.controller import TrajectoryRecorder

        recorder = TrajectoryRecorder()
        assert recorder is not None

    def test_recorder_start_stop(self):
        """测试录制开始停止"""
        from lansi_arm.controller import TrajectoryRecorder

        recorder = TrajectoryRecorder()

        recorder.start()
        assert recorder.is_recording is True

        recorder.stop()
        assert recorder.is_recording is False

    def test_recorder_clear(self):
        """测试录制清空"""
        from lansi_arm.controller import TrajectoryRecorder

        recorder = TrajectoryRecorder()

        recorder.start()
        recorder.clear()
        assert recorder.is_recording is False


class TestTrajectoryPlayer:
    """轨迹播放器测试类"""

    def test_player_creation(self):
        """测试播放器创建"""
        from lansi_arm.controller import TrajectoryPlayer

        player = TrajectoryPlayer()
        assert player is not None

    def test_player_state(self):
        """测试播放器状态"""
        from lansi_arm.controller import TrajectoryPlayer, PlaybackState

        player = TrajectoryPlayer()
        assert player.state == PlaybackState.IDLE

    def test_player_load_trajectory(self):
        """测试加载轨迹"""
        from lansi_arm.controller import TrajectoryPlayer, Trajectory

        player = TrajectoryPlayer()

        trajectory = Trajectory()
        trajectory.add_point(TrajectoryPoint([0.0] * 7, time=0.0))
        trajectory.add_point(TrajectoryPoint([0.1] * 7, time=1.0))

        player.load(trajectory)
        assert player.trajectory is not None


class TestPlaybackState:
    """播放状态测试类"""

    def test_playback_state_values(self):
        """测试播放状态值"""
        from lansi_arm.controller import PlaybackState

        assert PlaybackState.IDLE.value == "idle"
        assert PlaybackState.PLAYING.value == "playing"
        assert PlaybackState.PAUSED.value == "paused"
        assert PlaybackState.COMPLETED.value == "completed"

    def test_playback_state_count(self):
        """测试播放状态数量"""
        from lansi_arm.controller import PlaybackState

        assert len(PlaybackState) == 4


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
