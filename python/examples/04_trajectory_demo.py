#!/usr/bin/env python3
"""
轨迹规划示例

演示如何使用Lansi Arm SDK进行轨迹录制、插值和播放。
"""


import time
from lansi_arm import (
    GroupController,
    TrajectoryPlanner,
    TrajectoryRecorder,
    TrajectoryManager,
    Trajectory,
    TrajectoryPoint,
    InterpolationType,
    MotorMode,
)


def on_progress(progress: dict):
    """进度回调"""
    print(f"  进度: {progress['current']}/{progress['total']}")


def example_trajectory_interpolation():
    """轨迹插值示例"""
    print("\n" + "=" * 60)
    print("轨迹插值示例")
    print("=" * 60)

    planner = TrajectoryPlanner()

    # 点到点规划
    print("\n[1] 点到点轨迹规划")
    start = [0.0] * 7
    end = [0.5, -0.5, 0.3, 1.0, -0.5, 0.3, 0.0]

    trajectory = planner.plan_point_to_point(
        start_positions=start,
        end_positions=end,
        duration=2.0,
        num_points=50,
        interpolation=InterpolationType.QUINTIC
    )

    print(f"  起点: {start}")
    print(f"  终点: {end}")
    print(f"  点数: {len(trajectory.points)}")

    # 打印前3个点
    print("\n  前3个点:")
    for i in range(min(3, len(trajectory.points))):
        wp = trajectory.points[i]
        print(f"    点{i}: {[f'{p:.4f}' for p in wp.positions]}")

    # 直线轨迹规划
    print("\n[2] 双臂直线轨迹规划")
    start_pose = {
        "left": [0.0] * 7,
        "right": [0.0] * 7
    }
    end_pose = {
        "left": [0.3, -0.5, 0.2, 1.0, -0.3, 0.2, 0.1],
        "right": [0.3, 0.5, -0.2, -1.0, 0.3, -0.2, -0.1]
    }

    linear_trajectory = planner.plan_linear(
        start_pose=start_pose,
        end_pose=end_pose,
        duration=3.0,
        num_points=100
    )

    print(f"  左臂: {len(linear_trajectory.points)} 点")
    print(f"  右臂: 同时规划")

    # 轨迹平滑
    print("\n[3] 轨迹平滑")
    smooth_trajectory = planner.smooth_trajectory(
        linear_trajectory,
        smoothing_factor=0.2,
        iterations=5
    )
    print(f"  平滑后: {len(smooth_trajectory.points)} 点")


def example_teaching():
    """示教录制示例"""
    print("\n" + "=" * 60)
    print("示教录制示例")
    print("=" * 60)

    # 创建群控器
    group = GroupController()

    try:
        # 连接
        print("\n[1] 连接机械臂...")
        group.connect_all()
        group.initialize_all()
        group.enable_all(True)

        # 创建录制器
        recorder = TrajectoryRecorder(group)

        # 模拟录制过程 (实际使用时手动移动机械臂)
        print("\n[2] 开始示教录制...")

        # 设置控制目标
        group.set_target("left")
        recorder.start_recording()

        # 模拟录制几个点 (实际中用户手动移动机械臂)
        print("  录制点1 (当前位置)...")
        recorder.record_point("point_1")

        # 模拟移动到新位置
        print("  移动到目标位置...")
        group.set_joint_positions([0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0])
        time.sleep(0.5)

        print("  录制点2...")
        recorder.record_point("point_2")

        # 移动到下一个位置
        print("  移动到目标位置...")
        group.set_joint_positions([0.2, -0.3, 0.1, 1.0, -0.5, 0.3, 0.0])
        time.sleep(0.5)

        print("  录制点3...")
        recorder.record_point("point_3")

        # 停止录制
        print("\n[3] 生成轨迹...")
        trajectory = recorder.stop_recording()

        print(f"  录制点数: {len(trajectory.points)}")
        for i, pt in enumerate(trajectory.points):
            print(f"    {pt.name}: {pt.delay}s")

        # 保存轨迹
        print("\n[4] 保存轨迹...")
        manager = TrajectoryManager("./trajectories")
        filepath = manager.save(trajectory, "teaching_demo.json")
        print(f"  保存到: {filepath}")

    except Exception as e:
        print(f"\n错误: {e}")

    finally:
        group.disconnect_all()


def example_trajectory_playback():
    """轨迹播放示例"""
    print("\n" + "=" * 60)
    print("轨迹播放示例")
    print("=" * 60)

    # 创建群控器
    group = GroupController()

    try:
        # 连接
        print("\n[1] 连接机械臂...")
        group.connect_all()
        group.initialize_all()
        group.enable_all(True)

        # 加载轨迹
        print("\n[2] 加载轨迹...")
        manager = TrajectoryManager("./trajectories")
        trajectory = manager.load("teaching_demo.json")

        if trajectory is None:
            # 创建测试轨迹
            print("  未找到保存的轨迹，创建测试轨迹...")
            trajectory = Trajectory(
                name="test",
                points=[
                    TrajectoryPoint(
                        name="start",
                        positions={"left": [0.0] * 7},
                        delay=1.0
                    ),
                    TrajectoryPoint(
                        name="point_1",
                        positions={"left": [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]},
                        delay=1.0
                    ),
                    TrajectoryPoint(
                        name="point_2",
                        positions={"left": [0.2, -0.3, 0.1, 1.0, -0.5, 0.3, 0.0]},
                        delay=1.0
                    ),
                    TrajectoryPoint(
                        name="end",
                        positions={"left": [0.0] * 7},
                        delay=1.0
                    ),
                ]
            )

        print(f"  轨迹: {trajectory.name}")
        print(f"  点数: {len(trajectory.points)}")

        # 设置控制目标
        group.set_target("left")

        # 播放轨迹
        print("\n[3] 播放轨迹...")
        group.play_trajectory({"left": trajectory}, speed=1.0)

        # 等待播放完成
        print("  等待播放完成...")
        time.sleep(5.0)

        print("  播放完成!")

    except Exception as e:
        print(f"\n错误: {e}")

    finally:
        group.disconnect_all()


def example_trajectory_management():
    """轨迹管理示例"""
    print("\n" + "=" * 60)
    print("轨迹管理示例")
    print("=" * 60)

    manager = TrajectoryManager("./trajectories")

    # 列出所有轨迹
    print("\n[1] 列出所有轨迹...")
    trajectories = manager.list()

    if trajectories:
        for traj in trajectories:
            print(f"  - {traj['name']}: {traj['points_count']} 点")
    else:
        print("  没有保存的轨迹")

    # 创建示例轨迹
    print("\n[2] 创建示例轨迹...")
    trajectory = Trajectory(
        name="example_trajectory",
        description="这是一个示例轨迹",
        points=[
            TrajectoryPoint(name="start", positions={"left": [0.0]*7}, delay=1.0),
            TrajectoryPoint(name="home", positions={"left": [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]}, delay=1.0),
            TrajectoryPoint(name="end", positions={"left": [0.0]*7}, delay=1.0),
        ],
        loop=False
    )

    # 保存
    filepath = manager.save(trajectory)
    print(f"  保存到: {filepath}")

    # 重新加载
    loaded = manager.load(filepath)
    if loaded:
        print(f"  重新加载: {loaded.name}")
        print(f"  点数: {len(loaded.points)}")

    # 删除
    print("\n[3] 删除示例轨迹...")
    if manager.delete(filepath):
        print("  删除成功!")


def main():
    """主函数"""
    print("=" * 60)
    print("Lansi Arm SDK - 轨迹规划示例")
    print("=" * 60)

    # 1. 轨迹插值
    example_trajectory_interpolation()

    # 2. 示教录制 (需要机械臂连接)
    # example_teaching()

    # 3. 轨迹播放 (需要机械臂连接)
    # example_trajectory_playback()

    # 4. 轨迹管理
    example_trajectory_management()

    print("\n" + "=" * 60)
    print("示例完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
