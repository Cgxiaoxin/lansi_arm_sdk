#!/usr/bin/env python3
"""
双臂协同控制示例

演示如何使用Lansi Arm SDK进行双臂协同控制。
"""


import time
from lansi_arm import (
    GroupController,
    Trajectory,
    TrajectoryPoint,
    PlaybackState,
    MotorMode,
)


def on_error(message: str):
    """错误回调"""
    print(f"[错误] {message}")


def on_progress(progress: dict):
    """进度回调"""
    print(f"  进度: {progress['current']}/{progress['total']} ({progress['progress']:.1f}%)")


def example_basic_dual_arm():
    """基础双臂控制示例"""
    print("=" * 60)
    print("双臂协同控制示例")
    print("=" * 60)

    # 创建群控器
    group = GroupController()

    # 设置回调
    group.set_callbacks(on_error=on_error)

    try:
        # 连接双臂
        print("\n[1] 连接双臂...")
        results = group.connect_all()
        print(f"  左臂: {'成功' if results['left'] else '失败'}")
        print(f"  右臂: {'成功' if results['right'] else '失败'}")

        # 初始化双臂
        print("\n[2] 初始化双臂...")
        group.initialize_all(mode=MotorMode.PROFILE_POSITION)
        print("  初始化成功!")

        # 使能
        print("\n[3] 使能电机...")
        group.enable_all(True)
        print("  使能成功!")

        # 获取关节位置
        print("\n[4] 读取当前位置...")
        positions = group.get_joint_positions()
        if "left" in positions:
            print(f"  左臂: {[f'{p:.4f}' for p in positions['left']]}")
        if "right" in positions:
            print(f"  右臂: {[f'{p:.4f}' for p in positions['right']]}")

        # 双臂同时移动到零位
        print("\n[5] 双臂同时回零...")
        group.go_to_zero_all()
        print("  移动完成!")

        time.sleep(1.0)

        # 双臂同时移动到目标位置
        print("\n[6] 双臂同步移动...")
        left_target = [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]
        right_target = [0.1, 0.5, -0.3, -1.2, 0.8, -0.5, 0.0]

        print(f"  左臂目标: {[f'{p:.4f}' for p in left_target]}")
        print(f"  右臂目标: {[f'{p:.4f}' for p in right_target]}")

        group.move_both(left_target, right_target, speed=0.5, sync=True)
        print("  移动完成!")

        time.sleep(1.0)

        # 获取最终位置
        print("\n[7] 读取最终位置...")
        positions = group.get_joint_positions()
        if "left" in positions:
            print(f"  左臂: {[f'{p:.4f}' for p in positions['left']]}")
        if "right" in positions:
            print(f"  右臂: {[f'{p:.4f}' for p in positions['right']]}")

        # 获取状态
        print("\n[8] 获取状态...")
        state = group.get_state()
        print(f"  控制目标: {state['target']}")
        print(f"  左臂状态: {state['arms']['left']['state']}")
        print(f"  右臂状态: {state['arms']['right']['state']}")

    except Exception as e:
        print(f"\n错误: {e}")
        raise

    finally:
        # 断开连接
        print("\n[9] 断开连接...")
        group.disconnect_all()
        print("  断开成功!")

        print("\n" + "=" * 60)
        print("示例完成!")
        print("=" * 60)


def example_trajectory_playback():
    """轨迹播放示例"""
    print("\n" + "=" * 60)
    print("轨迹播放示例")
    print("=" * 60)

    # 创建群控器
    group = GroupController()

    try:
        # 连接并初始化
        print("\n[1] 连接并初始化...")
        group.connect_all()
        group.initialize_all()

        # 创建示教轨迹
        print("\n[2] 创建示教轨迹...")

        # 左臂轨迹
        left_trajectory = Trajectory(
            name="left_pick_place",
            points=[
                TrajectoryPoint(
                    name="start",
                    positions={"left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                    delay=1.0
                ),
                TrajectoryPoint(
                    name="pick",
                    positions={"left": [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]},
                    delay=1.0
                ),
                TrajectoryPoint(
                    name="place",
                    positions={"left": [0.2, -0.3, 0.1, 1.0, -0.5, 0.3, 0.0]},
                    delay=1.0
                ),
            ],
            loop=False
        )

        # 右臂轨迹
        right_trajectory = Trajectory(
            name="right_pick_place",
            points=[
                TrajectoryPoint(
                    name="start",
                    positions={"right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                    delay=1.0
                ),
                TrajectoryPoint(
                    name="pick",
                    positions={"right": [0.1, 0.5, -0.3, -1.2, 0.8, -0.5, 0.0]},
                    delay=1.0
                ),
                TrajectoryPoint(
                    name="place",
                    positions={"right": [0.2, 0.3, -0.1, -1.0, 0.5, -0.3, 0.0]},
                    delay=1.0
                ),
            ],
            loop=False
        )

        print("  左臂轨迹: 3个点")
        print("  右臂轨迹: 3个点")

        # 设置回调
        group.set_callbacks(on_playback_progress=on_progress)

        # 使能
        print("\n[3] 使能电机...")
        group.enable_all(True)

        # 播放轨迹
        print("\n[4] 播放轨迹...")
        group.play_trajectory(
            {"left": left_trajectory, "right": right_trajectory},
            speed=1.0,
            loop=False
        )

        # 等待播放完成
        while group.is_playing:
            time.sleep(0.5)

        print("  播放完成!")

    except Exception as e:
        print(f"\n错误: {e}")
        raise

    finally:
        print("\n[5] 清理...")
        group.disconnect_all()
        print("  完成!")


def example_individual_control():
    """单臂独立控制示例"""
    print("\n" + "=" * 60)
    print("单臂独立控制示例")
    print("=" * 60)

    group = GroupController()

    try:
        # 只连接左臂
        print("\n[1] 只连接左臂...")
        group.connect_left()
        group.initialize_left()
        group.enable_left(True)

        # 控制目标设为左臂
        group.set_target("left")

        # 移动左臂
        print("\n[2] 移动左臂...")
        group.set_joint_positions({
            "left": [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]
        })

        print("  移动完成!")

        # 状态
        state = group.get_state()
        print(f"\n[3] 状态: {state['target']}")
        print(f"  左臂: {state['arms']['left']['state']}")
        print(f"  右臂: {'未连接' if state['arms']['right'] is None else state['arms']['right']['state']}")

    except Exception as e:
        print(f"\n错误: {e}")

    finally:
        group.disconnect_all()


def main():
    """主函数"""
    print("Lansi Arm SDK - 双臂协同控制示例")
    print()

    # 基础双臂控制
    example_basic_dual_arm()

    # 轨迹播放
    example_trajectory_playback()

    # 单臂独立控制
    example_individual_control()

    print("\n" + "=" * 60)
    print("所有示例完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
