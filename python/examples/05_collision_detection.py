#!/usr/bin/env python3
"""
碰撞检测示例

演示如何使用Lansi Arm SDK进行碰撞检测。
"""


import time
import numpy as np
from lansi_arm import (
    CollisionDetector,
    CollisionConfig,
    CollisionResult,
    CollisionType,
    BoundingBox,
    GroupController,
    MotorMode,
)


def example_bounding_box():
    """包围盒示例"""
    print("\n" + "=" * 60)
    print("包围盒示例")
    print("=" * 60)

    # 创建包围盒
    box1 = BoundingBox(
        min_corner=[0.0, 0.0, 0.0],
        max_corner=[1.0, 1.0, 1.0]
    )

    box2 = BoundingBox(
        center=[1.5, 1.5, 1.5],
        size=[1.0, 1.0, 1.0]
    )

    print(f"\nBox1:")
    print(f"  Center: {box1.center}")
    print(f"  Size: {box1.size}")
    print(f"  Volume: {box1.volume}")

    print(f"\nBox2:")
    print(f"  Center: {box2.center}")
    print(f"  Size: {box2.size}")

    # 检查相交
    intersect = box1.intersect(box2)
    print(f"\n相交检查: {intersect}")

    # 计算距离
    distance = box1.distance_to(box2)
    print(f"距离: {distance:.4f} m")

    # 扩展包围盒
    box1_expanded = box1.expand(0.1)
    print(f"\n扩展后Box1:")
    print(f"  Min: {box1_expanded.min}")
    print(f"  Max: {box1_expanded.max}")


def example_environment_collision():
    """环境碰撞检测示例"""
    print("\n" + "=" * 60)
    print("环境碰撞检测示例")
    print("=" * 60)

    # 创建碰撞检测器
    config = CollisionConfig(
        enable_self_collision=False,
        enable_environment_collision=True,
        padding=0.05
    )
    detector = CollisionDetector(config)

    # 添加环境障碍物
    table = BoundingBox(
        min_corner=[-0.5, -0.5, 0.0],
        max_corner=[0.5, 0.5, 0.5]
    )
    detector.add_obstacle("table", table, padding=0.1)

    wall1 = BoundingBox(
        min_corner=[-2.0, -2.0, 0.0],
        max_corner=[-1.5, 2.0, 3.0]
    )
    detector.add_obstacle("wall_left", wall1, padding=0.05)

    wall2 = BoundingBox(
        min_corner=[1.5, -2.0, 0.0],
        max_corner=[2.0, 2.0, 3.0]
    )
    detector.add_obstacle("wall_right", wall2, padding=0.05)

    print("\n[1] 环境障碍物:")
    print(f"  Table: {table.to_dict()}")
    print(f"  Wall1: {wall1.to_dict()}")
    print(f"  Wall2: {wall2.to_dict()}")

    # 检查点的碰撞
    print("\n[2] 点碰撞检测:")

    safe_point = [0.0, 0.0, 1.0]
    collision, obstacles, distance = detector.check_point_collision(safe_point)
    print(f"  点 {safe_point}: 碰撞={collision}, 障碍物={obstacles}, 距离={distance:.4f}")

    unsafe_point = [0.0, 0.0, 0.2]
    collision, obstacles, distance = detector.check_point_collision(unsafe_point)
    print(f"  点 {unsafe_point}: 碰撞={collision}, 障碍物={obstacles}, 距离={distance:.4f}")


def example_self_collision():
    """自碰撞检测示例"""
    print("\n" + "=" * 60)
    print("自碰撞检测示例")
    print("=" * 60)

    # 创建碰撞检测器
    config = CollisionConfig(
        enable_self_collision=True,
        enable_environment_collision=False,
        padding=0.02
    )
    detector = CollisionDetector(config)

    # 正常姿态 (无碰撞)
    print("\n[1] 正常姿态:")
    normal_joints = {
        "left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    }

    result = detector.check_collision(normal_joints)
    print(f"  碰撞: {result.collision}")
    print(f"  类型: {result.collision_type.value}")
    print(f"  碰撞对: {result.colliding_links}")
    print(f"  距离: {result.distance:.4f} m")

    # 危险姿态 (可能碰撞)
    print("\n[2] 危险姿态 (双臂交叉):")
    dangerous_joints = {
        "left": [1.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0],
        "right": [-1.5, -0.5, 0.0, -0.5, 0.0, 0.0, 0.0]
    }

    result = detector.check_collision(dangerous_joints)
    print(f"  碰撞: {result.collision}")
    print(f"  类型: {result.collision_type.value}")
    print(f"  碰撞对: {result.colliding_links}")
    print(f"  距离: {result.distance:.4f} m")

    # 实时安全检查
    print("\n[3] 实时安全检查:")

    def is_safe_pose(joints, safety_margin=0.05):
        result = detector.check_collision(joints)
        return result.distance > safety_margin

    poses = [
        ({"left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
         "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]}),
        ({"left": [0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0],
         "right": [0.5, -0.5, 0.0, -0.5, 0.0, 0.0, 0.0]}),
        ({"left": [1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0],
         "right": [-1.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0]}),
    ]

    for i, joints in enumerate(poses):
        result = detector.check_collision(joints)
        safety = result.distance > 0.05
        print(f"  姿态{i+1}: 安全={safety}, 距离={result.distance:.4f} m")


def example_with_robot():
    """结合机械臂的碰撞检测示例"""
    print("\n" + "=" * 60)
    print("结合机械臂的碰撞检测示例")
    print("=" * 60)

    # 创建群控器
    group = GroupController()

    try:
        # 连接机械臂
        print("\n[1] 连接机械臂...")
        group.connect_all()
        group.initialize_all()
        group.enable_all(True)

        # 创建碰撞检测器
        config = CollisionConfig(
            enable_self_collision=True,
            enable_environment_collision=True,
            padding=0.03
        )
        detector = CollisionDetector(config)

        # 添加环境障碍物 (桌子)
        table = BoundingBox(
            min_corner=[-0.3, -0.3, -0.1],
            max_corner=[0.8, 0.3, 0.4]
        )
        detector.add_obstacle("table", table, padding=0.05)

        # 读取当前关节角度
        print("\n[2] 读取当前关节角度...")
        positions = group.get_joint_positions()
        joints = {
            "left": positions.get("left", [0.0] * 7),
            "right": positions.get("right", [0.0] * 7)
        }

        # 检查碰撞
        print("\n[3] 碰撞检测...")
        result = detector.check_collision(joints)

        print(f"  碰撞: {result.collision}")
        print(f"  类型: {result.collision_type.value}")
        print(f"  碰撞对: {result.colliding_links}")
        print(f"  距离: {result.distance:.4f} m")
        print(f"  安全: {detector.is_safe(joints, safety_margin=0.05)}")

        # 移动到新位置并检查
        print("\n[4] 移动并检查...")
        group.set_target("left")
        group.set_joint_positions([0.3, -0.5, 0.2, 1.0, -0.3, 0.2, 0.0])

        positions = group.get_joint_positions()
        joints = {"left": positions}

        result = detector.check_collision(joints)
        print(f"  碰撞: {result.collision}")
        print(f"  距离: {result.distance:.4f} m")

    except Exception as e:
        print(f"\n错误: {e}")

    finally:
        group.disconnect_all()


def example_collision_history():
    """碰撞历史示例"""
    print("\n" + "=" * 60)
    print("碰撞历史示例")
    print("=" * 60)

    detector = CollisionDetector()

    # 模拟碰撞检测
    print("\n[1] 模拟多次碰撞检测...")

    test_joints = [
        {"left": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "right": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
        {"left": [0.5, 0.5, 0.0, 0.5, 0.0, 0.0, 0.0], "right": [0.5, -0.5, 0.0, -0.5, 0.0, 0.0, 0.0]},
        {"left": [1.0, 1.0, 0.0, 1.0, 0.0, 0.0, 0.0], "right": [-1.0, -1.0, 0.0, -1.0, 0.0, 0.0, 0.0]},
    ]

    for i, joints in enumerate(test_joints):
        result = detector.check_collision(joints)
        print(f"  检测{i+1}: 碰撞={result.collision}, 距离={result.distance:.4f} m")

    # 获取历史
    print("\n[2] 获取碰撞历史...")
    history = detector.get_collision_history()
    print(f"  历史记录数: {len(history)}")

    # 清除历史
    print("\n[3] 清除历史...")
    detector.clear_history()
    history = detector.get_collision_history()
    print(f"  清除后: {len(history)}")

    # 获取状态
    print("\n[4] 获取碰撞检测器状态...")
    status = detector.get_status()
    print(f"  配置: 自碰撞={status['config']['enable_self_collision']}, "
          f"环境碰撞={status['config']['enable_environment_collision']}")
    print(f"  连杆数: {status['links_count']}")
    print(f"  碰撞对数: {status['collision_pairs_count']}")


def main():
    """主函数"""
    print("=" * 60)
    print("Lansi Arm SDK - 碰撞检测示例")
    print("=" * 60)

    # 包围盒示例
    example_bounding_box()

    # 环境碰撞检测
    example_environment_collision()

    # 自碰撞检测
    example_self_collision()

    # 结合机械臂 (需要连接)
    # example_with_robot()

    # 碰撞历史
    example_collision_history()

    print("\n" + "=" * 60)
    print("示例完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
