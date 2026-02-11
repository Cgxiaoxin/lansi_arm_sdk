#!/usr/bin/env python3
"""
运动学示例

演示如何使用Lansi Arm SDK进行正运动学和逆运动学计算。
"""


import math
from lansi_arm import (
    Kinematics,
    Pose,
    ForwardKinematics,
    InverseKinematics,
    get_dh_parameters,
    get_joint_limits,
    LEFT_ARM,
    RIGHT_ARM,
)


def print_pose(pose: Pose, prefix: str = ""):
    """打印位姿"""
    print(f"{prefix}位置: [{pose.position[0]:.4f}, {pose.position[1]:.4f}, {pose.position[2]:.4f}] m")
    print(f"{prefix}姿态: [{pose.orientation[0]:.4f}, {pose.orientation[1]:.4f}, {pose.orientation[2]:.4f}] rad")
    print(f"{prefix}       (R={math.degrees(pose.orientation[0]):.1f}°, P={math.degrees(pose.orientation[1]):.1f}°, Y={math.degrees(pose.orientation[2]):.1f}°)")


def example_forward_kinematics():
    """正向运动学示例"""
    print("\n" + "=" * 60)
    print("正向运动学示例")
    print("=" * 60)

    # 创建运动学实例 (左臂)
    kinematics = Kinematics(arm_type="left")

    # 零位姿态
    zero_positions = [0.0] * 7
    print("\n[1] 零位姿态 (所有关节角度=0)")
    pose = kinematics.forward(zero_positions)
    print_pose(pose, "  ")

    # 中间姿态
    mid_positions = [0.5, 0.3, 0.2, -0.5, 0.2, 0.3, 0.1]
    print(f"\n[2] 中间姿态 {mid_positions}")
    pose = kinematics.forward(mid_positions)
    print_pose(pose, "  ")

    # 分析奇异性
    print(f"\n[3] 奇异性分析")
    result = kinematics.analyze(mid_positions)
    print(f"  最小奇异值: {result.min_singular_value:.6f}")
    print(f"  是否奇异: {result.is_singular}")


def example_inverse_kinematics():
    """逆向运动学示例"""
    print("\n" + "=" * 60)
    print("逆向运动学示例")
    print("=" * 60)

    kinematics = Kinematics(arm_type="left")

    # 目标位姿 (零位)
    target_pose = Pose(
        position=[0.3, 0.2, 0.8],  # 约30cm前，20cm右，80cm高
        orientation=[0.0, 0.0, 0.0]
    )

    print(f"\n[1] 目标位姿")
    print_pose(target_pose, "  ")

    # 求解逆运动学
    print(f"\n[2] 求解逆运动学")
    joint_positions = kinematics.inverse(
        target_pose,
        seed=[0.0] * 7,  # 使用零位作为初始猜测
    )

    if joint_positions:
        print(f"  求解成功!")
        print(f"  关节角度: {[f'{j:.4f}' for j in joint_positions]}")
        print(f"  关节角度: {[f'{math.degrees(j):.1f}°' for j in joint_positions]}")

        # 验证
        print(f"\n[3] 验证 (FK)")
        verified_pose = kinematics.forward(joint_positions)
        print_pose(verified_pose, "  ")
    else:
        print(f"  求解失败!")


def example_arm_configs():
    """机械臂配置示例"""
    print("\n" + "=" * 60)
    print("机械臂配置示例")
    print("=" * 60)

    print("\n[1] 左臂关节限制")
    limits = get_joint_limits("left")
    for joint_name, limit in limits.items():
        print(f"  {joint_name}: [{limit['lower']:.3f}, {limit['upper']:.3f}] rad")
        print(f"           [{math.degrees(limit['lower']):.1f}°, {math.degrees(limit['upper']):.1f}°]")

    print(f"\n[2] 左臂电机ID")
    for jc in LEFT_ARM.joint_configs:
        print(f"  {jc.name}: 电机 {jc.motor_id}")

    print(f"\n[3] 右臂电机ID")
    for jc in RIGHT_ARM.joint_configs:
        print(f"  {jc.name}: 电机 {jc.motor_id}")


def main():
    """主函数"""
    print("=" * 60)
    print("Lansi Arm SDK - 运动学示例")
    print("=" * 60)

    example_forward_kinematics()
    example_inverse_kinematics()
    example_arm_configs()

    print("\n" + "=" * 60)
    print("示例完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
