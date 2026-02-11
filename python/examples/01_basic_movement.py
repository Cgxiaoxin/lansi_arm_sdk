#!/usr/bin/env python3
"""
基础移动示例

演示如何使用Lansi Arm SDK进行基础的机械臂控制。
"""


import time
from lansi_arm import ArmController, MotorMode


def main():
    """主函数"""
    print("=" * 60)
    print("Lansi Arm SDK - 基础移动示例")
    print("=" * 60)

    # 创建控制器
    arm = ArmController(
        motor_ids=[51, 52, 53, 54, 55, 56, 57],
        can_channel="can0"
    )

    try:
        # 连接
        print("\n[1] 连接机械臂...")
        arm.connect()
        print("    连接成功!")

        # 初始化
        print("\n[2] 初始化机械臂...")
        arm.initialize(mode=MotorMode.PROFILE_POSITION)
        print("    初始化成功!")

        # 使能
        print("\n[3] 使能电机...")
        arm.enable(True)
        print("    使能成功!")

        # 读取当前位置
        print("\n[4] 读取当前位置...")
        positions = arm.get_joint_positions()
        print(f"    当前位置: {[f'{p:.4f}' for p in positions]}")

        # 移动到零位
        print("\n[5] 移动到零位...")
        arm.go_to_zero()
        print("    移动完成!")

        # 等待到达零位
        time.sleep(1.0)

        # 移动到目标位置
        print("\n[6] 移动到目标位置...")
        target = [0.1, -0.5, 0.3, 1.2, -0.8, 0.5, 0.0]
        print(f"    目标位置: {[f'{p:.4f}' for p in target]}")
        arm.set_joint_positions(target)
        print("    移动完成!")

        # 等待到达目标位置
        time.sleep(1.0)

        # 读取最终位置
        print("\n[7] 读取最终位置...")
        positions = arm.get_joint_positions()
        print(f"    最终位置: {[f'{p:.4f}' for p in positions]}")

        # 获取状态
        print("\n[8] 获取状态...")
        state = arm.get_state()
        print(f"    状态: {state['state']}")
        print(f"    模式: {state['mode']}")

    except Exception as e:
        print(f"\n错误: {e}")
        raise

    finally:
        # 断开连接
        print("\n[9] 断开连接...")
        arm.disconnect()
        print("    断开成功!")

        print("\n" + "=" * 60)
        print("示例完成!")
        print("=" * 60)


if __name__ == "__main__":
    main()
