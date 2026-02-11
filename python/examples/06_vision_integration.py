#!/usr/bin/env python3
"""
视觉集成示例

演示如何使用Lansi Arm SDK进行手眼标定和视觉引导控制。
"""


import time
import numpy as np
from lansi_arm import (
    HandEyeCalibration,
    VisionGuidedController,
    DepthCamera,
    Transform3D,
    CameraIntrinsics,
    CalibrationType,
    VisionType,
    GroupController,
    Pose,
)


def example_hand_eye_calibration():
    """手眼标定示例"""
    print("\n" + "=" * 60)
    print("手眼标定示例")
    print("=" * 60)

    # 创建标定器
    calibrator = HandEyeCalibration(calibration_type=CalibrationType.EYE_IN_HAND)

    print("\n[1] 添加标定数据...")

    # 模拟标定数据 (实际中从机械臂和相机获取)
    for i in range(5):
        # 机械臂末端位姿
        robot_pose = Transform3D.from_pose(
            position=[0.3, 0.2, 0.5],
            orientation=[0, 0, 0]  # 欧拉角
        )

        # 相机检测到的标定板位姿
        camera_pose = Transform3D.from_pose(
            position=[0.1, 0.05, 0.3],
            orientation=[0.1, 0.05, 0.02]
        )

        calibrator.add_calibration_data(robot_pose, camera_pose)
        print(f"  添加数据点 {i+1}")

    print(f"  总数据点: {calibrator.data_count}")

    # 执行标定
    print("\n[2] 执行手眼标定...")
    result = calibrator.calibrate()

    if result.success:
        print(f"  标定成功!")
        print(f"  重投影误差: {result.error:.6f} m")
        print(f"  数据点数: {result.details['data_points']}")

        # 保存标定结果
        print("\n[3] 保存标定结果...")
        calibrator.save_calibration("hand_eye_calibration.json")
        print("  保存到: hand_eye_calibration.json")

        # 获取标定矩阵
        matrix = calibrator.get_calibration_matrix()
        print(f"  4x4标定矩阵:")
        print(matrix)

    else:
        print(f"  标定失败: {result.details['error']}")

    # 清除数据
    print("\n[4] 清除标定数据...")
    calibrator.clear_data()
    print(f"  数据点数: {calibrator.data_count}")


def example_vision_guided_control():
    """视觉引导控制示例"""
    print("\n" + "=" * 60)
    print("视觉引导控制示例")
    print("=" * 60)

    # 创建手眼标定
    calibrator = HandEyeCalibration(calibration_type=CalibrationType.EYE_IN_HAND)

    # 模拟标定结果
    calibrator.add_calibration_data(
        Transform3D.from_pose([0.3, 0.2, 0.5], [0, 0, 0]),
        Transform3D.from_pose([0.1, 0.05, 0.3], [0.1, 0.05, 0.02])
    )
    calibrator.calibrate()

    # 创建视觉引导控制器
    print("\n[1] 创建视觉引导控制器...")
    guided_controller = VisionGuidedController(
        vision_interface=None,  # 需要实际视觉设备
        hand_eye_calibration=calibrator
    )

    # 获取状态
    status = guided_controller.get_status()
    print(f"  标定已加载: {status['calibration_loaded']}")

    # 模拟目标检测
    print("\n[2] 模拟目标检测...")

    # 目标在相机坐标系下的位姿
    target_in_camera = Transform3D.from_pose(
        position=[0.2, 0.1, 0.5],
        orientation=[0, 0, 0]
    )

    # 转换到基座坐标系
    print("\n[3] 坐标系转换...")
    target_in_base = guided_controller.get_object_pose_in_base(target_in_camera)

    if target_in_base:
        print(f"  目标在相机坐标系: {target_in_camera.position}")
        print(f"  目标在基座坐标系: {target_in_base.position}")

    # 计算接近位姿
    print("\n[4] 计算接近位姿...")
    approach_pose = guided_controller.move_to_object(
        target_pose=target_in_camera,
        approach_distance=0.1,
        speed=0.5
    )

    if approach_pose[0]:
        print(f"  接近位姿: {approach_pose[1].position}")

    # 视觉伺服
    print("\n[5] 视觉伺服...")
    current_pose = Transform3D.from_pose([0.3, 0.2, 0.6], [0, 0, 0])
    target_pose = Transform3D.from_pose([0.2, 0.1, 0.5], [0, 0, 0])

    velocity = guided_controller.visual_servoing(
        current_pose=current_pose,
        target_pose=target_pose,
        Kp=1.0,
        max_velocity=0.1
    )

    print(f"  末端速度: {velocity}")


def example_depth_camera():
    """深度相机示例"""
    print("\n" + "=" * 60)
    print("深度相机示例")
    print("=" * 60)

    # 创建深度相机
    print("\n[1] 创建深度相机...")
    camera = DepthCamera(camera_type=VisionType.REALSENSE)

    # 连接相机
    print("\n[2] 连接相机...")
    connected = camera.connect()

    if not connected:
        print("  相机连接失败 (请安装pyrealsense2)")
        print("  跳过相机连接测试")
        return

    print("  连接成功!")

    # 获取点云
    print("\n[3] 获取点云...")
    try:
        point_cloud = camera.get_point_cloud()
        print(f"  点云形状: {point_cloud.shape}")

        # 获取RGBD
        print("\n[4] 获取RGBD图像...")
        color, depth = camera.get_rgbd()
        print(f"  彩色图像: {color.shape}")
        print(f"  深度图像: {depth.shape}")

        # 投影测试
        print("\n[5] 3D到2D投影测试...")
        test_points = np.array([[0.1, 0.1, 0.5]])
        pixels = camera.project_points(test_points)
        print(f"  3D点: {test_points[0]}")
        print(f"  2D像素: {pixels[0]}")

    except Exception as e:
        print(f"  错误: {e}")

    finally:
        # 断开连接
        print("\n[6] 断开连接...")
        camera.disconnect()


def example_coordinate_transformation():
    """坐标系转换示例"""
    print("\n" + "=" * 60)
    print("坐标系转换示例")
    print("=" * 60)

    # 创建变换
    print("\n[1] 创建变换...")
    transform = Transform3D.from_pose(
        position=[0.1, 0.2, 0.3],
        orientation=[0.1, 0.2, 0.3]  # 欧拉角
    )

    print(f"  位置: {transform.position}")
    print(f"  旋转矩阵:")
    print(transform.rotation)

    # 变换相乘
    print("\n[2] 变换相乘...")
    transform1 = Transform3D.from_pose([0.1, 0, 0], [0, 0, 0])
    transform2 = Transform3D.from_pose([0.2, 0, 0], [0, 0, 0])

    result = transform1.multiply(transform2)
    print(f"  T1 * T2 = {result.position}")

    # 逆变换
    print("\n[3] 逆变换...")
    inverse = transform.inverse()
    print(f"  原位置: {transform.position}")
    print(f"  逆位置: {inverse.position}")

    # 字典转换
    print("\n[4] 字典转换...")
    data = transform.to_dict()
    restored = Transform3D.from_dict(data)
    print(f"  原始位置: {transform.position}")
    print(f"  恢复位置: {restored.position}")


def example_calibration_save_load():
    """标定保存加载示例"""
    print("\n" + "=" * 60)
    print("标定保存加载示例")
    print("=" * 60)

    # 创建标定器
    calibrator = HandEyeCalibration(calibration_type=CalibrationType.EYE_IN_HAND)

    # 添加数据并标定
    print("\n[1] 执行标定...")
    for i in range(3):
        calibrator.add_calibration_data(
            Transform3D.from_pose([0.3, 0.2, 0.5 + i*0.01], [0, 0, i*0.01]),
            Transform3D.from_pose([0.1, 0.05, 0.3 + i*0.01], [0.1, 0.05, i*0.01])
        )

    result = calibrator.calibrate()

    if result.success:
        # 保存
        print("\n[2] 保存标定...")
        calibrator.save_calibration("calibration_test.json")

        # 清除
        print("\n[3] 清除数据...")
        calibrator.clear_data()
        print(f"  数据点数: {calibrator.data_count}")

        # 重新创建并加载
        print("\n[4] 重新加载...")
        new_calibrator = HandEyeCalibration(calibration_type=CalibrationType.EYE_IN_HAND)
        new_calibrator.load_calibration("calibration_test.json")

        if new_calibrator.result:
            print(f"  加载成功!")
            print(f"  误差: {new_calibrator.result.error:.6f} m")
            print(f"  数据点数: {new_calibrator.data_count}")


def main():
    """主函数"""
    print("=" * 60)
    print("Lansi Arm SDK - 视觉集成示例")
    print("=" * 60)

    # 坐标系转换
    example_coordinate_transformation()

    # 手眼标定
    example_hand_eye_calibration()

    # 标定保存加载
    example_calibration_save_load()

    # 视觉引导控制
    example_vision_guided_control()

    # 深度相机 (需要实际设备)
    # example_depth_camera()

    print("\n" + "=" * 60)
    print("示例完成!")
    print("=" * 60)


if __name__ == "__main__":
    main()
