"""
测试运动学计算
"""
import pytest
import numpy as np
from lansi_arm.kinematics import (
    Pose,
    DHParameters,
    get_dh_parameters,
)


class TestPose:
    """位姿测试类"""

    def test_pose_creation(self):
        """测试位姿创建"""
        pose = Pose(
            position=[0.5, 0.3, 0.2],
            orientation=[0, 0, 0, 1]
        )
        assert pose.position[0] == 0.5
        assert pose.position[1] == 0.3
        assert pose.position[2] == 0.2

    def test_pose_default_orientation(self):
        """测试默认单位四元数"""
        pose = Pose(position=[1.0, 0.0, 0.0])
        assert pose.orientation == [0, 0, 0, 1]

    def test_pose_to_matrix(self):
        """测试位姿转矩阵"""
        pose = Pose(
            position=[0.0, 0.0, 0.0],
            orientation=[0, 0, 0, 1]
        )
        matrix = pose.to_matrix()
        assert matrix.shape == (4, 4)
        assert matrix[3, 3] == 1.0

    def test_pose_from_matrix(self):
        """测试从矩阵创建位姿"""
        matrix = np.eye(4)
        matrix[0, 3] = 0.5
        matrix[1, 3] = 0.3
        matrix[2, 3] = 0.2
        pose = Pose.from_matrix(matrix)
        assert abs(pose.position[0] - 0.5) < 1e-6
        assert abs(pose.position[1] - 0.3) < 1e-6
        assert abs(pose.position[2] - 0.2) < 1e-6


class TestDHParameters:
    """DH参数测试类"""

    def test_dh_parameters_creation(self):
        """测试DH参数创建"""
        dh = DHParameters(
            a=[0, 0, 0, 0, 0, 0, 0],
            d=[0.1, 0.2, 0.3, 0.1, 0.1, 0.1, 0.05],
            alpha=[-np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2, -np.pi/2, 0],
            theta=[0, 0, 0, 0, 0, 0, 0]
        )
        assert len(dh.a) == 7
        assert len(dh.d) == 7
        assert len(dh.alpha) == 7
        assert len(dh.theta) == 7

    def test_dh_parameters_left_arm(self):
        """测试左臂DH参数"""
        dh = get_dh_parameters("left")
        assert len(dh.a) == 7
        assert len(dh.d) == 7
        assert len(dh.alpha) == 7
        assert len(dh.theta) == 7

    def test_dh_parameters_right_arm(self):
        """测试右臂DH参数"""
        dh = get_dh_parameters("right")
        assert len(dh.a) == 7
        assert len(dh.d) == 7
        assert len(dh.alpha) == 7
        assert len(dh.theta) == 7

    def test_dh_parameters_invalid_arm(self):
        """测试无效机械臂类型"""
        with pytest.raises(ValueError):
            get_dh_parameters("invalid")


class TestForwardKinematics:
    """正向运动学测试类"""

    def test_fk_home_position(self):
        """测试原点位置的正解"""
        from lansi_arm.kinematics import ForwardKinematics

        dh = get_dh_parameters("left")
        fk = ForwardKinematics(dh)

        # 所有关节角度为0时的末端位置
        joint_angles = [0.0] * 7
        pose = fk.compute(joint_angles)

        # 验证返回类型
        assert isinstance(pose, Pose)
        assert len(pose.position) == 3
        assert len(pose.orientation) == 4

    def test_fk_single_joint_movement(self):
        """测试单关节运动的正解"""
        from lansi_arm.kinematics import ForwardKinematics

        dh = get_dh_parameters("left")
        fk = ForwardKinematics(dh)

        # 第一个关节旋转
        joint_angles = [0.1] + [0.0] * 6
        pose = fk.compute(joint_angles)

        # 验证末端位置
        assert pose.position[0] is not None
        assert pose.position[1] is not None
        assert pose.position[2] is not None

    def test_fk_invalid_joint_count(self):
        """测试无效关节数量"""
        from lansi_arm.kinematics import ForwardKinematics

        dh = get_dh_parameters("left")
        fk = ForwardKinematics(dh)

        # 错误的关节数量
        with pytest.raises(ValueError):
            fk.compute([0.1, 0.2, 0.3])  # 只有3个关节


class TestInverseKinematics:
    """逆向运动学测试类"""

    def test_ik_home_position(self):
        """测试原点位置的逆解"""
        from lansi_arm.kinematics import InverseKinematics

        dh = get_dh_parameters("left")
        ik = InverseKinematics(dh)

        # 目标位姿接近原点
        target_pose = Pose(
            position=[0.0, 0.0, 0.5],
            orientation=[0, 0, 0, 1]
        )

        solution = ik.compute(target_pose, max_iterations=100)

        # 验证返回类型
        assert solution is not None
        assert len(solution) == 7

    def test_ik_reachable_position(self):
        """测试可达位置的逆解"""
        from lansi_arm.kinematics import InverseKinematics

        dh = get_dh_parameters("left")
        ik = InverseKinematics(dh)

        # 目标位姿在工作空间内
        target_pose = Pose(
            position=[0.3, 0.1, 0.4],
            orientation=[0, 0, 0, 1]
        )

        solution = ik.compute(target_pose, max_iterations=100)

        # 验证解的有效性
        if solution is not None:
            assert len(solution) == 7
            # 验证解是否满足关节限制
            for angle in solution:
                assert -np.pi <= angle <= np.pi

    def test_ik_invalid_pose(self):
        """测试无效位姿"""
        from lansi_arm.kinematics import InverseKinematics

        dh = get_dh_parameters("left")
        ik = InverseKinematics(dh)

        # 无效的四元数（未归一化）
        target_pose = Pose(
            position=[1.0, 0.0, 0.0],
            orientation=[0.5, 0.5, 0.5, 0.5]  # 未归一化
        )

        # 应该抛出异常或返回None
        solution = ik.compute(target_pose, max_iterations=10)
        # 对于不可达位置，IK可能返回None或抛出异常


class TestJacobian:
    """雅可比矩阵测试类"""

    def test_jacobian_computation(self):
        """测试雅可比矩阵计算"""
        from lansi_arm.kinematics import Jacobian

        dh = get_dh_parameters("left")
        jacobian = Jacobian(dh)

        joint_angles = [0.1] * 7
        J = jacobian.compute(joint_angles)

        # 雅可比矩阵应该是6x7的矩阵
        assert J.shape == (6, 7)

    def test_jacobian_zero_velocity(self):
        """测试零速度时的雅可比矩阵"""
        from lansi_arm.kinematics import Jacobian

        dh = get_dh_parameters("left")
        jacobian = Jacobian(dh)

        joint_angles = [0.0] * 7
        J = jacobian.compute(joint_angles)

        # 验证矩阵形状
        assert J.shape == (6, 7)

    def test_jacobian_singularity_detection(self):
        """测试奇异性检测"""
        from lansi_arm.kinematics import Jacobian

        dh = get_dh_parameters("left")
        jacobian = Jacobian(dh)

        joint_angles = [0.0] * 7
        J = jacobian.compute(joint_angles)

        # 计算可操作度
        manipulability = jacobian.compute_manipulability(J)

        # 可操作度应该是一个非负数
        assert manipulability >= 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
