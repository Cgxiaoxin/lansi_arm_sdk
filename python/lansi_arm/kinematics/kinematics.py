"""
运动学模块

提供正向运动学和逆向运动学计算。
"""


from dataclasses import dataclass
from typing import List, Optional, Tuple, Dict, Any
import math
import numpy as np

from .dh_parameters import DHParameters, DHParameter, get_dh_parameters


@dataclass
class Pose:
    """末端位姿"""
    position: List[float]      # [x, y, z] (米)
    orientation: List[float]    # [roll, pitch, yaw] (弧度)

    def __init__(
        self,
        position: Optional[List[float]] = None,
        orientation: Optional[List[float]] = None
    ):
        self.position = position or [0.0, 0.0, 0.0]
        self.orientation = orientation or [0.0, 0.0, 0.0]

    def to_matrix(self) -> np.ndarray:
        """转换为4x4变换矩阵"""
        return create_transform_matrix(self.position, self.orientation)

    @classmethod
    def from_matrix(cls, matrix: np.ndarray) -> "Pose":
        """从4x4变换矩阵创建"""
        position = matrix[:3, 3].tolist()
        orientation = matrix_to_euler(matrix)
        return cls(position=position, orientation=orientation)

    def __repr__(self) -> str:
        return (
            f"Pose(position=[{self.position[0]:.4f}, "
            f"{self.position[1]:.4f}, {self.position[2]:.4f}], "
            f"orientation=[{self.orientation[0]:.4f}, "
            f"{self.orientation[1]:.4f}, {self.orientation[2]:.4f}])"
        )


@dataclass
class JacobianResult:
    """雅可比矩阵计算结果"""
    jacobian: np.ndarray
    singular_values: List[float]
    is_singular: bool
    min_singular_value: float


class KinematicsError(Exception):
    """运动学异常"""
    pass


class IKError(KinematicsError):
    """逆运动学异常"""
    pass


def create_transform_matrix(
    position: List[float],
    orientation: List[float]
) -> np.ndarray:
    """
    创建4x4变换矩阵

    Args:
        position: [x, y, z]
        orientation: [roll, pitch, yaw]

    Returns:
        np.ndarray: 4x4变换矩阵
    """
    x, y, z = position
    roll, pitch, yaw = orientation

    # Roll (X轴旋转)
    cos_r, sin_r = math.cos(roll), math.sin(roll)
    # Pitch (Y轴旋转)
    cos_p, sin_p = math.cos(pitch), math.sin(pitch)
    # Yaw (Z轴旋转)
    cos_y, sin_y = math.cos(yaw), math.sin(yaw)

    # 构建旋转矩阵
    R = np.array([
        [cos_y * cos_p, cos_y * sin_p * sin_r - sin_y * cos_r,
         cos_y * sin_p * cos_r + sin_y * sin_r, x],
        [sin_y * cos_p, sin_y * sin_p * sin_r + cos_y * cos_r,
         sin_y * sin_p * cos_r - cos_y * sin_r, y],
        [-sin_p, cos_p * sin_r, cos_p * cos_r, z],
        [0, 0, 0, 1]
    ])

    return R


def matrix_to_euler(matrix: np.ndarray) -> List[float]:
    """从旋转矩阵提取欧拉角 [roll, pitch, yaw]"""
    R = matrix[:3, :3]

    # Pitch (Y轴)
    pitch = math.atan2(-R[2, 0], math.sqrt(R[0, 0]**2 + R[1, 0]**2))

    if abs(pitch) < 1e-6:
        #  gimbal lock
        roll = 0
        yaw = math.atan2(R[1, 2], R[0, 2])
    elif abs(pitch - math.pi/2) < 1e-6:
        pitch = math.pi/2
        roll = math.atan2(R[0, 1], R[0, 0])
    else:
        roll = math.atan2(R[2, 1], R[2, 2])
        yaw = math.atan2(R[1, 0], R[0, 0])

    return [roll, pitch, yaw]


class ForwardKinematics:
    """
    正向运动学

    将关节角度转换为末端位姿
    """

    def __init__(self, arm_type: str = "left", dh_params: Optional[DHParameters] = None):
        """
        Args:
            arm_type: "left" 或 "right"
            dh_params: DH参数 (可选，默认使用标准参数)
        """
        self.arm_type = arm_type
        self.dh = dh_params or get_dh_parameters(arm_type)
        self.n_joints = self.dh.n_joints

    def compute(
        self,
        joint_positions: List[float],
        return_all: bool = False
    ) -> Pose | Tuple[Pose, List[np.ndarray]]:
        """
        计算正向运动学

        Args:
            joint_positions: 7个关节角度 (弧度)
            return_all: 是否返回所有变换矩阵

        Returns:
            Pose: 末端位姿
            或 (Pose, List[np.ndarray]): 末端位姿 + 所有变换矩阵
        """
        if len(joint_positions) != self.n_joints:
            raise ValueError(
                f"需要{self.n_joints}个关节角度，实际{len(joint_positions)}个"
            )

        # 累积变换矩阵
        T = np.eye(4)
        T_list = [T.copy()]

        for i in range(self.n_joints):
            dh = self.dh.params[i]
            theta = joint_positions[i]

            # DH变换
            T_i = self._dh_transform(dh.a, dh.alpha, dh.d, theta)
            T = T @ T_i
            T_list.append(T.copy())

        pose = Pose.from_matrix(T)

        if return_all:
            return pose, T_list
        else:
            return pose

    def _dh_transform(
        self,
        a: float,
        alpha: float,
        d: float,
        theta: float
    ) -> np.ndarray:
        """
        计算DH变换矩阵

        标准DH变换:
        T = Tz(d) * Tz(theta) * Tx(a) * Rx(alpha)
        """
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        T = np.array([
            [ct, -st * ca,  st * ca, a * ct],
            [st,  ct * ca, -ct * ca, a * st],
            [0,      sa,      ca,     d],
            [0,       0,       0,      1]
        ])

        return T


class InverseKinematics:
    """
    逆向运动学

    将末端位姿转换为关节角度
    """

    def __init__(
        self,
        arm_type: str = "left",
        dh_params: Optional[DHParameters] = None
    ):
        """
        Args:
            arm_type: "left" 或 "right"
            dh_params: DH参数 (可选，默认使用标准参数)
        """
        self.arm_type = arm_type
        self.dh = dh_params or get_dh_parameters(arm_type)
        self.n_joints = self.dh.n_joints
        self.fk = ForwardKinematics(arm_type, dh_params)

        # IK参数
        self.max_iterations = 1000
        self.tolerance = 1e-4
        self.damping = 0.01

    def solve(
        self,
        target_pose: Pose,
        seed: Optional[List[float]] = None,
        use_null_space: bool = False,
        constraints: Optional[Dict] = None
    ) -> Optional[List[float]]:
        """
        求解逆运动学

        Args:
            target_pose: 目标末端位姿
            seed: 初始猜测关节角度
            use_null_space: 是否使用零空间控制
            constraints: 关节约束 {position_min, position_max}

        Returns:
            List[float]: 关节角度，或None(求解失败)
        """
        # 初始猜测
        if seed is None:
            seed = [0.0] * self.n_joints

        q = np.array(seed, dtype=np.float64)

        # 目标变换矩阵
        T_target = target_pose.to_matrix()

        # 约束
        if constraints:
            joint_min = np.array(constraints.get("position_min", [-math.pi] * self.n_joints))
            joint_max = np.array(constraints.get("position_max", [math.pi] * self.n_joints))
        else:
            joint_min = np.array([-math.pi] * self.n_joints)
            joint_max = np.array([math.pi] * self.n_joints)

        # 迭代求解
        for iteration in range(self.max_iterations):
            # 计算当前末端位姿
            current_pose = self.fk.compute(q.tolist())
            T_current = current_pose.to_matrix()

            # 计算位置误差
            position_error = np.array(target_pose.position) - current_pose.position

            # 计算姿态误差 (简化处理)
            orientation_error = np.array(target_pose.orientation) - current_pose.orientation

            # 总误差
            total_error = np.linalg.norm(position_error)

            # 检查收敛
            if total_error < self.tolerance:
                return q.tolist()

            # 计算雅可比矩阵
            J = self._compute_jacobian(q)

            # 阻尼最小二乘法
            try:
                J_T = J.T
                # 误差向量
                error = np.hstack([position_error, orientation_error[:3]])

                # (J^T J + λI) dq = J^T error
                JJT = J_T @ J
                regularization = self.damping * np.eye(self.n_joints)

                if use_null_space:
                    # 零空间优化
                    null_space_proj = np.eye(self.n_joints) - np.linalg.pinv(J @ J_T) @ J
                    dq_null = null_space_proj @ (joint_min + joint_max - 2 * q) * 0.1
                    dq = np.linalg.solve(JJT + regularization, J_T @ error) + dq_null
                else:
                    dq = np.linalg.solve(JJT + regularization, J_T @ error)

                # 更新关节角度
                q = q + dq

                # 应用约束
                q = np.clip(q, joint_min, joint_max)

            except np.linalg.LinAlgError:
                return None

        # 未收敛
        return None

    def _compute_jacobian(self, q: np.ndarray) -> np.ndarray:
        """
        计算几何雅可比矩阵

        Args:
            q: 关节角度

        Returns:
            np.ndarray: 6xN 雅可比矩阵
        """
        n = self.n_joints
        J = np.zeros((6, n))

        # 计算所有关节位置和姿态
        T = np.eye(4)

        # 末端位置
        T_ee = self.fk.compute(q.tolist())
        p_ee = np.array(T_ee.position)

        # 计算每个关节的雅可比列
        for i in range(n):
            # 计算当前关节位置
            T_i = np.eye(4)
            for j in range(i + 1):
                dh = self.dh.params[j]
                T_j = self.fk._dh_transform(dh.a, dh.alpha, dh.d, q[j])
                T_i = T_i @ T_j

            p_i = T_i[:3, 3]

            # 关节轴线方向
            if i == 0:
                z_i = np.array([0, 0, 1])
            else:
                z_i = T_i[:3, 2]

            # 线速度雅可比列
            J[:3, i] = np.cross(z_i, p_ee - p_i)

            # 角速度雅可比列
            J[3:, i] = z_i

        return J


class Jacobian:
    """雅可比矩阵计算工具"""

    @staticmethod
    def compute(
        joint_positions: List[float],
        dh_params: DHParameters
    ) -> np.ndarray:
        """计算雅可比矩阵"""
        fk = ForwardKinematics(dh_params=dh_params)
        jacobian = np.zeros((6, len(joint_positions)))

        # 计算末端位置
        pose = fk.compute(joint_positions)
        p_ee = np.array(pose.position)

        # 累积变换
        T = np.eye(4)

        for i in range(len(joint_positions)):
            dh = dh_params.params[i]
            T_i = fk._dh_transform(dh.a, dh.alpha, dh.d, joint_positions[i])
            T = T @ T_i

            p_i = T[:3, 3]
            z_i = T[:3, 2] if i > 0 else np.array([0, 0, 1])

            jacobian[:3, i] = np.cross(z_i, p_ee - p_i)
            jacobian[3:, i] = z_i

        return jacobian

    @staticmethod
    def analyze(jacobian: np.ndarray) -> JacobianResult:
        """
        分析雅可比矩阵

        Args:
            jacobian: 6xN 雅可比矩阵

        Returns:
            JacobianResult: 分析结果
        """
        # 计算奇异值
        singular_values = np.linalg.svdvals(jacobian)

        # 检查奇异性
        threshold = 1e-3
        is_singular = singular_values[-1] < threshold
        min_sv = singular_values[-1]

        return JacobianResult(
            jacobian=jacobian,
            singular_values=singular_values.tolist(),
            is_singular=is_singular,
            min_singular_value=min_sv
        )


class Kinematics:
    """
    统一运动学接口

    提供正向和逆向运动学的便捷访问
    """

    def __init__(self, arm_type: str = "left"):
        """
        Args:
            arm_type: "left" 或 "right"
        """
        self.arm_type = arm_type
        self.fk = ForwardKinematics(arm_type)
        self.ik = InverseKinematics(arm_type)

    def forward(
        self,
        joint_positions: List[float],
        return_all: bool = False
    ) -> Pose | Tuple[Pose, List[np.ndarray]]:
        """
        正向运动学

        Args:
            joint_positions: 关节角度
            return_all: 是否返回所有变换矩阵

        Returns:
            Pose: 末端位姿
        """
        return self.fk.compute(joint_positions, return_all)

    def inverse(
        self,
        target_pose: Pose,
        seed: Optional[List[float]] = None,
        constraints: Optional[Dict] = None
    ) -> Optional[List[float]]:
        """
        逆向运动学

        Args:
            target_pose: 目标位姿
            seed: 初始猜测
            constraints: 关节约束

        Returns:
            List[float]: 关节角度
        """
        return self.ik.solve(target_pose, seed, constraints=constraints)

    def jacobian(self, joint_positions: List[float]) -> np.ndarray:
        """计算雅可比矩阵"""
        return Jacobian.compute(joint_positions, self.fk.dh)

    def analyze(
        self,
        joint_positions: List[float]
    ) -> JacobianResult:
        """分析运动学特性"""
        J = self.jacobian(joint_positions)
        return Jacobian.analyze(J)
