"""
测试配置模块
"""
import pytest
from lansi_arm.config import (
    ArmType,
    ArmConfig,
    JointConfig,
    LEFT_ARM,
    RIGHT_ARM,
    get_arm_config,
    get_joint_limits,
    get_motor_id_map,
    DEFAULT_JOINT_LIMITS,
)


class TestArmType:
    """机械臂类型测试类"""

    def test_arm_type_values(self):
        """测试机械臂类型值"""
        assert ArmType.LEFT.value == "left"
        assert ArmType.RIGHT.value == "right"
        assert ArmType.BOTH.value == "both"

    def test_arm_type_count(self):
        """测试机械臂类型数量"""
        assert len(ArmType) == 3


class TestJointConfig:
    """关节配置测试类"""

    def test_joint_config_creation(self):
        """测试关节配置创建"""
        config = JointConfig(
            index=0,
            name="joint_1",
            min_position=-3.14,
            max_position=3.14,
            max_velocity=2.0,
            max_acceleration=5.0
        )
        assert config.index == 0
        assert config.name == "joint_1"
        assert config.min_position == -3.14
        assert config.max_position == 3.14


class TestArmConfig:
    """机械臂配置测试类"""

    def test_arm_config_creation(self):
        """测试机械臂配置创建"""
        joints = [
            JointConfig(
                index=i,
                name=f"joint_{i}",
                min_position=-3.14,
                max_position=3.14
            )
            for i in range(7)
        ]

        config = ArmConfig(
            arm_type=ArmType.LEFT,
            joints=joints,
            motor_ids=list(range(51, 58)),
            base_link="left_base_link",
            end_link="left_end_link"
        )

        assert config.arm_type == ArmType.LEFT
        assert len(config.joints) == 7
        assert len(config.motor_ids) == 7


class TestArmConstants:
    """机械臂常量测试类"""

    def test_left_arm_constants(self):
        """测试左臂常量"""
        assert LEFT_ARM["type"] == "left"
        assert len(LEFT_ARM["motor_ids"]) == 7
        assert LEFT_ARM["motor_ids"][0] == 51

    def test_right_arm_constants(self):
        """测试右臂常量"""
        assert RIGHT_ARM["type"] == "right"
        assert len(RIGHT_ARM["motor_ids"]) == 7
        assert RIGHT_ARM["motor_ids"][0] == 61


class TestConfigFunctions:
    """配置函数测试类"""

    def test_get_arm_config_left(self):
        """测试获取左臂配置"""
        config = get_arm_config("left")
        assert config is not None
        assert config.arm_type == ArmType.LEFT

    def test_get_arm_config_right(self):
        """测试获取右臂配置"""
        config = get_arm_config("right")
        assert config is not None
        assert config.arm_type == ArmType.RIGHT

    def test_get_arm_config_invalid(self):
        """测试无效机械臂配置"""
        with pytest.raises(ValueError):
            get_arm_config("invalid")

    def test_get_joint_limits(self):
        """测试获取关节限制"""
        limits = get_joint_limits("left")
        assert limits is not None
        assert len(limits) == 7

    def test_get_motor_id_map(self):
        """测试获取电机ID映射"""
        motor_map = get_motor_id_map("left")
        assert motor_map is not None
        assert 51 in motor_map
        assert 57 in motor_map


class TestDefaultJointLimits:
    """默认关节限制测试类"""

    def test_default_limits_exist(self):
        """测试默认限制存在"""
        assert DEFAULT_JOINT_LIMITS is not None
        assert len(DEFAULT_JOINT_LIMITS) == 7

    def test_default_limits_values(self):
        """测试默认限制值"""
        limits = DEFAULT_JOINT_LIMITS
        for i in range(7):
            assert limits[i]["min"] < limits[i]["max"]
            assert limits[i]["max_velocity"] > 0
            assert limits[i]["max_acceleration"] > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
