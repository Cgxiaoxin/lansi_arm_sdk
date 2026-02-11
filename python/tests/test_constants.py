"""
测试常量定义
"""
import pytest
from lansi_arm.constants import (
    MotorMode,
    ArmState,
    ControlTarget,
    InterpolationType,
    CommandCode,
    ErrorCode,
)


class TestMotorMode:
    """电机模式测试类"""

    def test_motor_mode_values(self):
        """测试电机模式枚举值"""
        assert MotorMode.TORQUE.value == 0
        assert MotorMode.MIT.value == 1
        assert MotorMode.VELOCITY.value == 2
        assert MotorMode.PROFILE_VELOCITY.value == 3
        assert MotorMode.POSITION.value == 4
        assert MotorMode.PROFILE_POSITION.value == 5

    def test_motor_mode_count(self):
        """测试电机模式数量"""
        assert len(MotorMode) == 6


class TestArmState:
    """机械臂状态测试类"""

    def test_arm_state_values(self):
        """测试机械臂状态枚举值"""
        assert ArmState.DISCONNECTED.value == "disconnected"
        assert ArmState.CONNECTED.value == "connected"
        assert ArmState.INITIALIZED.value == "initialized"
        assert ArmState.ENABLED.value == "enabled"
        assert ArmState.ERROR.value == "error"

    def test_arm_state_count(self):
        """测试机械臂状态数量"""
        assert len(ArmState) == 5


class TestControlTarget:
    """控制目标测试类"""

    def test_control_target_values(self):
        """测试控制目标枚举值"""
        assert ControlTarget.LEFT.value == "left"
        assert ControlTarget.RIGHT.value == "right"
        assert ControlTarget.BOTH.value == "both"

    def test_control_target_count(self):
        """测试控制目标数量"""
        assert len(ControlTarget) == 3


class TestInterpolationType:
    """插值类型测试类"""

    def test_interpolation_type_values(self):
        """测试插值类型枚举值"""
        assert InterpolationType.LINEAR.value == "linear"
        assert InterpolationType.CUBIC.value == "cubic"
        assert InterpolationType.QUINTIC.value == "quintic"
        assert InterpolationType.B_SPLINE.value == "b_spline"

    def test_interpolation_type_count(self):
        """测试插值类型数量"""
        assert len(InterpolationType) == 4


class TestCommandCode:
    """命令码测试类"""

    def test_command_code_not_none(self):
        """测试命令码定义不为空"""
        assert CommandCode.HAND_SHAKE == 0x00
        assert CommandCode.SET_POSITION == 0x0A
        assert CommandCode.SET_MIT == 0x0B
        assert CommandCode.SET_PT_V == 0x1F

    def test_command_code_range(self):
        """测试命令码范围"""
        assert 0x00 <= CommandCode.HAND_SHAKE <= 0xFF
        assert 0x00 <= CommandCode.R_ALARM <= 0xFF


class TestErrorCode:
    """错误码测试类"""

    def test_error_code_success(self):
        """测试成功错误码"""
        assert ErrorCode.SUCCESS == 0

    def test_error_code_ranges(self):
        """测试错误码范围"""
        # 通信错误 (1xx)
        assert 100 <= ErrorCode.ERR_CAN_NOT_CONNECTED <= 199
        # 控制错误 (2xx)
        assert 200 <= ErrorCode.ERR_NOT_INITIALIZED <= 299
        # 运动学错误 (3xx)
        assert 300 <= ErrorCode.ERR_IK_NO_SOLUTION <= 399
        # 参数错误 (4xx)
        assert 400 <= ErrorCode.ERR_INVALID_PARAMETER <= 499


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
