"""
测试数据转换器
"""
import pytest
import struct
from lansi_arm.core.data_converter import DataConverter


class TestDataConverter:
    """数据转换器测试类"""

    def test_float_to_bytes_positive(self):
        """测试正数转换"""
        value = 1.5
        result = DataConverter.float_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_float(result)
        assert abs(converted_back - value) < 1e-6

    def test_float_to_bytes_negative(self):
        """测试负数转换"""
        value = -2.7
        result = DataConverter.float_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_float(result)
        assert abs(converted_back - value) < 1e-6

    def test_float_to_bytes_zero(self):
        """测试零值转换"""
        value = 0.0
        result = DataConverter.float_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_float(result)
        assert abs(converted_back - value) < 1e-9

    def test_float_to_bytes_pi(self):
        """测试π值转换"""
        value = 3.14159265359
        result = DataConverter.float_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_float(result)
        assert abs(converted_back - value) < 1e-5

    def test_position_to_bytes(self):
        """测试位置值转换"""
        value = 1.234
        result = DataConverter.position_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_position(result)
        assert abs(converted_back - value) < 1e-4

    def test_velocity_to_bytes(self):
        """测试速度值转换"""
        value = 2.5
        result = DataConverter.velocity_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_velocity(result)
        assert abs(converted_back - value) < 1e-4

    def test_current_to_bytes(self):
        """测试电流值转换"""
        value = 1.5
        result = DataConverter.current_to_bytes(value)
        assert len(result) == 4
        converted_back = DataConverter.bytes_to_current(result)
        assert abs(converted_back - value) < 1e-4

    def test_bytes_to_invalid_length(self):
        """测试无效字节长度"""
        with pytest.raises(ValueError):
            DataConverter.bytes_to_float(b'\x00\x01\x02')

    def test_rad_to_deg(self):
        """测试弧度转角度"""
        import math
        assert abs(DataConverter.rad_to_deg(math.pi) - 180.0) < 1e-6
        assert abs(DataConverter.rad_to_deg(math.pi / 2) - 90.0) < 1e-6

    def test_deg_to_rad(self):
        """测试角度转弧度"""
        import math
        assert abs(DataConverter.deg_to_rad(180) - math.pi) < 1e-6
        assert abs(DataConverter.deg_to_rad(90) - math.pi / 2) < 1e-6


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
