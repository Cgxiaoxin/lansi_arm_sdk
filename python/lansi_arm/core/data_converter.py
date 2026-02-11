"""
数据转换工具

提供浮点数与字节数组之间的转换，支持IEEE 754格式（小端序）。
"""


import struct
from typing import List, Tuple


class DataConverter:
    """
    数据转换器

    支持：
    - 浮点数 ↔ 字节数组 (IEEE 754, 小端序)
    - 整数 ↔ 字节数组
    - 位置/速度/电流数据转换
    """

    # 格式字符串
    _FLOAT_LE = "<f"    # 小端序float
    _DOUBLE_LE = "<d"   # 小端序double
    _INT_LE = "<i"      # 小端序int
    _UINT_LE = "<I"     # 小端序unsigned int
    _SHORT_LE = "<h"    # 小端序short
    _USHORT_LE = "<H"   # 小端序unsigned short

    def __init__(self):
        """初始化数据转换器"""
        pass

    def float_to_bytes(self, value: float) -> bytes:
        """
        将浮点数转换为4字节数组 (IEEE 754, 小端序)

        Args:
            value: 浮点数值

        Returns:
            bytes: 4字节数组

        Example:
            >>> converter = DataConverter()
            >>> converter.float_to_bytes(3.14159)
            b'\\xd0\\x0f\\x49@'
        """
        return struct.pack(self._FLOAT_LE, value)

    def bytes_to_float(self, data: bytes | List[int]) -> float:
        """
        将4字节数组转换为浮点数

        Args:
            data: 字节数据 (至少4字节)

        Returns:
            float: 浮点数值

        Raises:
            ValueError: 数据长度不足4字节

        Example:
            >>> converter = DataConverter()
            >>> converter.bytes_to_float([0xD0, 0x0F, 0x49, 0x40])
            3.1415927...
        """
        if len(data) < 4:
            raise ValueError(f"数据长度不足4字节，实际: {len(data)}")

        if isinstance(data, list):
            data = bytes(data)

        return struct.unpack(self._FLOAT_LE, data[:4])[0]

    def double_to_bytes(self, value: float) -> bytes:
        """将双精度浮点数转换为8字节数组"""
        return struct.pack(self._DOUBLE_LE, value)

    def bytes_to_double(self, data: bytes | List[int]) -> float:
        """将8字节数组转换为双精度浮点数"""
        if len(data) < 8:
            raise ValueError(f"数据长度不足8字节，实际: {len(data)}")

        if isinstance(data, list):
            data = bytes(data)

        return struct.unpack(self._DOUBLE_LE, data[:8])[0]

    def int_to_bytes(self, value: int, length: int = 4) -> bytes:
        """
        将整数转换为字节数组

        Args:
            value: 整数值
            length: 字节长度 (1, 2, 4)

        Returns:
            bytes: 字节数组

        Raises:
            ValueError: 无效的字节长度
        """
        if length == 1:
            return bytes([value & 0xFF])
        elif length == 2:
            return struct.pack(self._SHORT_LE, value)
        elif length == 4:
            return struct.pack(self._INT_LE, value)
        else:
            raise ValueError(f"不支持的字节长度: {length}")

    def bytes_to_int(self, data: bytes | List[int], length: int = 4) -> int:
        """
        将字节数组转换为整数

        Args:
            data: 字节数据
            length: 字节长度 (1, 2, 4)

        Returns:
            int: 整数值
        """
        if isinstance(data, list):
            data = bytes(data)

        if length == 1:
            return data[0]
        elif length == 2:
            return struct.unpack(self._SHORT_LE, data[:2])[0]
        elif length == 4:
            return struct.unpack(self._INT_LE, data[:4])[0]
        else:
            raise ValueError(f"不支持的字节长度: {length}")

    def uint_to_bytes(self, value: int, length: int = 4) -> bytes:
        """将无符号整数转换为字节数组"""
        if length == 1:
            return bytes([value & 0xFF])
        elif length == 2:
            return struct.pack(self._USHORT_LE, value)
        elif length == 4:
            return struct.pack(self._UINT_LE, value)
        else:
            raise ValueError(f"不支持的字节长度: {length}")

    def bytes_to_uint(self, data: bytes | List[int], length: int = 4) -> int:
        """将字节数组转换为无符号整数"""
        if isinstance(data, list):
            data = bytes(data)

        if length == 1:
            return data[0]
        elif length == 2:
            return struct.unpack(self._USHORT_LE, data[:2])[0]
        elif length == 4:
            return struct.unpack(self._UINT_LE, data[:4])[0]
        else:
            raise ValueError(f"不支持的字节长度: {length}")

    def position_to_bytes(self, position: float) -> bytes:
        """
        将位置值转换为4字节

        Args:
            position: 位置值 (弧度)

        Returns:
            bytes: 4字节数组
        """
        return self.float_to_bytes(position)

    def bytes_to_position(self, data: bytes | List[int]) -> float:
        """将4字节转换为位置值"""
        return self.bytes_to_float(data)

    def velocity_to_bytes(self, velocity: float) -> bytes:
        """
        将速度值转换为4字节

        Args:
            velocity: 速度值 (rad/s)

        Returns:
            bytes: 4字节数组
        """
        return self.float_to_bytes(velocity)

    def bytes_to_velocity(self, data: bytes | List[int]) -> float:
        """将4字节转换为速度值"""
        return self.bytes_to_float(data)

    def current_to_bytes(self, current: float) -> bytes:
        """
        将电流值转换为4字节

        Args:
            current: 电流值 (A)

        Returns:
            bytes: 4字节数组
        """
        return self.float_to_bytes(current)

    def bytes_to_current(self, data: bytes | List[int]) -> float:
        """将4字节转换为电流值"""
        return self.bytes_to_float(data)

    def command_to_bytes(self, cmd: int, data: bytes | List[int] = b"\x00") -> bytes:
        """
        将命令码和数据转换为CAN消息格式

        Args:
            cmd: 命令码 (0x00-0xFF)
            data: 数据部分

        Returns:
            bytes: 完整的8字节CAN数据
        """
        if isinstance(data, list):
            data = bytes(data)

        # 构建消息: [CMD, data..., padding]
        message = bytes([cmd]) + data
        padding_needed = 8 - len(message)

        if padding_needed > 0:
            message = message + bytes(padding_needed)
        elif padding_needed < 0:
            message = message[:8]

        return message

    def parse_command_response(
        self,
        data: bytes | List[int]
    ) -> Tuple[int, bytes]:
        """
        解析命令响应

        Args:
            data: 原始响应数据

        Returns:
            Tuple[命令码, 数据部分]
        """
        if isinstance(data, list):
            data = bytes(data)

        if len(data) < 1:
            raise ValueError("响应数据为空")

        cmd = data[0]
        response_data = data[1:]

        return cmd, response_data

    @staticmethod
    def create_crc(data: bytes, polynomial: int = 0x1021, init_val: int = 0xFFFF) -> int:
        """
        简单的CRC校验计算

        Args:
            data: 输入数据
            polynomial: 多项式
            init_val: 初始值

        Returns:
            int: CRC值
        """
        crc = init_val
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ polynomial) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc
