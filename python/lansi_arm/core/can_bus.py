"""
CAN总线通信模块

提供CAN总线的发送和接收功能，支持Linux SocketCAN。
"""


import threading
import time
from typing import Callable, Optional, Dict, List
from dataclasses import dataclass, field
from collections import defaultdict
import can
from can import Message as CANMessage

from .data_converter import DataConverter
from ..constants import CommandCode
from ..exceptions import (
    CANError,
    CANNotConnectedError,
    CANSendError,
    CANReceiveError,
)


@dataclass
class CANConfig:
    """CAN配置"""
    channel: str = "can0"
    bitrate: int = 1000000
    timeout: float = 0.1
    auto_reconnect: bool = True
    reconnect_interval: float = 1.0


@dataclass
class CANMessageData:
    """CAN消息数据"""
    arbitration_id: int
    data: bytes
    timestamp: float = 0.0
    is_extended_id: bool = False
    is_remote: bool = False


class CanBus:
    """
    CAN总线通信类

    封装CAN总线的发送和接收操作，支持：
    - Linux SocketCAN
    - 消息发送和接收
    - 回调函数注册
    - 自动重连
    """

    def __init__(
        self,
        config: Optional[CANConfig] = None,
        data_converter: Optional[DataConverter] = None
    ):
        """
        Args:
            config: CAN配置
            data_converter: 数据转换器
        """
        self.config = config or CANConfig()
        self.converter = data_converter or DataConverter()

        self._bus: Optional[can.interface.Bus] = None
        self._running = False
        self._recv_thread: Optional[threading.Thread] = None
        self._callbacks: Dict[int, List[Callable]] = defaultdict(list)
        self._lock = threading.RLock()

    @property
    def is_connected(self) -> bool:
        """检查是否已连接"""
        return self._bus is not None and self._running

    def connect(self) -> bool:
        """
        连接CAN总线

        Returns:
            bool: 是否连接成功

        Raises:
            CANError: 连接失败
        """
        with self._lock:
            try:
                # 创建CAN总线接口
                self._bus = can.interface.Bus(
                    channel=self.config.channel,
                    interface="socketcan",
                    bitrate=self.config.bitrate
                )

                # 启动接收线程
                self._running = True
                self._recv_thread = threading.Thread(
                    target=self._receive_loop,
                    daemon=True
                )
                self._recv_thread.start()

                return True

            except Exception as e:
                self._bus = None
                self._running = False
                raise CANError(f"CAN连接失败: {e}")

    def disconnect(self) -> None:
        """断开CAN总线"""
        with self._lock:
            self._running = False

            if self._recv_thread and self._recv_thread.is_alive():
                self._recv_thread.join(timeout=2.0)

            if self._bus:
                try:
                    self._bus.shutdown()
                except Exception:
                    pass
                self._bus = None

    def send(
        self,
        msg_id: int,
        data: bytes,
        timeout: Optional[float] = None
    ) -> bool:
        """
        发送CAN消息

        Args:
            msg_id: CAN消息ID (电机ID)
            data: 数据载荷 (1-8字节)
            timeout: 超时时间 (秒)

        Returns:
            bool: 是否发送成功

        Raises:
            CANNotConnectedError: CAN总线未连接
            CANSendError: 发送失败
        """
        if not self.is_connected:
            raise CANNotConnectedError("CAN总线未连接")

        timeout = timeout or self.config.timeout

        try:
            # 构建CAN消息
            message = CANMessage(
                arbitration_id=msg_id,
                data=data[:8],  # 最多8字节
                is_extended_id=False,
                is_remote=False
            )

            # 发送消息
            self._bus.send(message, timeout=timeout)
            return True

        except Exception as e:
            raise CANSendError(f"CAN发送失败: {e}")

    def send_command(
        self,
        motor_id: int,
        command_code: int,
        data: bytes | List[int] = None
    ) -> bool:
        """
        发送命令到电机

        Args:
            motor_id: 电机ID (51-67)
            command_code: 命令码
            data: 数据部分

        Returns:
            bool: 是否发送成功
        """
        if data is None:
            data = b"\x00"

        if isinstance(data, list):
            data = bytes(data)

        # 构建命令消息
        message = self.converter.command_to_bytes(command_code, data)

        return self.send(motor_id, message)

    def recv(
        self,
        msg_id: Optional[int] = None,
        timeout: float = 0.1
    ) -> Optional[CANMessageData]:
        """
        接收CAN消息

        Args:
            msg_id: 消息ID过滤 (None表示接收所有)
            timeout: 超时时间 (秒)

        Returns:
            CANMessageData: 消息数据，超时返回None
        """
        if not self.is_connected:
            raise CANNotConnectedError("CAN总线未连接")

        try:
            start_time = time.time()
            timeout_seconds = timeout

            while time.time() - start_time < timeout_seconds:
                # 检查消息
                message = self._bus.recv(timeout=0.01)

                if message:
                    # 如果指定了msg_id，检查ID
                    if msg_id is None or message.arbitration_id == msg_id:
                        return CANMessageData(
                            arbitration_id=message.arbitration_id,
                            data=bytes(message.data),
                            timestamp=message.timestamp,
                            is_extended_id=message.is_extended_id,
                            is_remote=message.is_remote
                        )

            return None

        except Exception as e:
            raise CANReceiveError(f"CAN接收失败: {e}")

    def recv_response(
        self,
        motor_id: int,
        expected_command: int,
        timeout: float = 0.1
    ) -> Optional[bytes]:
        """
        接收电机响应

        Args:
            motor_id: 电机ID
            expected_command: 期望的命令码
            timeout: 超时时间

        Returns:
            bytes: 响应数据，超时返回None
        """
        response = self.recv(motor_id, timeout)

        if response is None:
            return None

        # 解析响应
        cmd, data = self.converter.parse_command_response(response.data)

        # 验证命令码匹配
        if cmd == expected_command:
            return data
        elif cmd == CommandCode.R_ALARM:
            # 报警响应，提取报警信息
            return data
        else:
            # 可能是异步状态更新，忽略
            return None

    def register_callback(
        self,
        msg_id: int,
        callback: Callable[[CANMessageData], None]
    ) -> None:
        """
        注册消息回调

        Args:
            msg_id: 消息ID
            callback: 回调函数
        """
        with self._lock:
            self._callbacks[msg_id].append(callback)

    def unregister_callback(
        self,
        msg_id: int,
        callback: Callable[[CANMessageData], None]
    ) -> None:
        """
        取消注册消息回调

        Args:
            msg_id: 消息ID
            callback: 回调函数
        """
        with self._lock:
            if callback in self._callbacks[msg_id]:
                self._callbacks[msg_id].remove(callback)

    def _receive_loop(self) -> None:
        """接收消息循环"""
        while self._running:
            try:
                # 使用短超时以支持优雅退出
                message = self._bus.recv(timeout=0.1)

                if message:
                    # 构建消息数据
                    msg_data = CANMessageData(
                        arbitration_id=message.arbitration_id,
                        data=bytes(message.data),
                        timestamp=message.timestamp,
                        is_extended_id=message.is_extended_id,
                        is_remote=message.is_remote
                    )

                    # 触发回调
                    self._dispatch_message(msg_data)

            except Exception:
                # 接收错误，继续循环
                time.sleep(0.01)

    def _dispatch_message(self, message: CANMessageData) -> None:
        """分发消息到回调函数"""
        msg_id = message.arbitration_id

        # 获取对应的回调
        callbacks = []
        with self._lock:
            # 精确匹配
            if msg_id in self._callbacks:
                callbacks.extend(self._callbacks[msg_id])

        # 调用回调
        for callback in callbacks:
            try:
                callback(message)
            except Exception:
                pass

    def get_status(self) -> Dict:
        """
        获取CAN总线状态

        Returns:
            dict: 状态信息
        """
        return {
            "connected": self.is_connected,
            "channel": self.config.channel,
            "bitrate": self.config.bitrate,
            "callbacks_registered": sum(len(cb) for cb in self._callbacks.values())
        }

    def __enter__(self):
        """上下文管理器入口"""
        self.connect()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        """上下文管理器退出"""
        self.disconnect()
        return False
