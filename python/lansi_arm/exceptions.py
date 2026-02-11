"""
Lansi Arm SDK 异常定义
"""


class LansiArmError(Exception):
    """SDK基础异常类"""
    def __init__(self, message: str, details: dict | None = None):
        super().__init__(message)
        self.message = message
        self.details = details or {}
        self.timestamp: float = 0.0


class CommunicationError(LansiArmError):
    """通信异常"""
    pass


class CANError(CommunicationError):
    """CAN通信异常"""
    pass


class CANNotConnectedError(CANError):
    """CAN总线未连接"""
    pass


class CANSendError(CANError):
    """CAN发送失败"""
    pass


class CANReceiveError(CANError):
    """CAN接收失败"""
    pass


class ModbusError(CommunicationError):
    """Modbus通信异常"""
    pass


class ControlError(LansiArmError):
    """控制异常"""
    pass


class ArmInitializationError(ControlError):
    """机械臂初始化异常"""
    pass


class JointLimitError(ControlError):
    """关节限制异常"""
    pass


class MotorError(ControlError):
    """电机异常"""
    pass


class MotorNotEnabledError(MotorError):
    """电机未使能"""
    pass


class MotorAlarmError(MotorError):
    """电机报警"""
    pass


class KinematicsError(LansiArmError):
    """运动学异常"""
    pass


class IKError(KinematicsError):
    """逆运动学求解异常"""
    pass


class FKError(KinematicsError):
    """正运动学计算异常"""
    pass


class SingularityError(KinematicsError):
    """奇异性异常"""
    pass


class ConfigurationError(LansiArmError):
    """配置异常"""
    pass


class InvalidParameterError(ConfigurationError):
    """无效参数异常"""
    pass


class TimeoutError(LansiArmError):
    """超时异常"""
    pass
