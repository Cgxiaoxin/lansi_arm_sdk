"""
Lansi Arm SDK 常量定义
"""

from enum import Enum


class MotorMode(Enum):
    """电机控制模式"""
    TORQUE = 0          # 力矩模式
    MIT = 1             # MIT模式
    VELOCITY = 2        # 速度模式
    PROFILE_VELOCITY = 3  # 轨迹速度模式
    POSITION = 4        # 位置模式
    PROFILE_POSITION = 5  # 轨迹位置模式


class ArmState(Enum):
    """机械臂状态"""
    DISCONNECTED = "disconnected"
    CONNECTED = "connected"
    INITIALIZED = "initialized"
    ENABLED = "enabled"
    ERROR = "error"


class ControlTarget(Enum):
    """控制目标"""
    LEFT = "left"
    RIGHT = "right"
    BOTH = "both"


class InterpolationType(Enum):
    """插值类型"""
    LINEAR = "linear"
    CUBIC = "cubic"
    QUINTIC = "quintic"
    B_SPLINE = "b_spline"


class CommandCode:
    """命令码定义 (来自蓝思电机文档)"""

    # 基础命令
    HAND_SHAKE = 0x00           # 握手命令
    R_VERSION = 0x01            # 读取版本信息
    R_ADDR = 0x02               # 读取地址信息
    SAVE_PARA = 0x0D            # 保存参数
    SET_ID = 0x3D               # 设置设备ID
    GO_TO_BOOTLOADER = 0x97     # 进入引导程序模式

    # 电机状态监控命令
    R_CURRENT = 0x04            # 读取当前电流
    R_VELOCITY = 0x05           # 读取当前速度
    R_POSITION = 0x06           # 读取当前位置
    R_ON_OFF = 0x2B             # 读取电机使能状态
    R_VOLTAGE = 0x45            # 读取母线电压
    R_CURRENT_MODE = 0x55        # 读取控制模式
    R_CORE_TEMP = 0x5E           # 读取核心温度
    R_MOTOR_TEMP = 0x5F         # 读取电机温度
    R_INVERTER_TEMP = 0x60       # 读取逆变器温度
    R_CVP = 0x94                # 读取电流、速度、位置
    R_ALARM = 0xFF              # 读取报警信息

    # 电机控制命令
    SET_MODE = 0x07             # 设置控制模式
    SET_CURRENT = 0x08          # 设置电流
    SET_VELOCITY = 0x09         # 设置速度
    SET_POSITION = 0x0A         # 设置位置
    S_ON_OFF = 0x2A             # 设置电机使能

    # MIT控制协议
    SET_MIT = 0x0B              # MIT模式控制

    # 轨迹控制参数配置
    R_PT_V = 0x1C               # 读取位置轨迹最大速度
    R_PT_A = 0x1D               # 读取位置轨迹加速度
    R_PT_D = 0x1E               # 读取位置轨迹减速度
    SET_PT_V = 0x1F             # 设置位置轨迹最大速度
    SET_PT_A = 0x20             # 设置位置轨迹加速度
    SET_PT_D = 0x21             # 设置位置轨迹减速度
    R_VT_V = 0x22               # 读取速度轨迹最大速度
    R_VT_A = 0x23               # 读取速度轨迹加速度
    R_VT_D = 0x24               # 读取速度轨迹减速度
    SET_VT_V = 0x25             # 设置速度轨迹最大速度
    SET_VT_A = 0x26             # 设置速度轨迹加速度
    SET_VT_D = 0x27             # 设置速度轨迹减速度

    # PID参数配置命令
    SET_C_P = 0x0E              # 设置电流环P参数
    SET_C_I = 0x0F              # 设置电流环I参数
    SET_V_P = 0x10              # 设置速度环P参数
    SET_V_I = 0x11              # 设置速度环I参数
    SET_P_P = 0x12              # 设置位置环P参数
    SET_P_I = 0x13              # 设置位置环I参数
    SET_P_D = 0x14              # 设置位置环D参数
    R_C_P = 0x15                # 读取电流环P参数
    R_C_I = 0x16                # 读取电流环I参数
    R_V_P = 0x17                # 读取速度环P参数
    R_V_I = 0x18                # 读取速度环I参数
    R_P_P = 0x19                # 读取位置环P参数
    R_P_I = 0x1A                # 读取位置环I参数
    R_P_D = 0x1B                # 读取位置环D参数
    SET_C_PID_DL = 0x2E         # 设置电流环PID输出限制
    SET_V_PID_DL = 0x30         # 设置速度环PID输出限制
    SET_P_PID_DL = 0x32         # 设置位置环PID输出限制
    R_C_PID_DL = 0x34           # 读取电流环PID输出限制
    R_V_PID_DL = 0x36           # 读取速度环PID输出限制
    R_P_PID_DL = 0x38           # 读取位置环PID输出限制

    # 输入限制命令
    S_C_INPUT_LIMIT = 0x58      # 设置电流输入限制
    R_C_INPUT_LIMIT = 0x59      # 读取电流输入限制
    S_V_INPUT_LIMIT = 0x5A      # 设置速度输入限制
    R_V_INPUT_LIMIT = 0x5B      # 读取速度输入限制
    S_P_MAX_L = 0x83            # 设置位置上限
    S_P_MIN_L = 0x84            # 设置位置下限
    R_P_MAX_L = 0x85            # 读取位置上限
    R_P_MIN_L = 0x86            # 读取位置下限

    # 保护参数命令
    S_INV_PROTECT_TEMP = 0x61   # 设置逆变器保护温度
    R_INV_PROTECT_TEMP = 0x62   # 读取逆变器保护温度
    S_MOTOR_PROTECT_TEMP = 0x6B # 设置电机保护温度
    R_MOTOR_PROTECT_TEMP = 0x6C # 读取电机保护温度
    S_CORE_PROTECT_TEMP = 0x70  # 设置核心保护温度
    R_CORE_PROTECT_TEMP = 0x71   # 读取核心保护温度
    S_UNDER_VOLT = 0x72          # 设置欠压保护值
    R_UNDER_VOLT = 0x73          # 读取欠压保护值
    S_OVER_VOLT = 0x74          # 设置过压保护值
    R_OVER_VOLT = 0x75           # 读取过压保护值
    S_OVER_VEL = 0x76           # 设置超速保护值
    R_OVER_VEL = 0x77           # 读取超速保护值
    S_OVER_CUR = 0x78           # 设置过流保护值
    R_OVER_CUR = 0x79           # 读取过流保护值

    # 电机参数命令
    SET_HOME = 0x87              # 设置原点
    S_MOTOR_L = 0xC0            # 设置电机电感
    R_MOTOR_L = 0xC1            # 读取电机电感
    S_MOTOR_R = 0xC2            # 设置电机电阻
    R_MOTOR_R = 0xC3            # 读取电机电阻
    S_GEAR_RATIO = 0xC4         # 设置减速比
    R_GEAR_RATIO = 0xC5         # 读取减速比
    S_CALI_START = 0xC7          # 开始校准
    S_CALI_CURRENT = 0xC8       # 设置校准电流
    R_CALI_CURRENT = 0xC9       # 读取校准电流
    S_TORQUE_COEFF = 0xCA       # 设置转矩系数
    R_TORQUE_COEFF = 0xCB       # 读取转矩系数

    # MIT范围参数命令
    S_MIT_P_UPPER = 0xA0        # 设置mit位置上限
    S_MIT_P_LOWER = 0xA1        # 设置mit位置下限
    S_MIT_V_UPPER = 0xA2        # 设置mit速度上限
    S_MIT_V_LOWER = 0xA3        # 设置mit速度下限
    S_MIT_T_UPPER = 0xA4        # 设置mit力矩上限
    S_MIT_T_LOWER = 0xA5        # 设置mit力矩下限
    S_MIT_KP_RANGE = 0xA6        # 设置mit Kp范围
    S_MIT_KD_RANGE = 0xA7        # 设置mit Kd范围
    R_MIT_P_UPPER = 0xA8        # 读取mit位置上限
    R_MIT_P_LOWER = 0xA9        # 读取mit位置下限
    R_MIT_V_UPPER = 0xAA        # 读取mit速度上限
    R_MIT_V_LOWER = 0xAB        # 读取mit速度下限
    R_MIT_T_UPPER = 0xAC        # 读取mit力矩上限
    R_MIT_T_LOWER = 0xAD        # 读取mit力矩下限
    R_MIT_KP_RANGE = 0xAE       # 读取mit Kp范围
    R_MIT_KD_RANGE = 0xAF       # 读取mit Kd范围

    # 报警处理命令
    CLEAR_ALARM = 0xFE          # 清除报警
    R_ALARM = 0xFF             # 读取报警信息


class ErrorCode:
    """错误码定义"""
    SUCCESS = 0

    # 通信错误 (1xx)
    ERR_CAN_NOT_CONNECTED = 100
    ERR_CAN_SEND_FAILED = 101
    ERR_CAN_RECV_TIMEOUT = 102
    ERR_CAN_INVALID_ID = 103

    # 控制错误 (2xx)
    ERR_NOT_INITIALIZED = 200
    ERR_NOT_ENABLED = 201
    ERR_JOINT_LIMIT = 202
    ERR_MOTOR_ALARM = 203
    ERR_EMERGENCY_STOP = 204

    # 运动学错误 (3xx)
    ERR_IK_NO_SOLUTION = 300
    ERR_IK_MAX_ITERATIONS = 301
    ERR_SINGULARITY = 302
    ERR_INVALID_POSE = 303

    # 参数错误 (4xx)
    ERR_INVALID_PARAMETER = 400
    ERR_INVALID_JOINT_INDEX = 401
    ERR_INVALID_POSITION = 402


class MotorSpec:
    """电机规格常量"""
    # 默认关节限制 (弧度)
    DEFAULT_POSITION_LIMIT = 3.14  # ±180度
    DEFAULT_VELOCITY_LIMIT = 2.0  # rad/s
    DEFAULT_ACCELERATION_LIMIT = 5.0  # rad/s²

    # 数据格式
    FLOAT_BYTES = 4
    POSITION_BYTES = 4
    VELOCITY_BYTES = 4
    CURRENT_BYTES = 4
