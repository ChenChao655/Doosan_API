from ctypes import *
import enum


NUM_JOINT = 6
NUMBER_OF_JOINT = 6
NUM_TASK = 6
NUM_FLANGE_IO = 6
NUM_BUTTON = 5
NUM_BUTTON_EX = 6
NUMBER_OF_TASK = 6
MAX_STRING_SIZE = 256


class IntEnum(enum.IntEnum):

    # 注意必须将添加这个方法，ctyps 要求，将obj转为 init型
    @classmethod
    def from_param(cls, obj):
        return int(obj)


class ROBOT_STATE(IntEnum):
    STATE_INITIALIZING = 0
    STATE_STANDBY = 1
    STATE_MOVING = 2
    STATE_SAFE_OFF = 3
    STATE_TEACHING = 4
    STATE_SAFE_STOP = 5
    STATE_EMERGENCY_STOP = 6
    STATE_HOMMING = 7
    STATE_RECOVERY = 8
    STATE_SAFE_STOP2 = 9
    STATE_SAFE_OFF2 = 10
    STATE_RESERVED1 = 11
    STATE_RESERVED2 = 12
    STATE_RESERVED3 = 13
    STATE_RESERVED4 = 14
    STATE_NOT_READY = 15
    STATE_LAST = 16


class SAFE_STOP_RESET_TYPE(IntEnum):
    SAFE_STOP_RESET_TYPE_DEFAULT = 0
    SAFE_STOP_RESET_TYPE_PROGRAM_STOP = SAFE_STOP_RESET_TYPE_DEFAULT
    SAFE_STOP_RESET_TYPE_PROGRAM_RESUME = 1


class ROBOT_CONTROL(IntEnum):
    CONTROL_INIT_CONFIG = 0
    CONTROL_ENABLE_OPERATION = 1
    CONTROL_RESET_SAFET_STOP = 2
    CONTROL_RESET_SAFET_OFF = 3
    CONTROL_RECOVERY_SAFE_STOP = 4
    CONTROL_RECOVERY_SAFE_OFF = 5
    CONTROL_RECOVERY_BACKDRIVE = 6
    CONTROL_RESET_RECOVERY = 7
    CONTROL_LAST = 8


class STOP_TYPE(IntEnum):
    STOP_TYPE_QUICK_STO = 0
    STOP_TYPE_QUICK = 1
    STOP_TYPE_SLOW = 2
    STOP_TYPE_HOLD = 3
    STOP_TYPE_EMERGENCY = STOP_TYPE_HOLD


class MANAGE_ACCESS_CONTROL(IntEnum):
    MANAGE_ACCESS_CONTROL_FORCE_REQUEST = 0
    MANAGE_ACCESS_CONTROL_REQUEST = 1
    MANAGE_ACCESS_CONTROL_RESPONSE_YES = 2
    MANAGE_ACCESS_CONTROL_RESPONSE_NO = 3


class MONITORING_ACCESS_CONTROL(IntEnum):
    MONITORING_ACCESS_CONTROL_REQUEST = 0
    MONITORING_ACCESS_CONTROL_DENY = 1
    MONITORING_ACCESS_CONTROL_GRANT = 2
    MONITORING_ACCESS_CONTROL_LOSS = 3
    MONITORING_ACCESS_CONTROL_LAST = 4


class ROBOT_MODE(IntEnum):
    ROBOT_MODE_MANUAL = 0  # executing a single action (e.g., jog action) for which the TCP velocity of
    # the edge of the robot is restricted to 250 mm/sec for safety
    ROBOT_MODE_AUTONOMOUS = 1
    ROBOT_MODE_RECOVERY = 2
    ROBOT_MODE_BACKDRIVE = 3
    ROBOT_MODE_MEASURE = 4
    ROBOT_MODE_INITIALIZE = 5
    ROBOT_MODE_LAST = 6


class ROBOT_SYSTEM(IntEnum):
    ROBOT_SYSTEM_REAL = 0
    ROBOT_SYSTEM_VIRTUAL = 1
    ROBOT_SYSTEM_LAST = 2


class SAFETY_MODE(IntEnum):
    SAFETY_MODE_MANUAL = 0
    SAFETY_MODE_AUTONOMOUS = 1
    SAFETY_MODE_RECOVERY = 2
    SAFETY_MODE_BACKDRIVE = 3
    SAFETY_MODE_MEASURE = 4
    SAFETY_MODE_INITIALIZE = 5
    SAFETY_MODE_LAST = 6


class SAFETY_MODE_EVENT(IntEnum):
    SAFETY_MODE_EVENT_ENTER = 0
    SAFETY_MODE_EVENT_MOVE = 1
    SAFETY_MODE_EVENT_STOP = 2
    SAFETY_MODE_EVENT_LAST = 3


class JOG_AXIS(IntEnum):
    JOG_AXIS_JOINT_1 = 0
    JOG_AXIS_JOINT_2 = 1
    JOG_AXIS_JOINT_3 = 2
    JOG_AXIS_JOINT_4 = 3
    JOG_AXIS_JOINT_5 = 4
    JOG_AXIS_JOINT_6 = 5
    JOG_AXIS_TASK_X = 6
    JOG_AXIS_TASK_Y = 7
    JOG_AXIS_TASK_Z = 8
    JOG_AXIS_TASK_RX = 9
    JOG_AXIS_TASK_RY = 10
    JOG_AXIS_TASK_RZ = 11


class COORDINATE_SYSTEM(IntEnum):
    COORDINATE_SYSTEM_BASE = 0
    COORDINATE_SYSTEM_TOOL = 1
    COORDINATE_SYSTEM_WORLD = 2
    COORDINATE_SYSTEM_USER_MIN = 101
    COORDINATE_SYSTEM_USER_MAX = 200


class MOVE_REFERENCE(IntEnum):
    MOVE_REFERENCE_BASE = COORDINATE_SYSTEM.COORDINATE_SYSTEM_BASE
    MOVE_REFERENCE_TOOL = COORDINATE_SYSTEM.COORDINATE_SYSTEM_TOOL
    MOVE_REFERENCE_WORLD = COORDINATE_SYSTEM.COORDINATE_SYSTEM_WORLD
    MOVE_REFERENCE_USER_MIN = COORDINATE_SYSTEM.COORDINATE_SYSTEM_USER_MIN
    MOVE_REFERENCE_USER_MAX = COORDINATE_SYSTEM.COORDINATE_SYSTEM_USER_MAX


class STOP_TYPE(IntEnum):
    STOP_TYPE_QUICK_STO = 0
    STOP_TYPE_QUICK = 1
    STOP_TYPE_SLOW = 2
    STOP_TYPE_HOLD = 3
    STOP_TYPE_EMERGENCY = STOP_TYPE_HOLD


class MOVE_MODE(IntEnum):
    MOVE_MODE_ABSOLUTE = 0
    MOVE_MODE_RELATIVE = 1


class BLENDING_SPEED_TYPE(IntEnum):
    BLENDING_SPEED_TYPE_DUPLICATE = 0
    BLENDING_SPEED_TYPE_OVERRIDE = 1


class ROBOT_SPACE(IntEnum):
    ROBOT_SPACE_JOINT = 0
    ROBOT_SPACE_TASK = 1


class GPIO_CTRLBOX_DIGITAL_INDEX(IntEnum):
    GPIO_CTRLBOX_DIGITAL_INDEX_1 = 0
    GPIO_CTRLBOX_DIGITAL_INDEX_2 = 1
    GPIO_CTRLBOX_DIGITAL_INDEX_3 = 2
    GPIO_CTRLBOX_DIGITAL_INDEX_4 = 3
    GPIO_CTRLBOX_DIGITAL_INDEX_5 = 4
    GPIO_CTRLBOX_DIGITAL_INDEX_6 = 5
    GPIO_CTRLBOX_DIGITAL_INDEX_7 = 6
    GPIO_CTRLBOX_DIGITAL_INDEX_8 = 7
    GPIO_CTRLBOX_DIGITAL_INDEX_9 = 8
    GPIO_CTRLBOX_DIGITAL_INDEX_10 = 9
    GPIO_CTRLBOX_DIGITAL_INDEX_11 = 10
    GPIO_CTRLBOX_DIGITAL_INDEX_12 = 11
    GPIO_CTRLBOX_DIGITAL_INDEX_13 = 12
    GPIO_CTRLBOX_DIGITAL_INDEX_14 = 13
    GPIO_CTRLBOX_DIGITAL_INDEX_15 = 14
    GPIO_CTRLBOX_DIGITAL_INDEX_16 = 15


class MANAGE_ACCESS_CONTROL(IntEnum):
    MANAGE_ACCESS_CONTROL_FORCE_REQUEST = 0
    MANAGE_ACCESS_CONTROL_REQUEST = 1
    MANAGE_ACCESS_CONTROL_RESPONSE_YES = 2
    MANAGE_ACCESS_CONTROL_RESPONSE_NO = 3


class MOVE_HOME(IntEnum):
    MOVE_HOME_MECHANIC = 0
    MOVE_HOME_USER = 1


class ROBOT_POSE(Structure):
    _fields_ = [('_fPosition', c_float * NUM_JOINT)]


class ROBOT_TASK_POSE(Structure):
    _fields_ = [('_fTargetPos', c_float * NUM_TASK), ('_iTargetSol', c_char)]


class ROBOT_VEL(Structure):
    _fields_ = [('_fVelocity', c_float * NUM_TASK)]


class ROBOT_FORCE(Structure):
    _fields_ = [('_fForce', c_float * NUM_TASK)]


class LOG_ALARM(Structure):
    _fields_ = [
        ("_iLevel", c_ubyte),                           # message level
        ("_iGroup", c_ubyte),                           # group no
        ("_iIndex", c_uint),                            # message no
        ("_szParam", (c_char * MAX_STRING_SIZE) * 3)  # message param
    ]


class RT_OUTPUT_DATA_LIST(Structure):
    _fields_ = [
        ('time_stamp', c_double),
        ('actual_joint_position', c_float * NUM_JOINT),
        ('actual_joint_position_abs', c_float * NUM_JOINT),
        ('actual_joint_velocity', c_float * NUM_JOINT),
        ('actual_joint_velocity_abs', c_float * NUM_JOINT),
        ('actual_tcp_position', c_float * NUM_TASK),
        ('actual_tcp_velocity', c_float * NUM_TASK),
        ('actual_flange_position', c_float * NUM_TASK),
        ('actual_flange_velocity', c_float * NUM_TASK),
        ('actual_motor_torque', c_float * NUM_JOINT),
        ('actual_joint_torque', c_float * NUM_JOINT),
        ('raw_joint_torque', c_float * NUM_JOINT),
        ('raw_force_torque', c_float * NUM_JOINT),
        ('external_joint_torque', c_float * NUM_JOINT),
        ('external_tcp_force', c_float * NUM_TASK),
        ('target_joint_position', c_float * NUM_JOINT),
        ('target_joint_velocity', c_float * NUM_JOINT),
        ('target_joint_acceleration', c_float * NUM_JOINT),
        ('target_motor_torque', c_float * NUM_JOINT),
        ('target_tcp_position', c_float * NUM_TASK),
        ('target_tcp_velocity', c_float * NUM_TASK),
        ('jacobian_matrix', (c_float * NUM_JOINT) * NUM_JOINT),
        ('gravity_torque', c_float * NUM_JOINT),
        ('coriolis_matrix', (c_float * NUM_JOINT) * NUM_JOINT),
        ('mass_matrix', (c_float * NUM_JOINT) * NUM_JOINT),
        ('solution_space', c_short),
        ('singularity', c_float),
        ('operation_speed_rate', c_float),
        ('joint_temperature', c_float * NUM_JOINT),
        ('controller_digital_input', c_short),
        ('controller_digital_output', c_short),
        ('controller_analog_input_type', c_char * 2),
        ('controller_analog_input', c_float * 2),
        ('controller_analog_output_type', c_char * 2),
        ('controller_analog_output', c_float * 2),
        ('flange_digital_input', c_char),
        ('flange_digital_output', c_char),
        ('flange_analog_input', c_float * 4),
        ('external_encoder_strobe_count', c_char * 2),
        ('external_encoder_count', c_int * 2),
        ('goal_joint_position', c_float * NUM_JOINT),
        ('goal_tcp_position', c_float * NUM_TASK),
        ('robot_mode', c_char),
        ('robot_state', c_char),
        ('control_mode', c_short),
        ('reserved', c_char * 256),
    ]


class TEST(Structure):
    _fields_ = [
        ("value", (c_float * 7) * 6)
    ]


TOnLogAlarmCB = CFUNCTYPE(None, POINTER(LOG_ALARM))
TOnRTMonitoringDataCB = CFUNCTYPE(None, POINTER(RT_OUTPUT_DATA_LIST))



