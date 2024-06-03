import sys
import os
import time
import platform
from typing import List
import numpy as np
from envs.real_env.robot.doosan.doosan_api.api_types import *


class DoosanAPI:

    def __init__(self, ip, dt, max_vel=0.03, max_acc=20 * 3.14 / 180):
        self.ip = ip
        self.dt = dt
        fpath = os.path.dirname(os.path.realpath(__file__))

        if sys.platform.startswith("linux"):
            bit_size = platform.architecture()[0]

            if bit_size == '32bit':
                dll = CDLL(os.path.join(fpath, "dll/linux/32bits/doosan.so"))
            elif bit_size == '64bit':
                dll = CDLL(os.path.join(fpath, "dll/linux/64bits/doosan.so"))
            else:
                print("无法确定系统位数。")
                assert False

        elif sys.platform.startswith("win"):
            dll = CDLL(os.path.join(fpath, "dll/win/doosan.dll"))
        else:
            assert False, sys.platform

        self.dll = dll

        # self.on_rt_monitoring_data_callback = self.get_on_rt_monitoring_data_callback()
        self.on_log_alarm_callback = self.get_on_log_alarm_callback()
        # ################################ BASIC FUNCTION ##########################################################
        self.open_connection = self.dll.open_connection
        self.open_connection.argtypes, self.open_connection.restype = [c_char_p, c_int], c_bool

        self.close_connection = self.dll.close_connection
        self.close_connection.argtypes, self.close_connection.restype = [], c_bool

        self.get_robot_mode = self.dll.get_robot_mode
        self.get_robot_mode.argtypes, self.get_robot_mode.restype = [], ROBOT_MODE

        self.set_robot_mode = self.dll.set_robot_mode
        self.set_robot_mode.argtypes, self.set_robot_mode.restype = [ROBOT_MODE], c_bool
        self.SetRobotMode = self.dll.SetRobotMode
        self.SetRobotMode.argtypes, self.SetRobotMode.restype = [ROBOT_MODE], c_bool

        self.set_safe_stop_reset_type = self.dll.set_safe_stop_reset_type
        self.set_safe_stop_reset_type.argtypes, self.set_safe_stop_reset_type.restype = [SAFE_STOP_RESET_TYPE], \
                                                                                        c_bool
        self.SetSafeStopResetType = self.dll.SetSafeStopResetType
        self.SetSafeStopResetType.argtypes, self.SetSafeStopResetType.restype = [SAFE_STOP_RESET_TYPE], c_bool

        self.get_robot_state = self.dll.get_robot_state
        self.get_robot_state.argtypes, self.get_robot_state.restype = [], ROBOT_STATE
        self.GetRobotState = self.dll.GetRobotState
        self.GetRobotState.argtypes, self.GetRobotState.restype = [], ROBOT_STATE

        self.set_robot_system = self.dll.set_robot_system
        self.set_robot_system.argtypes, self.set_robot_system.restype = [ROBOT_SYSTEM], c_bool
        self.SetRobotSystem = self.dll.SetRobotSystem
        self.SetRobotSystem.argtypes, self.SetRobotSystem.restype = [ROBOT_SYSTEM], c_bool

        self.set_safety_mode = self.dll.set_safety_mode
        self.set_safety_mode.argtypes, self.set_safety_mode.restype = [SAFETY_MODE, SAFETY_MODE_EVENT], c_bool

        self.set_robot_control = self.dll.set_robot_control
        self.set_robot_control.argtypes, self.set_robot_control.restype = [ROBOT_CONTROL], c_bool
        self.SetRobotControl = self.dll.SetRobotControl
        self.SetRobotControl.argtypes, self.SetRobotControl.restype = [ROBOT_CONTROL], c_bool

        self.get_library_version = self.dll.get_library_version
        self.get_library_version.argtypes, self.get_library_version.restype = [], c_char_p

        self.check_motion = self.dll.check_motion
        self.check_motion.argtypes, self.check_motion.restype = [], c_int

        self.ManageAccessControl = self.dll.ManageAccessControl
        self.ManageAccessControl.argtypes, self.ManageAccessControl.restype = [MANAGE_ACCESS_CONTROL], c_bool

        self.set_digital_output = self.dll.set_digital_output
        self.set_digital_output.argtypes, self.set_digital_output.restype = \
            [GPIO_CTRLBOX_DIGITAL_INDEX, c_bool], c_bool

        self.servo_off = self.dll.servo_off
        self.servo_off.argtypes, self.servo_off.restype = [STOP_TYPE], c_bool

        self.change_collision_sensitivity = self.dll.change_collision_sensitivity
        self.change_collision_sensitivity.argtypes, self.change_collision_sensitivity.restype = \
            [c_float], c_bool

        self.release_compliance_ctrl = self.dll.release_compliance_ctrl
        self.release_compliance_ctrl.argtypes, self.release_compliance_ctrl.restype = [], c_bool
        # #################################### SINGLE CONTROL ##################################################
        self.move_home = self.dll.move_home
        self.move_home.argtypes, self.move_home.restype = [MOVE_HOME, c_int], c_bool

        self.stop = self.dll.stop
        self.stop.argtypes, self.stop.restype = [STOP_TYPE], c_bool

        self.movel = self.dll.movel
        self.movel.argtypes, self.movel.restype = [c_float * 6, c_float * 2, c_float * 2,
                                                   c_float, MOVE_MODE, MOVE_REFERENCE, c_float,
                                                   BLENDING_SPEED_TYPE], c_bool

        self.servoj = self.dll.servoj
        self.servoj.argtypes, self.servoj.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                     c_float * NUM_JOINT, c_float], c_bool

        self.speedj = self.dll.speedj
        self.speedj.argtypes, self.speedj.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT, c_float], c_bool

        # ######################################## RT CONTROL ########################################################
        self.connect_rt_control = self.dll.connect_rt_control
        self.connect_rt_control.argtypes, self.connect_rt_control.restype = \
            [c_char_p, c_int], c_bool

        self.disconnect_rt_control = self.dll.disconnect_rt_control
        self.disconnect_rt_control.argtypes, self.disconnect_rt_control.restype = [], c_bool

        self.set_rt_control_input = self.dll.set_rt_control_input
        self.set_rt_control_input.argtypes, self.set_rt_control_input.restype = \
            [c_char_p, c_float, c_int], c_bool

        self.set_rt_control_output = self.dll.set_rt_control_output
        self.set_rt_control_output.argtypes, self.set_rt_control_output.restype = \
            [c_char_p, c_float, c_int], c_bool

        self.start_rt_control = self.dll.start_rt_control
        self.start_rt_control.argtypes, self.start_rt_control.restype = [], c_bool

        self.stop_rt_control = self.dll.stop_rt_control
        self.stop_rt_control.argtypes, self.stop_rt_control.restype = [], c_bool

        self.set_velj_rt = self.dll.set_velj_rt
        self.set_velj_rt.argtypes, self.set_velj_rt.restype = [c_float * NUM_JOINT], c_bool

        self.set_accj_rt = self.dll.set_accj_rt
        self.set_accj_rt.argtypes, self.set_accj_rt.restype = [c_float * NUM_JOINT], c_bool

        self.servoj_rt = self.dll.servoj_rt
        self.servoj_rt.argtypes, self.servoj_rt.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                           c_float * NUM_JOINT, c_float], c_bool

        self.speedj_rt = self.dll.speedj_rt
        self.speedj_rt.argtypes, self.speedj_rt.restype = [c_float * NUM_JOINT, c_float * NUM_JOINT,
                                                           c_float], c_bool

        # ######################################## RT CONTROL ########################################################
        self.initialize(ip=ip, dt=dt, max_vel=max_vel, max_acc=max_acc)

    def get_on_rt_monitoring_data_callback(self):
        @CFUNCTYPE(None, POINTER(RT_OUTPUT_DATA_LIST))
        def on_rt_monitoring_data_callback(rt_output_data_list: POINTER(RT_OUTPUT_DATA_LIST)):
            self.rt_output_data_list = rt_output_data_list.contents
            # print("xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx")
        return on_rt_monitoring_data_callback

    def get_on_log_alarm_callback(self):
        @CFUNCTYPE(None, POINTER(LOG_ALARM))
        def on_log_alarm_callback(log: POINTER(LOG_ALARM)):
            iLevel = log.contents._iLevel
            iGroup = log.contents._iGroup
            iIndex = log.contents._iIndex
            szParam = log.contents._szParam
            szParam = [string_at(szParam[i]).decode("utf-8") for i in range(3)]
            # warnings.warn(f"alarm -- iLevel: {iLevel}, iGroup: {iGroup}, iIndex: {iIndex}, szParam: {szParam}")

        return on_log_alarm_callback

    def initialize(self, ip, dt, max_vel=0.03, max_acc=20 * 3.14 / 180):
        assert self.open_connection(create_string_buffer(ip.encode("utf-8")), 12345), \
            f"fail to connect robot doosan with tcp: {ip}!"
        initialize = self.dll.initialize
        initialize.argtypes, initialize.restype = [], None
        initialize()

        set_on_rt_monitoring_data = self.dll.set_on_rt_monitoring_data
        set_on_rt_monitoring_data.argtypes, set_on_rt_monitoring_data.restype = [TOnRTMonitoringDataCB], None
        # set_on_rt_monitoring_data(self.on_rt_monitoring_data_callback)

        set_on_log_alarm = self.dll.set_on_log_alarm
        set_on_log_alarm.argtypes, set_on_log_alarm.restype = [TOnLogAlarmCB], None
        set_on_log_alarm(self.on_log_alarm_callback)

        assert self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
        assert self.set_robot_system(ROBOT_SYSTEM.ROBOT_SYSTEM_REAL)
        tool = self.get_tool()
        print("tool: ", tool)
        if tool != "RG6":
            print("add tool: ", self.add_tool(name="RG6", weight=1.25, cog=[0.0, 0, 134.3]))
        print("set tool: ", self.set_tool("RG6"))
        print("tool: ", self.get_tool())
        # tcp = self.get_tcp()
        # print("tcp: ", tcp)
        # if tcp != "TCP":
        #     print("add tcp: ", self.add_tcp(name="TCP", position=[0, 0, 0., 0, 0, 0]))
        print("set tcp: ", self.set_tcp("RG6 TCP"))
        print("tcp: ", self.get_tcp())
        assert self.change_collision_sensitivity(10)

        assert self.connect_rt_control(create_string_buffer(ip.encode("utf-8")), 12347), \
            f"fail to connect doosan robot with udp!"
        assert self.set_rt_control_output(create_string_buffer("v1.0".encode("utf-8")), dt, 0)
        assert self.set_velj_rt((c_float * NUM_JOINT)(*((max_vel * 1000,) * NUM_JOINT)))
        assert self.set_accj_rt((c_float * NUM_JOINT)(*((max_acc * 180 / 3.14,) * NUM_JOINT)))
        assert self.start_rt_control()

    def switch_mode(self, control_mode):
        # print(f"old control_mode: {self.control_mode}, new control_mode: {control_mode}")
        if control_mode == "normal":
            self.stop(STOP_TYPE.STOP_TYPE_SLOW)
            time.sleep(0.3)
            self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
            self.set_safety_mode(SAFETY_MODE.SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_STOP)
        elif control_mode == "rt":
            self.set_robot_mode(ROBOT_MODE.ROBOT_MODE_AUTONOMOUS)
            self.set_safety_mode(SAFETY_MODE.SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_MOVE)
        else:
            assert False

    def get_tool(self):
        get_tool = self.dll.get_tool
        get_tool.argtypes, get_tool.restype = [], c_char_p
        return string_at(get_tool()).decode("utf-8")

    def add_tool(self, name: str, weight: float, cog: List[float], inertia: List[float] = None):
        add_tool = self.dll.add_tool
        add_tool.argtypes, add_tool.restype = [c_char_p, c_float, c_float * 3, c_float * NUM_TASK], bool
        cog = np.asarray(cog)
        cog_arr = (c_float * 3)(*cog.tolist())
        if inertia is None:
            inertia = [0., ] * NUM_TASK
        inertia = np.asarray(inertia)
        inertia_arr = (c_float * NUM_TASK)(*inertia.tolist())
        return add_tool(create_string_buffer(name.encode("utf-8")), weight, cog_arr, inertia_arr)

    def set_tool(self, name):
        set_tool = self.dll.set_tool
        set_tool.argtypes, set_tool.restype = [c_char_p], bool
        return set_tool(create_string_buffer(name.encode("utf-8")))

    def get_tcp(self):
        get_tcp = self.dll.get_tcp
        get_tcp.argtypes, get_tcp.restype = [], c_char_p
        return string_at(get_tcp()).decode("utf-8")

    def add_tcp(self, name: str, position: List[float]):
        add_tcp = self.dll.add_tcp
        add_tcp.argtypes, add_tcp.restype = [c_char_p, c_float * NUM_TASK], bool
        position = np.asarray(position)
        position = (c_float * NUM_TASK)(*position.tolist())
        return add_tcp(create_string_buffer(name.encode("utf-8")), position)

    def set_tcp(self, name):
        set_tcp = self.dll.set_tcp
        set_tcp.argtypes, set_tcp.restype = [c_char_p], bool
        return set_tcp(create_string_buffer(name.encode("utf-8")))

    def get_current_posj(self):
        if not hasattr(self, "_get_current_posj"):
            self._get_current_posj = self.dll.get_current_posj
            self._get_current_posj.argtypes = []
            self._get_current_posj.restype = POINTER(ROBOT_POSE)

        p = self._get_current_posj()
        joint_pos = np.array(p.contents._fPosition)
        return joint_pos

    def get_current_velj(self):
        if not hasattr(self, "_get_current_velj"):
            self._get_current_velj = self.dll.get_current_velj
            self._get_current_velj.argtypes = []
            self._get_current_velj.restype = POINTER(ROBOT_VEL)

        p = self._get_current_velj()
        joint_vel = np.array(p.contents._fVelocity)
        return joint_vel

    def get_current_posx(self):
        if not hasattr(self, "_get_current_posx"):
            self._get_current_posx = self.dll.get_current_posx
            self._get_current_posx.argtypes = [COORDINATE_SYSTEM]
            self._get_current_posx.restype = POINTER(ROBOT_TASK_POSE)

        p = self._get_current_posx(COORDINATE_SYSTEM.COORDINATE_SYSTEM_BASE)
        actual_tcp_position = np.array(p.contents._fTargetPos)
        return actual_tcp_position

    def get_current_velx(self):
        if not hasattr(self, "_get_current_velx"):
            self._get_current_velx = self.dll.get_current_velx
            self._get_current_velx.argtypes = []
            self._get_current_velx.restype = POINTER(ROBOT_VEL)

        p = self._get_current_velx()
        actual_tcp_velocity = np.array(p.contents._fVelocity)
        return actual_tcp_velocity

    def get_joint_torque(self):
        if not hasattr(self, "_get_joint_torque"):
            self._get_joint_torque = self.dll.get_joint_torque
            self._get_joint_torque.argtypes, self._get_joint_torque.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_joint_torque()
        return np.array(p.contents._fForce)

    def get_external_torque(self):
        if not hasattr(self, "_get_external_torque"):
            self._get_external_torque = self.dll.get_external_torque
            self._get_external_torque.argtypes, self._get_external_torque.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_external_torque()
        return np.array(p.contents._fForce)

    def get_tool_force(self):
        if not hasattr(self, "_get_tool_force"):
            self._get_tool_force = self.dll.get_tool_force
            self._get_tool_force.argtypes, self._get_tool_force.restype = [], POINTER(ROBOT_FORCE)

        p = self._get_tool_force()
        return np.array(p.contents._fForce)

    def read_data_rt(self):
        if not hasattr(self, "_read_data_rt"):
            self._read_data_rt = self.dll.read_data_rt
            self._read_data_rt.argtypes, self._read_data_rt.restype = [], POINTER(RT_OUTPUT_DATA_LIST)
        p_rt_data_list = self._read_data_rt()
        rt_data_list = p_rt_data_list.contents
        return rt_data_list

    def movej(self, jpos, max_vel=30, max_acc=20, ):
        if not hasattr(self, "_movej"):
            self._movej = self.dll.movej
            self._movej.argtypes, self._movej.restype = [c_float * NUM_JOINT, c_float, c_float,
                                                         c_float, MOVE_MODE, c_float, BLENDING_SPEED_TYPE], c_bool
        jpos = np.asarray(jpos)
        arr_jpos = (c_float * NUM_JOINT)(*jpos.tolist())
        ret = self._movej(arr_jpos, max_vel, max_acc, 0.0, MOVE_MODE.MOVE_MODE_ABSOLUTE, 0,
                          BLENDING_SPEED_TYPE.BLENDING_SPEED_TYPE_DUPLICATE)
        return ret

    def task_compliance_ctrl(self, stiffness, target_time=0.0):
        if not hasattr(self, "_task_compliance_ctrl"):
            self._task_compliance_ctrl = self.dll.task_compliance_ctrl
            self._task_compliance_ctrl.argtypes, self._task_compliance_ctrl.restype = \
                [c_float * NUM_TASK, COORDINATE_SYSTEM, c_float], c_bool
        stiffness = np.asarray(stiffness)
        arr_stiffness = (c_float * NUM_TASK)(*stiffness.tolist())
        self._task_compliance_ctrl(arr_stiffness, COORDINATE_SYSTEM.COORDINATE_SYSTEM_BASE, target_time)

    def amovej(self, jpos, max_vel=30, max_acc=20, target_time=0.0):
        if not hasattr(self, "_amovej"):
            self._amovej = self.dll.amovej
            self._amovej.argtypes, self._amovej.restype = [c_float * NUM_JOINT, c_float, c_float,
                                                           c_float, MOVE_MODE, BLENDING_SPEED_TYPE], c_bool
        jpos = np.asarray(jpos)
        arr_jpos = (c_float * NUM_JOINT)(*jpos.tolist())
        ret = self._amovej(arr_jpos, max_vel, max_acc, target_time, MOVE_MODE.MOVE_MODE_ABSOLUTE,
                           BLENDING_SPEED_TYPE.BLENDING_SPEED_TYPE_DUPLICATE)
        return ret

    def speedl_rt(self, target_vel: np.ndarray, target_acc, target_time):
        assert target_time > 0
        if not hasattr(self, "_speedl_rt"):
            self._speedl_rt = self.dll.speedl_rt
            self._speedl_rt.argtypes, self._speedl_rt.restype = [c_float * NUM_TASK, c_float * NUM_TASK, c_float], c_bool
        target_vel = np.asarray(target_vel)
        # target_vel = np.concatenate([target_vel[:3] * 1000, target_vel[3:] * 180 / 3.14])
        arr_vel = (c_float * NUM_TASK)(*target_vel.tolist())

        target_acc = np.asarray(target_acc)
        # target_acc = np.concatenate([target_acc[:3] * 1000, target_acc[3:] * 180 / 3.14])
        arr_acc = (c_float * NUM_TASK)(*target_acc.tolist())
        # print(f"speedl_rt-target_vel: {target_vel}, target_acc: {target_acc}")
        return self._speedl_rt(arr_vel, arr_acc, target_time)

    def torque_rt(self, torques: np.ndarray, target_time=0.0):
        if not hasattr(self, "_torque_rt"):
            self._torque_rt = self.dll.torque_rt
            self._torque_rt.argtypes, self._torque_rt.restype = [c_float * NUM_JOINT, c_float], c_bool
        torques = np.asarray(torques)
        arr_torque = (c_float * NUM_JOINT)(*torques.tolist())
        return self._torque_rt(arr_torque, target_time)


if __name__ == "__main__":
    import time
    import numpy as np

    np.set_printoptions(precision=3, suppress=True)

    gravity_torque = None

    # on_rt_monitoring_data_callback = self.get_on_rt_monitoring_data_callback()
    # on_log_alarm_callback = self.get_on_log_alarm_callback()

    ip = "192.168.5.110"
    api = DoosanAPI(ip, dt=0.002)
    print("----------------------------2")
    api.stop(STOP_TYPE.STOP_TYPE_SLOW)
    time.sleep(0.3)
    api.set_robot_mode(ROBOT_MODE.ROBOT_MODE_MANUAL)
    assert api.set_safety_mode(SAFETY_MODE.SAFETY_MODE_MANUAL, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_STOP)
    api.movej(np.array([175, -17.5, -84.15, -1.2, -78.28, -92.61]) * 3.14 / 180)
    # api.task_compliance_ctrl([300, 300, 300, 200, 200, 200])
    time.sleep(50)
    target_vel = np.array([0.00, 0.01, 0.0] + [0.0] * 3)
    target_acc = np.array([0.1] * 6)
    api.set_robot_mode(ROBOT_MODE.ROBOT_MODE_AUTONOMOUS)
    assert api.set_safety_mode(SAFETY_MODE.SAFETY_MODE_AUTONOMOUS, SAFETY_MODE_EVENT.SAFETY_MODE_EVENT_MOVE)
    while True:
        if gravity_torque is not None:
            api.torque_rt(api._gravity_torque)
        # api.speedl_rt(target_vel, target_acc, target_time=0.02)
        time.sleep(0.02)
        # print(gravity_torque)
        # if gravity_torque is not None:
        #     assert api.torque_rt(gravity_torque)
        # print(f"mass_matrix: {mass_matrix}")

        # api.task_compliance_ctrl([300, 300, 300, 200, 200, 200])

