import numpy as np
import numpy.typing as npt
import vectormath as vmath
from scipy.spatial.transform import Slerp, Rotation as R
from enum import Enum, auto
import threading
from queue import Queue, Empty
import time
from collections import deque
from typing import Deque, List


from .kinematics import Kinematics
from .comms_manager import CommsManager
from . import defs


class InterpMode(Enum):
    LINEAR = auto()


class MotionState(Enum):
    INIT = auto()
    READY = auto()
    RUNNING_ACTIVE_COMPLIANCE = auto()
    RUNNING_MOTION_DEMO = auto()
    RUNNING_REMOTE_CONTROL = auto()


class PosOri:
    def __init__(self, pos: vmath.Vector3, ori: R):
        self._pos = pos
        self._ori = ori
        self._start_timestamp = 0.0
        self._duration = 0.0
        self._interp_mode = InterpMode.LINEAR

    @property
    def pos(self):
        return self._pos
    
    @property
    def ori(self):
        return self._ori
    
    def __repr__(self) -> str:
        ret = "(" + str(self._pos.x) + ", " + str(self._pos.y) + ", " + str(self._pos.z) + "); " + str(self._ori.as_rotvec()) + "; " + str(self._start_timestamp) + "; " + str(self._duration)
        return ret


class MotionPlanner:
    def __init__(self, kinematics: Kinematics):
        self._kinematics = kinematics
        self._state_lock = threading.Lock()
        self._state = MotionState.INIT
        self._prior_state = MotionState.INIT
        self._latest_pos_ori = PosOri(vmath.Vector3(0, 0, 215), R.from_rotvec([0, 0, 0]))
        self._waypoints: List[PosOri] = []
        self._calib_joint_angles: np.ndarray = np.full(6, defs.CALIB_ANGLE_OFFSET)
        self._state_queue = Queue()
        
        self._loop_thread = threading.Thread(target=self._loop_thread_func, daemon=True)
        self._loop_thread.start()

        # CommsManager.send_message("CT")

    def home(self):
        if self._state == MotionState.INIT:
            CommsManager.send_message("MAP150")
            CommsManager.send_message("MAI0")
            CommsManager.send_message("MAD0.1")
            CommsManager.send_message("MLU5")
            CommsManager.send_message("CH")
            CommsManager.send_message("CP")
            # CommsManager.send_message("@1")
            # CommsManager.send_message("#6")
            # time.sleep(0.1)
            # CommsManager.get_all_replies()
            # CommsManager.send_message("MMG6")
            # time.sleep(0.1)
            # angles = CommsManager.get_all_replies()
            # # print(angles)
            # angles_float = []
            # if len(angles) != 6:
            #     print("Error, failed to calibrate")
            #     return
            # for i, angle in enumerate(angles):
            #     try:
            #         angle_float = float(angle)
            #     except ValueError:
            #         print("Error, failed to parse joint angle from joint " + str(i))
            #         return
            #     angles_float.append(defs.CALIB_ANGLE_OFFSET - angle_float)
            # self._calib_joint_angles = np.array(angles_float)
            # # print(self._calib_joint_angles)

            # CommsManager.send_message("CV0.1")
            # CommsManager.send_message("CA0.000005")
            # time.sleep(0.1)
            # CommsManager.send_motor_angles(np.array([0, 0, 0, 0, 0, 0]) - self._calib_joint_angles)
            # time.sleep(2.0)

            CommsManager.send_motor_angles(np.array([0, 0, 0, 0, 0, 0]) - self._calib_joint_angles)
            self._state = MotionState.READY
    
    def skip_home(self):
        if self._state == MotionState.INIT:
            CommsManager.send_message("MAP150")
            CommsManager.send_message("MAI0")
            CommsManager.send_message("MAD0.1")
            CommsManager.send_message("MLU5")
            CommsManager.send_message("CP")
            CommsManager.send_motor_angles(np.array([0, 0, 0, 0, 0, 0]) - self._calib_joint_angles)
            self._state = MotionState.READY
        

    def _add_waypoint(self, pos_ori: PosOri, duration: float, interp_mode: InterpMode = InterpMode.LINEAR):
        if len(self._waypoints) <= 0:
            self._waypoints.append(self._latest_pos_ori)
        
        last_point = self._waypoints[-1]
        last_point._duration = duration
        last_point._interp_mode = interp_mode
        self._waypoints.append(pos_ori)

    def _clear_waypoints(self):
        # print("clearing")
        self._waypoints.clear()

    def _get_pos_ori_angles(self, pos_ori: PosOri):
        return self._kinematics.calc_ik(pos_ori.pos, pos_ori.ori) - self._calib_joint_angles

    def _go_to_pos_ori(self, pos_ori: PosOri):
        if self._state != MotionState.INIT:
            angles = self._get_pos_ori_angles(pos_ori)
            CommsManager.send_motor_angles(angles)
            self._latest_pos_ori = pos_ori
            # print(pos_ori)
            
    def is_ready(self):
        return self._state != MotionState.INIT
    
    def waypoints_complete(self):
        return len(self._waypoints) <= 0

    def _loop_thread_func(self):
    # def update(self):
        while True:
            try:
                new_state = self._state_queue.get_nowait()
                self._state = new_state
            except Empty:
                pass

            do_init = self._prior_state != self._state
            self._prior_state = self._state

            if do_init:
                # print("doinit true")
                self._clear_waypoints()

            if self._state == MotionState.INIT:
                time.sleep(0.01)
            elif self._state == MotionState.READY:
                time.sleep(0.01)
            elif self._state == MotionState.RUNNING_ACTIVE_COMPLIANCE:
                self._run_active_compliance(do_init)
            elif self._state == MotionState.RUNNING_MOTION_DEMO:
                self._run_motion_demo(do_init)
            elif self._state == MotionState.RUNNING_REMOTE_CONTROL:
                pass
            
            if self.is_ready():
                self._run_waypoints()
            
            time.sleep(0)

    def _run_waypoints(self):
        now = time.monotonic()
        if len(self._waypoints) > 0:
            first_point = self._waypoints[0]
            if first_point._start_timestamp <= 0:
                first_point._start_timestamp = now
            # print(str(first_point._start_timestamp) + " = " + str(now) + " - " + str(first_point._duration))
            if now >= first_point._start_timestamp + first_point._duration:
                # print("removing first point")
                self._waypoints = self._waypoints[1:]
                first_point = self._waypoints[0]
                first_point._start_timestamp = now
        else:
            return

        if len(self._waypoints) > 1:
            curr_point = self._waypoints[0]
            next_point = self._waypoints[1]

            interp_factor = (now - curr_point._start_timestamp) / curr_point._duration
            interp_factor = max(0, min(1, interp_factor))
            # print(str(interp_factor) + " = " + str(now) + " - " + str(curr_point._start_timestamp))

            pos_interp = (next_point.pos - curr_point.pos) * interp_factor + curr_point.pos
            ori_interp = Slerp([0, 1], R.concatenate([curr_point.ori, next_point.ori]))(interp_factor)
            self._go_to_pos_ori(PosOri(pos_interp, ori_interp))
        else:
            self._waypoints.clear()

    def run_active_compliance(self):
        self._state_queue.put(MotionState.RUNNING_ACTIVE_COMPLIANCE)

    def run_motion_demo(self):
        self._state_queue.put(MotionState.RUNNING_MOTION_DEMO)

    def run_remote_control(self):
        self._state_queue.put(MotionState.RUNNING_REMOTE_CONTROL)

    def run_idle(self):
        self._state_queue.put(MotionState.READY)

    def _run_motion_demo(self, do_init: bool):
        if do_init:
            CommsManager.send_message("MAP100")
            CommsManager.send_message("MAI0")
            CommsManager.send_message("MAD0.1")
            CommsManager.send_message("MLU9")

            # CommsManager.send_message("CV1")
            # CommsManager.send_message("CA0.5")
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 3.0)

        if self.waypoints_complete():
            # self._add_waypoint(PosOri(vmath.Vector3([0, 0, 300]), R.from_rotvec([0, 0, 0])), 1.0)
            # self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 1.0)
            # self._add_waypoint(PosOri(vmath.Vector3([0, 0, 300]), R.from_rotvec([0, 0, 0])), 1.0)
            # self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 1.0)

            self._add_waypoint(PosOri(vmath.Vector3([50, 0, 350]), R.from_rotvec([0, 0, 0])), 0.5)
            self._add_waypoint(PosOri(vmath.Vector3([50, 0, 300]), R.from_rotvec([0, 0, 0])), 0.5)
            self._add_waypoint(PosOri(vmath.Vector3([-50, 0, 300]), R.from_rotvec([0, 0, 0])), 1.0)
            self._add_waypoint(PosOri(vmath.Vector3([-50, 0, 400]), R.from_rotvec([0, 0, 0])), 1.0)
            self._add_waypoint(PosOri(vmath.Vector3([50, 0, 400]), R.from_rotvec([0, 0, 0])), 1.0)
            self._add_waypoint(PosOri(vmath.Vector3([50, 0, 350]), R.from_rotvec([0, 0, 0])), 0.5)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 0.5)
            
            # self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 0.5)

            tilt_angle = 20
            rot_angle = 40
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([tilt_angle, 0, 0], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([-tilt_angle, 0, 0], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, tilt_angle, 0], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, -tilt_angle, 0], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, rot_angle], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, -rot_angle], True)), 1)
            self._add_waypoint(PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0])), 0.5)


        # for i in range(100):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 250 + i]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.001)

        # for i in range(100):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 350 - i]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.001)




        # for i in range(30):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 300]), vmath.Vector3([1, 0, 0]), i) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.01)

        # for i in range(30):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 300]), vmath.Vector3([1, 0, 0]), 30 - i) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.01)

        # for i in range(30):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 300]), vmath.Vector3([1, 0, 0]), -i) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.01)

        # for i in range(30):
        #     angles = kin.calc_ik(vmath.Vector3([0, 0, 300]), vmath.Vector3([1, 0, 0]), -30 + i) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.01)





        # for i in range(50):
        #     angles = kin.calc_ik(vmath.Vector3([i, 0, 300]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.005)

        # for i in range(50):
        #     angles = kin.calc_ik(vmath.Vector3([50, i, 300]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.005)

        # for i in range(50):
        #     angles = kin.calc_ik(vmath.Vector3([50 - i, 50, 300]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.005)
            
        # for i in range(50):
        #     angles = kin.calc_ik(vmath.Vector3([0, 50 - i, 300]), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "M", angles)
        #     time.sleep(0.005)

    
    def _run_active_compliance(self, do_init):
        if do_init:
            CommsManager.send_message("MAP50")
            CommsManager.send_message("MAI0")
            CommsManager.send_message("MAD0.1")
            CommsManager.send_message("MLU10")

            CommsManager.send_message("CV0.1")
            CommsManager.send_message("CA0.000005")
            CommsManager.get_all_replies()
            
            init_posori = PosOri(vmath.Vector3([0, 0, 350]), R.from_rotvec([0, 0, 0]))
            self._add_waypoint(init_posori, 3.0)
            self._angles = self._get_pos_ori_angles(init_posori)
            self._buffer = [""] * len(self._angles)
        
        if self.waypoints_complete():
            CommsManager.send_message("MMG6")
            for i, reply in enumerate(CommsManager.get_all_replies()):
                self._buffer[i] += reply
                strs = self._buffer[i].split("\r\n")
                if len(strs) > 1:
                    try:
                        self._angles[i] = float(strs[-2])
                    except ValueError:
                        pass
                    self._buffer[i] = strs[-1]

            # print(self._angles)

            self._kinematics.calc_fk(self._latest_pos_ori.pos, self._latest_pos_ori.ori, self._angles)

            


        # global error_buffer
        # setpoint = vmath.Vector3([0, 0, 350])
        # # setpoint = vmath.Vector3([0, 0, 0])
        # angles = kin.calc_ik(setpoint, vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        # send_array_open(open_ports, "T", angles)
        # # send_array_open(open_ports, "M", np.array([0, 0, 0, 0, 0, 0]))


        # send_message_open(open_ports, "@1")
        # send_message_open(open_ports, "#6")
        # send_message_open(open_ports, "CV0.1")
        # send_message_open(open_ports, "CA0.000005")

        # for port in open_ports:
        #         port.read_all()

        # last_time = time.time()
        # counter = 0
        # angles = [0] * 6
        # buffer = [""] * 6

        # b, a = iirfilter(20, Wn=30, fs=80, btype="low", ftype="butter")
        # lpf = LiveLFilter(b, a)
        # while True:
        #     send_message_open(open_ports, "MMG6")
        #     for i, port in enumerate(open_ports):
        #         buffer[i] += port.read_all().decode('UTF-8')
        #         strs = buffer[i].split("\r\n")
        #         if len(strs) > 1:
        #             try:
        #                 angles[i] = float(strs[-2])
        #             except ValueError:
        #                 pass
        #             buffer[i] = strs[-1]
        #     func_timer = time.perf_counter()
        #     actual = kin.calc_fk(setpoint, vmath.Vector3([0, 0, 1]), 0, np.array(angles) - math.radians(10))
        #     func_timer = time.perf_counter() - func_timer
        #     # error = (setpoint - actual[0:3]) * 0.1
        #     error = vmath.Vector3(setpoint - actual[0:3])
        #     # error[2] = 0

        #     # mag = lpf(error.length)
        #     # if error.length > 0:
        #     #     error = error.as_length(mag)

        #     with error_buffer_lock:
        #         error_buffer[time.time()] = np.array([func_timer, error.length, 0])
            
        #     error[2] = error[2] - 2.25

        #     k_term = 0.5
        #     max_offset = 2
        #     offset_coefficient = 0.8
        #     for i in range(3):
        #         if actual[i] - setpoint[i] > max_offset:
        #             # setpoint[i] += (actual[i] - max_offset) * k_term
        #             # setpoint[i] -= (error[i] + max_offset * offset_coefficient) * k_term
        #             setpoint[i] -= (error[i]) * k_term
        #         elif setpoint[i] - actual[i] > max_offset:
        #             # setpoint[i] = (actual[i] + max_offset) * k_term
        #             # setpoint[i] -= (error[i] - max_offset * offset_coefficient) * k_term
        #             setpoint[i] -= (error[i]) * k_term

            


        #     if setpoint[0] > 30:
        #         setpoint[0] = 30
        #     elif setpoint[0] < -30:
        #         setpoint[0] = -30

        #     if setpoint[1] > 30:
        #         setpoint[1] = 30
        #     elif setpoint[1] < -30:
        #         setpoint[1] = -30

        #     # commanded_angles = kin.calc_ik(setpoint + vmath.Vector3(error), vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     commanded_angles = kin.calc_ik(setpoint, vmath.Vector3([0, 0, 1]), 0) + math.radians(10)
        #     send_array_open(open_ports, "T", commanded_angles)

        #     counter = counter + 1
        #     if (time.time() > last_time + 1):
        #         last_time = time.time()
        #         # print(counter)
        #         counter = 0

        #     # print(angles)
        #     time.sleep(0)
