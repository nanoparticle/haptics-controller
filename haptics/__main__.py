import time
import numpy as np
import math
import vectormath as vmath
from scipy.spatial.transform import Slerp, Rotation as R

from .kinematics import Hexagon, Kinematics
from . import defs
from .motion_planner import MotionPlanner
from .comms_manager import CommsManager
from .gui import GUI


if __name__ == "__main__":
    np.set_printoptions(linewidth=5000, suppress=True)
    # libusb_package.get_libusb1_backend() # Loads libusb, pyusb will not find the libusb backend without calling this function
    
    CommsManager.connect()
    base = Hexagon(defs.DIST_BETWEEN_ADJACENT_ARMS_MM, defs.DIST_FROM_CENTER_TO_MOTOR_AXIS_MM, defs.DIST_FROM_FLOOR_TO_MOTOR_AXIS_MM)
    toolhead = Hexagon(defs.DIST_BETWEEN_ADJACENT_BALL_JOINTS_MM, defs.DIST_FROM_CENTER_TO_BALL_JOINT_AXIS_MM, -defs.DIST_FROM_TOOLHEAD_CENTER_TO_BALL_JOINT_PLANE_MM)
    kin = Kinematics(base, toolhead, defs.DIST_FROM_ARM_AXIS_TO_BALL_JOINT_MM, defs.DIST_BETWEEN_EACH_BALL_JOINT_MM)
    mp = MotionPlanner(kin)

    # h = kin.calc_fk(vmath.Vector3(0, 0, 162), R.from_rotvec([0, 0, 0]) , np.array([0, 0, 0, 0, 0, 0]))
    # print(h)

    # CommsManager.go_to_UF2_mode()
    # CommsManager.clear_prefs()
    # exit()

    # CommsManager.send_message("CH")
    # exit()
    
    last_time = time.monotonic_ns()
    # mp.home()
    GUI.init_gui_blocking(mp)

    

    # CommsManager.send_message("MAP150")
    # CommsManager.send_message("MAP50")
    # CommsManager.send_message("MAI0")
    # CommsManager.send_message("MAD0.1")
    # CommsManager.send_message("MLU3")

    # CommsManager.send_message("CP")

    # angle = math.radians(-117.456348)
    # CommsManager.send_motor_angles(np.full(6, angle))
