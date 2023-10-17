# from dataclasses import dataclass
import math
from typing import List
import vectormath as vmath
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import fsolve, root, broyden1, broyden2
import warnings

from . import defs


class Hexagon:
    def __init__(self, axis_width: float, axis_offset: float, origin_offset: float = 0):
        self._axis_width = axis_width
        self._axis_offset = axis_offset
        self._origin_offset = origin_offset

        
        base_point_a = [self._axis_offset, self._axis_width / 2, self._origin_offset]
        base_point_b = [self._axis_offset, -self._axis_width / 2, self._origin_offset]

        self._points = vmath.Vector3Array([
            base_point_a,
            self._rotate_2d(base_point_b, 120),
            self._rotate_2d(base_point_a, 120),
            self._rotate_2d(base_point_b, 240),
            self._rotate_2d(base_point_a, 240),
            base_point_b,
        ])
        
        # print(self._points)
    
    @property
    def points(self):
        return self._points

    @classmethod
    def _rotate_2d(cls, point: List[float], degrees: float):
        rad = math.radians(degrees)
        return [
            point[0] * math.cos(rad) - point[1] * math.sin(rad),
            point[0] * math.sin(rad) + point[1] * math.cos(rad),
            point[2],
        ]


class Kinematics:
    def __init__(self, base: Hexagon, toolhead: Hexagon, arm_length: float, ball_joint_rod_length: float):
        self._base = base
        self._toolhead = toolhead
        self._arm_length = arm_length
        self._ball_joint_rod_length = ball_joint_rod_length

    @property
    def base(self):
        return self._base
    
    @property
    def toolhead(self):
        return self._toolhead
    
    @property
    def arm_length(self):
        return self._arm_length
    
    @property
    def ball_joint_rod_length(self):
        return self._ball_joint_rod_length

    def calc_fk(self, pos_offset: vmath.Vector3, orientation: R, joint_angles: np.ndarray):
        fk = _FKSolver(self, joint_angles)
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            angles = orientation.as_euler("ZXZ", degrees=False)
        # return fsolve(fk._equation, (pos_offset.x, pos_offset.y, pos_offset.z, angles[0], angles[1], angles[2]), maxfev=1000)
        return root(fk.equation, (pos_offset.x, pos_offset.y, pos_offset.z, angles[0], angles[1], angles[2]), method = "lm").x
        # return broyden2(fk._equation, (pos_offset.x, pos_offset.y, pos_offset.z, angles[0], angles[1], angles[2]))

    def calc_ik(self, pos_offset: vmath.Vector3, orientation: R):
        #transform using axis and angle approach [x, y, z, theta]
        # axis_norm = axis.as_unit()
        # rad = math.radians(degrees)
        # rotation_matrix = R.from_rotvec(axis.as_unit() * degrees, degrees=True).as_matrix()
        # print(rotation_matrix)
        
        # rotated = self._toolhead.points * math.cos(rad) + np.cross(axis_norm, self._toolhead.points) * math.sin(rad) + np.broadcast_to(axis_norm, (6, 3)) * np.dot(self._toolhead.points, axis_norm)[np.newaxis].T * (1 - math.cos(rad))
        # rotated = np.matmul(rotation_matrix, self._toolhead.points.T).T
        rotated = orientation.apply(self._toolhead.points)
        translated = rotated + pos_offset
        L_i = vmath.Vector3Array(translated)

        phi = np.deg2rad(np.array([0, 120, 120, 240, 240, 0]))
        # calculate A, B, and C vectors
        diff_x = L_i.x - self._base.points.x
        diff_y = L_i.y - self._base.points.y
        diff_z = L_i.z - self._base.points.z

        vec_a = 2 * self._arm_length * diff_z
        vec_b = 2 * self._arm_length * ((np.sin(phi) * diff_y) + (np.cos(phi) * diff_x))
        vec_c = np.concatenate((np.full((6, 1), self._arm_length), diff_x[np.newaxis].T, diff_y[np.newaxis].T, diff_z[np.newaxis].T), axis=1)
        vec_c = np.concatenate((np.full((6, 1), pow(self._ball_joint_rod_length, 2)), -np.square(vec_c)), axis=1)
        vec_c = np.sum(vec_c, axis=-1)
        
        theta = np.arccos(vec_c / np.sqrt(np.square(vec_a) + np.square(vec_b))) + np.arctan2(vec_a, vec_b)

        return np.deg2rad(np.rad2deg(theta) - 180)

class _FKSolver:
    def __init__(self, kinematics: Kinematics, joint_angles: np.ndarray):
        self._kinematics = kinematics
        
        phi = np.deg2rad(np.array([0, 120, 120, 240, 240, 0]))
        cos_phi = np.cos(phi)
        sin_phi = np.sin(phi)
        cos_angles = np.cos(joint_angles)
        sin_angles = np.sin(joint_angles)
        self._P_i = np.zeros((6, 3))
        for i in range(6):
            self._P_i[i, 0] = self._kinematics.arm_length * cos_angles[i] * cos_phi[i]
            self._P_i[i, 1] = self._kinematics.arm_length * cos_angles[i] * sin_phi[i]
            self._P_i[i, 2] = self._kinematics.arm_length * sin_angles[i]
        self._P_i = self._P_i + self._kinematics.base.points

        # print(self._kinematics.base.points)
        # print(self._P_i)
        

    
    def equation(self, parameters):
        x, y, z, a, b, c = parameters
        # print(parameters)

        L_i = R.from_euler("ZXZ", np.array([a, b, c]), degrees=False).apply(self._kinematics.toolhead.points) + np.array([x, y, z])
        # print(L_i)
        # print(self._P_i)
        # print(L_i - self._P_i)
        # print(np.square(L_i - self._P_i))
        # print(np.sum(np.square(L_i - self._P_i), axis=1))
        result = np.sum(np.square(L_i - self._P_i), axis=1) - np.square(self._kinematics.ball_joint_rod_length)
        # print(result)
        return result