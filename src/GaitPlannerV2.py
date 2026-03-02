from src.HardwareInterface import HardwareInterface
from src.Rotation import orientation_kinematics
from src.StancePlannerV2 import StancePlanner
from src.SwingPlannerV2 import SwingPlanner
from src.InertiaBalancer import InertiaBalancer
from src.Kinematics import inverse_kinematics
import numpy as np
import time
import ctypes
import os

class ServoAngles(ctypes.Structure):
    _fields_ = [
        ("theta_ab", ctypes.c_double),
        ("theta_thigh", ctypes.c_double),
        ("theta_shin", ctypes.c_double)
    ]

lib_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'leg_ik.so')
ik_lib = ctypes.CDLL(lib_path)

ik_lib.LegIK_new.restype = ctypes.c_void_p
ik_lib.LegIK_ik.argtypes = [ctypes.c_void_p, ctypes.c_double, ctypes.c_double, ctypes.c_double]
ik_lib.LegIK_ik.restype = ServoAngles
ik_lib.LegIK_delete.argtypes = [ctypes.c_void_p]

class CppLegIK:
    def __init__(self):
        self.obj = ik_lib.LegIK_new()
        
    def get_ik(self, x, y, z, leg_index):
        res = ik_lib.LegIK_ik(self.obj, float(x), float(y), float(z), int(leg_index))
        return res.theta_ab, res.theta_thigh, res.theta_shin
        
    def __del__(self):
        ik_lib.LegIK_delete(self.obj)

fast_ik = CppLegIK()

# TODO: Implement terrain adaptation

class GaitPlanner:
    def __init__(self, config, state, command):
        self.config = config
        self.state = state
        self.command = command
        self.hardware_interface = HardwareInterface()
        self.stance_planner = StancePlanner(self.state, self.config)
        self.swing_planner = SwingPlanner(self.state, self.config)
        self.balancer = InertiaBalancer(self.config)
        self.state.legpair_start_time = [time.time(), time.time()]
        self.last_loop_time = time.time()
        self.driven_velocityX = 0.0
        self.driven_velocityY = 0.0

    def get_swing_trajectory(self, dt):
        method = self.config.swing_method.upper()

        if method == "BEZIER":
            return self.swing_planner.bezier_discretizer(dt)
        elif method == "TRIANGULAR":
            return self.swing_planner.triangular_discretizer(dt)
        elif method == "CHEETAH":
            return self.swing_planner.MIT_cheetah_bezier_discretizer(dt)
        else:
            raise ValueError("Wrong method in config")

    def get_stance_trajectory(self, dt):
        method = self.config.stance_method.upper()
        if method == "LINEAR":
            return self.stance_planner.linear_discretizer(dt)
        elif method == "BEZIER":
            return self.stance_planner.stance_bezier_discretizer(dt)
        else:
            raise ValueError("Wrong method in config")

    def trot_begin(self):
        self.state.leg_pair_in_swing = [False, False]
        now = time.time()
        self.state.legpair_start_time = [now, now - 3.0/3.0 * self.config.stancetime]
        self.state.velocityX = 0.0
        self.state.velocityY = 0.0
        self.state.trot_yaw = 0.0
        self.state.stance_yaw_pair = [0.0, 0.0]
        self.last_loop_time = now
        self.actuate_pair(0, now)
        self.actuate_pair(1, now)

    def actuate_pair(self, pair_index, current_time):
        dt = current_time - self.state.legpair_start_time[pair_index]

        if self.state.leg_pair_in_swing[pair_index]:
            dt = min(dt, self.config.swingtime) # Max time can be swing time
            x, y, z = self.get_swing_trajectory(dt)
            s = min(dt/self.config.swingtime,1.0)
            smooth_factor = 6 * s ** 5 - 15 * s ** 4 + 10 * s ** 3
            self.state.stance_yaw_pair[pair_index] = self.config.stancetime * self.state.trot_yaw * (0.5 - smooth_factor)

        else:
            dt = min(dt, self.config.stancetime) # Max time can be stance time
            x, y, z = self.get_stance_trajectory(dt)
            s = min(dt/self.config.stancetime,1.0)
            self.state.stance_yaw_pair[pair_index] = self.state.trot_yaw * self.config.stancetime * (s - 0.5)


        # Clip Yaw limits
        limit = abs(self.state.trot_yaw * self.config.stancetime)
        self.state.stance_yaw_pair[pair_index] = np.clip(self.state.stance_yaw_pair[pair_index], -limit, limit)

        pos_vec = np.array([x, y, z - self.config.body_height])
        for leg_index in self.config.leg_pairs[pair_index, :]:
            pos_with_offset = pos_vec + np.array([0, self.config.abduction_offsets[leg_index], 0])
            final_pos = orientation_kinematics(pos_with_offset, self.state.stance_yaw_pair[pair_index],0, 0, leg_index, self.config)
            angles = fast_ik.get_ik(final_pos[0],final_pos[1],final_pos[2], leg_index)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(angles[motor_index], leg_index, motor_index)
                self.state.foot_locations[motor_index, leg_index] = final_pos[motor_index]

    def trot_cycle_actuated(self):
        current_time = time.time()
        for i in range(2):
            dt = current_time - self.state.legpair_start_time[i]
            duration = self.config.swingtime if self.state.leg_pair_in_swing[i] else self.config.stancetime

            if dt >= duration:
                self.state.leg_pair_in_swing[i] = not self.state.leg_pair_in_swing[i]
                self.state.legpair_start_time[i] = current_time

        loop_dt = current_time - self.last_loop_time
        self.last_loop_time = current_time

        diff_x = self.command.velocityX - self.driven_velocityX
        diff_y = self.command.velocityY - self.driven_velocityY
        magnitude = np.sqrt(diff_x ** 2 + diff_y ** 2)

        max_step = self.config.max_acceleration * loop_dt

        if magnitude > max_step:
            scale = max_step / magnitude
            self.driven_velocityX += diff_x * scale
            self.driven_velocityY += diff_y * scale
        else:
            self.driven_velocityX = self.command.velocityX
            self.driven_velocityY = self.command.velocityY

        diff_yaw = self.command.trot_yaw - self.state.trot_yaw
        max_yaw_step = self.config.max_yaw_acceleration * loop_dt

        if abs(diff_yaw) > max_yaw_step:
            self.state.trot_yaw += np.sign(diff_yaw) * max_yaw_step
        else:
            self.state.trot_yaw = self.command.trot_yaw

        roll, pitch, gx, gy = self.hardware_interface.get_imu_tilt()

        #emergency_stop(roll, pitch)

        velocity_offsetx, velocity_offsety = self.balancer.velocity_offset(loop_dt, pitch, roll, gx, gy)

        self.state.velocityX = self.driven_velocityX + velocity_offsetx
        self.state.velocityY = self.driven_velocityY + velocity_offsety

        self.state.velocityX = np.clip(self.state.velocityX, -self.config.max_velocityX, self.config.max_velocityX)
        self.state.velocityY = np.clip(self.state.velocityY, -self.config.max_velocityY, self.config.max_velocityY)

        self.actuate_pair(0, current_time)
        self.actuate_pair(1, current_time)
