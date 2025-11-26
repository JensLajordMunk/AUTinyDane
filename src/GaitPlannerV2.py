from src.HardwareInterface import HardwareInterface
from src.Kinematics import inverse_kinematics
from src.Rotation import complete_kinematics
from src.StancePlannerV2 import StancePlanner
from src.SwingPlannerV2 import SwingPlanner
from src.InertiaBalancer import InertiaBalancer
from src.FallExit import emergency_stop
import numpy as np
import time


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
        self.state.legpair_start_time = [now, now - 2.0/3.0 * self.config.stancetime]
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
            ratio = min(dt/self.config.swingtime,1.0)
            self.state.stance_yaw_pair[pair_index] = self.config.stancetime * self.state.trot_yaw * (1-ratio)

        else:
            dt = min(dt, self.config.stancetime) # Max time can be stance time
            x, y, z = self.get_stance_trajectory(dt)
            ratio = min(dt/self.config.stancetime,1.0)
            self.state.stance_yaw_pair[pair_index] = self.state.trot_yaw * self.config.stancetime * ratio

        # Clip Yaw limits
        limit = abs(self.state.trot_yaw * self.config.stancetime)
        self.state.stance_yaw_pair[pair_index] = np.clip(self.state.stance_yaw_pair[pair_index], -limit, limit)

        pos_vec = np.array([x, y, z])
        for leg_index in self.config.leg_pairs[pair_index, :]:
            pos_with_offset = pos_vec + np.array([0, self.config.abduction_offsets[leg_index], 0])
            final_pos = complete_kinematics(pos_with_offset, self.state.stance_yaw_pair[pair_index],0, 0, leg_index, self.config)
            angles = inverse_kinematics(final_pos, leg_index, self.config)
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

                if not self.state.leg_pair_in_swing[i]:
                    self.state.stance_yaw_pair[i] = 0

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

        emergency_stop(roll, pitch)

        velocity_offsetx, velocity_offsety = self.balancer.velocity_offset(loop_dt, pitch, roll, gx, gy)

        self.state.velocityX = self.driven_velocityX + velocity_offsetx
        self.state.velocityY = self.driven_velocityY + velocity_offsety

        self.state.velocityX = np.clip(self.state.velocityX, -self.config.max_velocityX, self.config.max_velocityX)
        self.state.velocityY = np.clip(self.state.velocityY, -self.config.max_velocityY, self.config.max_velocityY)

        self.actuate_pair(0, current_time)
        self.actuate_pair(1, current_time)