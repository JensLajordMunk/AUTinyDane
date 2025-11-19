from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from StancePlannerV2 import StancePlanner
from SwingPlannerV2 import SwingPlanner
import numpy as np
import time


class GaitPlanner:
    def __init__(self, config, state, command):
        self.config = config
        self.state = state
        self.command = command
        self.hardware_interface = HardwareInterface()
        self.stance_planner = StancePlanner(self.state, self.config)
        self.swing_planner = SwingPlanner(self.state, self.config)
        self.state.legpair_start_time = [time.time(), time.time()]
        self.last_loop_time = time.time()

    def get_swing_trajectory(self, dt):
        method = self.config.swing_method.upper()

        if method == "BEZIER":
            return self.swing_planner.bezier_discretizer(dt)
        elif method == "TRIANGULAR":
            return self.swing_planner.triangular_discretizer(dt)
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
            ratio = dt/self.config.swingtime
            self.state.stance_yaw_pair[pair_index] -= self.state.trot_yaw * ratio

        else:
            dt = min(dt, self.config.stancetime) # Max time can be stance time
            x, y, z = self.stance_planner.linear_discretizer(dt)
            ratio = dt/self.config.stancetime
            self.state.stance_yaw_pair[pair_index] += self.state.trot_yaw * ratio

        # Clip Yaw limits
        limit = abs(self.state.trot_yaw)
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
                dt = current_time - self.state.legpair_start_time[i]

                if self.state.leg_pair_in_swing[i]:
                    self.get_swing_trajectory(dt)

                else:
                    self.stance_planner.linear_discretizer(dt)
                    self.state.stance_yaw_pair[i] = 0


        loop_dt = current_time - self.last_loop_time
        self.last_loop_time = current_time

        diff_x = self.command.velocityX - self.state.velocityX
        diff_y = self.command.velocityY - self.state.velocityY
        magnitude = np.sqrt(diff_x ** 2 + diff_y ** 2)

        max_step = self.config.max_acceleration * loop_dt

        if magnitude > max_step:
            scale = max_step / magnitude
            self.state.velocityX += diff_x * scale
            self.state.velocityY += diff_y * scale
        else:
            self.state.velocityX = self.command.velocityX
            self.state.velocityY = self.command.velocityY

        diff_yaw = self.command.trot_yaw - self.state.trot_yaw
        max_yaw_step = self.config.max_yaw_acceleration * loop_dt

        if abs(diff_yaw) > max_yaw_step:
            self.state.trot_yaw += np.sign(diff_yaw) * max_yaw_step
        else:
            self.state.trot_yaw = self.command.trot_yaw

        self.actuate_pair(0, current_time)
        self.actuate_pair(1, current_time)