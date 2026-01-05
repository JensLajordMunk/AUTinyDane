from src.HardwareInterface import HardwareInterface
from src.Kinematics import inverse_kinematics
from src.Rotation import orientation_kinematics
import numpy as np


class StandRotationPlanner:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.max_degree_change = 0.35
        self.driven_roll = 0.0
        self.driven_pitch = 0.0
        self.driven_yaw = 0.0

    def run_rotation(self):

        diff_yaw = self.command.stand_yaw - self.state.stand_yaw

        if abs(diff_yaw) > self.max_degree_change:
            self.state.stand_yaw += np.sign(diff_yaw) * self.max_degree_change
        else:
            self.state.stand_yaw = self.command.stand_yaw

        diff_roll = self.command.stand_roll - self.state.stand_roll

        if abs(diff_roll) > self.max_degree_change:
            self.state.stand_roll += np.sign(diff_roll) * self.max_degree_change
        else:
            self.state.stand_roll = self.command.stand_roll

        diff_pitch = self.command.stand_pitch - self.state.stand_pitch

        if abs(diff_pitch) > self.max_degree_change:
            self.state.stand_pitch += np.sign(diff_pitch) * self.max_degree_change
        else:
            self.state.stand_pitch = self.command.stand_pitch

        for leg_index in range(4):
            pos = orientation_kinematics([-self.state.stand_x, self.state.stand_y + self.config.abduction_offsets[leg_index], - self.state.stand_z - self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):                
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
