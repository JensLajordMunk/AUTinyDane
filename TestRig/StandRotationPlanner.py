from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from Command import Command
import numpy as np

class StandRotaionPlanner:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.command = Command(self.config)

    def smoother(self, command, state):
        max_step = max(2,0.01 / self.config.frequency)
        array = np.arange(0, 1, max_step)
        smoothed_array = 6 * array ** 5 - 15 * array ** 4 + 10 * array ** 3
        smoothed_transition = command + (state - command) * smoothed_array
        return smoothed_transition

    def run_rotation(self):
        if self.command.yaw_array == [] and abs(self.command.stand_yaw - self.state.stand_yaw):
            self.command.yaw_array = self.smoother(self.command.stand_yaw, self.state.stand_yaw)

        if self.command.pitch_array == [] and abs(self.command.stand_pitch - self.state.stand_pitch):
            self.command.pitch_array = self.smoother(self.command.stand_pitch, self.state.stand_pitch)

        if self.command.roll_array == [] and abs(self.command.stand_roll - self.state.stand_roll):
            self.command.roll_array = self.smoother(self.command.stand_roll, self.state.stand_roll)

        if self.command.yaw_array != []:
            self.state.stand_yaw = self.command.yaw_array[0]
            self.command.yaw_array.pop(0)

        if self.command.roll_array != []:
            self.state.stand_roll = self.command.roll_array[0]
            self.command.roll_array.pop(0)

        if self.command.pitch_array != []:
            self.state.stand_pitch = self.command.pitch_array[0]
            self.command.pitch_array.pop(0)

        for motor_index in range(3):
            for leg_index in range(4):
                pos = complete_kinematics([0, self.config.abduction_offsets[leg_index], -self.config.body_height], self.state.stand_yaw, self.state.pitch_yaw, self.state.roll_yaw, leg_index, self.config)
                current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index,leg_index], leg_index, motor_index)


