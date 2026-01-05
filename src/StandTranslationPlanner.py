from src.HardwareInterface import HardwareInterface
from src.Kinematics import inverse_kinematics
from src.Rotation import orientation_kinematics
import numpy as np

class StandTranslationPlanner:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.max_translation_change = 0.001

    def run_translation(self):

        diff_x = self.command.stand_x - self.state.stand_x

        if abs(diff_x) > self.max_translation_change:
            self.state.stand_x += np.sign(diff_x) * self.max_translation_change
        else:
            self.state.stand_x = self.command.stand_x


        diff_y = self.command.stand_y - self.state.stand_y

        if abs(diff_y) > self.max_translation_change:
            self.state.stand_y += np.sign(diff_y) * self.max_translation_change
        else:
            self.state.stand_y = self.command.stand_y


        diff_z = self.command.stand_z - self.state.stand_z

        if abs(diff_z) > self.max_translation_change:
            self.state.stand_z += np.sign(diff_z) * self.max_translation_change
        else:
            self.state.stand_z = self.command.stand_z


        for leg_index in range(4):
            pos = orientation_kinematics([-self.state.stand_x, self.state.stand_y + self.config.abduction_offsets[leg_index], - self.state.stand_z - self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
