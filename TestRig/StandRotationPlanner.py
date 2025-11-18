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

    def run_rotation(self):
        self.state.stand_yaw =self.command.stand_yaw
        self.state.stand_roll =self.command.stand_roll
        self.state.stand_pitch =self.command.stand_pitch

        for leg_index in range(4):
            pos = complete_kinematics([0, self.config.abduction_offsets[leg_index], -self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):                
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)


