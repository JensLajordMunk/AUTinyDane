from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from Command import Command
import numpy as np

class StandTranslationPlanner:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()

    def run_translation(self):
        self.state.stand_x =self.command.stand_x
        self.state.stand_y =self.command.stand_y
        self.state.stand_z =self.command.stand_z

        desired_roll = 0
        desired_pitch = 0
        roll_imu, pitch_imu, gx, gy = self.hardware_interface.get_imu_tilt()

        self.state.stand_pitch += self.config.k_p_stand_pitch*(desired_pitch - pitch_imu) - self.config.k_d_stand_pitch*gy
        self.state.stand_roll += self.config.k_p_stand_roll*(desired_roll - roll_imu) - self.config.k_d_stand_roll*gx

        print(self.state.stand_pitch, self.state.stand_roll)

        for leg_index in range(4):
            pos = complete_kinematics([-self.state.stand_x, self.state.stand_y + self.config.abduction_offsets[leg_index], - self.state.stand_z - self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
