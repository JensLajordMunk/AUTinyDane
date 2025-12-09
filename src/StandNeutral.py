from src.HardwareInterface import HardwareInterface
from src.Kinematics import inverse_kinematics
from src.Rotation import orientation_kinematics

class StandNeutral:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()

    def run_neutral(self):
        self.state.stand_x = 0
        self.state.stand_y = 0
        self.state.stand_z = 0

        self.state.stand_yaw = 0
        self.state.stand_pitch = 0
        self.state.stand_roll = 0

        for leg_index in range(4):
            pos = orientation_kinematics([self.state.stand_x, self.state.stand_y + self.config.abduction_offsets[leg_index], - self.state.stand_z - self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)