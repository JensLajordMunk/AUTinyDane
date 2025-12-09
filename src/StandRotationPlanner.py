from src.HardwareInterface import HardwareInterface
from src.Kinematics import inverse_kinematics
from src.Rotation import orientation_kinematics


class StandRotationPlanner:

    def __init__(self, config, state, command):
        self.config = config
        self.command = command
        self.state = state
        self.hardware_interface = HardwareInterface()

    def run_rotation(self):
        self.state.stand_yaw = self.command.stand_yaw

        #Without PD controller:
        #self.state.stand_roll = self.command.stand_roll
        #self.state.stand_pitch = self.command.stand_pitch

        # PD controller:
        desired_roll = self.command.stand_roll
        desired_pitch = self.command.stand_pitch

        roll_imu, pitch_imu, gx, gy = self.hardware_interface.get_imu_tilt()

        self.state.stand_pitch += desired_pitch - pitch_imu
        self.state.stand_roll += desired_roll - roll_imu

        print(self.state.stand_pitch, self.state.stand_roll)

        for leg_index in range(4):
            pos = orientation_kinematics([-self.state.stand_x, self.state.stand_y + self.config.abduction_offsets[leg_index], - self.state.stand_z - self.config.body_height], self.state.stand_yaw, self.state.stand_pitch, self.state.stand_roll, leg_index, self.config)
            current_angles_rad = inverse_kinematics(pos,leg_index,self.config)
            for motor_index in range(3):                
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)