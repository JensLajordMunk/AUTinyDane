import numpy as np
import time


class HardwareInterface:
    def __init__(self):
        self.last_time = time.time()

    def set_actuator_position(self, joint_angle, leg_index, motor_index):
        self.servo_command(leg_index, motor_index, joint_angle)
