import numpy as np
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig, ServoParams
from Kinematics import inverse_kinematics


def servos_to_neutral():
    hardware_interface = HardwareInterface()
    for leg_index in range(4):
        for motor_index in range(3):
            hardware_interface.set_actuator_position0(0, leg_index, motor_index)

def legs_to_neutral():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    x = 0
    z = - configuration.body_height
    for leg_index in range(4):
        y = configuration.abduction_offsets[leg_index]
        target_angles = inverse_kinematics(np.array([x,y,z]), leg_index, configuration)
        for motor_index in range(3):
            hardware_interface.set_actuator_position(target_angles[motor_index], leg_index,motor_index)

servos_to_neutral()