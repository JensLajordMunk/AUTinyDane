import numpy as np
import time
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from State import State

# TODO: Enable ease in/out for motion

def test_main():
    start_time = time.time() # TODO: Why is time not working properly? Fix
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State()

    leg_index = int(input("Enter leg index: "))
    x, y, z = map(float, input("Enter position (unit [m]): x y z").split())
    duration = float(input("Enter duration (unit [s]): "))
    update_rate =  int(input("Enter update rate (unit [Hz]): "))

    phases = int(duration*update_rate)
    time_per_phase = 1/update_rate

    initial_angles = state.joint_angles.copy()

    target_angles = inverse_kinematics(np.array([x,y,z]), leg_index, configuration)

    for i in range(phases+1):
        time = time.time()-start_time
        progress = time/duration
        current_angle = initial_angles + (target_angles-initial_angles)*progress
        hardware_interface.set_actuator_position(current_angle, leg_index)
        time.sleep(time_per_phase)

    hardware_interface.set_actuator_position(target_angles, leg_index)
    state.joint_angles=target_angles



