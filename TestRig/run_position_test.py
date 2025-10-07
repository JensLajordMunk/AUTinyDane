import numpy as np
import time
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from State import State


def test_main():
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

    start_time = time.time()
    for i in range(phases+1):
        elapsed_time = time.time()-start_time
        progress = np.clip(elapsed_time/duration,0,1)
        smoothed_progress = 6*progress**5-15*progress**4+10*progress**3
        current_angles = initial_angles + (target_angles-initial_angles)*smoothed_progress
        for motor_index in range(3):
            hardware_interface.set_actuator_position(current_angles[motor_index], leg_index,motor_index)
            time.sleep(time_per_phase)

    hardware_interface.set_actuator_position(target_angles, leg_index)
    state.joint_angles[:,leg_index]=target_angles

    #Smoother step: https://en.wikipedia.org/wiki/Smoothstep



