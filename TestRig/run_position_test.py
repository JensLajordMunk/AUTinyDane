import numpy as np
import time
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from State import State


def test_main():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State()

    while True:
        leg_index = 1 #int(input("UPDATE NEUTRAL ANGLES BEFORE. Enter leg index: "))
        x, y, z = map(float, input("Enter position (unit [m]): x y z").split())
        duration = float(input("Enter duration (unit [s]): "))
        update_rate =  int(input("Enter update rate (unit [Hz]): "))

        phases = int(duration*update_rate)
        time_per_phase = 1/update_rate

        initial_angles = state.joint_angles.copy() *np.pi/180
        for motor_index in range(3):
            hardware_interface.set_actuator_position(initial_angles[motor_index,leg_index], leg_index,motor_index)

        target_angles = inverse_kinematics(np.array([x,y,z]), leg_index, configuration)

        start_time = time.time()
        for i in range(phases+1):
            elapsed_time = time.time()-start_time
            progress = np.clip(elapsed_time/duration,0,1)
            smoothed_progress = 6*progress**5-15*progress**4+10*progress**3
            current_angles_rad = initial_angles[:,leg_index] + (target_angles-initial_angles[:,leg_index])*smoothed_progress
            for motor_index in range(3):
                hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index,motor_index)
            time.sleep(time_per_phase)
        
        state.joint_angles[:,leg_index]=target_angles*180/np.pi
    

def test_main_2():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State()

    leg_index_1 = int(input("UPDATE NEUTRAL ANGLES BEFORE. Enter first leg index: "))
    x1, y1, z1 = map(float, input("Enter position (unit [m]): x y z").split())
    leg_index_2 = int(input("Enter second leg index: "))
    x2, y2, z2 = map(float, input("Enter position (unit [m]): x y z").split())
    duration = float(input("Enter duration (unit [s]): "))
    update_rate =  int(input("Enter update rate (unit [Hz]): "))

    phases = int(duration*update_rate)
    time_per_phase = 1/update_rate

    initial_angles = state.joint_angles.copy() *np.pi/180

    target_angles_1 = inverse_kinematics(np.array([x1,y1,z1]), leg_index_1, configuration)
    target_angles_2 = inverse_kinematics(np.array([x2,y2,z2]), leg_index_2, configuration)

    start_time = time.time()
    for i in range(phases+1):
        elapsed_time = time.time()-start_time
        progress = np.clip(elapsed_time/duration,0,1)
        smoothed_progress = 6*progress**5-15*progress**4+10*progress**3
        current_angles_rad_1 = initial_angles[:,leg_index_1] + (target_angles_1-initial_angles[:,leg_index_1])*smoothed_progress
        current_angles_rad_2 = initial_angles[:,leg_index_2] + (target_angles_2-initial_angles[:,leg_index_2])*smoothed_progress
        for motor_index in range(3):
            hardware_interface.set_actuator_position(current_angles_rad_1[motor_index], leg_index_1,motor_index)
            hardware_interface.set_actuator_position(current_angles_rad_2[motor_index], leg_index_2,motor_index)
        time.sleep(time_per_phase)

    state.joint_angles[:,leg_index_1]=target_angles_1
    state.joint_angles[:,leg_index_2]=target_angles_2
    for motor_index in range(3):
            hardware_interface.set_actuator_position(initial_angles[motor_index,leg_index_1], leg_index_1,motor_index)
            hardware_interface.set_actuator_position(initial_angles[motor_index,leg_index_2], leg_index_2,motor_index)

def test_main_4():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State(configuration)


    x1, y1, z1 = map(float, input("UPDATE NEUTRAL ANGLES BEFORE. Enter position for left legs(unit [m]): x y z").split())
    x2, y2, z2 = map(float, input("Enter position for right legs (unit [m]): x y z").split())
    duration = float(input("Enter duration (unit [s]): "))
    update_rate =  int(input("Enter update rate (unit [Hz]): "))

    phases = int(duration*update_rate)
    time_per_phase = 1/update_rate

    initial_angles = state.joint_angles.copy() *np.pi/180

    target_angles_1 = inverse_kinematics(np.array([x1,y1,z1]), 1, configuration)
    target_angles_2 = inverse_kinematics(np.array([x2,y2,z2]), 2, configuration)

    start_time = time.time()
    for i in range(phases+1):
        elapsed_time = time.time()-start_time
        progress = np.clip(elapsed_time/duration,0,1)
        smoothed_progress = 6*progress**5-15*progress**4+10*progress**3
        current_angles_rad_1 = initial_angles[:,1] + (target_angles_1-initial_angles[:,1])*smoothed_progress
        current_angles_rad_2 = initial_angles[:,2] + (target_angles_2-initial_angles[:,2])*smoothed_progress
        for motor_index in range(3):
            hardware_interface.set_actuator_position(current_angles_rad_1[motor_index], 1,motor_index)
            hardware_interface.set_actuator_position(current_angles_rad_2[motor_index], 2,motor_index)
            hardware_interface.set_actuator_position(current_angles_rad_1[motor_index], 3,motor_index)
            hardware_interface.set_actuator_position(current_angles_rad_2[motor_index], 4,motor_index)
        time.sleep(time_per_phase)


    for motor_index in range(3):
            hardware_interface.set_actuator_position(initial_angles[motor_index,1], 1,motor_index)
            hardware_interface.set_actuator_position(initial_angles[motor_index,2], 2,motor_index)
            hardware_interface.set_actuator_position(initial_angles[motor_index,3], 3,motor_index)
            hardware_interface.set_actuator_position(initial_angles[motor_index,4], 4,motor_index)
    #Smoother step: https://en.wikipedia.org/wiki/Smoothstep

def complete_kinematics_test():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State(configuration)

    x, y, z = map(float,
                     input("Enter body center movement: x y z").split())
    yaw, pitch, roll = map(float,
                     input("Enter body rotation in degrees: yaw pitch roll").split())
    duration = float(input("Enter duration (unit [s]): "))
    update_rate = int(input("Enter update rate (unit [Hz]): "))

    phases = int(duration * update_rate)
    time_per_phase = 1 / update_rate

    # Positions:
    P = np.zeros((3, 4))
    P[0,:] = x
    P[1, :] = y + configuration.abduction_offsets
    P[2,:] = z - configuration.body_height

    # Servo angles:
    initial_angles = state.joint_angles.copy() * np.pi / 180
    thetas = np.zeros((3, 4))

    for i in range(4):
        P[:,i] = complete_kinematics(P[:,i], yaw, pitch, roll, i, configuration)
        thetas[:,i] = inverse_kinematics(P[:,i], i, configuration)
    print(P)
    thetas_deg = thetas*180/np.pi
    print(thetas_deg)
    input("Press Enter to continue...")

    start_time = time.time()
    for i in range(phases + 1):
        target_time = start_time + i * time_per_phase
        elapsed_time = time.time() - start_time
        progress = np.clip(elapsed_time / duration, 0, 1)
        smoothed_progress = 6 * progress ** 5 - 15 * progress ** 4 + 10 * progress ** 3
        current_angles_rad = initial_angles + (thetas - initial_angles) * smoothed_progress

        for motor_index in range(3):
            for leg_index in range(4):
                hardware_interface.set_actuator_position(current_angles_rad[motor_index,leg_index], leg_index, motor_index)
        sleep_time = target_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    start_time = time.time()
    for i in range(phases + 1):
        target_time = start_time + i * time_per_phase
        elapsed_time = time.time() - start_time
        progress = np.clip(elapsed_time / duration, 0, 1)
        smoothed_progress = 6 * progress ** 5 - 15 * progress ** 4 + 10 * progress ** 3
        current_angles_rad = thetas + (initial_angles - thetas) * smoothed_progress

        for motor_index in range(3):
            for leg_index in range(4):
                hardware_interface.set_actuator_position(current_angles_rad[motor_index,leg_index], leg_index, motor_index)

        sleep_time = target_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

complete_kinematics_test()