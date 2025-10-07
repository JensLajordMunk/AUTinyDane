# Install on Raspberry Pi:
# sudo apt-get update
#sudo apt-get install -y python3-pip
# Low-level: pip3 install --upgrade adafruit-circuitpython-pca9685 adafruit-circuitpython-motor adafruit-blinka
# High-level: pip3 install adafruit-circuitpython-servokit

# High-level:
from adafruit_servokit import ServoKit
import time
import numpy as np
from HardwareInterface import HardwareInterface
from Configuration import RobotConfig
from Kinematics import inverse_kinematics
from State import State


kit = ServoKit(channels=16)

def test_servo_basic():
    """Move servo on channel 0 through a few positions"""
    kit.servo[0].angle = 90
    time.sleep(1)
    kit.servo[0].angle = 0
    time.sleep(1)
    kit.servo[0].angle = 180
    time.sleep(1)

def test_servo_sweep():
    """Sweep servo from 0 to 180 and back"""
    for angle in range(0, 181, 10):
        kit.servo[0].angle = angle
        time.sleep(0.05)
    for angle in range(180, -1, -10):
        kit.servo[0].angle = angle
        time.sleep(0.05)

#if __name__ == "__main__":
    # Run only ONE test by calling the function
    #test_servo_basic()


def test_leg_servo
    hardware_interface = HardwareInterface()
    state = State()

    leg_index = int(input("Enter leg index: "))
    motor_index = int(input("Enter motor index: "))
    desired_angle_degrees = int(input("Enter desired angle in degrees: "))

    desired_angle = desired_angle_degrees / 180.0 * np.pi

    hardware_interface.set_actuator_position(desired_angle, leg_index, motor_index)
    state.joint_angles[motor_index,leg_index] = desired_angle

def test_leg_movement
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()
    state = State()

    leg_index = int(input("Enter leg index: "))
    x, y, z = map(float, input("Enter position (unit [m]): x y z").split())

    target_angles = inverse_kinematics(np.array([x,y,z]), leg_index, configuration)

    for motor_index in range(3):
        hardware_interface.set_actuator_position(target_angles[motor_index], leg_index,motor_index)

    state.joint_angles[:,leg_index]=target_angles
