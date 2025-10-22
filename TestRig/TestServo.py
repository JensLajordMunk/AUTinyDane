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
from Configuration import RobotConfig, ServoParams
from ServoCalibration import ServoCalibration
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

def test_leg_servo():
    hardware_interface = HardwareInterface()

    leg_index = int(input("Enter leg index: "))
    motor_index = int(input("Enter motor index: "))
    desired_angle_degrees = int(input("Enter desired angle in degrees: "))

    desired_angle = desired_angle_degrees / 180.0 * np.pi

    hardware_interface.set_actuator_position(desired_angle, leg_index, motor_index)

def test_leg_movement():
    hardware_interface = HardwareInterface()
    configuration = RobotConfig()

    leg_index = int(input("Enter leg index: "))
    x, y, z = map(float, input("Enter position (unit [m]): x y z").split())

    target_angles = inverse_kinematics(np.array([x,y,z]), leg_index, configuration)

    for motor_index in range(3):
        hardware_interface.set_actuator_position(target_angles[motor_index], leg_index,motor_index)


def test_leg_params():
    hardware_interface = HardwareInterface()
    calibration = ServoCalibration()
    config = ServoParams()

    leg_index = int(input("Which leg index?"))
    motor_index = int(input("Which motor index?"))

    while True:

        min_angle = input(
            "Search for the lower bound angle, insert new lower angle until amp increses. If lower angle found type 'y': ")
        if min_angle == "y":
            break
        min_angle = float(min_angle) / 180.0 * np.pi
        hardware_interface.set_actuator_position(min_angle, leg_index, motor_index)
        min_angle_prev = float(min_angle)

    while True:

        max_angle = input(
            "Search for the upper bound angle, insert new upper angle until amp increses. If upper angle found type 'y': ")
        if max_angle == "y":
            break
        max_angle = float(max_angle) / 180.0 * np.pi
        hardware_interface.set_actuator_position(max_angle, leg_index, motor_index)
        max_angle_prev = float(max_angle)

    micros_per_rad = calibration.MICROS_PER_RAD[motor_index,leg_index]
    new_neutral_PWM = config.neutral_position_pwm[motor_index,leg_index] + ((max_angle_prev + min_angle_prev) * micros_per_rad / 2)
    new_micros_per_rad = (config.neutral_position_pwm[motor_index,leg_index] + max_angle_prev * micros_per_rad - new_neutral_PWM) / 90
    print(f"The new neutral PWM is: {new_neutral_PWM}")
    print(f"The new micros per rad is: {new_micros_per_rad}")
test_leg_movement()