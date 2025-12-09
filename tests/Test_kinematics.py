import numpy as np
from src.Configuration import RobotConfig
from src.Kinematics import inverse_kinematics
from src.Rotation import orientation_kinematics
configuration = RobotConfig()

def test_kinematics():
    # leg_index = int(input("Enter leg index: "))
    # x, y, z = map(float, input("Enter position (unit [m]): x y z").split())
    # target_angles = inverse_kinematics(np.array([x, y, z]), leg_index, configuration)

    # Insert your test point here (x forward, y left, z upward)
    r_abductor_foot = (-0.0265, -0.04235, -0.144)

    # Choose which leg to test (typically 0–3)
    leg_index = 1

    # Call the inverse kinematics function
    thetas = inverse_kinematics(r_abductor_foot, leg_index, configuration)

    # Print results in both radians and degrees for easier interpretation
    print("Input coordinates (x, y, z):", r_abductor_foot)
    print("Leg joint angles (radians):", thetas)
    print("Leg joint angles (degrees):", np.degrees(thetas))

def test_complete_kinematics():
    configuration = RobotConfig()
    r_abductor_foot = (0, 0.04, -0.1131)
    leg_index = 2
    r = orientation_kinematics(r_abductor_foot,0,0,10,leg_index,configuration)

    print("Output coordinates (x1, y1, z1):", r)

test_kinematics()