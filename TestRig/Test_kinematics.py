import numpy as np
from Configuration import RobotConfig, ServoParams
from Kinematics import inverse_kinematics

configuration = RobotConfig()

# leg_index = int(input("Enter leg index: "))
# x, y, z = map(float, input("Enter position (unit [m]): x y z").split())
# target_angles = inverse_kinematics(np.array([x, y, z]), leg_index, configuration)

# Insert your test point here (x forward, y left, z upward)
r_abductor_foot = (0.04, 0.04, -0.08)

# Choose which leg to test (typically 0–3)
leg_index = 0

# Call the inverse kinematics function
thetas = inverse_kinematics(r_abductor_foot, leg_index, configuration)

# Print results in both radians and degrees for easier interpretation
print("Input coordinates (x, y, z):", r_abductor_foot)
print("Leg joint angles (radians):", thetas)
print("Leg joint angles (degrees):", np.degrees(thetas))