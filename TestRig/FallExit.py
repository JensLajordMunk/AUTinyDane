def emergency_stop(roll, pitch):
    if abs(roll) > 45.0 or abs(pitch) > 45.0:
        input("Fall detected. If the robot is standing, check calibration. Press ENTER to proceed")
    else:
        pass