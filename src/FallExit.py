def emergency_stop(roll, pitch):
    if abs(roll) > 80.0 or abs(pitch) > 80.0:
        input("Fall detected. If the robot is standing, check calibration. Press ENTER to proceed")
    else:
        pass