from HardwareInterface import HardwareInterface
import time

def main():
    print("IMU test running... Move the robot to see pitch/roll changes.\n")

    while True:
        try:
            pitch, roll = HardwareInterface.get_imu_tilt()
            print(f"Pitch: {pitch:.2f}°   Roll: {roll:.2f}°")
            time.sleep(0.1)
        except KeyboardInterrupt:
            print("\nIMU test stopped.")
            break

if __name__ == "__main__":
    main()
