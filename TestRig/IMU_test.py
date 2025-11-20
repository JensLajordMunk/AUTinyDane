from HardwareInterface import HardwareInterface
import time

def main():
    hw = HardwareInterface()

    print("Måler vinkler... (Ctrl+C for stop)")

    try:
        while True:
            # Hent de beregnede vinkler
            pitch, roll = hw.get_imu_tilt()

            # Print pænt med 1 decimal
            print(f"Pitch: {pitch:.1f}°  |  Roll: {roll:.1f}°")

            time.sleep(0.2)

    except KeyboardInterrupt:
        print("Stoppet.")

main()
