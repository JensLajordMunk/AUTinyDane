import RPi.GPIO as GPIO
import os
import time

# ---------------- Configuration ---------------- #

BUTTON_PIN = 17          # GPIO pin 17 (physical pin 11)
HOLD_TIME = 1.5          # Seconds the button must be held
POLL_INTERVAL = 0.05     # Poll interval for checking button state


# ---------------- Main program ----------------- #

def shutdown_button():
    # Set GPIO mode and activate internal pull-up
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    print("[shutdown_button] Running. Hold button to power off.")

    pressed_since = None

    try:
        while True:
            # LOW = button pressed (connected to GND)
            if GPIO.input(BUTTON_PIN) == GPIO.LOW:
                # First moment of press
                if pressed_since is None:
                    pressed_since = time.time()

                # If held long enough → shutdown
                elif time.time() - pressed_since >= HOLD_TIME:
                    print("[shutdown_button] Button held long enough, shutting down now...")
                    os.system("sudo shutdown -h now")
                    time.sleep(5)  # Prevent repeated triggering if shutdown is delayed

            else:
                # Button released → reset timer
                pressed_since = None

            time.sleep(POLL_INTERVAL)

    except KeyboardInterrupt:
        print("[shutdown_button] Exiting (KeyboardInterrupt).")

    finally:
        # Always clean up GPIO on exit
        GPIO.cleanup()


shutdown_button()