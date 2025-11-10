from .DogController import DogController
from threading import Thread
import time

# Define controller from DogController
controller = DogController(interface="/dev/input/js0", connecting_using_ds4drv=False)


def controller_listen(config, listen_time=0.03):
    # Start separate thread to listen for controller input
    listener_thread = Thread(target=controller.listen, daemon=True)
    listener_thread.start()

    # Listen for control input until time is out
    time.sleep(listen_time)

    # Stop listening and join thread
    controller.stop = True
    listener_thread.join()

    # Reset to listen later
    controller.stop = False
    return [controller.L3_coordinates, controller.R3_coordinates]


if __name__ == "__main__":
    test = controller_listen(listen_time=2)
