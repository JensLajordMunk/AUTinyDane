from .DogController import DogController
from threading import Thread


def controller_listen(config):
    # Define controller from DogController
    controller = DogController(
        config, interface="/dev/input/js0", connecting_using_ds4drv=False
    )

    # Start separate thread to listen for controller input
    listener_thread = Thread(target=controller.listen)
    listener_thread.start()

    # Return controller and thread so that the thread can be shut down
    control_panel = {"controller": controller, "thread": listener_thread}
    return control_panel


def controller_stop(control_panel):
    # Stop listening and join thread
    control_panel["controller"].stop = True
    control_panel["thread"].join()

    # Dont know if this is needed?
    control_panel["controller"].stop = False
