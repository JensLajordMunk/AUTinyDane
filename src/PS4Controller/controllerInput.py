from .DogController import DogController
from threading import Thread


def controller_listen(command_config, debug=False):
    # Define controller from DogController
    controller = DogController(
        command_config, interface="/dev/input/js0", connecting_using_ds4drv=False
    )

    # Start separate thread to listen for controller input
    listener_thread = Thread(target=controller.listen)
    listener_thread.start()

    # Return controller and thread so that the thread can be shut down
    control_panel = {"controller": controller, "thread": listener_thread}

    # Tell command that controller is listening
    command_config.controller_connected = True

    if debug:
        print("Controller thread started - Now listening for input.")

    return control_panel


def controller_stop(command_config, control_panel, debug=False):
    # Stop listening and join thread
    control_panel["controller"].stop = True
    control_panel["thread"].join()

    # Dont know if this is needed?
    control_panel["controller"].stop = False

    # Tell command that controller is not listening anymore
    command_config.controller_connected = False

    # Print to console (Optional)
    if debug:
        print("Controller has been stopped!")
