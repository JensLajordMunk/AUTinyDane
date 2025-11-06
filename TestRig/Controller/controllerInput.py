from pyPS4Controller.controller import Controller
from DogController import DogController
from threading import Thread
import time


controller = DogController(
    interface="/dev/input/js0",
    connecting_using_ds4drv=False
)

# controller.listen(timeout=60)

"""
listener_thread = Thread(target=controller.listen, daemon=True)
listener_thread.start()

# Main loop continues normally
for i in range(10):
    print(f"Main loop tick {i}")
    time.sleep(1)

print("Requesting stop...")
controller.stop = True  # ✅ This stops the listen loop cleanly
listener_thread.join()
print("Controller listener exited.")
"""