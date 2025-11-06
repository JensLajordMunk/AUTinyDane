from pyPS4Controller.controller import Controller
from DogController import DogController


controller = DogController(
    interface="/dev/input/js0",
    connecting_using_ds4drv=False
)

controller.listen(timeout=60)
