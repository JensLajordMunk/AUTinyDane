from pyPS4Controller.controller import Controller


class MyController(Controller):
    DEADZONE = 5000  # adjust between 1000–10000 as needed

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
        print("Hello world")

    def on_x_release(self):
        print("Goodbye world")

    def on_L3_up(self, value):
        if abs(value) > self.DEADZONE:
            print(f"L3 up: {value}")

    def on_L3_down(self, value):
        if abs(value) > self.DEADZONE:
            print(f"L3 down: {value}")

    def on_L3_left(self, value):
        if abs(value) > self.DEADZONE:
            print(f"L3 left: {value}")

    def on_L3_right(self, value):
        if abs(value) > self.DEADZONE:
            print(f"L3 right: {value}")

    def on_L3_x_at_rest(self):
        # optional: handle when joystick returns to rest
        pass

    def on_L3_y_at_rest(self):
        pass


controller = MyController(
    interface="/dev/input/js0",
    connecting_using_ds4drv=False,
    debug=False
)

controller.listen(timeout=60)
