from pyPS4Controller.controller import Controller
from threading import Lock


class DogController(Controller):
    # Add dead zone to limit the noise of controller
    DEAD_ZONE = 5000

    # Indexing for axis
    HORIZONTAL = 0
    VERTICAL = 1

    # Maximum and Minimum max positions of the controller joystick
    MAX_POS = 2**15 - 1

    def __init__(self, command_config, **kwargs):
        super().__init__(**kwargs)
        self.command_config = command_config

    def on_x_press(self):
        self.command_config.mode = Modes.TROT

    def on_circle_press(self):
        self.command_config.mode = "ROTATE"

    def on_square_press(self):
        self.command_config.mode = "TRANSLATE"

    def on_L3_up(self, value):
        self.modify_coordinates(self.command_config.L3, value, self.VERTICAL)

    def on_L3_down(self, value):
        self.modify_coordinates(self.command_config.L3, value, self.VERTICAL)

    def on_L3_left(self, value):
        self.modify_coordinates(self.command_config.L3, value, self.HORIZONTAL)

    def on_L3_right(self, value):
        self.modify_coordinates(self.command_config.L3, value, self.HORIZONTAL)

    # Reduce noise from console
    def on_L3_x_at_rest(self):
        pass

    # Reduce noise from console
    def on_L3_y_at_rest(self):
        pass

    def on_R3_up(self, value):
        self.modify_coordinates(self.command_config.R3, value, self.VERTICAL)

    def on_R3_down(self, value):
        self.modify_coordinates(self.command_config.R3, value, self.VERTICAL)

    def on_R3_left(self, value):
        self.modify_coordinates(self.command_config.R3, value, self.HORIZONTAL)

    def on_R3_right(self, value):
        self.modify_coordinates(self.command_config.R3, value, self.HORIZONTAL)

    # Reduce noise from console
    def on_R3_x_at_rest(self):
        pass

    # Reduce noise from console
    def on_R3_y_at_rest(self):
        pass

    def modify_coordinates(self, coordinates, value, direction):
        # Check if value is outside of dead zone
        if abs(value) > self.DEAD_ZONE:
            # Map controller input depending on direction
            if direction == self.HORIZONTAL:
                coordinates[direction] = self.map_coordinates_y(value)
            elif direction == self.VERTICAL:
                coordinates[direction] = self.map_coordinates_y(value)
            # print(f"L3: {self.config.L3}\n R3: {self.config.R3}\n")
        else:
            # If not outside dead zone the direction should be 0
            coordinates[direction] = 0


        return

    # Map from -1 to 1 for sideways speed
    def map_coordinates_y(self, value):
        mapped_value = value / self.MAX_POS
        # Primarily for testing // EDGE CASE when velocity is 0
        if abs(mapped_value) < 0.1:
            mapped_value = 0.1
        return mapped_value
"""
    # Return value 1 or -1 for forward and backwards
    @staticmethod
    def map_coordinates_x(value):
        if value > 0:
            return -1
        if value < 0:
            return 1
        else:
            return 0
"""