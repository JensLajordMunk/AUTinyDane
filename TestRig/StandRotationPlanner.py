from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
from Command import Command
import numpy as np
import time

class StandRotaionPlanner:

    def __init__(self, config, state):
        self.config = config
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.command = Command(self.config)

    def changeCheck(self,command,state):
        if abs(state - command) > 0.0001:

            change = command - state

            max_step = 0.01 / self.config.frequency

            # Limit the velocity change per iteration
            if change > max_step:
                scale = max_step / change
                state = state + change * scale
            else:
                state = command

        return state

    def run_rotation(self):
        self.state.yaw_stand = self.changeCheck(self.command.yaw_stand, self.state.yaw_stand)
        self.state.pitch_stand = self.changeCheck(self.command.pitch_stand, self.state.pitch_stand)
        self.state.roll_stand = self.changeCheck(self.command.roll_stand, self.state.roll_stand)

