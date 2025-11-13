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

    def run_rotation(self):
        if abs(self.state.yaw_stand - self.command.yaw_stand) > 0.0001:

            yaw_change = self.command.yaw_stand - self.state.yaw_stand

            max_step = 0.01 / self.config.frequency

            # Limit the velocity change per iteration
            if yaw_change > max_step:
                scale = max_step / yaw_change
                self.state.yaw_stand = self.state.yaw_stand + yaw_change * scale
            else:
                self.state.yaw_stand= self.command.yaw_stand


        if abs(self.state.pitch_stand - self.command.pitch_stand) > 0.0001:

            yaw_change = self.command.pitch_stand - self.state.pitch_stand

            max_step = 0.01 / self.config.frequency

            # Limit the velocity change per iteration
            if yaw_change > max_step:
                scale = max_step / yaw_change
                self.state.yaw_stand = self.state.yaw_stand + yaw_change * scale
            else:
                self.state.yaw_stand= self.command.yaw_stand


        if abs(self.state.yaw_stand - self.command.yaw_stand) > 0.0001:

            yaw_change = self.command.yaw_stand - self.state.yaw_stand

            max_step = 0.01 / self.config.frequency

            # Limit the velocity change per iteration
            if yaw_change > max_step:
                scale = max_step / yaw_change
                self.state.yaw_stand = self.state.yaw_stand + yaw_change * scale
            else:
                self.state.yaw_stand= self.command.yaw_stand
