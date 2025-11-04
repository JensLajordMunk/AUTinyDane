from Configuration import RobotConfig
from SwingPlanner import SwingPlanner
from StancePlanner import StancePlanner
from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
import numpy as np
import time

class GaitPlanner():

    def __init__(self,config):
        self.config = config
        self.hardware_interface = HardwareInterface()
        self.stance_planner = StancePlanner(self.config)
        self.swing_planner = SwingPlanner(self.config)

    def trot_begin(self):
        velocity = self.config.velocity
        self.config.swingtime = self.config.swingtime * 3 #Equal swing and stance time
        self.config.velocity = 0.5*velocity # puts leg at the front touchdown location

        xswing,zswing = self.swing_planner.discretizer()
        xswing += self.swing_planner.touchdown_location()
        self.config.swingtime = self.config.swingtime/3 # resets swingtime

        self.config.velocity = velocity/6 # puts leg in rightposition 2/3 of the way back from desired touchdown
        xstance,zstance = self.stance_planner.linear_discretizer()
        xstance -= self.swing_planner.touchdown_location()

        self.config.velocity = velocity #resets velocity
        initial_phases = int(self.config.frequency * self.config.stancetime)

        self.config.legpair_phases_remaining[0] = initial_phases
        self.config.legpair_phases_remaining[1] = initial_phases

        return xstance, zstance, xswing, zswing


    def trot(self, InSwing, pair_index):

        if self.config.firstIt:

            x1, z1 = self.stance_planner.linear_discretizer()
            self.config.legpair_phases_remaining[0] = int(self.config.frequency * self.config.swingtime)

            td = self.swing_planner.touchdown_location()
            self.config.velocity = self.config.velocity
            self.config.stancetime = self.config.stancetime/3
            x0, z0 = self.stance_planner.linear_discretizer()
            self.config.legpair_phases_remaining[1] = int(self.config.frequency * self.config.stancetime)
            x0 -=  2*td/3

            self.config.stancetime = self.config.stancetime*3
            self.config.firstIt = False
            return x0, z0, x1, z1

        else:
            if not self.config.leg_pair_in_swing[pair_index]:
                xswing, zswing = self.swing_planner.discretizer()
                self.config.legpair_phases_remaining[pair_index] = int(self.config.frequency * self.config.swingtime)
                self.config.leg_pair_in_swing[pair_index] = True
                return xswing, zswing

            else:
                xstance, zstance = self.stance_planner.linear_discretizer()
                self.config.legpair_phases_remaining[pair_index] = int(self.config.frequency * self.config.stancetime)
                self.config.leg_pair_in_swing[pair_index] = False
                return xstance, zstance



    def trot_end(self):

        initial_angles = state.joint_angles.copy() * np.pi / 180

        target_angles = np.array[np.transpose(inverse_kinematics(np.array([0, 0, -self.config.Z_zero]), 0, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.Z_zero]), 1, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.Z_zero]), 2, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.Z_zero]), 3, self.config))]

        start_time = time.time()
        loop_time = 0.0
        duration = 0.75

        for i in range(phases + 1):
            loop_time = time.time() - (start_time + loop_time)

            elapsed_time = time.time() - start_time
            progress = np.clip(elapsed_time / duration, 0, 1)
            smoothed_progress = 6 * progress ** 5 - 15 * progress ** 4 + 10 * progress ** 3
            current_angles_rad = initial_angles + (target_angles - initial_angles) * smoothed_progress

            for leg_index in range(4):
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index,leg_index], leg_index, motor_index)

            time.sleep(time_per_phase-loop_time)







