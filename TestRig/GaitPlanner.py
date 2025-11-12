from SwingPlanner import SwingPlanner
from StancePlanner import StancePlanner
from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
import numpy as np
import time

class GaitPlanner():

    def __init__(self, config, state):
        self.config = config
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.stance_planner = StancePlanner(self.state,self.config)
        self.swing_planner = SwingPlanner(self.state, self.config)

    def trot_begin(self):
        velocityX = self.state.velocityX
        velocityY = self.state.velocityY
        self.config.swingtime = self.config.swingtime * 3 #Equal swing and stance time
        self.state.velocityX = 0.5 * velocityX # puts leg at the front touchdown location
        self.state.velocityY = 0.5 * velocityY

        xswing, yswing, zswing = self.swing_planner.discretizer()
        TDX, TDY = self.swing_planner.touchdown_location()
        xswing += TDX
        yswing += TDY
        self.config.swingtime = self.config.swingtime/3 # resets swingtime

        self.state.velocityX = velocityX/6 # puts leg in rightposition 2/3 of the way back from desired touchdown
        self.state.velocityY = velocityY/6
        xstance, ystance, zstance = self.stance_planner.linear_discretizer()
        TDX, TDY = self.swing_planner.touchdown_location()
        xstance -= TDX
        ystance -= TDY

        self.state.velocityX = velocityX #resets velocity
        self.state.velocityY = velocityY  # resets velocity
        initial_phases = int(self.config.frequency * self.config.stancetime)

        self.state.legpair_phases_remaining[0] = initial_phases
        self.state.legpair_phases_remaining[1] = initial_phases

        return xstance, ystance, zstance, xswing, yswing, zswing


    def trot(self, InSwing, pair_index):


        if self.state.firstIt:

            x1, y1, z1 = self.stance_planner.linear_discretizer()
            self.state.legpair_phases_remaining[1] = int(self.config.frequency * self.config.stancetime)

            TDX, TDY = self.swing_planner.touchdown_location()
            self.config.stancetime = self.config.stancetime/3
            x0, y0, z0 = self.stance_planner.linear_discretizer()
            self.state.legpair_phases_remaining[0] = int(self.config.frequency * self.config.stancetime)
            x0 -= 2*TDX/3
            y0 -= 2*TDY/3

            self.config.stancetime = self.config.stancetime*3
            self.state.firstIt = False
            return x0, y0, z0, x1, y1, z1

        else:
            if not self.state.leg_pair_in_swing[pair_index]:
                xswing, yswing, zswing = self.swing_planner.discretizer()
                self.state.legpair_phases_remaining[pair_index] = int(len(xswing))
                self.state.leg_pair_in_swing[pair_index] = True
                return xswing, yswing, zswing

            else:
                xstance, ystance, zstance = self.stance_planner.linear_discretizer()
                self.state.legpair_phases_remaining[pair_index] = int(len(xstance))
                self.state.leg_pair_in_swing[pair_index] = False
                return xstance, ystance, zstance



    def trot_end(self):

        initial_angles = state.joint_angles.copy() * np.pi / 180

        target_angles = np.array[np.transpose(inverse_kinematics(np.array([0, 0, -self.config.body_height]), 0, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.body_height]), 1, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.body_height]), 2, self.config)),
                                 np.transpose(inverse_kinematics(np.array([0, 0, -self.config.body_height]), 3, self.config))]

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







