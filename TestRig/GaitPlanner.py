from SwingPlanner import SwingPlanner
from StancePlanner import StancePlanner
from HardwareInterface import HardwareInterface
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from Command import Command
import numpy as np
import time

class GaitPlanner:

    def __init__(self, config, state):
        self.config = config
        self.state = state
        self.hardware_interface = HardwareInterface()
        self.stance_planner = StancePlanner(self.state,self.config)
        self.swing_planner = SwingPlanner(self.state, self.config)
        self.command = Command(self.config)

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


    def trot(self, pair_index):


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


    def trot_begin_actuated(self):
        xstance, ystance, zstance, xswing, yswing, zswing = self.trot_begin()
        n=min(len(xswing),len(xstance))

        for phase in range(n):

            loop_time = time.time()

            for leg_index in self.config.leg_pairs[0, :]:
                positions_legpair0 = np.array([xstance[phase], ystance[phase], zstance[phase]])
                current_angles_rad = inverse_kinematics(positions_legpair0, leg_index, self.config)
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index,leg_index] = positions_legpair0[motor_index]

            for leg_index in self.config.leg_pairs[1, :]:
               positions_legpair1 = np.array([xswing[phase], yswing[phase], zswing[phase]])
               current_angles_rad = inverse_kinematics(positions_legpair1, leg_index, self.config)
               for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index, leg_index] = positions_legpair1[motor_index]

            time.sleep((1.0 / self.config.frequency) - (time.time()-loop_time))


    def trot_cycle_actuated(self):
        if self.state.firstIt:
            x0, y0, z0, x1, y1, z1 = self.trot(0)
            lengthx0 = len(x0)
            lengthx1 = len(x1)

        ################# CHECKING VELOCITY CHANGES IN BOTH DIRECTIONS ######################
        elif abs(self.state.velocityX - self.command.velocityX) > 0.0001 or abs(self.state.velocityY - self.command.velocityY) > 0.0001:

            vel_change_x = self.command.velocityX - self.state.velocityX
            vel_change_y = self.command.velocityY - self.state.velocityY
            vel_change_magnitude = np.sqrt(vel_change_x ** 2 + vel_change_y ** 2)

            max_step = 0.01 / self.config.frequency

            # Limit the velocity change per iteration
            if vel_change_magnitude > max_step:
                scale = max_step / vel_change_magnitude
                self.state.velocityX = self.state.velocityX + vel_change_x * scale
                self.state.velocityY = self.state.velocityY + vel_change_y * scale
            else:
                self.state.velocityX = self.command.velocityX
                self.state.velocityY = self.command.velocityY

            ########## CHANGING POSITIONS BASED ON NEW VELOCITY ##############
            if self.state.leg_pair_in_swing[0]:
                x0, y0, z0 = self.swing_planner.discretizer()
                lengthx0 = len(x0)
            else:
                n = self.state.legpair_phases_remaining[0]
                currentX0 = self.state.foot_locations[0, 0]
                currentY0 = self.state.foot_locations[1, 0]
                x0, y0, z0 = self.stance_planner.linear_discretizer_manual(currentX0, currentY0, n)
                x0, y0, z0 = x0[1:], y0[1:], z0[1:]
                lengthx0 = len(x0)
                self.state.legpair_phases_remaining[0] = lengthx0

            if self.state.leg_pair_in_swing[1]:
                x1, y1, z1 = self.swing_planner.discretizer()
                lengthx1 = len(x1)
            else:
                n = self.state.legpair_phases_remaining[1]
                currentX1 = self.state.foot_locations[0, 1]
                currentY1 = self.state.foot_locations[1, 1]
                x1, y1, z1 = self.stance_planner.linear_discretizer_manual(currentX1, currentY1, n)
                x1, y1, z1 = x1[1:], y1[1:], z1[1:]
                lengthx1 = len(x1)
                self.state.legpair_phases_remaining[1] = lengthx1

        ############## CHECKING YAW CHANGE ####################
        if abs(self.command.trot_yaw-self.state.trot_yaw) > 0.0001 and self.state.leg_pair_in_swing[0] == self.state.leg_pair_in_swing[1]:
            yaw_change = self.command.trot_yaw-self.state.trot_yaw

            max_step = 0.1 / self.config.frequency

            # Limit the yaw change per iteration
            if abs(yaw_change) > max_step:
                scale = max_step / yaw_change
                self.state.trot_yaw = self.state.trot_yaw + yaw_change * scale
            else:
                self.state.trot_yaw = self.command.trot_yaw

        ################# CHECK IF PHASE ENDED ###############3
        if self.state.legpair_phases_remaining[0] == 0:
            x0, y0, z0 = self.trot(0)
            lengthx0 = len(x0)
            if not self.state.leg_pair_in_swing[1]:
                self.state.stance_yaw_pair[0] = 0

        if self.state.legpair_phases_remaining[1] == 0:
            x1, y1, z1 = self.trot(1)
            lengthx1 = len(x1)
            if not self.state.leg_pair_in_swing[1]:
                self.state.stance_yaw_pair[1] = 0

        ####################### ACTUATING THE FIRST LEG PAIR ########################################
        positions_legpair0 = np.array([x0[lengthx0 - self.state.legpair_phases_remaining[0]],
                                       y0[lengthx0 - self.state.legpair_phases_remaining[0]],
                                       z0[lengthx0 - self.state.legpair_phases_remaining[0]]])

        # Changing yaw progressively
        if self.state.leg_pair_in_swing[0]:
            rate_swing = self.config.swingtime * self.config.frequency
            self.state.stance_yaw_pair[0] -= self.state.trot_yaw/rate_swing
            minimum = min(0, self.state.trot_yaw)
            maximum = max(0, self.state.trot_yaw)
            self.state.stance_yaw_pair[0] = np.clip(self.state.stance_yaw_pair[0], minimum, maximum)
        else:
            rate_stance = self.config.stancetime * self.config.frequency
            self.state.stance_yaw_pair[0] += self.state.trot_yaw/rate_stance
            minimum = min(0, self.state.trot_yaw)
            maximum = max(0, self.state.trot_yaw)
            self.state.stance_yaw_pair[0] = np.clip(self.state.stance_yaw_pair[0], minimum, maximum)

        # actuating next position
        for leg_index in self.config.leg_pairs[0, :]:
            positions_legpair0 = complete_kinematics(positions_legpair0,self.state.stance_yaw_pair[0],0,0,leg_index,self.config)
            current_angles_rad = inverse_kinematics(positions_legpair0, leg_index, self.config)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                self.state.foot_locations[motor_index, leg_index] = positions_legpair0[motor_index]

        ####################### ACTUATING THE SECOND LEG PAIR ########################################
        positions_legpair1 = np.array([x1[lengthx1 - self.state.legpair_phases_remaining[1]],
                                       y1[lengthx1 - self.state.legpair_phases_remaining[1]],
                                       z1[lengthx1 - self.state.legpair_phases_remaining[1]]])

        # Changing yaw progressively
        if self.state.leg_pair_in_swing[1]:
            rate_swing = self.config.swingtime * self.config.frequency
            self.state.stance_yaw_pair[1] -= self.state.trot_yaw/rate_swing
            minimum = min(0, self.state.trot_yaw)
            maximum = max(0, self.state.trot_yaw)
            self.state.stance_yaw_pair[1] = np.clip(self.state.stance_yaw_pair[1], minimum, maximum)
        else:
            rate_stance = self.config.stancetime * self.config.frequency
            self.state.stance_yaw_pair[1] += self.state.trot_yaw/rate_stance
            minimum = min(0, self.state.trot_yaw)
            maximum = max(0, self.state.trot_yaw)
            self.state.stance_yaw_pair[1] = np.clip(self.state.stance_yaw_pair[1], minimum, maximum)

        for leg_index in self.config.leg_pairs[1, :]:
            positions_legpair1 = complete_kinematics(positions_legpair1, self.state.stance_yaw_pair[1], 0, 0, leg_index, self.config)
            current_angles_rad = inverse_kinematics(positions_legpair1, leg_index, self.config)
            for motor_index in range(3):
                self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                self.state.foot_locations[motor_index, leg_index] = positions_legpair1[motor_index]

        ########## PROGRESSING TO NEXT ITERATION ################
        self.state.legpair_phases_remaining[0] -= 1
        self.state.legpair_phases_remaining[1] -= 1





