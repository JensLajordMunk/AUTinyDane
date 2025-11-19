import numpy as np


class SwingPlanner:

    def __init__(self, state, config):
        self.config = config
        self.state = state

    def touchdown_location(self):
        #################### READ ME ######################
        """ This i based on Raiberts touchdown location, and calculates how far the
            foot has drifted backwards during the stance and is placed the same distance
            forward in the swing phase"""

        TDX = self.state.velocityX * self.config.stancetime * 0.5
        TDY = self.state.velocityY * self.config.stancetime * 0.5
        return TDX, TDY  # Raibert et al.

    def thetaZ(self):
        #################### READ ME ######################
        """ This calculates the angle at which the initial arc should end"""
        absVel = np.sqrt(self.state.velocityX ** 2 + self.state.velocityY ** 2)
        return np.arctan2(2 * (self.config.step_height - 2 * self.config.arcR), abs(absVel * self.config.stancetime))

    def velAngle(self):
        #################### READ ME ######################
        """ This calculates the angle between the two velocities, essential
            to project the equations correctly"""
        return np.arctan2(self.state.velocityY, self.state.velocityX)

    def key_points(self):
        assert self.config.arcR * 2 < self.config.step_height, "The radius is too large compared to the step height"

        #################### READ ME ######################
        """ This calculates the coordinates for the beginning and end of the straight
            line. This is ofcourse symmetric so the sign can be changed for the other
            line:

            x1: The touchdown location but negative substracted by how far the arc
            stretches back projected onto the xz plane by the velocity angle

            x2: This is how far the arc stretches back projected onto xz

            y1: same logic as x1 but projected onto yz

            y2: same logic as x1 but projected onto yz

            z1: calculates the top point of the first arc

            z2: Takes the stepheigt - radius = center of circle and adds how far up
            the arc begins"""

        TDX, TDY = self.touchdown_location()

        x1 = -TDX - self.config.arcR * np.sin(self.thetaZ()) * round(np.cos(self.velAngle()), 10)
        x2 = -self.config.arcR * np.sin(self.thetaZ()) * round(np.cos(self.velAngle()), 10)
        y1 = -TDY - self.config.arcR * np.sin(self.thetaZ()) * round(np.sin(self.velAngle()), 10)
        y2 = -self.config.arcR * round(np.sin(self.velAngle()), 10) * np.sin(self.thetaZ())
        z1 = self.config.arcR + np.cos(self.thetaZ()) * self.config.arcR
        z2 = self.config.step_height - self.config.arcR + self.config.arcR * np.cos(self.thetaZ())
        return x1, x2, y1, y2, z1, z2

    def bottom_arc_length(self):  # Symmetry gives that the two bottom arcs are of equal length
        return 2 * 0.25 * self.config.arcR * np.pi + self.config.arcR * (0.5 * np.pi - self.thetaZ())

    def linear_section_length(self):
        x1, x2, y1, y2, z1, z2 = self.key_points()
        return np.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)

    def half_top_arc_length(self):
        return np.pi * self.config.arcR - self.bottom_arc_length()

    def total_swing_length(self):
        return 2 * (self.bottom_arc_length() + self.linear_section_length() + self.half_top_arc_length())

    def section_phase_times(self):
        #################### READ ME ######################
        """ This calculates the time based on the length of each arc, such
            that each section gets roughly the same amount of points, giving
            a somewhat even distribution of point across the swing"""

        bottom_arc_time = self.config.swingtime * self.bottom_arc_length() / self.total_swing_length()
        linear_length_time = self.config.swingtime * self.linear_section_length() / self.total_swing_length()
        top_arc_time = self.config.swingtime * 2 * self.half_top_arc_length() / self.total_swing_length()
        return bottom_arc_time, linear_length_time, top_arc_time

    def arc_angles(self):
        #################### READ ME ######################
        """ This returns the arc angles for the discretizer"""

        angle1_start = -np.pi / 2
        angle1_end = np.pi / 2 + self.thetaZ()

        angle2_start = np.pi / 2 + self.thetaZ()
        angle2_end = np.pi / 2 - self.thetaZ()

        angle3_start = np.pi / 2 - self.thetaZ()
        angle3_end = -np.pi / 2

        return angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end

    def linear_discretizer(self, start, end, ratio):
        #################### READ ME ######################
        """ This can be shortened such that f1 and f2 methods are one.
            Each gives a discretized version between the starting and end
            location by parameterizing the vector into a function of one
            variable and returns the x,y,z"""
        return start + (end - start) * ratio

    def circular_discretizer(self, angle_start, angle_end, xcenter, ycenter, zcenter, ratio):
        #################### READ ME ######################
        """ This first ensures the starting angle is correct so we dont go
            around the circle in the wrong direction and then calculates
            x,y,z making sure to project x,y to their respective planes"""

        sign_product = (1 if self.state.velocityX >= 0 else -1) * (1 if self.state.velocityY >= 0 else -1)

        if sign_product > 0:
            if angle_start < angle_end:
                angle_start += 2 * np.pi
        else:
            if angle_start < angle_end:
                angle_start += 2 * np.pi

        angle = angle_start + (angle_end-angle_start) * ratio
        r = self.config.arcR

        x = r * np.cos(angle) * round(np.cos(self.velAngle()), 10) + xcenter
        y = r * np.cos(angle) * round(np.sin(self.velAngle()), 10) + ycenter
        z = r * np.sin(angle) + zcenter
        return x, y, z

    def zero_velocity_discrete(self,ratio):
        #################### READ ME ######################
        """ Calculates the smooth trajectory of the legs when velocity is zero"""
        if ratio<0.5:
            s = 2*ratio
            h = self.config.step_height * (6 * s ** 5 - 15 * s ** 4 + 10 * s ** 3)
        else:
            s = 1 - (ratio - 0.5)*2
            h = self.config.step_height * (6 * s ** 5 - 15 * s ** 4 + 10 * s ** 3)
        return 0, 0 , h

    def triangular_discretizer(self,time):
        #################### READ ME ######################
        """ Discretizes based on velocity using above functions and returning to gait planner"""
        if round(self.state.velocityX + self.state.velocityY,10) == 0.0:
            ratio = time/self.config.swingtime
            x, y, z = self.zero_velocity_discrete(ratio)

        else:
            bottom_arc_time, linear_length_time, top_arc_time = self.section_phase_times()
            x1, x2, y1, y2, z1, z2 = self.key_points()
            TDX, TDY = self.touchdown_location()
            angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end = self.arc_angles()

            time_interval_1 = bottom_arc_time
            time_interval_2 = time_interval_1 + linear_length_time
            time_interval_3 = time_interval_2 + top_arc_time
            time_interval_4 = time_interval_3 + linear_length_time

            if time <= time_interval_1:
                ratio = time/bottom_arc_time
                x, y, z = self.circular_discretizer(angle1_start, angle1_end, -TDX, -TDY, self.config.arcR, ratio)

            elif time <= time_interval_2:
                ratio = (time-time_interval_1)/linear_length_time
                start = np.array([x1, y1, z1])
                end = np.array([x2, y2, z2])
                p = self.linear_discretizer(start, end, ratio)
                x, y, z = p[0], p[1], p[2]

            elif time <= time_interval_3:
                ratio = (time - time_interval_2) / top_arc_time
                x, y, z = self.circular_discretizer(angle2_start, angle2_end, 0, 0, self.config.step_height - self.config.arcR, ratio)

            elif time <= time_interval_4:
                ratio = (time-time_interval_3)/linear_length_time
                start = np.array([-x2, -y2, z2])
                end = np.array([-x1, -y1, z1])
                p = self.linear_discretizer(start, end, ratio)
                x, y, z = p[0], p[1], p[2]

            else:
                ratio = (time-time_interval_4) / bottom_arc_time
                ratio = min(1.0, ratio) # Clamp to ensure time does not run further in the circle
                x, y, z = self.circular_discretizer(angle3_start, angle3_end, TDX, TDY, self.config.arcR, ratio)

        return x, y, z-self.config.body_height

    def bezier_discretizer(self,time):

        ratio = time/self.config.swingtime
        t = np.clip(ratio, 0.0, 1.0)

        TDX, TDY = self.touchdown_location()

        P0 = np.array([-TDX, -TDY, 0])
        P1 = np.array([-TDX*1.6, -TDY*1.6, 0.25*np.sqrt(TDX**2+TDY**2)])
        P2 = np.array([-TDX, -TDY, self.config.step_height]) # Trækker opad
        P3 = np.array([TDX, TDY, self.config.step_height])
        P4 = np.array([TDX*1.6, TDY*1.6, 0.25*np.sqrt(TDX**2+TDY**2)])
        P5 = np.array([TDX, TDY, 0])

        u = 1 - t
        tt = t * t
        uu = u * u
        ttt = tt * t
        uuu = uu * u

        curve_pos = (uuu * uu) * P0 + (5 * t * uuu * u) * P1 + (10 * tt * uuu) * P2 + (10 * ttt * uu) * P3 + (5 * ttt * t * u) * P4 + (ttt * tt) * P5

        x, y, z = curve_pos[0], curve_pos[1], curve_pos[2]

        return x, y, z-self.config.body_height