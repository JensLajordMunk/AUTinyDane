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
        return TDX, TDY # Raibert et al.

    def thetaZ(self):
        #################### READ ME ######################
        """ This calculates the angle at which the initial arc should end"""
        absVel = np.sqrt( self.state.velocityX**2 + self.state.velocityY**2)
        return np.arctan2(2*(self.config.step_height - 2*self.config.arcR), abs(absVel*self.config.stancetime))

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
        x1 = -TDX - self.config.arcR * np.sin(self.thetaZ()) * round(np.cos(self.velAngle()),10)
        x2 = -self.config.arcR*np.sin(self.thetaZ()) * round(np.cos(self.velAngle()),10)
        y1 = -TDY - self.config.arcR*np.sin(self.thetaZ()) * round(np.sin(self.velAngle()),10)
        y2 = -self.config.arcR*round(np.sin(self.velAngle()),10)*np.sin(self.thetaZ())
        z1 = self.config.arcR + np.cos(self.thetaZ()) * self.config.arcR
        z2 = self.config.step_height - self.config.arcR + self.config.arcR*np.cos(self.thetaZ())
        return x1, x2, y1, y2, z1, z2

    def bottom_arc_length(self): #Symmetry gives that the two bottom arcs are of equal length
        return 2*0.25*self.config.arcR*np.pi + self.config.arcR*(0.5*np.pi-self.thetaZ())

    def linear_section_length(self):
        x1, x2, y1, y2, z1, z2 = self.key_points()
        return np.sqrt( (x2-x1)**2 + (y2-y1)**2 + (z2-z1)**2 )

    def half_top_arc_length(self):
        return np.pi*self.config.arcR-self.bottom_arc_length()

    def total_swing_length(self):
        return 2 * (self.bottom_arc_length() + self.linear_section_length() + self.half_top_arc_length())

    def section_phase_times(self):
        #################### READ ME ######################
        """ This calculates the time based on the length of each arc, such
            that each section gets roughly the same amount of points, giving
            a somewhat even distribution of point across the swing"""

        bottom_arc_time = self.config.swingtime*self.bottom_arc_length()/self.total_swing_length()
        linear_length_time = self.config.swingtime*self.linear_section_length()/self.total_swing_length()
        top_arc_time = self.config.swingtime*2*self.half_top_arc_length()/self.total_swing_length()
        return bottom_arc_time, linear_length_time, top_arc_time

    def arc_angles(self):
        #################### READ ME ######################
        """ This returns the arc angles for the discretizer"""

        angle1_start = -np.pi/2
        angle1_end = np.pi/2 + self.thetaZ()

        angle2_start = np.pi/2 + self.thetaZ()
        angle2_end = np.pi / 2 - self.thetaZ()

        angle3_start = np.pi/2 - self.thetaZ()
        angle3_end = -np.pi / 2

        return angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end

    def f1(self):
        #################### READ ME ######################
        """ This can be shortened such that f1 and f2 methods are one.
            Each gives a discretized version between the starting and end
            location by parameterizing the vector into a function of one
            variable and returns the x,y,z"""
        x1, x2, y1, y2, z1, z2 = self.key_points()
        bottom_arc_time, linear_length_time, top_arc_time = self.section_phase_times()
        n = max(2, int(linear_length_time * self.config.frequency))
        lamd = np.linspace(0,1,n)
        npvals = (1-lamd)*np.array([[x1],[y1],[z1]]) + lamd*np.array([[x2],[y2],[z2]])
        return npvals[0], npvals[1], npvals[2]

    def f2(self):
        x1, x2, y1, y2, z1, z2 = self.key_points()
        bottom_arc_time, linear_length_time, top_arc_time = self.section_phase_times()
        n = max(2, int(linear_length_time * self.config.frequency))
        lamd = np.linspace(0, 1, n)
        npvals = (1-lamd)*np.array([[-x2],[-y2],[z2]]) + lamd*np.array([[-x1],[-y1],[z1]])
        return npvals[0], npvals[1], npvals[2]

    def circular_discretizer(self, angle_start, angle_end, xcenter, ycenter, zcenter, duration):
        #################### READ ME ######################
        """ This first ensures the starting angle is correct so we dont go
            around the circle in the wrong direction and then calculates
            x,y,z making sure to project x,y to their respective planes"""

        n = max(2,int(duration * self.config.frequency))

        sign_product = (1 if self.state.velocityX >= 0 else -1) * (1 if self.state.velocityY >= 0 else -1)

        if sign_product > 0 :
            if angle_start < angle_end:
                angle_start += 2 * np.pi
        else:
            if angle_start < angle_end:
                angle_start += 2 * np.pi
        angles = np.linspace(angle_start, angle_end, n)

        unirform_arc = self.config.arcR*np.cos(angles)
        x_uni = unirform_arc * round(np.cos(self.velAngle()),10) + xcenter
        y_uni = unirform_arc * round(np.sin(self.velAngle()),10) + ycenter
        z_uni = self.config.arcR * np.sin(angles) + zcenter
        return x_uni, y_uni, z_uni

    def zero_velocity_discrete(self):
        #################### READ ME ######################
        """ Calculates the smooth trajectory of the legs when velocity is zero"""

        n = int(0.5 * self.config.swingtime * self.config.frequency)
        array_up = np.linspace(0, 1, n)
        array_down = np.linspace(1, 0, n)
        smoothed_up = 6 * array_up ** 5 - 15 * array_up ** 4 + 10 * array_up ** 3
        smoothed_down = 6 * array_down ** 5 - 15 * array_down ** 4 + 10 * array_down ** 3
        smoothed_motion_up = self.config.step_height * smoothed_up
        smoothed_motion_down = self.config.step_height * smoothed_down
        return np.concatenate((smoothed_motion_up, smoothed_motion_down))

    def discretizer(self):
        #################### READ ME ######################
        """ Discretizes based on velocity using above functions and returning to gait planner"""
        if self.state.velocityX + self.state.velocityY == 0:
            z_discrete = self.zero_velocity_discrete() - self.config.body_height
            x_discrete = np.zeros(len(z_discrete))
            y_discrete = np.zeros(len(z_discrete))
        else:
            bottom_arc_time, linear_length_time, top_arc_time = self.section_phase_times()
            angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end = self.arc_angles()
            TDX, TDY = self.touchdown_location()

            x1_discretized, y1_discretized, z1_discretized = self.circular_discretizer(angle1_start, angle1_end, -TDX, -TDY, self.config.arcR, bottom_arc_time)

            x2_discretized, y2_discretized, z2_discretized = self.f1()

            x3_discretized, y3_discretized, z3_discretized = self.circular_discretizer(angle2_start, angle2_end, 0, 0, self.config.step_height-self.config.arcR, top_arc_time)

            x4_discretized, y4_discretized, z4_discretized = self.f2()

            x5_discretized, y5_discretized, z5_discretized = self.circular_discretizer(angle3_start, angle3_end, TDX, TDY, self.config.arcR, bottom_arc_time)

            x_discrete = np.concatenate([x1_discretized, x2_discretized, x3_discretized, x4_discretized, x5_discretized])
            y_discrete = np.concatenate([y1_discretized, y2_discretized, y3_discretized, y4_discretized, y5_discretized])
            z_discrete = np.concatenate([z1_discretized, z2_discretized, z3_discretized, z4_discretized, z5_discretized]) - self.config.body_height

        return x_discrete, y_discrete, z_discrete