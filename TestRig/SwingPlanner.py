import numpy as np

class SwingPlanner:

    def __init__(self, state, config):
        self.config = config
        self.state = state

    def touchdown_location(self):
        # TODO: Define values in Config
        TDX = self.state.velocityX * self.config.stancetime * 0.5
        TDY = self.state.velocityY * self.config.stancetime * 0.5
        return TDX, TDY # Raibert et al.

    def thetaZ(self):
        absVel = np.sqrt( self.state.velocityX**2 + self.state.velocityY**2)
        return np.arctan2(2*(self.config.step_height - 2*self.config.arcR), abs(absVel*self.config.stancetime))

    def velAngle(self):
        return np.arctan2(self.state.velocityY, self.state.velocityX)

    def key_points(self):
        assert self.config.arcR * 2 < self.config.step_height, "The radius is too large compared to the step height"

        TDX, TDY = self.touchdown_location()

        signx = 1 if self.state.velocityX >= 0 else -1
        signy = 1 if self.state.velocityY >= 0 else -1
        if signx<0 and signy>0:
            x1 = (-np.sqrt(TDX ** 2 + TDY ** 2) - self.config.arcR * np.sin(self.thetaZ())) * np.cos(self.velAngle())
            x2 = -self.config.arcR*np.cos(self.velAngle())*np.sin(self.thetaZ())*signy
            y1 = (-np.sqrt(TDX**2+TDY**2) - self.config.arcR*np.sin(self.thetaZ())) * np.sin(self.velAngle())
            y2 = -self.config.arcR*np.sin(self.velAngle())*np.sin(self.thetaZ())*signy
            z1 = self.config.arcR + np.cos(self.thetaZ()) * self.config.arcR
            z2 = self.config.step_height - self.config.arcR + self.config.arcR*np.cos(self.thetaZ())
        elif signx>0 and signy<0:
            x1 = (-np.sqrt(TDX ** 2 + TDY ** 2) - self.config.arcR * np.sin(self.thetaZ())) * np.cos(self.velAngle())
            x2 = -self.config.arcR*np.cos(self.velAngle())*np.sin(self.thetaZ())*signx
            y1 = (-np.sqrt(TDX**2+TDY**2) - self.config.arcR*np.sin(self.thetaZ())) * np.sin(self.velAngle())
            y2 = -self.config.arcR*np.sin(self.velAngle())*np.sin(self.thetaZ())*signx
            z1 = self.config.arcR + np.cos(self.thetaZ()) * self.config.arcR
            z2 = self.config.step_height - self.config.arcR + self.config.arcR*np.cos(self.thetaZ())
        else:
            x1 = (-np.sqrt(TDX ** 2 + TDY ** 2) - self.config.arcR * np.sin(self.thetaZ())) * np.cos(self.velAngle())
            x2 = -self.config.arcR*np.cos(self.velAngle())*np.sin(self.thetaZ())
            y1 = (-np.sqrt(TDX**2+TDY**2) - self.config.arcR*np.sin(self.thetaZ())) * np.sin(self.velAngle())
            y2 = -self.config.arcR*np.sin(self.velAngle())*np.sin(self.thetaZ())
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
        bottom_arc_time = self.config.swingtime*self.bottom_arc_length()/self.total_swing_length()
        linear_length_time = self.config.swingtime*self.linear_section_length()/self.total_swing_length()
        top_arc_time = self.config.swingtime*2*self.half_top_arc_length()/self.total_swing_length()
        return bottom_arc_time, linear_length_time, top_arc_time

    def arc_angles(self):
        signx = 1 if self.state.velocityX >= 0 else -1
        signy = 1 if self.state.velocityY >= 0 else -1

        angle1_start = -np.pi/2
        angle1_end = np.pi/2 + self.thetaZ()

        angle2_start = np.pi/2 + self.thetaZ()
        angle2_end = np.pi / 2 - self.thetaZ()

        angle3_start = np.pi/2 - self.thetaZ()
        angle3_end = -np.pi / 2

        return angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end

    def f_lims(self):
        x1, x2, y1, y2, z1, z2 = self.key_points()
        lim1 = np.array([x1,x2])
        assert np.all(np.isfinite(lim1)), "lim1 is not finite"
        lim2 = np.array([-x2,-x1])
        assert np.all(np.isfinite(lim2)), "lim2 is not finite"
        lim3 = np.array([y1,y2])
        assert np.all(np.isfinite(lim3)), "lim3 is not finite"
        lim4 = np.array([-y2,-y1])
        assert np.all(np.isfinite(lim4)), "lim4 is not finite"

        return lim1, lim2, lim3, lim4

    def f1(self):
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

        x_uni = unirform_arc * np.cos(self.velAngle()) + xcenter
        y_uni = unirform_arc * np.sin(self.velAngle()) + ycenter
        z_uni = self.config.arcR * np.sin(angles) + zcenter
        return x_uni, y_uni, z_uni

    def discretizer(self):

        lim1, lim2, lim3, lim4 = self.f_lims()
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