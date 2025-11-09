import numpy as np

class SwingPlanner:

    def __init__(self,config):
        self.config = config

    def touchdown_location(self):
        # TODO: Define values in Config
        return self.config.velocity*self.config.stancetime*0.5 # Raibert et al.

    def theta(self):
        return np.arctan2(2*(self.config.step_height - 2*self.config.arcR), abs(self.config.velocity*self.config.stancetime))

    def key_points(self):
        assert self.config.arcR * 2 < self.config.step_height, "The radius is too large compared to the step height"
        if self.config.velocity > 0:
            x1 = -self.touchdown_location() - self.config.arcR*np.sin(self.theta())
            x2 = -self.config.arcR*np.sin(self.theta())
            z1 = self.config.arcR + np.cos(self.theta())*self.config.arcR
            z2 = self.config.step_height - self.config.arcR + self.config.arcR*np.cos(self.theta())
        else:
            x1 = -self.touchdown_location() + self.config.arcR*np.sin(self.theta())
            x2 = self.config.arcR*np.sin(self.theta())
            z1 = self.config.arcR + np.cos(self.theta())*self.config.arcR
            z2 = self.config.step_height - self.config.arcR + self.config.arcR*np.cos(self.theta())
        return x1, x2, z1, z2

    def bottom_arc_length(self): #Symmetry gives that the two bottom arcs are of equal length
        return 2*0.25*self.config.arcR*np.pi + self.config.arcR*(0.5*np.pi-self.theta())

    def linear_section_length(self):
        x1, x2, z1, z2 = self.key_points()
        return np.sqrt( (x2-x1)**2 + (z2-z1)**2 )

    def half_top_arc_length(self):
        return np.pi*self.config.arcR-self.bottom_arc_length()

    def total_swing_length(self):
        return 2 * (self.bottom_arc_length() + self.linear_section_length() + self.half_top_arc_length())

    def section_phase_times(self):
        bottom_arc_time = self.config.swingtime*self.bottom_arc_length()/self.total_swing_length()
        linear_length_time = self.config.swingtime*self.linear_section_length()/self.total_swing_length()
        top_arc_time = self.config.swingtime*2*self.half_top_arc_length()/self.total_swing_length()
        return bottom_arc_time, linear_length_time, top_arc_time

    def m(self):
        x1, x2, z1, z2 = self.key_points()
        denominator = x2 - x1
        assert abs(denominator) > 1e-10, "Denomitor is zero in linear calculation"
        return (z2 - z1)/(x2 - x1)

    def arc_angles(self):
        x1, x2, z1, z2 = self.key_points()
        if self.config.velocity > 0:
            angle1_start = -np.pi/2
            angle1_end = np.pi/2 + self.theta()

            angle2_start = np.pi/2 + self.theta()
            angle2_end = np.pi / 2 - self.theta()

            angle3_start = np.pi/2 - self.theta()
            angle3_end = -np.pi / 2

        else:
            angle1_start = -np.pi / 2
            angle1_end = np.pi / 2 - self.theta()

            angle2_start = np.pi / 2 - self.theta()
            angle2_end = np.pi / 2 + self.theta()

            angle3_start = np.pi / 2 + self.theta()
            angle3_end = -np.pi / 2
        return angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end

    def f1(self,x):
        x1, x2, z1, z2 = self.key_points()
        return self.m() * (x - x1) + z1

    def f2(self,x):
        x1, x2, z1, z2 = self.key_points()
        return -self.m() * (x + x1) + z1

    def f_lims(self):
        x1, x2, z1, z2 = self.key_points()
        lim1 = np.array([x1,x2])
        assert np.all(np.isfinite(lim1)), "lim3 is not finite"
        lim2 = np.array([-x2,-x1])
        assert np.all(np.isfinite(lim2)), "lim5 is not finite"

        return lim1, lim2

    def circular_discretizer(self, angle_start, angle_end, xcenter, zcenter, duration):
        n = max(2,int(duration * self.config.frequency))

        if self.config.velocity > 0:
            if angle_start< angle_end:
                angle_start+=2*np.pi
        else:
            if angle_start > angle_end:
                angle_end += 2 * np.pi

        angles = np.linspace(angle_start,angle_end,n)

        x_uni = self.config.arcR*np.cos(angles) + xcenter
        z_uni = self.config.arcR*np.sin(angles) + zcenter
        return x_uni,z_uni

    def linear_discretizer(self,numpy_func,limit,duration):
        n = max(2,int(duration * self.config.frequency))
        x_uni=np.linspace(limit[0],limit[1],n)
        z_uni=numpy_func(x_uni)
        return x_uni, z_uni

    def discretizer(self):

        lim1, lim2 = self.f_lims()
        bottom_arc_time, linear_length_time, top_arc_time = self.section_phase_times()
        angle1_start, angle1_end, angle2_start, angle2_end, angle3_start, angle3_end = self.arc_angles()


        x1_discretized, z1_discretized = self.circular_discretizer(angle1_start, angle1_end, -self.touchdown_location(), self.config.arcR, bottom_arc_time)
        assert np.all(np.isfinite(x1_discretized)), "x1_discretized is not finite"
        assert np.all(np.isfinite(z1_discretized)), "z1_discretized is not finite"

        x2_discretized, z2_discretized = self.linear_discretizer(self.f1, lim1, linear_length_time)
        assert np.all(np.isfinite(x2_discretized)), "x3_discretized is not finite"
        assert np.all(np.isfinite(z2_discretized)), "z3_discretized is not finite"

        x3_discretized, z3_discretized = self.circular_discretizer(angle2_start, angle2_end, 0, self.config.step_height-self.config.arcR, top_arc_time)
        assert np.all(np.isfinite(x3_discretized)), "x4_discretized is not finite"
        assert np.all(np.isfinite(z3_discretized)), "z4_discretized is not finite"

        x4_discretized, z4_discretized = self.linear_discretizer(self.f2, lim2, linear_length_time)
        assert np.all(np.isfinite(x4_discretized)), "x5_discretized is not finite"
        assert np.all(np.isfinite(z4_discretized)), "z5_discretized is not finite"

        x5_discretized, z5_discretized = self.circular_discretizer(angle3_start, angle3_end, self.touchdown_location(), self.config.arcR, bottom_arc_time)
        assert np.all(np.isfinite(x5_discretized)), "x6_discretized is not finite"
        assert np.all(np.isfinite(z5_discretized)), "z6_discretized is not finite"

        x_discrete = np.concatenate([x1_discretized,x2_discretized,x3_discretized,x4_discretized,x5_discretized])
        z_discrete = np.concatenate([z1_discretized, z2_discretized, z3_discretized, z4_discretized, z5_discretized]) - self.config.Z_zero

        return x_discrete, z_discrete