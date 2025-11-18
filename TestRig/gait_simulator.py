import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from Configuration import RobotConfig
from State import State
from Kinematics import inverse_kinematics
from Rotation import complete_kinematics
from Command import Command
import time
import sys

class MockHardwareInterface:
    def set_actuator_position(self, angle, leg_index, motor_index):
        pass  # No-op for simulation

# Inject mock before importing GaitPlanner
sys.modules['HardwareInterface'] = type(sys)('HardwareInterface')
sys.modules['HardwareInterface'].HardwareInterface = MockHardwareInterface

from GaitPlanner import GaitPlanner

class GaitSimulator:

    def __init__(self):
        self.config = RobotConfig()
        self.state = State(self.config)
        self.command = Command(self.config)
        self.gait_planner = GaitPlanner(self.config, self.state,self.command)
        self.hardware_interface = MockHardwareInterface()
        self.front_right = []
        self.front_left = []

    def record_point(self, leg_index, pos_xyz):
        xyz = np.array([pos_xyz[0], pos_xyz[1], pos_xyz[2]])
        if leg_index == 0:
            self.front_left.append(xyz)
        elif leg_index == 1:
            self.front_right.append(xyz)

    def simulator(self):

        ####################### Begin ############################
        xstance, ystance, zstance, xswing, yswing, zswing = self.gait_planner.trot_begin()
        n=min(len(xswing),len(xstance))

        for phase in range(n):

            loop_time = time.time()

            for leg_index in self.config.leg_pairs[0, :]:
                positions_legpair0 = np.array([xstance[phase], ystance[phase], zstance[phase]])
                current_angles_rad = inverse_kinematics(positions_legpair0, leg_index, self.config)
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index,leg_index] = positions_legpair0[motor_index]
                    self.record_point(leg_index, positions_legpair0)

            for leg_index in self.config.leg_pairs[1, :]:
               positions_legpair1 = np.array([xswing[phase], yswing[phase], zswing[phase]])
               current_angles_rad = inverse_kinematics(positions_legpair1, leg_index, self.config)
               for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index, leg_index] = positions_legpair1[motor_index]
                    self.record_point(leg_index, positions_legpair1)

            time.sleep((1.0 / self.config.frequency) - (time.time()-loop_time))

        #################### Cycle ########################

        cycle_start = time.time()
        iter = 0

        while 5 > time.time()-cycle_start:
            loop_time = time.time()

            if self.state.firstIt:
                x0, y0, z0, x1, y1, z1 = self.gait_planner.trot(0)
                lengthx0 = len(x0)
                lengthx1 = len(x1)

            elif self.state.legpair_phases_remaining[0] == 0:
                x0, y0, z0 = self.gait_planner.trot(0)
                if not self.state.leg_pair_in_swing[0]:
                    self.state.stance_yaw_pair[0] = 0
                lengthx0 = len(x0)

            elif self.state.legpair_phases_remaining[1] == 0:
                x1, y1, z1 = self.gait_planner.trot(1)
                if not self.state.leg_pair_in_swing[1]:
                    self.state.stance_yaw_pair[1] = 0
                lengthx1 = len(x1)

            if abs(self.state.velocityX- self.command.velocityX) > 0.0001 or abs(self.state.velocityY - self.command.velocityY) > 0.0001:

                vel_change_x = self.command.velocityX - self.state.velocityX
                vel_change_y = self.command.velocityY - self.state.velocityY
                vel_change_magnitude = np.sqrt(vel_change_x ** 2 + vel_change_y ** 2)

                max_step = 0.05 / self.config.frequency

                # Limit the velocity change per iteration
                if vel_change_magnitude > max_step:
                    scale = max_step / vel_change_magnitude
                    self.state.velocityX = self.state.velocityX + vel_change_x * scale
                    self.state.velocityY = self.state.velocityY + vel_change_y * scale
                else:
                    self.state.velocityX = self.command.velocityX
                    self.state.velocityY = self.command.velocityY

                if self.state.leg_pair_in_swing[0]:
                    x0, y0, z0 = self.gait_planner.swing_planner.discretizer()
                    lengthx0 = len(x0)
                else:
                    n = self.state.legpair_phases_remaining[0]
                    currentX0 = self.state.foot_locations[0, 0]
                    currentY0 = self.state.foot_locations[1, 0]
                    x0, y0, z0 = self.gait_planner.stance_planner.linear_discretizer_manual(currentX0,currentY0,n)
                    if n > 1:
                        x0, y0, z0 = x0[1:], y0[1:], z0[1:]
                    lengthx0 = len(x0)
                    self.state.legpair_phases_remaining[0] = lengthx0

                if self.state.leg_pair_in_swing[1]:
                    x1, y1, z1 = self.gait_planner.swing_planner.discretizer()
                    lengthx1 = len(x1)
                else:
                    n = self.state.legpair_phases_remaining[1]
                    currentX1 = self.state.foot_locations[0, 1]
                    currentY1 = self.state.foot_locations[1, 1]
                    x1, y1, z1 = self.gait_planner.stance_planner.linear_discretizer_manual(currentX1, currentY1,n)
                    if n > 1:
                        x1, y1, z1 = x1[1:], y1[1:], z1[1:]
                    lengthx1 = len(x1)
                    self.state.legpair_phases_remaining[1] = lengthx1

            if abs(self.command.trot_yaw - self.state.trot_yaw) > 0.001 and self.state.leg_pair_in_swing[0] == self.state.leg_pair_in_swing[1]:
                yaw_change = self.command.trot_yaw - self.state.trot_yaw

                max_step = 10 / self.config.frequency

                # Limit the yaw change per iteration
                if abs(yaw_change) > max_step:
                    scale = max_step / yaw_change
                    self.state.trot_yaw = self.state.trot_yaw + yaw_change * scale
                else:
                    self.state.trot_yaw = self.command.trot_yaw

            positions_legpair0 = np.array([x0[lengthx0 - self.state.legpair_phases_remaining[0]],
                                           y0[lengthx0 - self.state.legpair_phases_remaining[0]],
                                           z0[lengthx0 - self.state.legpair_phases_remaining[0]]])

            # Changing yaw progressively
            if self.state.leg_pair_in_swing[0]:
                rate_swing = self.config.swingtime * self.config.frequency
                self.state.stance_yaw_pair[0] -= self.state.trot_yaw / rate_swing
                minimum = min(0, self.state.trot_yaw)
                maximum = max(0, self.state.trot_yaw)
                self.state.stance_yaw_pair[0] = np.clip(self.state.stance_yaw_pair[0],minimum,maximum)
            else:
                rate_stance = self.config.stancetime * self.config.frequency
                self.state.stance_yaw_pair[0] += self.state.trot_yaw / rate_stance
                minimum = min(0, self.state.trot_yaw)
                maximum = max(0, self.state.trot_yaw)
                self.state.stance_yaw_pair[0] = np.clip(self.state.stance_yaw_pair[0],minimum,maximum)

            for leg_index in self.config.leg_pairs[0, :]:
                positions_legpair0 = complete_kinematics(positions_legpair0, self.state.stance_yaw_pair[0], 0, 0, leg_index, self.config)
                current_angles_rad = inverse_kinematics(positions_legpair0, leg_index, self.config)
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index, leg_index] = positions_legpair0[motor_index]
                    self.record_point(leg_index, positions_legpair0)


            positions_legpair1 = np.array([x1[lengthx1 - self.state.legpair_phases_remaining[1]],
                                           y1[lengthx1 - self.state.legpair_phases_remaining[1]],
                                           z1[lengthx1 - self.state.legpair_phases_remaining[1]]])

            # Changing yaw progressively
            if self.state.leg_pair_in_swing[1]:
                rate_swing = self.config.swingtime * self.config.frequency
                self.state.stance_yaw_pair[1] -= self.state.trot_yaw / rate_swing
                minimum = min(0, self.state.trot_yaw)
                maximum = max(0, self.state.trot_yaw)
                self.state.stance_yaw_pair[1] = np.clip(self.state.stance_yaw_pair[1],minimum,maximum)
            else:
                rate_stance = self.config.stancetime * self.config.frequency
                self.state.stance_yaw_pair[1] += self.state.trot_yaw / rate_stance
                minimum = min(0, self.state.trot_yaw)
                maximum = max(0, self.state.trot_yaw)
                self.state.stance_yaw_pair[1] = np.clip(self.state.stance_yaw_pair[1],minimum,maximum)

            for leg_index in self.config.leg_pairs[1, :]:
                positions_legpair1 = complete_kinematics(positions_legpair1, self.state.stance_yaw_pair[1], 0, 0, leg_index, self.config)
                current_angles_rad = inverse_kinematics(positions_legpair1, leg_index, self.config)
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)
                    self.state.foot_locations[motor_index, leg_index] = positions_legpair1[motor_index]
                    self.record_point(leg_index, positions_legpair1)

            self.state.legpair_phases_remaining[0] -= 1
            self.state.legpair_phases_remaining[1] -= 1

            #if iter==200 or iter == 400:
            #    self.command.L3[1] += 2
            if 0<iter<100:
                self.command.R3[1] = 1
            else:
                self.command.R3[1] = 0
            iter+=1
            time.sleep((1.0 / self.config.frequency) - (time.time()-loop_time))

        return self.front_left, self.front_right


sim = GaitSimulator()
fl, fr = sim.simulator()
fl = np.asarray(fl)
fr = np.asarray(fr)

def test_swing_planner_animated():
    fl_x = fl[:, 0]
    fl_y = fl[:, 1]
    fl_z = fl[:, 2]
    fr_x = fr[:, 0]
    fr_y = fr[:, 1]
    fr_z = fr[:, 2]

    # Create figure with 2x2 grid: Left leg on left, Right leg on right
    # Top row: XZ view, Bottom row: XY view
    fig, axes = plt.subplots(2, 2, figsize=(14, 10))

    # Front Left XZ (top left)
    ax_fl_xz = axes[0, 0]
    x_margin = (max(fl_x) - min(fl_x)) * 0.1 if max(fl_x) != min(fl_x) else 0.1
    z_margin = (max(fl_z) - min(fl_z)) * 0.1 if max(fl_z) != min(fl_z) else 0.1
    ax_fl_xz.set_xlim(min(fl_x) - x_margin, max(fl_x) + x_margin)
    ax_fl_xz.set_ylim(min(fl_z) - z_margin, max(fl_z) + z_margin)
    ax_fl_xz.set_ylabel('Z Position (m)', fontsize=12)
    ax_fl_xz.set_title('Front Left - XZ Plane', fontsize=14)
    ax_fl_xz.grid(True, alpha=0.3)
    ax_fl_xz.set_aspect('equal')

    # Front Left XY (bottom left)
    ax_fl_xy = axes[1, 0]
    x_margin = (max(fl_x) - min(fl_x)) * 0.1 if max(fl_x) != min(fl_x) else 0.1
    y_margin = (max(fl_y) - min(fl_y)) * 0.1 if max(fl_y) != min(fl_y) else 0.1
    ax_fl_xy.set_xlim(min(fl_x) - x_margin, max(fl_x) + x_margin)
    ax_fl_xy.set_ylim(min(fl_y) - y_margin, max(fl_y) + y_margin)
    ax_fl_xy.set_xlabel('X Position (m)', fontsize=12)
    ax_fl_xy.set_ylabel('Y Position (m)', fontsize=12)
    ax_fl_xy.set_title('Front Left - XY Plane', fontsize=14)
    ax_fl_xy.grid(True, alpha=0.3)
    ax_fl_xy.set_aspect('equal')

    # Front Right XZ (top right)
    ax_fr_xz = axes[0, 1]
    x_margin = (max(fr_x) - min(fr_x)) * 0.1 if max(fr_x) != min(fr_x) else 0.1
    z_margin = (max(fr_z) - min(fr_z)) * 0.1 if max(fr_z) != min(fr_z) else 0.1
    ax_fr_xz.set_xlim(min(fr_x) - x_margin, max(fr_x) + x_margin)
    ax_fr_xz.set_ylim(min(fr_z) - z_margin, max(fr_z) + z_margin)
    ax_fr_xz.set_title('Front Right - XZ Plane', fontsize=14)
    ax_fr_xz.grid(True, alpha=0.3)
    ax_fr_xz.set_aspect('equal')

    # Front Right XY (bottom right)
    ax_fr_xy = axes[1, 1]
    x_margin = (max(fr_x) - min(fr_x)) * 0.1 if max(fr_x) != min(fr_x) else 0.1
    y_margin = (max(fr_y) - min(fr_y)) * 0.1 if max(fr_y) != min(fr_y) else 0.1
    ax_fr_xy.set_xlim(min(fr_x) - x_margin, max(fr_x) + x_margin)
    ax_fr_xy.set_ylim(min(fr_y) - y_margin, max(fr_y) + y_margin)
    ax_fr_xy.set_xlabel('X Position (m)', fontsize=12)
    ax_fr_xy.set_title('Front Right - XY Plane', fontsize=14)
    ax_fr_xy.grid(True, alpha=0.3)
    ax_fr_xy.set_aspect('equal')

    # Create plot elements for Front Left XZ
    line_fl_xz, = ax_fl_xz.plot([], [], 'b-', linewidth=2, alpha=0.5, label='Trajectory')
    points_fl_xz, = ax_fl_xz.plot([], [], 'b.', markersize=4, alpha=0.3)
    current_fl_xz, = ax_fl_xz.plot([], [], 'ro', markersize=10, label='Current', zorder=5)
    ax_fl_xz.legend(loc='best')

    # Create plot elements for Front Left XY
    line_fl_xy, = ax_fl_xy.plot([], [], 'b-', linewidth=2, alpha=0.5, label='Trajectory')
    points_fl_xy, = ax_fl_xy.plot([], [], 'b.', markersize=4, alpha=0.3)
    current_fl_xy, = ax_fl_xy.plot([], [], 'ro', markersize=10, label='Current', zorder=5)
    ax_fl_xy.legend(loc='best')

    # Create plot elements for Front Right XZ
    line_fr_xz, = ax_fr_xz.plot([], [], 'r-', linewidth=2, alpha=0.5, label='Trajectory')
    points_fr_xz, = ax_fr_xz.plot([], [], 'r.', markersize=4, alpha=0.3)
    current_fr_xz, = ax_fr_xz.plot([], [], 'bo', markersize=10, label='Current', zorder=5)
    ax_fr_xz.legend(loc='best')

    # Create plot elements for Front Right XY
    line_fr_xy, = ax_fr_xy.plot([], [], 'r-', linewidth=2, alpha=0.5, label='Trajectory')
    points_fr_xy, = ax_fr_xy.plot([], [], 'r.', markersize=4, alpha=0.3)
    current_fr_xy, = ax_fr_xy.plot([], [], 'bo', markersize=10, label='Current', zorder=5)
    ax_fr_xy.legend(loc='best')

    # Animation initialization function
    def init():
        line_fl_xz.set_data([], [])
        points_fl_xz.set_data([], [])
        current_fl_xz.set_data([], [])
        line_fl_xy.set_data([], [])
        points_fl_xy.set_data([], [])
        current_fl_xy.set_data([], [])
        line_fr_xz.set_data([], [])
        points_fr_xz.set_data([], [])
        current_fr_xz.set_data([], [])
        line_fr_xy.set_data([], [])
        points_fr_xy.set_data([], [])
        current_fr_xy.set_data([], [])
        return (line_fl_xz, points_fl_xz, current_fl_xz, line_fl_xy, points_fl_xy, current_fl_xy,
                line_fr_xz, points_fr_xz, current_fr_xz, line_fr_xy, points_fr_xy, current_fr_xy)

    # Animation update function
    def update(frame):
        TAIL = max(0, frame - 150)

        # Update Front Left XZ
        line_fl_xz.set_data(fl_x[TAIL:frame + 1], fl_z[TAIL:frame + 1])
        points_fl_xz.set_data(fl_x[TAIL:frame + 1], fl_z[TAIL:frame + 1])
        current_fl_xz.set_data([fl_x[frame]], [fl_z[frame]])

        # Update Front Left XY
        line_fl_xy.set_data(fl_x[TAIL:frame + 1], fl_y[TAIL:frame + 1])
        points_fl_xy.set_data(fl_x[TAIL:frame + 1], fl_y[TAIL:frame + 1])
        current_fl_xy.set_data([fl_x[frame]], [fl_y[frame]])

        # Update Front Right XZ
        line_fr_xz.set_data(fr_x[TAIL:frame + 1], fr_z[TAIL:frame + 1])
        points_fr_xz.set_data(fr_x[TAIL:frame + 1], fr_z[TAIL:frame + 1])
        current_fr_xz.set_data([fr_x[frame]], [fr_z[frame]])

        # Update Front Right XY
        line_fr_xy.set_data(fr_x[TAIL:frame + 1], fr_y[TAIL:frame + 1])
        points_fr_xy.set_data(fr_x[TAIL:frame + 1], fr_y[TAIL:frame + 1])
        current_fr_xy.set_data([fr_x[frame]], [fr_y[frame]])

        return (line_fl_xz, points_fl_xz, current_fl_xz, line_fl_xy, points_fl_xy, current_fl_xy,
                line_fr_xz, points_fr_xz, current_fr_xz, line_fr_xy, points_fr_xy, current_fr_xy)

    # Create animation
    num_frames = min(len(fl_x), len(fr_x))
    anim = FuncAnimation(fig, update, init_func=init, frames=num_frames,
                         interval=0.1, blit=False, repeat=True)

    plt.tight_layout()
    plt.show()
    return fl_x, fl_z, fr_x, fr_z, anim

def plot_front_leg_joint_angles():
    cfg = RobotConfig()

    # --- helper: IK over one leg's (x,z) trajectory ---
    def ik_over_traj(xyz: np.ndarray, leg_index: int, offset) -> np.ndarray:
        qs = []
        for x, y, z in xyz:
            qs.append(inverse_kinematics(np.array([x, y+offset, z]), leg_index, cfg))
        return np.asarray(qs)

    # Inverse kinematics for front left (leg 0) and front right (leg 1)
    fl_q_rad = ik_over_traj(fl, 0,cfg.abduction_offset)
    fr_q_rad = ik_over_traj(fr, 1,-cfg.abduction_offset)

    # Unwrap & convert to degrees
    fl_q_deg = np.degrees(np.unwrap(fl_q_rad, axis=0))
    fr_q_deg = np.degrees(np.unwrap(fr_q_rad, axis=0))

    # Time vectors
    t_fl = np.arange(fl_q_deg.shape[0], dtype=float) / cfg.frequency
    t_fr = np.arange(fr_q_deg.shape[0], dtype=float) / cfg.frequency

    # --- global limits for angle plots ---
    gmin = min(fl_q_deg.min(), fr_q_deg.min())
    gmax = max(fl_q_deg.max(), fr_q_deg.max())
    pad = 0.05 * (gmax - gmin if gmax > gmin else 1.0)
    gmin -= pad
    gmax += pad

    # --- build figure: 5 rows × 2 cols ---
    fig, axes = plt.subplots(
        nrows=5, ncols=2, figsize=(14, 14),
        gridspec_kw={'height_ratios': [1, 1, 1, 0.7, 0.7]},
        constrained_layout=True
    )

    motor_names = ['Motor 0', 'Motor 1', 'Motor 2']

    # ---------- Front Left column ----------
    for i in range(3):
        ax = axes[i, 0]
        ax.plot(t_fl, fl_q_deg[:, i], color='tab:blue', lw=1.8)
        if i == 0:
            ax.set_title('Front Left – Joint Angles (deg)')
        ax.set_ylabel(f'{motor_names[i]} [deg]')
        ax.set_ylim(gmin, gmax)
        ax.grid(True, alpha=0.3)

    # X position vs time
    ax = axes[3, 0]
    ax.plot(t_fl, fl[:, 0], color='tab:blue', lw=1.8)
    ax.set_ylabel('X [m]')
    ax.grid(True, alpha=0.3)

    # Z position vs time
    ax = axes[4, 0]
    ax.plot(t_fl, fl[:, 1], color='tab:blue', lw=1.8)
    ax.set_xlabel('Time [s]')
    ax.set_ylabel('Z [m]')
    ax.grid(True, alpha=0.3)

    # ---------- Front Right column ----------
    for i in range(3):
        ax = axes[i, 1]
        ax.plot(t_fr, fr_q_deg[:, i], color='tab:red', lw=1.8)
        if i == 0:
            ax.set_title('Front Right – Joint Angles (deg)')
        ax.set_ylim(gmin, gmax)
        ax.grid(True, alpha=0.3)

    # X position vs time
    ax = axes[3, 1]
    ax.plot(t_fr, fr[:, 0], color='tab:red', lw=1.8)
    ax.grid(True, alpha=0.3)

    # Z position vs time
    ax = axes[4, 1]
    ax.plot(t_fr, fr[:, 1], color='tab:red', lw=1.8)
    ax.set_xlabel('Time [s]')
    ax.grid(True, alpha=0.3)

    # Align time limits for both columns
    tmax_fl = t_fl[-1] if t_fl.size else 0
    tmax_fr = t_fr[-1] if t_fr.size else 0
    for r in range(5):
        axes[r, 0].set_xlim(0, tmax_fl)
        axes[r, 1].set_xlim(0, tmax_fr)

    plt.show()


def animate_foot_3d(foot_data, foot_name='Front Left'):
    """
    Create a 3D animation of foot trajectory with moving camera.

    Parameters:
    -----------
    foot_data : np.ndarray
        Array of shape (N, 3) containing [x, y, z] positions
    foot_name : str
        Name of the foot for the title ('Front Left' or 'Front Right')
    """
    from mpl_toolkits.mplot3d import Axes3D

    x = foot_data[:, 0]
    y = foot_data[:, 1]
    z = foot_data[:, 2]

    fig = plt.figure(figsize=(12, 9))
    ax = fig.add_subplot(111, projection='3d')

    # Set axis limits with margins
    x_margin = (max(x) - min(x)) * 0.2 if max(x) != min(x) else 0.1
    y_margin = (max(y) - min(y)) * 0.2 if max(y) != min(y) else 0.1
    z_margin = (max(z) - min(z)) * 0.2 if max(z) != min(z) else 0.1

    ax.set_xlim(min(x) - x_margin, max(x) + x_margin)
    ax.set_ylim(min(y) - y_margin, max(y) + y_margin)
    ax.set_zlim(min(z) - z_margin, max(z) + z_margin)

    ax.set_xlabel('X Position (m)', fontsize=11)
    ax.set_ylabel('Y Position (m)', fontsize=11)
    ax.set_zlabel('Z Position (m)', fontsize=11)
    ax.set_title(f'{foot_name} - 3D Trajectory', fontsize=14, pad=20)

    # Color based on foot
    color = 'tab:blue' if 'Left' in foot_name else 'tab:red'

    # Create plot elements
    trail_line, = ax.plot([], [], [], color=color, linewidth=2, alpha=0.6, label='Trail')
    full_traj, = ax.plot(x, y, z, color=color, linewidth=0.5, alpha=0.15, label='Full Path')
    current_point, = ax.plot([], [], [], 'o', color='yellow', markersize=12,
                             markeredgecolor='black', markeredgewidth=2, label='Current', zorder=10)

    # Add ground plane for reference
    if min(z) < 0:
        xx, yy = np.meshgrid(
            np.linspace(min(x) - x_margin, max(x) + x_margin, 10),
            np.linspace(min(y) - y_margin, max(y) + y_margin, 10)
        )
        zz = np.zeros_like(xx)
        ax.plot_surface(xx, yy, zz, alpha=0.1, color='gray')

    ax.legend(loc='upper left')
    ax.grid(True, alpha=0.3)

    # Initial camera position
    initial_elev = 20
    initial_azim = 45

    def init():
        trail_line.set_data([], [])
        trail_line.set_3d_properties([])
        current_point.set_data([], [])
        current_point.set_3d_properties([])
        ax.view_init(elev=initial_elev, azim=initial_azim)
        return trail_line, current_point

    def update(frame):
        TAIL = max(0, frame - 80)

        # Update trail
        trail_line.set_data(x[TAIL:frame + 1], y[TAIL:frame + 1])
        trail_line.set_3d_properties(z[TAIL:frame + 1])

        # Update current point
        current_point.set_data([x[frame]], [y[frame]])
        current_point.set_3d_properties([z[frame]])

        # Smooth camera rotation: oscillate azimuth, slowly change elevation
        progress = frame / len(x)
        azim = initial_azim + 60 * np.sin(2 * np.pi * progress * 2)  # 2 full rotations
        elev = initial_elev + 10 * np.sin(2 * np.pi * progress)  # Gentle up-down

        ax.view_init(elev=elev, azim=azim)

        return trail_line, current_point

    num_frames = len(x)
    anim = FuncAnimation(fig, update, init_func=init, frames=num_frames,
                         interval=1, blit=False, repeat=True)

    plt.tight_layout()
    plt.show()

    return anim

#plot_front_leg_joint_angles()
#test_swing_planner_animated()
animate_foot_3d(fr, 'Front Right')
