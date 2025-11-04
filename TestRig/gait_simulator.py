import matplotlib
matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from Configuration import RobotConfig
from State import State
from collections import deque
from Kinematics import inverse_kinematics
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
        self.gait_planner = GaitPlanner(self.config)
        self.state = State()
        self.hardware_interface = MockHardwareInterface()
        self.front_right = []
        self.front_left = []

    def simulator(self):

        ####################### Begin ############################
        xstance, zstance, xswing, zswing = self.gait_planner.trot_begin()

        blockLeft=np.column_stack([xstance,zstance])
        self.front_left.append(blockLeft)
        blockRight=np.column_stack([xswing,zswing])
        self.front_right.append(blockRight)

        n=min(len(xswing),len(xstance))

        for phase in range(n):

            loop_time = time.time()

            for leg_index in self.config.leg_pairs[0, :]:
                current_angles_rad = inverse_kinematics(np.array([xstance[phase], 0, zstance[phase]]), leg_index, self.config)
                for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)

            for leg_index in self.config.leg_pairs[1, :]:
               current_angles_rad = inverse_kinematics(np.array([xswing[phase], 0, zswing[phase]]), leg_index, self.config)
               for motor_index in range(3):
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)

            time.sleep((1.0 / self.config.frequency) - (time.time()-loop_time))

        #################### Cycle ########################
        cycle_start = time.time()
        while 3 > time.time()-cycle_start:
            loop_time = time.time()

            if self.config.firstIt:
                x0, z0, x1, z1 = self.gait_planner.trot(self.config.leg_pair_in_swing[0], 0)

                blockRight = np.column_stack([x1, z1])
                self.front_right.append(blockRight)

                blockLeft = np.column_stack([x0, z0])
                self.front_left.append(blockLeft)

                self.config.firstIt = False
                lengthx0 = len(x0)
                lengthx1 = len(x1)

            if self.config.legpair_phases_remaining[0] == 0:
                x0, z0 = self.gait_planner.trot(self.config.leg_pair_in_swing[0], 0)
                blockLeft = np.column_stack([x0, z0])
                self.front_left.append(blockLeft)
                lengthx0 = len(x0)

            if self.config.legpair_phases_remaining[1] == 0:
                x1, z1 = self.gait_planner.trot(self.config.leg_pair_in_swing[1], 1)
                blockRight = np.column_stack([x1, z1])
                self.front_right.append(blockRight)
                lengthx1 = len(x1)

            for leg_index in self.config.leg_pairs[0, :]:
                for motor_index in range(3):
                    current_angles_rad = inverse_kinematics(np.array([x0[lengthx0-self.config.legpair_phases_remaining[0]], 0, z0[lengthx0-self.config.legpair_phases_remaining[0]]]), leg_index, self.config)
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)

            for leg_index in self.config.leg_pairs[1, :]:
                for motor_index in range(3):
                    current_angles_rad = inverse_kinematics(np.array([x1[lengthx1-self.config.legpair_phases_remaining[1]], 0, z1[lengthx1-self.config.legpair_phases_remaining[1]]]), leg_index, self.config)
                    self.hardware_interface.set_actuator_position(current_angles_rad[motor_index], leg_index, motor_index)

            time.sleep((1.0 / self.config.frequency) - (time.time()-loop_time))


            self.config.legpair_phases_remaining[0] -= 1
            self.config.legpair_phases_remaining[1] -= 1

        return self.front_left, self.front_right


sim = GaitSimulator()
fl, fr = sim.simulator()
fl = np.vstack(fl)
fr = np.vstack(fr)

def test_swing_planner_animated():

    config = RobotConfig()

    fl_x = fl[:,0]
    fl_z = fl[:,1]
    fr_x = fr[:,0]
    fr_z = fr[:,1]

    # Create figure with two subplots side by side
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 10))

    # Set up the left plot (Front Left)
    x_margin = (max(fl_x) - min(fl_x)) * 0.1
    z_margin = (max(fl_z) - min(fl_z)) * 0.1
    ax1.set_xlim(min(fl_x) - x_margin, max(fl_x) + x_margin)
    ax1.set_ylim(min(fl_z) - z_margin, max(fl_z) + z_margin)
    ax1.set_ylabel('Z Position (m)', fontsize=12)
    ax1.set_title(f'Front Left Leg Trajectory ({len(fl_x)} points)', fontsize=14)
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect('equal')

    # Set up the right plot (Front Right)
    x_margin = (max(fr_x) - min(fr_x)) * 0.1
    z_margin = (max(fr_z) - min(fr_z)) * 0.1
    ax2.set_xlim(min(fr_x) - x_margin, max(fr_x) + x_margin)
    ax2.set_ylim(min(fr_z) - z_margin, max(fr_z) + z_margin)
    ax2.set_xlabel('X Position (m)', fontsize=12)
    ax2.set_ylabel('Z Position (m)', fontsize=12)
    ax2.set_title(f'Front Right Leg Trajectory ({len(fr_x)} points)', fontsize=14)
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect('equal')

    # Create plot elements for left leg
    line_l, = ax1.plot([], [], 'b-', linewidth=2, alpha=0.5, label='Trajectory')
    points_l, = ax1.plot([], [], 'b.', markersize=4, alpha=0.3)
    current_l, = ax1.plot([], [], 'ro', markersize=10, label='Current', zorder=5)
    ax1.legend(loc='best')

    # Create plot elements for right leg
    line_r, = ax2.plot([], [], 'r-', linewidth=2, alpha=0.5, label='Trajectory')
    points_r, = ax2.plot([], [], 'r.', markersize=4, alpha=0.3)
    current_r, = ax2.plot([], [], 'bo', markersize=10, label='Current', zorder=5)
    ax2.legend(loc='best')

    # Animation initialization function
    def init():
        line_l.set_data([], [])
        points_l.set_data([], [])
        current_l.set_data([], [])
        line_r.set_data([], [])
        points_r.set_data([], [])
        current_r.set_data([], [])
        return line_l, points_l, current_l, line_r, points_r, current_r

    # Animation update function
    def update(frame):
        # Update left leg
        line_l.set_data(fl_x[:frame+1], fl_z[:frame+1])
        points_l.set_data(fl_x[:frame+1], fl_z[:frame+1])
        current_l.set_data([fl_x[frame]], [fl_z[frame]])
        ax1.set_title(f'Front Left - Point {frame + 1}/{len(fl_x)}', fontsize=14)

        # Update right leg
        line_r.set_data(fr_x[:frame+1], fr_z[:frame+1])
        points_r.set_data(fr_x[:frame+1], fr_z[:frame+1])
        current_r.set_data([fr_x[frame]], [fr_z[frame]])
        ax2.set_title(f'Front Right - Point {frame + 1}/{len(fr_x)}', fontsize=14)

        return line_l, points_l, current_l, line_r, points_r, current_r

    # Create animation (using the minimum length in case arrays differ)
    num_frames = min(len(fl_x), len(fr_x))
    anim = FuncAnimation(fig, update, init_func=init, frames=num_frames,
                         interval=5, blit=False, repeat=True)

    plt.tight_layout()
    plt.show()

    return fl_x, fl_z, fr_x, fr_z, anim

def plot_front_leg_joint_angles():
    cfg = RobotConfig()

    # --- helper: IK over one leg's (x,z) trajectory ---
    def ik_over_traj(xz: np.ndarray, leg_index: int, offset) -> np.ndarray:
        qs = []
        for x, z in xz:
            qs.append(inverse_kinematics(np.array([x, offset, z]), leg_index, cfg))
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


plot_front_leg_joint_angles()
#test_swing_planner_animated()