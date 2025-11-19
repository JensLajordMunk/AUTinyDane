import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import sys


class MockHardwareInterface:
    def set_actuator_position(self, angle, leg_index, motor_index):
        pass


class VirtualClock:
    def __init__(self):
        self.current_time = 0.0

    def time(self):
        return self.current_time

    def sleep(self, seconds):
        self.current_time += seconds


sys.modules['HardwareInterface'] = type(sys)('HardwareInterface')
sys.modules['HardwareInterface'].HardwareInterface = MockHardwareInterface

from Configuration import RobotConfig
from State import State
from Command import Command, Modes
import GaitPlannerV2 as GP_Module


class KinematicMonitor:
    @staticmethod
    def check_workspace_safety(x, y, z, config):
        min_safe_z = -config.body_height + 0.02
        if z > min_safe_z:
            z = min_safe_z

        max_reach = (config.leg_up + config.leg_low) * 0.95

        dist = np.sqrt(x ** 2 + y ** 2 + z ** 2)
        if dist > max_reach:
            scale = max_reach / dist
            x *= scale
            y *= scale
            z *= scale

        return x, y, z

    @staticmethod
    def analyze_trajectory(positions, dt):
        pos = np.array(positions)
        vel = np.gradient(pos, axis=0) / dt
        acc = np.gradient(vel, axis=0) / dt
        jerk = np.gradient(acc, axis=0) / dt
        jerk_norm = np.linalg.norm(jerk, axis=1)
        return vel, acc, jerk, jerk_norm


class GaitSimulator:
    def __init__(self):
        self.config = RobotConfig()
        self.state = State(self.config)
        self.command = Command(self.config)
        self.clock = VirtualClock()
        GP_Module.time = self.clock
        self.gait_planner = GP_Module.GaitPlanner(self.config, self.state, self.command)
        self.log_fl = []
        self.log_fr = []
        self.time_log = []

    def record_state(self):
        self.log_fl.append(self.state.foot_locations[:, 0].copy())
        self.log_fr.append(self.state.foot_locations[:, 1].copy())
        self.time_log.append(self.clock.time())

    def set_virtual_joystick(self, target_vx, target_vy, target_yaw_rate):
        lx = target_vx / self.config.max_velocityX
        ly = target_vy / self.config.max_velocityY
        rx = target_yaw_rate / self.config.max_yaw_rate

        self.command.L3[0] = np.clip(lx, -1.0, 1.0)
        self.command.L3[1] = np.clip(ly, -1.0, 1.0)
        self.command.R3[1] = np.clip(rx, -1.0, 1.0)

    def run(self, duration_seconds=15.0):
        dt = 1.0 / 100
        steps = int(duration_seconds / dt)
        self.gait_planner.trot_begin()

        for i in range(steps):
            t = self.clock.time()

            if t < 1.0:
                self.set_virtual_joystick(0.0, 0.0, 0.0)
            elif t < 5.0:
                self.set_virtual_joystick(0.4, 0.0, 0.0)
            elif t < 10.0:
                self.set_virtual_joystick(0.0, 0.3, 0.0)
            elif t < 15.0:
                self.set_virtual_joystick(0.3, 0.3, 1.0)

            self.gait_planner.trot_cycle_actuated()
            self.record_state()
            self.clock.sleep(dt)

        return np.array(self.log_fl), np.array(self.log_fr), dt


def animate_foot_3d(foot_data, title):
    x, y, z = foot_data[:, 0], foot_data[:, 1], foot_data[:, 2]

    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title(title)

    mid_x = np.mean(x)
    mid_y = np.mean(y)

    ax.set_xlim(mid_x - 0.3, mid_x + 0.3)
    ax.set_ylim(mid_y - 0.3, mid_y + 0.3)
    ax.set_zlim(min(z) - 0.02, max(z) + 0.05)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    line, = ax.plot([], [], [], lw=2, color='r', label='Trajectory')
    point, = ax.plot([], [], [], 'bo', markersize=8, label='Foot')

    ax.legend()

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        point.set_data([], [])
        point.set_3d_properties([])
        return line, point

    def update(frame):
        tail = max(0, frame - 150)
        line.set_data(x[tail:frame], y[tail:frame])
        line.set_3d_properties(z[tail:frame])

        point.set_data([x[frame]], [y[frame]])
        point.set_3d_properties([z[frame]])

        return line, point

    anim = FuncAnimation(fig, update, init_func=init, frames=len(x), interval=10, blit=False)
    plt.show()


def visualize_results(fl, fr, dt):
    fl_x, fl_y, fl_z = fl[:, 0], fl[:, 1], fl[:, 2]
    fr_x, fr_y, fr_z = fr[:, 0], fr[:, 1], fr[:, 2]

    vel, acc, jerk, jerk_norm = KinematicMonitor.analyze_trajectory(fr, dt)

    fig = plt.figure(figsize=(14, 10))
    gs = fig.add_gridspec(3, 2)

    ax1 = fig.add_subplot(gs[0, 0])
    ax1.plot(fl_x, fl_y, label='Front Left')
    ax1.plot(fr_x, fr_y, label='Front Right')
    ax1.set_title("Pos: Top View (XY)")
    ax1.axis('equal')
    ax1.legend()
    ax1.grid(True)

    ax2 = fig.add_subplot(gs[1, 0])
    ax2.plot(fl_x, fl_z, label='FL')
    ax2.plot(fr_x, fr_z, label='FR')
    ax2.set_title("Pos: Side View (XZ)")
    ax2.grid(True)

    ax4 = fig.add_subplot(gs[0, 1])
    time_axis = np.arange(len(jerk_norm)) * dt
    ax4.plot(time_axis, jerk_norm, color='purple', lw=1)
    ax4.set_title(f"JERK Magnitude")
    ax4.set_ylabel("m/s^3")
    ax4.grid(True)
    ax4.axhline(y=500, color='r', linestyle='--', alpha=0.5, label='Threshold')
    ax4.legend()

    ax5 = fig.add_subplot(gs[1, 1])
    acc_norm = np.linalg.norm(acc, axis=1)
    ax5.plot(time_axis, acc_norm, color='orange', lw=1)
    ax5.set_title("Acceleration Magnitude")
    ax5.set_ylabel("m/s^2")
    ax5.grid(True)

    plt.tight_layout()
    plt.show()

    print("Starting 3D Animation...")
    animate_foot_3d(fr, 'Front Right - 3D Animation')


if __name__ == "__main__":
    sim = GaitSimulator()
    fl, fr, dt = sim.run(duration_seconds=15.0)
    visualize_results(fl, fr, dt)