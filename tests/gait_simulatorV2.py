import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import cv2
import mpl_toolkits.mplot3d.art3d as art3d
import sys


# ----------------- Mocking Hardware -----------------
class MockHardwareInterface:
    def __init__(self):
        self.angles = np.zeros((3, 4))

    def set_actuator_position(self, angle, leg_index, motor_index):
        self.angles[motor_index, leg_index] = angle

    def get_imu_tilt(self):
        return 0, 0, 0, 0


class VirtualClock:
    def __init__(self):
        self.current_time = 0.0

    def time(self):
        return self.current_time

    def sleep(self, seconds):
        self.current_time += seconds


sys.modules['src.HardwareInterface'] = type(sys)('src.HardwareInterface')
sys.modules['src.HardwareInterface'].HardwareInterface = MockHardwareInterface

from src.Configuration import RobotConfig
from src.State import State
from src.Command import Command
import src.GaitPlannerV2 as GP_Module


# ----------------- Kinematics Helper -----------------
def get_joint_positions(angles, leg_index, config):
    theta_abd, theta_hip = angles[0], angles[1]
    l_up, offset = config.leg_up, config.abduction_offsets[leg_index]

    # 1. Abduction Link
    hip_pos = np.array([0, offset * np.cos(theta_abd), offset * np.sin(theta_abd)])

    # 2. Upper Leg
    thigh_loc = np.array([-l_up * np.sin(theta_hip), 0, -l_up * np.cos(theta_hip)])

    rot_y = thigh_loc[1] * np.cos(theta_abd) - thigh_loc[2] * np.sin(theta_abd)
    rot_z = thigh_loc[1] * np.sin(theta_abd) + thigh_loc[2] * np.cos(theta_abd)

    knee_pos = hip_pos + np.array([thigh_loc[0], rot_y, rot_z])
    return hip_pos, knee_pos


# ----------------- Offline Simulator -----------------

class GaitSimulator:
    def __init__(self):
        self.config = RobotConfig()
        self.state = State(self.config)
        self.command = Command(self.config)
        self.clock = VirtualClock()
        GP_Module.time = self.clock

        self.gait_planner = GP_Module.GaitPlanner(self.config, self.state, self.command)
        self.mock_hw = self.gait_planner.hardware_interface

        self.foot_logs = [[] for _ in range(4)]
        self.angle_logs = [[] for _ in range(4)]
        self.time_log = []

    def record_state(self):
        for i in range(4):
            self.foot_logs[i].append(self.state.foot_locations[:, i].copy())
            self.angle_logs[i].append(self.mock_hw.angles[:, i].copy())
        self.time_log.append(self.clock.time())

    def set_virtual_joystick(self, vx, vy, yaw_rate):
        self.command.L3[1] = np.clip(vx, -1.0, 1.0)
        self.command.L3[0] = np.clip(vy, -1.0, 1.0)
        self.command.R3[0] = np.clip(yaw_rate, -1.0, 1.0)

    def run(self, duration_seconds=20.0):
        dt = 1.0 / 100
        steps = int(duration_seconds / dt)
        self.gait_planner.trot_begin()

        print(f"Calculating {duration_seconds} seconds of motion...")

        for i in range(steps):
            t = self.clock.time()

            if t < 1.0:
                self.set_virtual_joystick(1.0, 0.0, 0.0)
            elif t < 4.0:
                self.set_virtual_joystick(1.0, 0.0, 0.0)
            elif t < 7.0:
                self.set_virtual_joystick(0.0, 1.0, 1.0)
            elif t < 10.0:
                self.set_virtual_joystick(0.0, -1.0, 0.0)
            else:
                self.set_virtual_joystick(0.0, 0.0, 0.0)

            self.gait_planner.trot_cycle_actuated()
            self.record_state()
            self.clock.sleep(dt)

        print("Calculation finished.")
        return [np.array(l) for l in self.foot_logs], [np.array(l) for l in self.angle_logs], dt


# ----------------- Visualization -----------------

def animate_robot_3d(foot_logs, angle_logs, dt):
    config = RobotConfig()
    L, W = config.body_length, config.body_width

    # Setup Plot
    # DPI and figsize determine the resolution. 10*100 x 8*100 = 1000x800 pixels
    dpi = 100
    fig = plt.figure(figsize=(10, 8), dpi=dpi)
    ax = fig.add_subplot(111, projection='3d')

    # ----------------- Video Writer Setup -----------------
    fps = int(1.0 / dt)

    # Get the actual size from the figure to avoid rounding errors
    width, height = fig.get_size_inches() * fig.get_dpi()
    width, height = int(width), int(height)

    video_filename = 'robot_gait_opencv.mp4'
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    video = cv2.VideoWriter(video_filename, fourcc, fps, (width, height))

    print(f"Generating video '{video_filename}' ({width}x{height}) with {fps} FPS...")

    # ----------------- Visual Elements -----------------
    ax.set_xlim(-0.3, 0.3)
    ax.set_ylim(-0.3, 0.3)
    ax.set_zlim(-0.25, 0.1)
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title("Robot Simulation")

    body_poly = None
    # Trails (The new lines)
    # alpha=0.5 makes them transparent, lw=1 makes them thin
    trails = [ax.plot([], [], [], '-', lw=1, alpha=0.5)[0] for _ in range(4)]
    abd_links = [ax.plot([], [], [], lw=4, color='k')[0] for _ in range(4)]
    upper_legs = [ax.plot([], [], [], lw=3, color='gray')[0] for _ in range(4)]
    lower_legs = [ax.plot([], [], [], lw=2, color='lightgray')[0] for _ in range(4)]
    feet_points = [ax.plot([], [], [], 'o', markersize=5)[0] for _ in range(4)]

    colors = ['g', 'orange', 'b', 'r']
    for i, p in enumerate(feet_points):
        p.set_color(colors[i])
        trails[i].set_color(colors[i])

    body_attachments = [
        np.array([L / 2, W / 2, 0]), np.array([L / 2, -W / 2, 0]),
        np.array([-L / 2, W / 2, 0]), np.array([-L / 2, -W / 2, 0])
    ]

    # ----------------- Animation Loop -----------------
    num_frames = len(foot_logs[0])

    for frame in range(num_frames):
        if frame % 50 == 0:
            print(f"Processing frame {frame}/{num_frames}")

        # --- Update Camera Angle (Rotation) ---
        # Rotates 0.5 degrees per frame. Adjust multiplier to change speed.
        ax.view_init(elev=30, azim=45 + frame * 0.5)

        # --- Same update logic ---
        att_points = []
        for i in range(4):
            p_attach = body_attachments[i]
            att_points.append(p_attach)

            angles = angle_logs[i][frame]
            foot_local = foot_logs[i][frame]

            p_hip_rel, p_knee_rel = get_joint_positions(angles, i, config)
            p_hip = p_attach + p_hip_rel
            p_knee = p_attach + p_knee_rel
            p_foot = p_attach + foot_local

            # Trails
            tail = max(0, frame - 25)
            history_local = foot_logs[i][tail:frame + 1]
            if len(history_local) > 0:
                history_global = p_attach + history_local
                trails[i].set_data(history_global[:, 0], history_global[:, 1])
                trails[i].set_3d_properties(history_global[:, 2])

            # Robot
            abd_links[i].set_data([p_attach[0], p_hip[0]], [p_attach[1], p_hip[1]])
            abd_links[i].set_3d_properties([p_attach[2], p_hip[2]])

            upper_legs[i].set_data([p_hip[0], p_knee[0]], [p_hip[1], p_knee[1]])
            upper_legs[i].set_3d_properties([p_hip[2], p_knee[2]])

            lower_legs[i].set_data([p_knee[0], p_foot[0]], [p_knee[1], p_foot[1]])
            lower_legs[i].set_3d_properties([p_knee[2], p_foot[2]])

            feet_points[i].set_data([p_foot[0]], [p_foot[1]])
            feet_points[i].set_3d_properties([p_foot[2]])

        # Body
        bx = [att_points[0][0], att_points[1][0], att_points[3][0], att_points[2][0], att_points[0][0]]
        by = [att_points[0][1], att_points[1][1], att_points[3][1], att_points[2][1], att_points[0][1]]
        bz = [att_points[0][2], att_points[1][2], att_points[3][2], att_points[2][2], att_points[0][2]]

        if body_poly: body_poly.remove()
        verts = [list(zip(bx, by, bz))]
        body_poly = art3d.Poly3DCollection(verts, alpha=0.5, facecolor='gray')
        ax.add_collection3d(body_poly)

        # ----------------- Save Frame -----------------
        fig.canvas.draw()

        # Get buffer as RGBA (4 channels)
        img = np.frombuffer(fig.canvas.buffer_rgba(), dtype=np.uint8)

        # Reshape with 4 channels instead of 3
        img = img.reshape(height, width, 4)

        # Convert RGBA -> BGR (OpenCV uses BGR and does not support Alpha in mp4v)
        img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)

        video.write(img)

    video.release()
    plt.close()
    print(f"Done! Video saved as '{video_filename}'")


if __name__ == "__main__":
    sim = GaitSimulator()
    foot_logs, angle_logs, dt = sim.run(duration_seconds=12.0)
    animate_robot_3d(foot_logs, angle_logs, dt)