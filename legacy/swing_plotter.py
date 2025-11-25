import matplotlib
matplotlib.use('TkAgg')  # Set backend before importing pyplot
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from src.Configuration import RobotConfig
from SwingPlanner import SwingPlanner


def test_swing_planner_animated():
    """
    Test that animates swing trajectory points one at a time in the order they occur.
    """
    # Initialize with RobotConfig
    config = RobotConfig()
    planner = SwingPlanner()
    planner.config = config

    # Collect all trajectory points
    print("Generating trajectory points...")
    print(f"Config: velocity={config.velocity}, stancetime={config.stancetime}")
    print(f"Config: step_height={config.step_height}, arcR={config.arcR}")
    print(f"Config: swingtime={config.swingtime}, frequency={config.frequency}")

    # Suppress sqrt warnings for out-of-domain values
    all_x, all_y = planner.discretizer()
    print(f"SUMMARY:")
    print(f"Total trajectory points: {len(all_x)}")

    # Create figure and axis
    fig, ax = plt.subplots(figsize=(12, 8))

    # Set up the plot limits based on data
    x_margin = (max(all_x) - min(all_x)) * 0.1
    y_margin = (max(all_y) - min(all_y)) * 0.1
    print(min(all_x))
    print(max(all_x))
    ax.set_xlim(min(all_x) - x_margin, max(all_x) + x_margin)
    ax.set_ylim(min(all_y) - y_margin, max(all_y) + y_margin)

    ax.set_xlabel('X Position (m)', fontsize=12)
    ax.set_ylabel('Y Position (m)', fontsize=12)
    ax.set_title(f'Swing Trajectory Animation ({len(all_x)} points)', fontsize=14)
    ax.grid(True, alpha=0.3)
    ax.set_aspect('equal')

    # Initialize plot elements
    line, = ax.plot([], [], 'b-', linewidth=2, alpha=0.6, label='Trajectory')
    points = ax.scatter([], [], c=[], cmap='viridis', s=20, alpha=0.7)
    current_point, = ax.plot([], [], 'ro', markersize=10, label='Current Point', zorder=5)
    start_point, = ax.plot([all_x[0]], [all_y[0]], 'go', markersize=15, label='Start', zorder=5)

    ax.legend(loc='best')

    # Animation initialization function
    def init():
        line.set_data([], [])
        points.set_offsets(np.empty((0, 2)))
        current_point.set_data([], [])
        return line, points, current_point, start_point

    # Animation update function
    def update(frame):
        # Update line (continuous trajectory)
        line.set_data(all_x[:frame + 1], all_y[:frame + 1])

        # Update scattered points
        xy = np.c_[all_x[:frame + 1], all_y[:frame + 1]]
        points.set_offsets(xy)
        points.set_array(np.arange(frame + 1))

        # Update current point marker
        current_point.set_data([all_x[frame]], [all_y[frame]])

        # Update title with progress
        ax.set_title(f'Swing Trajectory Animation - Point {frame + 1}/{len(all_x)}', fontsize=14)

        return line, points, current_point, start_point

    # Create animation
    # interval is in milliseconds - adjust for desired speed
    # Lower interval = faster animation
    anim = FuncAnimation(fig, update, init_func=init, frames=len(all_x),
                         interval=20, blit=False, repeat=True)

    plt.tight_layout()
    plt.show()

    return all_x, all_y, anim


# Run the animated test
test_swing_planner_animated()