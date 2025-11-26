import numpy as np
import matplotlib.pyplot as plt
from src.Configuration import RobotConfig
from src.State import State
from src.SwingPlannerV2 import SwingPlanner
from src.StancePlannerV2 import StancePlanner


def visualize_bezier_curves():
    config = RobotConfig()
    state = State(config)

    # Define to make plot somewhat realistic
    state.velocityX = 0.3
    state.velocityY = 0.0
    config.step_height = 0.04
    config.stancetime = 0.3
    config.swingtime = config.stancetime / 3

    swing_planner = SwingPlanner(state, config)
    stance_planner = StancePlanner(state, config)

    swing_t = np.linspace(0, config.swingtime, 200)
    swing_x = []
    swing_z = []

    for t in swing_t:
        x, y, z = swing_planner.MIT_cheetah_bezier_discretizer(t)
        swing_x.append(x)
        swing_z.append(z)

    stance_t = np.linspace(0, config.stancetime, 200)
    stance_x = []
    stance_z = []

    for t in stance_t:
        x, y, z = stance_planner.stance_bezier_discretizer(t)

        stance_x.append(x)
        stance_z.append(z)

    plt.figure(figsize=(12, 6))

    plt.plot(swing_x, swing_z, 'b-', linewidth=2, label='Foot Trajectory (Swing)')
    plt.plot(stance_x, stance_z, 'orange', linewidth=2, label='Foot Trajectory (Stance)')
    plt.title('Complete trajectory')
    plt.xlabel('X [m]')
    plt.ylabel('Z [m]')
    plt.grid(True)
    plt.axis('equal')
    plt.legend()

    plt.tight_layout()
    plt.show()

visualize_bezier_curves()