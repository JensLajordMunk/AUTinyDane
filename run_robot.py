from src.GaitPlannerV2 import GaitPlanner
from src.Configuration import RobotConfig
from src.State import State
from src.Command import Command

def main():
    config = RobotConfig()
    state = State(config)
    command = Command(config)
    gait_planner = GaitPlanner(config,state,command)

    while True:
        gait_planner.trot_begin()
        while True:
            gait_planner.trot_cycle_actuated()


main()


