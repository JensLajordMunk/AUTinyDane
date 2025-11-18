from GaitPlanner import GaitPlanner
from Configuration import RobotConfig
from State import State
from Command import Command
from StandRotationPlanner import StandRotaionPlanner
from PS4Controller.controllerInput import controller_listen, controller_stop
import time

def main():
    config = RobotConfig()
    state = State(config)
    command = Command(config)
    gait_planner = GaitPlanner(config,state)
    control_panel = controller_listen(config)
    stand_rotation_planner = StandRotaionPlanner(config, state, command)

    while True:
        # MAIN LOOP
        if command.mode == "Trot":
            gait_planner.trot_begin_actuated()
            while True:
                loop_time = time.time()
                gait_planner.trot_cycle_actuated()
                if state.velocityY + state.velocityX == 0 and mode != trot:
                    break
                time.sleep((1.0 / config.frequency) - (time.time() - loop_time))

        if mode == stand_rotation:
            while True:
                stand_rotation_planner.run_rotation()
                if mode != stand_rotation:
                    break

        if mode == stand_rotation:
            # TODO: Fix standrotation and make compatible with controller


main()


