from GaitPlanner import GaitPlanner
from Configuration import RobotConfig
from State import State
from Command import Command, Modes
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
        match command.mode:
            case Modes.TROT:
                gait_planner.trot_begin_actuated()
                while True:
                    loop_time = time.time()
                    gait_planner.trot_cycle_actuated()
                    if state.velocityY + state.velocityX == 0 and command.mode != Modes.TROT:
                        break
                    time.sleep((1.0 / config.frequency) - (time.time() - loop_time))
            case Modes.ROTATE:
                while True:
                    stand_rotation_planner.run_rotation()
                    if command.mode != Modes.ROTATE:
                        break

            case Modes.TRANSLATE:
                return
                # TODO: Fix standrotation and make compatible with controller.
                # THIS SHOULD BE TRANSLATION

main()


