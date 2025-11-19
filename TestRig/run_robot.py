from GaitPlanner import GaitPlanner
from Configuration import RobotConfig
from State import State
from Command import Command, Modes
from StandRotationPlanner import StandRotationPlanner
from StandTranslationPlanner import StandTranslationPlanner
from PS4Controller.controllerInput import controller_listen, controller_stop
import time

def main():
    config = RobotConfig()
    state = State(config)
    command = Command(config)
    gait_planner = GaitPlanner(config,state,command)
    control_panel = controller_listen(command)
    stand_rotation_planner = StandRotationPlanner(config, state, command)
    stand_translation_planner = StandTranslationPlanner(config, state, command)

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
                while True:
                    stand_translation_planner.run_translation()
                    if command.mode != Modes.TRANSLATE:
                        break

main()


