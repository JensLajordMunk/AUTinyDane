from PS4Controller.controllerInput import controller_listen, controller_stop
import Command
import Configuration
import time

if __name__ == "__main__":
    testConfig = Configuration.RobotConfig()
    testCommand = Command.Command(testConfig)

    control_panel = controller_listen(command_config=testCommand)

    i = 0
    while i <= 200:
        print(testCommand.L3, testCommand.R3)
        time.sleep(0.1)
        i += 1

    controller_stop(testCommand, control_panel)
