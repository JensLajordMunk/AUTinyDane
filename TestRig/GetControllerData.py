from Controller.controllerInput import controller_listen, controller_stop
import Configuration
import time

if __name__ == "__main__":
    testConfig = Configuration.RobotConfig()
    startTime = time.time()
    print("Thread started.")
    control_panel = controller_listen(config=testConfig)
    endTime = time.time()
    print(endTime - startTime)

i = 0
while i <= 200:
    print(testConfig.L3, testConfig.R3)
    time.sleep(0.1)
    i += 1

controller_stop(control_panel)
