from Controller.controllerInput import controller_listen
import Configuration
import time




if __name__ == "__main__":
    testConfig = Configuration.RobotConfig()
    startTime = time.time()
    controller_listen(testConfig)
    endTime = time.time()
    print(endTime-startTime)
    print(testConfig.R3, testConfig.L3)
