from Controller.controllerInput import controller_listen
import Configuration
import time

def modify_config(config):
    data = controller_listen(listen_time=2)
    config.R3 = data[1]
    config.L3 = data[0]

if __name__ == "__main__":
    testConfig = Configuration.RobotConfig
    startTime = time.time()
    modify_config(testConfig)
    endTime = time.time()
    print(endTime-startTime)
    print(testConfig.R3, testConfig.L3)
