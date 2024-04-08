class SwerveDrive:
    def __init__(self, modules, gyro):
        self.modules = modules
    
    def setModuleStatesFromSpeeds(self, Vx, Vy, Vtheta):
        #convert speeds to states from kinematics here
        pass
    
    def setModuleStates(self, states):
        for i in range(self.modules):
            self.modules[i].setModuleState(states[i * 2][0], states[i * 2 + 1][1])