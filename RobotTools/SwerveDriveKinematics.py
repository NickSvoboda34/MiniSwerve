import numpy, math

class SwerveDriveKinematics:
    #pass module positions as nested list where n is number of modules {{x1, y1}, {x2, y2}, ..., {xn, yn}}
    def __init__(self, modulePositions) -> None:
        self.modulePositions = modulePositions
        self.numModules = len(modulePositions)
        self.moduleHeadings = numpy.zeros(self.numModules)
        self.inverseKinematics = numpy.zeros((self.numModules * 2, 3))
        self.prevCoR = [0, 0]

        for i in range(self.numModules):
            self.inverseKinematics[i * 2 + 0, :] = [1, 0, -self.modulePositions[i][1]]
            self.inverseKinematics[i * 2 + 1, :] = [0, 1, +self.modulePositions[i][0]]
        
    #chassisSpeeds = [vx, vy, vtheta]
    #centerOfRotation = [x, y] 
    def toModuleStates(self, chassisSpeeds, centerOfRotation):
        moduleStates = numpy.zeros((self.numModules, 2))

        if chassisSpeeds[0] == 0 and chassisSpeeds[1] == 0 and chassisSpeeds[2] == 0:
            for i in range(self.numModules):
                moduleStates[i] = [0, self.moduleHeadings[i]]
            return moduleStates
    
        if not centerOfRotation == self.prevCoR:
            for i in range(self.numModules):
                self.inverseKinematics[i * 2 + 0, :] = [1, 0, -self.modulePositions[i][1] + centerOfRotation[1]]
                self.inverseKinematics[i * 2 + 1, :] = [0, 1, +self.modulePositions[i][0] - centerOfRotation[0]]
            self.prevCoR = centerOfRotation

        chassisSpeedsVector = numpy.zeros((3, 1))
        chassisSpeedsVector[:, 0] = [chassisSpeeds[0], chassisSpeeds[1], chassisSpeeds[2]]

        moduleStatesMatrix = self.inverseKinematics.dot(chassisSpeedsVector)

        for i in range(self.numModules):
            x = moduleStatesMatrix[i * 2]
            y = moduleStatesMatrix[i * 2 + 1]

            speed = math.hypot(x, y)
            angle = math.atan2(y, x)

            moduleStates[i] = [speed, angle]
            self.moduleHeadings[i] = angle
        
        return moduleStates