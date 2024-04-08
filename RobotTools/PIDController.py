import RobotMath, time as t

class PIDController:
    kP, kI, kD = 0
    isContinuous = False
    setpoint = 0
    positionError = 0
    prevError = 0
    totalError = 0
    lastTime = 0
    velocityError = 0

    def __init__(self, kP:float, kI:float, kD:float):
        self.kP = kP
        self.kI = kI
        self.kD = kD
    
    def enableContinuous(self, lowerBound:float, upperBound:float):
        self.isContinuous = True
        self.lowerBound = lowerBound
        self.upperBound = upperBound

    def calculate(self, setpoint:float, measurement:float) -> float:
        self.prevError = self.positionError

        if self.isContinuous:
            errorBound = (self.upperBound - self.lowerBound) / 2
            positionError = RobotMath.inputModulus(setpoint - measurement, -errorBound, errorBound)
        else:
            positionError = setpoint - measurement
        
        time = t.time()
        if self.lastTime <= 0: self.lastTime = time
        period = time - self.lastTime
        if period <= 0: return 0

        self.totalError += positionError * period
        self.velocityError = (positionError - self.prevError) / period
        output = (self.kP * positionError) + (period * self.kI * self.totalError) + self.velocityError * self.kD
        self.lastTime = time

        return output
    
    def reset(self):
        self.setpoint = 0
        self.positionError = 0
        self.prevError = 0
        self.totalError = 0
        self.lastTime = 0
        self.velocityError = 0
