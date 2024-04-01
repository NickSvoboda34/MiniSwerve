import serial, time

def inputModulus(input, min, max):
    mod = max - min

    numMax = int((input - min) / mod)
    input -= numMax * mod

    numMin = int((input - max) / mod)
    input -= numMin * mod

    return input

class RobotBase:
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.02)
        self.ser.reset_input_buffer()
        time.sleep(1)

class SwerveDrive:
    def __init__(self):
        pass

class SwerveModule:
    def __init__(self, leftMotor, rightMotor, turnEncoder):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.turnEncoder = turnEncoder
    
    def setMotorSpeeds(self, leftSpeed, rightSpeed):
        self.leftMotor.setSpeed(leftSpeed)
        self.rightMotor.setSpeed(rightSpeed)
        
class MotorController(RobotBase):
    def __init__(self, speedPin, leftDirPin, rightDirPin):
        self.speedPin = speedPin
        self.leftDirPin = leftDirPin
        self.rightDirPin = rightDirPin

    def setPinVal(self, pin, value):
        self.ser.write(('0' * (2 - len(str(pin))) + str(pin) + ',' + '0' * (3 - len(str(value))) + str(value)).encode())
    
    def setSpeed(self, speed):
        if speed > 0:
            self.setPinVal(self.leftDirPin, 255)
            self.setPinVal(self.rightDirPin, 0)
        else:
            self.setPinVal(self.leftDirPin, 0)
            self.setPinVal(self.rightDirPin, 255)

        speed = (int) (abs(speed * 255))
        self.setPinVal(self.speedPin, speed)      

class PIDController:
    kP, kI, kD = 0
    isContinuous = False
    setpoint = 0
    positionError = 0
    prevError = 0
    totalError = 0
    lastTime = 0
    velocityError = 0

    def __init__(self, kP, kI, kD):
        self.kP = kP
        self.kI = kI
        self.kD = kD
    
    def enableContinuous(self, lowerBound, upperBound):
        self.isContinuous = True
        self.lowerBound = lowerBound
        self.upperBound = upperBound

    def calculate(self, setpoint, measurement, time):
        self.prevError = self.positionError

        if self.isContinuous:
            errorBound = (self.upperBound - self.lowerBound) / 2
            positionError = inputModulus(setpoint - measurement, -errorBound, errorBound)
        else:
            positionError = setpoint - measurement
        
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
