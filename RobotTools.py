import serial, time as t

def inputModulus(input, min, max):
    mod = max - min

    numMax = int((input - min) / mod)
    input -= numMax * mod

    numMin = int((input - max) / mod)
    input -= numMin * mod

    return input

class RobotBase:
    #Port usually /dev/ttyACM0
    def __init__(self, serialPort):
        self.ser = serial.Serial(serialPort, 115200, timeout=0.02)
        self.ser.reset_input_buffer()
        time.sleep(1)


class SwerveDrive:
    def __init__(self, modules, gyro):
        self.modules = modules
    
    def setModuleStatesFromSpeeds(self, Vx, Vy, Vtheta):
        #convert speeds to states from kinematics here
        pass
    
    def setModuleStates(self, states):
        for i in range(self.modules):
            self.modules[i].setModuleState(states[i].velocity, states[i].angle)

    
class SwerveModule:
    def __init__(self, leftMotor, rightMotor, turnEncoder, anglePID, drivePID):
        self.leftMotor = leftMotor
        self.rightMotor = rightMotor
        self.turnEncoder = turnEncoder
        self.anglePID = anglePID
        self.drivePID = drivePID
    
    def setMotorSpeeds(self, leftSpeed, rightSpeed):
        self.leftMotor.setSpeed(leftSpeed)
        self.rightMotor.setSpeed(rightSpeed)

    def setModuleState(self, driveVelocity, angle):
        
        return
        
class MotorController(RobotBase):
    def __init__(self, speedPin, leftDirPin, rightDirPin, encoder):
        self.speedPin = speedPin
        self.leftDirPin = leftDirPin
        self.rightDirPin = rightDirPin
        self.encoder = encoder

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

    def getMotorPosition(self):
        return self.encoder.getPosition()

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

    def calculate(self, setpoint, measurement):
        self.prevError = self.positionError

        if self.isContinuous:
            errorBound = (self.upperBound - self.lowerBound) / 2
            positionError = inputModulus(setpoint - measurement, -errorBound, errorBound)
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
