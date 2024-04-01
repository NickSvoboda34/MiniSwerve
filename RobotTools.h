#ifndef PID_H
#define PID_H

class PIDController {
  public:
    double _kP = 0;
    double _kI = 0;
    double _kD = 0;
    double upperBound;
    double lowerBound;
    bool isContinuous = false;


    /// @brief Creates a PIDControler with (0, 0, 0).
    PIDController() {
      _kP = 0;
      _kI = 0;
      _kD = 0;
    }

    /// @brief Creates a new PIDController with contants kP, kI, kD.
    /// @param kP The proportional constant.
    /// @param kI The integral constant.
    /// @param kD The derivative constant.
    PIDController(double kP, double kI, double kD) {
      _kP = kP;
      _kI = kI;
      _kD = kD;
    }

    void enableContinuous(double lowerBound, double upperBound) {
      isContinuous = true;
      this->lowerBound = lowerBound;
      this->upperBound = upperBound;
    }

    double inputModulus(double input, double min, double max) {
      double mod = max - min;

      int numMax = (int) ((input - min) / mod);
      input -= numMax * mod;

      int numMin = (int) ((input - max) / mod);
      input -= numMin * mod;

      return input;
    }

    /// @brief Calculates new output based on setpoint and measuremnt.
    /// @param setpoint The goal position.
    /// @param measuerment The current position.
    /// @param time The time since the last update.
    /// @return The calculated output.
    double calculate(double setpoint, double measuerment, double time) {

      prevError = positionError;

      if(isContinuous) {
        double errorBound = (upperBound - lowerBound) / 2;
        positionError = inputModulus(setpoint - measuerment, -errorBound, errorBound);
      } else {
        positionError = setpoint - measuerment;
      }
      
      if(lastTime <= 0) { lastTime = time; }
      double period = time - lastTime;      
      if(period <= 0) { return 0; }

      totalError += positionError * period;
      velocityError = (positionError - prevError) / period;

      double output = (_kP * positionError) + (period * _kI * totalError) + velocityError * _kD;

      double lastTime = time;

      return output;
    }

    /// @brief Resets the controller
    void reset() {
      setpoint = 0;
      positionError = 0;
      prevError = 0;
      totalError = 0;
      lastTime = 0;
      velocityError = 0;
    }

  private:
    double setpoint = 0;
    double positionError = 0;
    double prevError = 0;
    double totalError = 0;
    double lastTime = 0;
    double velocityError = 0;
};

class trapezoidProfile {
  public:
    trapezoidProfile(double maxVelocity, double maxAcceleration) {
      maxVel = maxVelocity;
      maxAccel = maxAcceleration;
    }
  
  private:
    double maxVel;
    double maxAccel;
};

/// @brief CURRENTLY NOT WORKING!
class profiledPIDController {
  public:
    profiledPIDController(double kP, double kI, double kD, double maxVelocity, double maxAcceleration) {
      pid = PIDController(kP, kI, kD);
      maxAccel = maxAcceleration;
      maxVel = maxVelocity;
    }

  private:
    PIDController pid;
    double maxAccel;
    double maxVel;
};

#endif