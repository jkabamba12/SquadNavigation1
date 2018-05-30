
#include "PID.h"

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

PID::PID(double *input, double *output, double setpoint, double kP, double kI, double kD, int direction) {

  // Set the variables that were passed in
  _inputPtr = input;
  _outputPtr = output;
  _setPoint = setpoint;
  _kP = kP;
  _kI = kI;
  _kD = kD;
  _direction = direction;
}

bool PID::update(uint32_t currentMicros) {
  // Compute the elapsed time
  unsigned long deltaTimeUS = currentMicros - _lastTime;

  // If elapsed time equals or exceeds sample time, compute a new output
  if (deltaTimeUS >= _sampleTimeUS) {
    // Capture a snapshot of the input in case it changes in the middle of our computation
    double input = *_inputPtr;
    // Calculate our error over the elapsed time e(t)
    double error = input - _setPoint;
//    double errorOverTime = error / (deltaTimeUS / 1000.0);

    // Add the I term value which is our accumulated error over time using seconds as our dT --> e(t)*dt
    _sumOfErrors += error * (deltaTimeUS / US_PER_S);
    // Calculate our I term
    double iTerm = _kI * _sumOfErrors;
    // Constrain the I term within the limits of our output range
    if (iTerm > _maxOutput) {
      iTerm = _maxOutput;
    } else if (iTerm < _minOutput) {
      iTerm = _minOutput;
    }

    // Calculate the D term which is the rate of change in error over the elapsed time --> de(t)/dt
    double dTerm = (error - _lastError) / (deltaTimeUS / US_PER_S);

    // The final equation, this is how the PID output is calculated --> kP*e(t) + kI*sum(e(t)*dt) + kD*(de(t)/dt)
    double output = _kP * error + _kI * iTerm - _kD * dTerm;

    // Constrain the final output to a value within the limits of our output range
    if (output > _maxOutput) {
      output = _maxOutput;
    } else if (output < _minOutput) {
      output = _minOutput;
    }
    *_outputPtr = output;

    // Adjust the reused variables for the next iteration
    _lastError = error;
    _lastTime += deltaTimeUS;

    return true;
  } else {
    // Sample time hasn't been exceeded
    return false;
  }
}

void PID::setOutputRange(double min, double max) {
  // Update the internal values
  _minOutput = min;
  _maxOutput = max;

  // Update the current output
  if (*_outputPtr > _maxOutput) {
    *_outputPtr = _maxOutput;
  } else if (*_outputPtr < _minOutput) {
    *_outputPtr = _minOutput;
  }

  // Update the I term sum of errors
  if (_sumOfErrors > _maxOutput) {
    _sumOfErrors = _maxOutput;
  } else if (_sumOfErrors < _minOutput) {
    _sumOfErrors = _minOutput;
  }
}

void PID::init() {
  _sumOfErrors = *_outputPtr;
  _lastError = _setPoint - *_inputPtr;

  if (_sumOfErrors > _maxOutput) {
    _sumOfErrors = _maxOutput;
  } else if (_sumOfErrors < _minOutput) {
    _sumOfErrors = _minOutput;
  }

  // Update the last time so that our first update isn't an extremely large elapsed time
  _lastTime = micros() - _sampleTimeUS;
}

void PID::setDirection(int direction) {
  // By inverting the gain values we avoid needing to check direction in each update iteration.
  if (direction != _direction) {
    _kP = (0 - _kP);
    _kI = (0 - _kI);
    _kD = (0 - _kD);
  }
  _direction = direction;
}

void PID::setSetPoint(double setPoint) {
  _setPoint = setPoint;
}
