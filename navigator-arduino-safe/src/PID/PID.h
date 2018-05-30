#ifndef PID_H
#define PID_H

#include <stdint.h>

#define DIRECT  0
#define INDIRECT  1
#define US_PER_S 1000000.0

class PID {

public:

  /**
   * The constructor for the PID class object. Provide pointers to necessary variables and optionally set parameters.
   * @param input the variable containing the pid input values
   * @param output the variable which will be updated with the output after calling update
   * @param setpoint the goal setpoint to reach
   * @param kP the proportional gain value
   * @param kI the integral gain value, optional as it is defaulted to zero
   * @param kD the derivative gain value, optional as it is defaulted to zero
   * @param direction the direction to adjust for the given error, optional as it is defaulted to DIRECT
   */
  PID(double *input, double *output, double setpoint, double kP, double kI = 0, double kD = 0, int direction = DIRECT);

  /**
   * Should be called once at the very end of setup(). This method allows the PID object to update values so that the
   * first calculation is not skewed by a large elapsed time gap. This should also be called if you haven't called
   * update for a significant amount of time for whatever reason.
   */
  void init();

  /**
   * Call this method during each iteration of your loop, if the elapsed time has passed the output variable will
   * be populated with a new value and a value of true will be returned. If the required time has not elapsed to meet
   * your update time, nothing will be computed and instead the function will return a value of false.
   * @return a value of true or false depending on if the output was updated
   */
  bool update(uint32_t currentMicros);


  /* Getter and Setters */

  /**
   * Output range is defaulted to a range of 0-255 as that is the most common PWM range for Arduino compliant boards.
   * You can adjust the range to your own setting here. For example, if you have a motor that ranges from -255 to 255
   * you don't want your PID adjustment to exceed those values. In another case you may have  a range of -100 to 100
   * particularly in cases where you represent a percentage of output power. Essentially, set this to the min and max
   * output of whatever it is you are controlling with the PID adjustment value.
   * @param min the minimum output you want to provide
   * @param max the maximum output you want to provide
   */
  void setOutputRange(double min, double max);

  /**
   * Set the PID values, this is handy if you want to adjust the values dynamically. For instance, maybe you want to
   * have a higher gain for extremely sharp turns, when that turn is detected you can increase the PID valuse and
   * return them back when the turn is complete.
   * @param kP the gain value of P
   * @param kI the gain value of I
   * @param kD the gain value of D
   */
  void setPID(double kP, double kI, double kD) {
    _kP = kP;
    _kI = kI;
    _kD = kD;
  };

  /**
   * Set the direction of your output to remain in line with the error values or inverted. For example, if you are line
   * following and your robot is avoiding the line, it is possible that you are increasing motor speed when you should
   * be decreasing it. Rather than changing your code you can simply change the direction of the output adjustment.
   * @param direction the direction you want the output to correspond with the error (DIRECT or REVERSE)
   */
  void setDirection(int direction);

  /**
   * Set the sample time in milliseconds. When calling update, if this time has not been exceeded then no computation
   * will be performed and no updates will be made to your PID output value. If you want to make more adjustments
   * over time, decrease this value giving you higher resolution of adjustments.
   * @param sampleTimeUS
   */
  void setSampleTimeUS(unsigned long sampleTimeUS) { _sampleTimeUS = sampleTimeUS; }

  void setSetPoint(double setPoint);

  /**
   * Get the current value of kP
   * @return the current value of kP
   */
  double getKp() { return _kP; }

  /**
   * Get the current value of kI
   * @return the current value of kI
   */
  double getKi() { return _kI; }

  /**
   * Get the current value of kD
   * @return the current value of kD
   */
  double getKd() { return _kD; }

  /**
   * Get the currently set direction (DIRECT or REVERSE)
   * @return the current direction
   */
  int getDirection() { return _direction; };

private:

  int _direction = DIRECT;
  double *_inputPtr;
  double *_outputPtr;
  double _kP;
  double _kI;
  double _kD;

  double _setPoint = 0;
  unsigned long _lastTime = 0;
  double _lastError = 0;
  double _sumOfErrors = 0;
  unsigned long _sampleTimeUS = 100;

  double _minOutput = 0;
  double _maxOutput = 255;
};

#endif // PID_H
