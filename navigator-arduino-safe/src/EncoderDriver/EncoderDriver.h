//
// Created by Geoff Berl on 3/12/17.
//

#ifndef ENCODER_NAVIGATION_ENCODERDRIVER_H
#define ENCODER_NAVIGATION_ENCODERDRIVER_H


#include <stdint.h>
#include <Motor.h>
#include <MotorDriver.h>
#include <Encoder.h>
#include "../PID/PID.h"

#define SAMPLE_RATE_US  1000
#define MICROS_PER_SECOND 1000000.0f
#define SECONDS_PER_MINUTE  60
#define LEFT_ENCODER  0
#define RIGHT_ENCODER 1

class EncoderDriver {

public:
  EncoderDriver(MotorDriver *driver, uint8_t leftEncPin1, uint8_t leftEncPin2, uint8_t rightEncPin1, uint8_t rightEncPin2);

  void init(float encoderCPR, float leftWheelDiameterMM, float rightWheelDiameterMM, float wheelBaseMM);

  void driveStraight(float goalMM, float velocityMMPerSecond);

  /**
   * Use encoders to make a turn, by default this function will rotate by spinning the left or right wheel forward to
   * perform a clockwise or counter-clockwise turn. If the optional parameter 'reverse' is used and set to true then
   * the motor opposite motor will be spun in reverse to achieve the same directional turn.
   * @param degrees the number of degrees to turn (positive for clockwise turns, negative for counter-clockwise turns)
   * @param velocityMMPerSecond the speed at which to execute the turn
   * @param reverse whether or not to perform the turn moving forward or reverse, defaulted to forward
   */
  void turn(float degrees, float velocityMMPerSecond, bool reverse = false);

  void spin(float degrees, float velocityMMPerSecond);

  void stop();

  bool isStopped();

  void calibrate();

  void update(uint32_t currentMicros);

  void setVelocityMMPerS(float goalMMPerS);

  double getVelocity(uint8_t encoder);

  float getDistanceTraveled(uint8_t encoder);

  float getActualRPM(uint8_t encoder);

  float getGoalDistance(uint8_t encoder);

  float getGoalVelocity(uint8_t encoder);

  bool achievedDistanceGoal();

  ~EncoderDriver() {
    delete(leftEncoder);
    delete(rightEncoder);
    delete(leftPID);
    delete(rightPID);
  }

private:

  void updateVelocity(uint8_t motorIndex, uint32_t currentMicros);

  void updateGoalMM(uint8_t motorIndex, Encoder *encoder);

  void updateStats(uint8_t motorIndex, Encoder *encoder);

  MotorDriver *driver = nullptr;
  Encoder *leftEncoder = nullptr;
  Encoder *rightEncoder = nullptr;
  PID *leftPID = nullptr;
  PID *rightPID = nullptr;
  double pidError[2];
  bool encInitialized = false;

  /** Static Statistical Data */
  float encoderCPR = 0;           // The number of counts in one wheel revolution
  float wheelRadiusMM[2];      // The radius of the wheel attached to this motor
  float countsPerMM[2];          // The number of encoder counts in one mm distance
  uint8_t minRequiredPower = 0;   // The minimum power required to move the wheel (weight, lever arm, etc factor into this)
  float maximumMMPerSecond = 0;   // The maximum velocity achievable by this motor
  float circumference[2];        // The circumference(s) of the wheel (so we don't need to keep computing it)
  float wheelBaseCircumference = 0;

  /** Time variables */
  uint32_t lastUpdateMicros = 0;  // The last micros time that an update was performed
  uint32_t elapsedMicros = 0;     // The number of micros that elapsed from the last update to the current update

  /** Goals (array of size two for each motor) */
  float goalMM[2];               // The desired mm of distance to travel at the wheel
  float goalVelocity[2];         // The desired speed (independent of having a goal mm distance)
  volatile float elapsedGoalMM[2];        // How much of the goal mm distance has elapsed so far

  /** Dynamic stats (array of size two for each motor) */
  volatile int32_t lastTickCount[2];      // The last tick count from the encoder
  volatile float mmTraveled[2];           // The number of mm traveled over the last update period
  double mmPerSecond[2];          // The mm per second traveled over the last update period
  volatile float actualRPM[2];            // The actual computed RPM based on encoder ticks

  /** Calibration Values */
  int8_t minimumPower = 0;
  float minimumVelocity = 0;
  float maximumVelocity = 0;
};


#endif //ENCODER_NAVIGATION_ENCODERDRIVER_H
