//
// Created by Geoff Berl on 3/12/17.
//

#include "EncoderDriver.h"

EncoderDriver::EncoderDriver(MotorDriver *driver, uint8_t leftEncPin1, uint8_t leftEncPin2, uint8_t rightEncPin1,
                             uint8_t rightEncPin2) {
  EncoderDriver::driver = driver;
  EncoderDriver::leftEncoder = new Encoder(leftEncPin1, leftEncPin2);
  EncoderDriver::rightEncoder = new Encoder(rightEncPin1, rightEncPin2);
  EncoderDriver::leftPID = new PID(&mmPerSecond[LEFT_ENCODER], &pidError[LEFT_ENCODER], goalVelocity[LEFT_ENCODER], 0.035, 0.00, 0.0, INDIRECT);
  EncoderDriver::rightPID = new PID(&mmPerSecond[RIGHT_ENCODER], &pidError[RIGHT_ENCODER], goalVelocity[RIGHT_ENCODER], 0.035, 0.00, 0.0, INDIRECT);
}

void EncoderDriver::init(float encoderCPR, float leftWheelDiameterMM, float rightWheelDiameterMM, float wheelBaseMM) {
  // Create a new encoder object
  EncoderDriver::encoderCPR = encoderCPR;
  EncoderDriver::wheelRadiusMM[LEFT_ENCODER] = leftWheelDiameterMM / 2;
  EncoderDriver::wheelRadiusMM[RIGHT_ENCODER] = rightWheelDiameterMM / 2;
  // Counts per millimeter is equal to CPR / Circumference
  EncoderDriver::circumference[LEFT_ENCODER] = (float) (2 * PI * wheelRadiusMM[LEFT_ENCODER]);
  EncoderDriver::countsPerMM[LEFT_ENCODER] = encoderCPR / circumference[LEFT_ENCODER];
  EncoderDriver::circumference[RIGHT_ENCODER] = (float) (2 * PI * wheelRadiusMM[RIGHT_ENCODER]);
  EncoderDriver::countsPerMM[RIGHT_ENCODER] = encoderCPR / circumference[RIGHT_ENCODER];
  // Wheelbase circumference
  wheelBaseCircumference = (float) (2 * PI * (wheelBaseMM / 2));

  // Initialize PID objects
  leftPID->init();
  leftPID->setOutputRange(-100, 100);
  leftPID->setSampleTimeUS(10);
  rightPID->init();
  rightPID->setOutputRange(-100, 100);
  rightPID->setSampleTimeUS(10);

  // Let the library know we initialized
  encInitialized = true;
}

void EncoderDriver::driveStraight(float goalMM, float velocityMMPerSecond) {
  // Set the goal mm distance
  EncoderDriver::goalMM[LEFT_ENCODER] = goalMM;
  EncoderDriver::goalMM[RIGHT_ENCODER] = goalMM;
  // Set the velocity goal
  setVelocityMMPerS(velocityMMPerSecond);
}

void EncoderDriver::spin(float degrees, float velocityMMPerSecond) {
  // Set the goal distances, degrees being +/- will adjust these directly
  goalMM[LEFT_ENCODER] = wheelBaseCircumference * (degrees / 360);
  goalMM[1] = wheelBaseCircumference * -1 * (degrees / 360);

  // If the angle is positive, turn right otherwise turn left
  if (degrees > 0) {
    // Set the goal speed
    goalVelocity[LEFT_ENCODER] = velocityMMPerSecond;
    goalVelocity[RIGHT_ENCODER] = velocityMMPerSecond * -1;
  } else {
    // Set the goal speed
    goalVelocity[LEFT_ENCODER] = velocityMMPerSecond * -1;
    goalVelocity[RIGHT_ENCODER] = velocityMMPerSecond;
  }

  // Update the PID set points
  leftPID->setSetPoint(goalVelocity[LEFT_ENCODER]);
  rightPID->setSetPoint(goalVelocity[RIGHT_ENCODER]);
}

void EncoderDriver::turn(float degrees, float velocityMMPerSecond, bool reverse) {
  // TODO: Possibly could clean this code up a bit
  if (degrees > 0 && !reverse) {
    // Clockwise forward (left motor positive)
    goalMM[LEFT_ENCODER] = (wheelBaseCircumference * 2) * (degrees / 360);  // Degrees already positive
    goalVelocity[LEFT_ENCODER] = velocityMMPerSecond;
  } else if (degrees < 0 && !reverse) {
    // Counter-clockwise forward (right motor positive)
    goalMM[RIGHT_ENCODER] = (wheelBaseCircumference * 2) * -1 * (degrees / 360);  // Reverse the degrees for forward
    goalVelocity[RIGHT_ENCODER] = velocityMMPerSecond;
  } else if (degrees > 0 && reverse) {
    // Clockwise reverse (right motor reverse)
    goalMM[RIGHT_ENCODER] = (wheelBaseCircumference * 2) * -1 * (degrees / 360); // Reverse the degrees for reverse
    goalVelocity[RIGHT_ENCODER] = velocityMMPerSecond * -1; // Reverse the velocity for reverse
  } else if (degrees < 0 && reverse) {
    // Counter-Clockwise reverse (left motor reverse)
    goalMM[LEFT_ENCODER] = (wheelBaseCircumference * 2) * (degrees / 360);  // Degrees already negative
    goalVelocity[LEFT_ENCODER] = velocityMMPerSecond;
  }

  // Update the PID set points
  leftPID->setSetPoint(goalVelocity[LEFT_ENCODER]);
  rightPID->setSetPoint(goalVelocity[RIGHT_ENCODER]);
}

void EncoderDriver::calibrate() {

  /** Determine the minimum speed for both motors to move together */
  int8_t power = 0;
  do {
    power++;
    driver->setAllBrakePower(power);

    // Wait for some updates (the initial update might be a false positive due to backlash allowing encoder movement)
    for (int i = 0; i < 10; i++) {
      delayMicroseconds(SAMPLE_RATE_US + 2);
      update(micros());
    }

    // Check the actual movement of the motors after some updates
  } while(actualRPM[LEFT_ENCODER] == 0 || actualRPM[RIGHT_ENCODER] == 0);

  // Store minimum calibration values and cut the motors
  minimumPower = power;
  minimumVelocity = (float) max(mmPerSecond[0], mmPerSecond[1]);
  stop();

  Serial.print("minimum power is ");
  Serial.println(minimumPower);
  Serial.print("minimum velocity is ");
  Serial.println(minimumVelocity);

  /** Determine the maximum speed that both motors can drive together */
  // Power motors on at full speed
  driver->setAllBrakePower(100);

  // Loop until numerous readings are the same and assume we've reached terminal velocity
  float lastLeftRPM = 0;
  float lastRightRPM = 0;
  uint8_t leftCount = 0;
  uint8_t rightCount = 0;
  do {
    delayMicroseconds(SAMPLE_RATE_US + 2);
    update(micros());
//    Serial.print("RPM L|R\t");
//    Serial.print(leftMotor->getActualRPM());
//    Serial.print("|");
//    Serial.println(rightMotor->getActualRPM());

    // If the RPM is within 10% of the last, increase the counter. Otherwise, reset the values.
    if (abs(1 - actualRPM[0] / lastLeftRPM) < 0.1) {
      leftCount++;
    } else {
      lastLeftRPM = actualRPM[0];
      leftCount = 0;
    }
    if (abs(1 - actualRPM[1] / lastRightRPM) < 0.1) {
      rightCount++;
    } else {
      lastRightRPM = actualRPM[1];
      rightCount = 0;
    }
  } while (leftCount < 14 && rightCount < 14);

  // If we've broken out of the loop, record the smaller of the last speeds (the fastest speed of the slowest motor)
  maximumVelocity = (float) min(mmPerSecond[1], mmPerSecond[0]);
  stop();
  Serial.print("maximum velocity is ");
  Serial.println(maximumVelocity);
}

void EncoderDriver::stop() {
  driver->brakeAll();
}

bool EncoderDriver::isStopped() {
  // FIXME: This can occasionally return zero when it should actually be moving, seems odd they both result in zero at the same time
  return actualRPM[LEFT_ENCODER] == 0 && actualRPM[RIGHT_ENCODER] == 0;
}

void EncoderDriver::update(uint32_t currentMicros) {
  // TODO: Fix this so we don't explicitly call the functions with their parameters
  // Record the elapsed micros and see if we should really update
  elapsedMicros = (currentMicros - lastUpdateMicros);
  if (elapsedMicros < SAMPLE_RATE_US || !encInitialized) {
    return;
  }
  lastUpdateMicros = currentMicros;

  updateStats(LEFT_ENCODER, leftEncoder);
  updateStats(RIGHT_ENCODER, rightEncoder);

  /** Update based Goals */
  updateVelocity(LEFT_ENCODER, currentMicros);
  updateVelocity(RIGHT_ENCODER, currentMicros);
  updateGoalMM(LEFT_ENCODER, leftEncoder);
  updateGoalMM(RIGHT_ENCODER, rightEncoder);
}

void EncoderDriver::updateVelocity(uint8_t motorIndex, uint32_t currentMicros) {
  // TODO: Update to remove explicit motor calls
  // If the goal velocity is zero, skip this update
  if (goalVelocity[motorIndex] == 0) {
    return;
  }

  // Compute PID and adjust the motor accordingly.
  if (motorIndex == 0) {
    if (leftPID->update(currentMicros)) {
      int8_t newPower = (int8_t) constrain(driver->getMotorA()->getCurrentPower() + pidError[motorIndex], -100, 100);
      driver->setMotorABrakePower(newPower);
    }
  } else {
    if (rightPID->update(currentMicros)) {
      int8_t newPower = (int8_t) constrain(driver->getMotorB()->getCurrentPower() + pidError[motorIndex], -100, 100);
      driver->setMotorBBrakePower(newPower);
    }
  }

}

void EncoderDriver::updateGoalMM(uint8_t motorIndex, Encoder *encoder) {
  // If there is no goal, just skip this
  if (goalMM[motorIndex] == 0) {
    return;
  }

  elapsedGoalMM[motorIndex] += (lastTickCount[motorIndex] / countsPerMM[motorIndex]);

  // If we have reached the goal, reset and brake the motor
  if (abs(elapsedGoalMM[motorIndex]) >= abs(goalMM[motorIndex])) {
    elapsedGoalMM[motorIndex] = 0;
    goalMM[motorIndex] = 0;
    if (motorIndex == LEFT_ENCODER) {
      driver->motorABrake();
    } else {
      driver->motorBBrake();
    }
    goalVelocity[motorIndex] = 0;
    leftPID->setSetPoint(motorIndex);
  }
}

void EncoderDriver::updateStats(uint8_t motorIndex, Encoder *encoder) {
  // Calculate the distance traveled using encoder values
  lastTickCount[motorIndex] = encoder->read();
  encoder->write(0);
  mmTraveled[motorIndex] += (lastTickCount[motorIndex] / countsPerMM[motorIndex]);

  // Calculate the velocity in mm/s
  mmPerSecond[motorIndex] = (lastTickCount[motorIndex] / countsPerMM[motorIndex]) / (elapsedMicros / MICROS_PER_SECOND);

  // Calculate the actual RPM
  actualRPM[motorIndex] = (float) ((mmPerSecond[motorIndex] * SECONDS_PER_MINUTE) / circumference[motorIndex]);
}

void EncoderDriver::setVelocityMMPerS(float goalMMPerS) {
  // Set both motors to the goal velocity
  goalVelocity[LEFT_ENCODER] = goalMMPerS;
  leftPID->setSetPoint(goalMMPerS);
  goalVelocity[RIGHT_ENCODER] = goalMMPerS;
  rightPID->setSetPoint(goalMMPerS);

  // If the velocity goal is zero, turn the motors off
  if (goalMMPerS == 0) {
    driver->brakeAll();
  }
}

double EncoderDriver::getVelocity(uint8_t encoder) {
  return mmPerSecond[encoder];
}

float EncoderDriver::getDistanceTraveled(uint8_t encoder) {
  float returnValue = mmTraveled[encoder];

  // reset the value for distance traveled each time it is read
  mmTraveled[encoder] = 0;

  return returnValue;
}

float EncoderDriver::getActualRPM(uint8_t encoder) {
  if (encoder == LEFT_ENCODER) {
    return actualRPM[LEFT_ENCODER];
  } else {
    return actualRPM[RIGHT_ENCODER];
  }
}

float EncoderDriver::getGoalDistance(uint8_t encoder) {
  return goalMM[encoder];
}

float EncoderDriver::getGoalVelocity(uint8_t encoder) {
  return goalVelocity[encoder];
}

bool EncoderDriver::achievedDistanceGoal() {
  // If both goal distances are zero, then we're not working on a goal
  return goalMM[LEFT_ENCODER] == 0 && goalMM[RIGHT_ENCODER] == 0;
}

