//
// Created by gberl on 1/31/2017.
//
// IMPORTANT: This library is dependent on the Encoder library from Paul S and the develop version of Dynamic Motor Driver, links below
// Dynamic Motor Driver:  https://github.com/mcc-robotics/Dynamic_Motor_Driver/tree/develop
// Encoder:               https://github.com/PaulStoffregen/Encoder
//

#include <DRV8835.h>
#include <Navigator.h>

#define MOTOR_A1 20
#define MOTOR_A2 21
#define MOTOR_B1 23
#define MOTOR_B2 22
#define ENCODER_A1 3
#define ENCODER_A2 4
#define ENCODER_B1 2
#define ENCODER_B2 1

/*
 * Setup motors and navigation method (Encoder Navigation)
 */
DRV8835 motorDriver(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
EncoderDriver encNavigator(&motorDriver, ENCODER_A1, ENCODER_A2, ENCODER_B1, ENCODER_B2);


void setup() {

  // Set the PWM frequency, all of my motor pins happen to be on the same timer so I only set this on one pin.
  analogWriteFrequency(MOTOR_A1, 10000);

  // Initialize the encoder navigator (ticks/rev, leftWheelDiameter, rightWheelDiameter, distanceBetween wheels)
  encNavigator.init(1212.6315789, 40.0, 40.0, 115.4465278);

  // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(914.4f, 200);  // Test driving 36 inches

  // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
  }

  // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.spin(90, 200);   // Turn (degrees, speedInMMPerSecond)

  // ****************************************************************************************************************
  // We don't need to wait here because we won't be doing anything else, the robot will stop when it has met the goal
  // ****************************************************************************************************************
}


void loop() {
  // No need for anything here, nothing is repetitively done in this simple example
}