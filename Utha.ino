//
// Created by gberl on 1/31/2017.
//
// IMPORTANT: This library is dependent on the Encoder library from Paul S and the develop version of Dynamic Motor Driver, links below
// Dynamic Motor Driver:  https://github.com/mcc-robotics/Dynamic_Motor_Driver/tree/develop
// Encoder:               https://github.com/PaulStoffregen/Encoder
//
#include <Wire.h>
#include <DRV8835.h>
#include <Navigator.h>
#include <EncoderDriver/EncoderDriver.h>
#include <Servo.h>
#include <Encoder.h>

#define MOTOR_A1 7
#define MOTOR_A2 8
#define MOTOR_B1 9
#define MOTOR_B2 10
#define ENCODER_A1 2
#define ENCODER_A2 3
#define ENCODER_B1 5
#define ENCODER_B2 4

/*
 * Setup motors and navigation method (Encoder Navigation)
 */
 Encoder myEnc(22, 23);
DRV8835 driver(22, 21, 38, 39);
DRV8835 motorDriver(MOTOR_A1, MOTOR_A2, MOTOR_B1, MOTOR_B2);
EncoderDriver encNavigator(&motorDriver, ENCODER_A1, ENCODER_A2, ENCODER_B1, ENCODER_B2);
Servo myservo; 

void setup() {
  myservo.attach(20);  // attaches the servo on pin 9 to the servo object 

  // Set the PWM frequency, all of my motor pins happen to be on the same timer so I only set this on one pin.
  //analogWriteFrequency(MOTOR_A1, 10000);

  // Initialize the encoder navigator (ticks/rev, leftWheelDiameter, rightWheelDiameter, distanceBetween wheels)
  encNavigator.init(1800, 60.0, 60.0, 176);

  // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(1985.63f, 200);  // Test driving 78.4 inches

   // Wait until it's done but update along the way
   while(!encNavigator.achievedDistanceGoal()) {
   encNavigator.update(micros());
   driver.setMotorAPower(50);
 }

   // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
   encNavigator.turn(90, 200);   // Turn (degrees, speedInMMPerSecond)
  
   while(!encNavigator.achievedDistanceGoal()) {
   encNavigator.update(micros());
    driver.setMotorAPower(50);
   }
     // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(360, 200);  // Test driving 40 inches  //SHORT DISTANCE 484
 
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
   }
    // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.turn(135, 200);   // Turn (degrees, speedInMMPerSecond) //-45

 while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
   }
  
  // drive in reverse(distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(94.12f, -200);  // Test driving 8 inches //203.2f
 
    
    // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
   myservo.write(30);
   delay(2000);
   myservo.write(0);
   delay(2000);
   myservo.write(30);
   delay(2000);

    
 
    // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.turn(-45, 200);   // Turn (degrees, speedInMMPerSecond) //-45



  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
   }
   
  // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(1985.63f, 200);  // Test driving 78.4 inches

  

   // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
   // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.turn(135, 200);   // Turn (degrees, speedInMMPerSecond) //-45

   // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
    // drive in reverse(distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(94.12f, -200);  // Test driving 8 inches //203.2f

   // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }

  myservo.write(30);
    delay(2000);
    myservo.write(0);
    delay(2000);
    myservo.write(30);
    delay(2000);

     // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.spin(-45, 200);   // Turn (degrees, speedInMMPerSecond)
   // Wait until it's done but update along the way
  
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
  // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(884.27, 200);  // Test driving 40inches
   
    // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
    // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.turn(135, 200);   // Turn (degrees, speedInMMPerSecond) //-45
  
  // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
     // drive in reverse(distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(94.12f, -200);  // Test driving 8 inches //203.2f

  // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
  myservo.write(30);
    delay(2000);
    myservo.write(0);
    delay(2000);
    myservo.write(30);
    delay(2000);
  // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.spin(-45, 200);   // Turn (degrees, speedInMMPerSecond)
  
   // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }
 
  // Initially drive straight (distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(1985.63f, 200);  // Test driving 78.4 inches
  
  // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }

    // The only way we've reached this point is if the robot finished driving straight and stopped, so let's turn
  encNavigator.turn(135, 200);   // Turn (degrees, speedInMMPerSecond) //-45

  // Wait until it's done but update along the way
  while(!encNavigator.achievedDistanceGoal()) {
    encNavigator.update(micros());
    driver.setMotorAPower(50);
  }

     // drive in reverse(distanceInMM, speedInMMPerSecond)
  encNavigator.driveStraight(94.12f, -200);  // Test driving 8 inches //203.2f

   myservo.write(30);
    delay(2000);
    myservo.write(0);
    delay(2000);
    myservo.write(30);
    delay(2000);

  
  // ****************************************************************************************************************
  // We don't need to wait here because we won't be doing anything else, the robot will stop when it has met the goal
  // ****************************************************************************************************************
}


void loop() {
  // No need for anything here, nothing is repetitively done in this simple example
}
