//
// Created by Geoff Berl on 2/12/17.
//

#ifndef NAVIGATOR_H
#define NAVIGATOR_H

#include <stdlib.h>
#include <Arduino.h>
// Include nested headers so they can be referenced from the Arduino IDE
#include "EncoderDriver/EncoderDriver.h"
#include "PID/PID.h"
//#include "SharpWallNavigator/SharpWallNavigator.h" // Removing temporarily since it isn't used yet
// END includes

class Navigator {
public:
  virtual void init(unsigned char *pins, unsigned char numSensors, unsigned int maxValue = 1023) {
    // init calibration variables and num sensors
    calibratedMinimum = 0;
    calibratedMaximum = 0;
    _numSensors = numSensors;

    // Allocate space for pins
    if (_pins == 0) {
      _pins = (unsigned char *) malloc(sizeof(unsigned char) * _numSensors);
      if (_pins == 0) {
        return;
      }
    }

    // init the pin values
    unsigned char i;
    for (i = 0; i < _numSensors; i++) {
      _pins[i] = pins[i];
      // Set the pins' mode to input
      pinMode(_pins[i], INPUT);
    }

    // Assume most AD converters will be 10 bit resolution (default parameter)
    _maxValue = maxValue;
  }

  virtual int readPosition(unsigned int *sensor_values, unsigned char inverted = 0) = 0;

  virtual void calibrate() {
    unsigned int sensor_values[_numSensors];
    unsigned int min_sensor_values[_numSensors];
    unsigned int max_sensor_values[_numSensors];
    int i;
    int j;

    // Allocate the arrays if necessary.
    if (calibratedMaximum == 0) {
      calibratedMaximum = (unsigned int *) malloc(sizeof(unsigned int) * _numSensors);

      // If the malloc failed, don't continue.
      if (calibratedMaximum == 0) {
        return;
      }

      // init the max and min calibrated values to values that
      // will cause the first reading to update them.

      for (i = 0; i < _numSensors; i++) {
        calibratedMaximum[i] = 0;
      }
    }
    if (calibratedMinimum == 0) {
      calibratedMinimum = (unsigned int *) malloc(sizeof(unsigned int) * _numSensors);

      // If the malloc failed, don't continue.
      if (calibratedMinimum == 0) {
        return;
      }

      for (i = 0; i < _numSensors; i++) {
        calibratedMinimum[i] = _maxValue;
      }
    }

    // Run the calibration with ten reads from each sensor
    for (j = 0; j < 10; j++) {
      read(sensor_values);
      for (i = 0; i < _numSensors; i++) {
        // Check if the value is greater than our current max
        if (j == 0 || max_sensor_values[i] < sensor_values[i]) {
          max_sensor_values[i] = sensor_values[i];
        }

        // Check if the value is less than our current min
        if (j == 0 || min_sensor_values[i] > sensor_values[i]) {
          min_sensor_values[i] = sensor_values[i];
        }
      }
    }

    // record the min and max calibration values
    for (i = 0; i < _numSensors; i++) {
      // Check if the max found is larger than a previously calibrated max value
      if (min_sensor_values[i] > (calibratedMaximum)[i]) {
        (calibratedMaximum)[i] = min_sensor_values[i];
      }

      // Check if the min found is less than a previously calibrated min value
      if (max_sensor_values[i] < (calibratedMinimum)[i]) {
        (calibratedMinimum)[i] = max_sensor_values[i];
      }
    }
  }

  virtual void read(unsigned int *sensor_values) {
    // Essentially this is here for different parent read modes but
    // currently there aren't any so this is just pointing to another
    // function
    readPrivate(sensor_values);
  }

  virtual void readCalibrated(unsigned int *sensor_values) {

    int i;

    // read the needed values
    read(sensor_values);

    for (i = 0; i < _numSensors; i++) {
      unsigned int denominator;

      denominator = calibratedMaximum[i] - calibratedMinimum[i];

      signed int x = 0;
      if (denominator != 0) {
        x = (((signed long) sensor_values[i]) - calibratedMinimum[i]) * 1000 / denominator;
      }
      if (x < 0) {
        x = 0;
      } else if (x > 1000) {
        x = 1000;
      }
      // Storing a signed integer is okay because we ensured it was a positive value above
      sensor_values[i] = x;
    }

  }

  virtual void setNumSamples(unsigned char numSamples) { _numSamplesPerSensor = numSamples; }

  virtual ~Navigator() {
    if (calibratedMaximum) {
      free(calibratedMaximum);
    }
    if (calibratedMinimum) {
      free(calibratedMinimum);
    }
    if (_pins) {
      free(_pins);
    }
  }

  unsigned int *calibratedMaximum;
  unsigned int *calibratedMinimum;

protected:
  /**
   * A private read method so we can handle sensors that aren't straight forward reads and require additional code
   * @param sensor_values the array of sensor values
   */
  virtual void readPrivate(unsigned int *sensor_values) = 0;

  unsigned char _numSamplesPerSensor = 3;
  unsigned char _numSensors;
  unsigned int _maxValue;
  unsigned char *_pins;
};

#endif //NAVIGATOR_H
