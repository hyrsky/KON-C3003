/*
  This file is part of the Arduino_LSM6DSOX library.
  Copyright (c) 2021 Arduino SA. All rights reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
*/

#ifndef LSM6DSOX_h
#define LSM6DSOX_h

#define LSM6DSOX_ADDRESS            0x6A

#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#define RAD_TO_DEG (180.0 / M_PI)
#define DEG_TO_RAD (M_PI / 180.0)
#define ACCL_DUE_TO_GRAVITY_MPS2 9.807 // According to wolfram alpha

struct LSM6DSOX_out {
  // gyro rates in radians per seconds.
  float p, q, r;
  // acceleration values in meters per second squared.
  float x, y, z;
};

class LSM6DSOX {
  public:
    LSM6DSOX(TwoWire& wire, int sdaPin, int sclPin, int freq, uint8_t slaveAddress);
    ~LSM6DSOX();

    int begin();
    void end();

    // Read sensor data in SI units.
    int calibrateGyro();

    // Read sensor data in SI units.
    int readSensor(LSM6DSOX_out& data);

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope

    // Temperature
    int readTemperature(int& temperature_deg);
    int readTemperatureFloat(float& temperature_deg);
    int temperatureAvailable();

  private:
    int readRegister(uint8_t address);
    int readRegisters(uint8_t address, uint8_t* data, size_t length);
    int writeRegister(uint8_t address, uint8_t value);


  private:
    TwoWire* _wire;
    uint8_t _slaveAddress;
    int _sdaPin;
    int _sclPin;
    int _freq;
};

#endif