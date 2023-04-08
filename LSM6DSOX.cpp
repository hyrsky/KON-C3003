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

#include "LSM6DSOX.h"

#include <math.h>

#define LSM6DSOX_WHO_AM_I_REG       0X0F
#define LSM6DSOX_CTRL1_XL           0X10
#define LSM6DSOX_CTRL2_G            0X11

#define LSM6DSOX_STATUS_REG         0X1E

#define LSM6DSOX_CTRL6_C            0X15
#define LSM6DSOX_CTRL7_G            0X16
#define LSM6DSOX_CTRL8_XL           0X17

#define LSM6DSOX_OUT_TEMP_L         0X20
#define LSM6DSOX_OUT_TEMP_H         0X21

#define LSM6DSOX_OUTX_L_G           0X22
#define LSM6DSOX_OUTX_H_G           0X23
#define LSM6DSOX_OUTY_L_G           0X24
#define LSM6DSOX_OUTY_H_G           0X25
#define LSM6DSOX_OUTZ_L_G           0X26
#define LSM6DSOX_OUTZ_H_G           0X27

#define LSM6DSOX_OUTX_L_XL          0X28
#define LSM6DSOX_OUTX_H_XL          0X29
#define LSM6DSOX_OUTY_L_XL          0X2A
#define LSM6DSOX_OUTY_H_XL          0X2B
#define LSM6DSOX_OUTZ_L_XL          0X2C
#define LSM6DSOX_OUTZ_H_XL          0X2D

#define GYRO_TO_RAD_PER_S 1000.0 / ((float)INT16_MAX) * DEG_TO_RAD
#define ACCEL_TO_MPS2 4.0 / ((float)INT16_MAX) * ACCL_DUE_TO_GRAVITY_MPS2

LSM6DSOX::LSM6DSOX(TwoWire& wire, int sdaPin, int sclPin, int freq, uint8_t slaveAddress) :
  _wire(&wire),
  _slaveAddress(slaveAddress),
  _sdaPin(sdaPin),
  _sclPin(sclPin),
  _freq(freq)
{
}

LSM6DSOX::~LSM6DSOX()
{
}

int LSM6DSOX::begin()
{
  _wire->begin(_sdaPin, _sclPin, _freq);

  if (!(readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x6C || readRegister(LSM6DSOX_WHO_AM_I_REG) == 0x69)) {
    end();
    return 0;
  }

  // Set the Accelerometer control register to work at 104 Hz, 4 g,and in bypass mode and enable ODR/4
  // low pass filter (check figure9 of LSM6DSOX's datasheet)
  // Datasheet: page 56 
  writeRegister(LSM6DSOX_CTRL1_XL, 0x4A);

  // Set the ODR config register to ODR/4
  writeRegister(LSM6DSOX_CTRL8_XL, 0x09);

  // Set the gyroscope control register to work at 104 Hz, 1000 dps and in bypass mode
  // Datasheet: page 57
  writeRegister(LSM6DSOX_CTRL2_G, 0x48);

  // set gyroscope power mode to high performance and bandwidth to 16 MHz
  writeRegister(LSM6DSOX_CTRL7_G, 0x00);

  return 1;
}

void LSM6DSOX::end()
{
  writeRegister(LSM6DSOX_CTRL2_G, 0x00);
  writeRegister(LSM6DSOX_CTRL1_XL, 0x00);
  _wire->end();
}

int LSM6DSOX::readSensor(LSM6DSOX_out& data)
{
  int16_t buffer[6];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)buffer, sizeof(buffer))) {
    memset(&data, 0, sizeof(data));
    return 0;
  }

  // Normalize gyro
  data.p = buffer[0] * GYRO_TO_RAD_PER_S;
  data.q = buffer[1] * GYRO_TO_RAD_PER_S;
  data.r = buffer[2] * GYRO_TO_RAD_PER_S;

  // Normalize accelerometer
  data.x = buffer[3] * ACCEL_TO_MPS2;
  data.y = buffer[4] * ACCEL_TO_MPS2;
  data.z = buffer[5] * ACCEL_TO_MPS2;

  return 1;
}

int LSM6DSOX::readAcceleration(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_XL, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 4.0 / 32768.0 * 9.81;
  y = data[1] * 4.0 / 32768.0 * 9.81;
  z = data[2] * 4.0 / 32768.0 * 9.81;

  return 1;
}

int LSM6DSOX::accelerationAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x01) {
    return 1;
  }

  return 0;
}

float LSM6DSOX::accelerationSampleRate()
{
  return 104.0F;
}

int LSM6DSOX::readGyroscope(float& x, float& y, float& z)
{
  int16_t data[3];

  if (!readRegisters(LSM6DSOX_OUTX_L_G, (uint8_t*)data, sizeof(data))) {
    x = NAN;
    y = NAN;
    z = NAN;

    return 0;
  }

  x = data[0] * 2000.0 / 32768.0 * (M_PI / 180.0);
  y = data[1] * 2000.0 / 32768.0 * (M_PI / 180.0);
  z = data[2] * 2000.0 / 32768.0 * (M_PI / 180.0);

  return 1;
}

int LSM6DSOX::gyroscopeAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x02) {
    return 1;
  }

  return 0;
}

int LSM6DSOX::readTemperature(int& temperature_deg)
{
  float temperature_float = 0;
  readTemperatureFloat(temperature_float);

  temperature_deg = static_cast<int>(temperature_float);

  return 1;
}

int LSM6DSOX::readTemperatureFloat(float& temperature_deg)
{
  /* Read the raw temperature from the sensor. */
  int16_t temperature_raw = 0;

  if (readRegisters(LSM6DSOX_OUT_TEMP_L, reinterpret_cast<uint8_t*>(&temperature_raw), sizeof(temperature_raw)) != 1) {
    return 0;
  }

  /* Convert to °C. */
  static int const TEMPERATURE_LSB_per_DEG = 256;
  static int const TEMPERATURE_OFFSET_DEG = 25;

  temperature_deg = (static_cast<float>(temperature_raw) / TEMPERATURE_LSB_per_DEG) + TEMPERATURE_OFFSET_DEG;

  return 1;
}

int LSM6DSOX::temperatureAvailable()
{
  if (readRegister(LSM6DSOX_STATUS_REG) & 0x04) {
    return 1;
  }

  return 0;
}

float LSM6DSOX::gyroscopeSampleRate()
{
  return 104.0F;
}

int LSM6DSOX::readRegister(uint8_t address)
{
  uint8_t value;
  
  if (readRegisters(address, &value, sizeof(value)) != 1) {
    return -1;
  }
  
  return value;
}

int LSM6DSOX::readRegisters(uint8_t address, uint8_t* data, size_t length)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);

  if (_wire->endTransmission(false) != 0) {
    return -1;
  }

  if (_wire->requestFrom(_slaveAddress, length) != length) {
    return 0;
  }

  for (size_t i = 0; i < length; i++) {
    *data++ = _wire->read();
  }

  return 1;
}

int LSM6DSOX::writeRegister(uint8_t address, uint8_t value)
{
  _wire->beginTransmission(_slaveAddress);
  _wire->write(address);
  _wire->write(value);
  if (_wire->endTransmission() != 0) {
    return 0;
  }

  return 1;
}
