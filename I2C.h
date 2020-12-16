#ifndef I2C_h
#define I2C_h

#include <Arduino.h>
#include <Wire.h>

extern const uint8_t IMUAddress;
extern const uint16_t I2C_TIMEOUT;

void i2cBegin();
uint8_t i2cWrite(uint8_t registerAddress, uint8_t data, bool sendStop);
uint8_t i2cWrite(uint8_t registerAddress, uint8_t *data, uint8_t length, bool sendStop);
uint8_t i2cRead(uint8_t registerAddress, uint8_t *data, uint8_t nbytes);

#endif
