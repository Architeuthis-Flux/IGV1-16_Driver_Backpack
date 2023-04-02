/*
 * I^2C implementation
 *
 * Copyright (c) 2022 Gregory Tomasch and Simon D. Levy.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. The names of Gregory Tomasch and his successors
 *     may not be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 */

#include <Arduino.h>
#include <Wire.h>

#include "i2c_api.h"

void i2cBegin(void)
{
    Wire.begin();
}

void i2cSetClock(uint32_t rate)
{
    Wire.setClock(rate);
}

uint8_t i2cReadByte(uint8_t address, uint8_t subAddress)
{
    Wire.beginTransmission(address);      
    Wire.write(subAddress);       
    Wire.endTransmission(false);     
    Wire.requestFrom(address, (size_t) 1);     
    return Wire.read();    
}

void i2cReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{  
    Wire.beginTransmission(address); 
    Wire.write(subAddress);
    Wire.endTransmission(false);     
    Wire.requestFrom(address, (size_t) count); 

    uint8_t i = 0;
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }       
}

void i2cWriteByte(uint8_t devAddr, uint8_t regAddr, uint8_t data)
{
    i2cWriteBytes(devAddr, regAddr, 1, &data);
}

void i2cWriteBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data)
{
    Wire.beginTransmission(devAddr);
    Wire.write((uint8_t) regAddr);

    for (uint8_t i=0; i < length; i++) {
        Wire.write((uint8_t)data[i]);
    }

    Wire.endTransmission();
}
