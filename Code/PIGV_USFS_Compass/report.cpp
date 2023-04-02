/*
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

#include "USFSMAX.h"
#include "report.h"

static void reportAngle(float angle, const char * label)
{
    Serial.print(":\t");
    Serial.print(label);
    Serial.print("=");
    Serial.print(angle > 0 ? "+" : "");
    Serial.print(angle, 2);
}

bool serialAvailable(void)
{
    return Serial.available() > 0;
}

void serialBegin(uint32_t baud)
{
    Serial.begin(baud);
}

uint8_t serialGetCharacter(void)
{
    return Serial.read();
}

void serialPrintMessage(const char * msg)
{
    Serial.println(msg);
}

void serialReportAngles(uint32_t count, float angles[3])
{
    Serial.print(count, 10);

    reportAngle(angles[0], "roll");

    reportAngle(angles[1], "pitch");

    Serial.print("\tyaw=");
    Serial.println(angles[2], 2);
}
void serialReportCalibrationResults(USFSMAX * usfsmax)
{
    Serial.println("Gyroscope Sensor Offsets (dps)");
    Serial.println(usfsmax->gyrocal.V[0], 4);
    Serial.println(usfsmax->gyrocal.V[1], 4);
    Serial.println(usfsmax->gyrocal.V[2], 4);
    Serial.println("");
    Serial.println("Gyroscope Calibration Tensor");
    Serial.print(usfsmax->gyrocal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->gyrocal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->gyrocal.invW[0][2], 4);
    Serial.print(usfsmax->gyrocal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->gyrocal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->gyrocal.invW[1][2], 4);
    Serial.print(usfsmax->gyrocal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->gyrocal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->gyrocal.invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Accelerometer Sensor Offsets (g)");
    Serial.println(usfsmax->accelcal.V[0], 4);
    Serial.println(usfsmax->accelcal.V[1], 4);
    Serial.println(usfsmax->accelcal.V[2], 4);
    Serial.println("");
    Serial.println("Accelerometer Calibration Tensor");
    Serial.print(usfsmax->accelcal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->accelcal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->accelcal.invW[0][2], 4);
    Serial.print(usfsmax->accelcal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->accelcal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->accelcal.invW[1][2], 4);
    Serial.print(usfsmax->accelcal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->accelcal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->accelcal.invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Magnetometer Sensor Offsets (uT)");
    Serial.println(usfsmax->ellipsoid_magcal.V[0], 4);
    Serial.println(usfsmax->ellipsoid_magcal.V[1], 4);
    Serial.println(usfsmax->ellipsoid_magcal.V[2], 4); 
    Serial.println("");
    Serial.println("Magnetometer Soft Iron Correction Tensor");
    Serial.print(usfsmax->ellipsoid_magcal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->ellipsoid_magcal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->ellipsoid_magcal.invW[0][2], 4);
    Serial.print(usfsmax->ellipsoid_magcal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->ellipsoid_magcal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->ellipsoid_magcal.invW[1][2], 4);
    Serial.print(usfsmax->ellipsoid_magcal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->ellipsoid_magcal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->ellipsoid_magcal.invW[2][2], 4);
    Serial.println("");
    Serial.println("");
    Serial.println("Magnetometer Residual Hard Iron Offsets (uT)");
    Serial.println(usfsmax->final_magcal.V[0], 4);
    Serial.println(usfsmax->final_magcal.V[1], 4);
    Serial.println(usfsmax->final_magcal.V[2], 4);
    Serial.println("");
    Serial.println("Magnetometer Fine Calibration/Alignment Tensor");
    Serial.print(usfsmax->final_magcal.invW[0][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->final_magcal.invW[0][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->final_magcal.invW[0][2], 4);
    Serial.print(usfsmax->final_magcal.invW[1][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->final_magcal.invW[1][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->final_magcal.invW[1][2], 4);
    Serial.print(usfsmax->final_magcal.invW[2][0], 4);
    Serial.print(",");
    Serial.print(usfsmax->final_magcal.invW[2][1], 4);
    Serial.print(",");
    Serial.println(usfsmax->final_magcal.invW[2][2], 4);
    Serial.println("");
    Serial.println("");
}

