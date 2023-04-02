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

#if defined(ARDUINO)
#include <Arduino.h>
#else
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <wiringPi.h>
#endif

#include "USFSMAX.h"
#include "i2c_api.h"

// AHRS filters follow ENU convention
void USFSMAX::accOrientation(float sensorValues[3], float accData[3])
{
    accData[USFSMAX::EAST]  = +sensorValues[0];
    accData[USFSMAX::NORTH] = +sensorValues[1];  
    accData[USFSMAX::UP]    = +sensorValues[2];
}

void USFSMAX::baroGetAdc()
{
    uint8_t bytes[3];

    i2cReadBytes(MAX32660_ADDR, BARO_XL, 3, bytes);
    baroADC = (int32_t)bytes[2] << 16 | (int32_t)bytes[1] << 8 | bytes[0];
}

void USFSMAX::begin(const bool enableDhiCorrector, const bool use2DDhiCorrector)
{
    uint8_t STAT;
    uint8_t ConfigByte;

    // Read the coprocessor's current fusion status
    STAT = i2cReadByte(MAX32660_ADDR, FUSION_STATUS);                                                                         
    delay(100);
//Serial.println(STAT,HEX);
    if (STAT == 0) {

        // Stop sensor fusion
        i2cWriteByte(MAX32660_ADDR, FUSION_START_STOP, 0x00);  

        delay(100);

        // Upload configuration structure variable
        USFSMAX::uploadConfig();

        // Re-start sensor fusion
        ConfigByte = ((0x01 | EulerQuatFlag << 1) | ScaledSensorDataFlag << 2);              
        // Set bit0 to re-start fusion; adjust bit1, bit2 for desired output options
        i2cWriteByte(MAX32660_ADDR, FUSION_START_STOP, ConfigByte);

        delay(100);

        // Poll the FUSION_STATUS register to see if fusion has resumed
        while(1)
        {
          
            delay(10);
            STAT = i2cReadByte(MAX32660_ADDR, FUSION_STATUS);
            if ((STAT & FUSION_RUNNING_MASK)) {break;}
        }
    }

    // Check for sensor errors
    STAT = i2cReadByte(MAX32660_ADDR, SENS_ERR_STAT);
    if (STAT !=0) {
        while(1) {;}
    }

    if (enableDhiCorrector) {
        if (use2DDhiCorrector) {
            // Enable DHI corrector, 2D (0x10|0x50)
            i2cWriteByte(MAX32660_ADDR, CALIBRATION_REQUEST, 0x51);
        } else {
            // Enable DHI corrector, 3D (0x10)
            i2cWriteByte(MAX32660_ADDR, CALIBRATION_REQUEST, 0x10);
        }
    }
    
    delay(100);

    retrieveCalibration();

    delay(500);
}

void USFSMAX::retrieveCalibration(void)
{
    USFSMAX::retrieveFullGyroCal();
    delay(100);
    USFSMAX::retrieveFullAccelCal();
    delay(100);
    USFSMAX::retrieveEllipMagCal();
    delay(100);
    USFSMAX::retrieveFinalMagCal();
    delay(100);
}

void USFSMAX::accGetAdc()
{
    uint8_t bytes[6];

    i2cReadBytes(MAX32660_ADDR, A_X_L, 6, bytes);
    accADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    accADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    accADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

float USFSMAX::gyroDpsPerCount(void)
{
    float f = 0;

    switch (config.Gscale) {
        case GYRO_SCALE_125_DPS:
            f = 125;
            break;
        case GYRO_SCALE_250_DPS:
            f = 250;
            break;
        case GYRO_SCALE_500_DPS:
            f = 500;
            break;
        case GYRO_SCALE_1000_DPS:
            f = 1000;
            break;
        case GYRO_SCALE_2000_DPS:
            f = 2000;
            break;
    }

    return f * 3.5e-5;
}

void USFSMAX::getDhiRsq()
{
    i2cReadByte(MAX32660_ADDR, CALIBRATION_STATUS);   

    uint8_t bytes[2];

    i2cReadBytes(MAX32660_ADDR, DHI_RSQ_L, 2, bytes);
    Rsq = ((float)((int16_t)bytes[1] << 8 | bytes[0]))/10000.0f;

    Serial.println (Rsq);
}

void USFSMAX::getMxMy()
{
    uint8_t bytes[4];
    int16_t H[2];

    i2cReadBytes(MAX32660_ADDR, MX_L, 4, bytes);

    H[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    H[1] = ((int16_t)bytes[3] << 8) | bytes[2];

    Mx = ((float)H[0])*UT_per_Count;
    My = ((float)H[1])*UT_per_Count;
}

void USFSMAX::update(void)
{
    // I2C instance 0, Sensor instance 0 (and implicitly USFSMAX instance 0)
    static uint8_t temp[1];

    // Read algorithm status and event status
    i2cReadBytes(MAX32660_ADDR, COMBO_DRDY_STAT, 1, temp);
    uint8_t eventStatus = temp[0];

    // Decode the event status to determine what data is ready and set the
    // appropriate DRDY flag
    got_gyro  = (bool)(eventStatus & 0x01);
    got_accel = (bool)(eventStatus & 0x02);
    got_mag   = (bool)(eventStatus & 0x04);
    got_baro  = (bool)(eventStatus & 0x08);
    got_quat  = (bool)(eventStatus & 0x10);

    switch (eventStatus & 0x0F) {

        case 0x01:
            gyroAccelGetAdc();
            break;
        case 0x02:
            gyroAccelGetAdc();
            break;
        case 0x03:
            gyroAccelGetAdc();
            break;
        case 0x07:
            gyroAccelMagBaroGetAdc();
            break;
        case 0x0B:
            gyroAccelMagBaroGetAdc();
            break;
        case 0x0F:
            gyroAccelMagBaroGetAdc();
            break;
        case 0x0C:
            magBaroGetAdc();
            break;
        case 0x04:
            magGetAdc();
            break;
        case 0x08:
            baroGetAdc();
            break;
        default:
            break;
    };

    static float sensorPoint[3];

    static float accData[3];
    static float gyroData[3];
    static float magData[3];

    if (got_mag) {

        // Calibration data is applied in the coprocessor; just scale
        if (ScaledSensorDataFlag) {
            for(uint8_t i=0; i<3; i++) {
                magData[i] = ((float)magADC[i])*UT_per_Count;
            }
        } else {       // Calibration data applied locally
            static float mag_calData[3];
            applyAdvCalibration( ellipsoid_magcal, magADC, UT_per_Count, mag_calData);
            applyAdvCalibration( final_magcal, mag_calData, 1, sensorPoint);
            USFSMAX::magOrientation(sensorPoint, magData);
        }
    }

    if (got_accel) {

        // Calibration data is applied in the coprocessor; just scale
        if (ScaledSensorDataFlag) {
            for(uint8_t i=0; i<3; i++) {
                accData[i] = ((float)accADC[i]) * gPerCount();
            } 
        } else {       // Calibration data applied locally
            applyAdvCalibration( accelcal, accADC, gPerCount(), sensorPoint);
            USFSMAX::accOrientation(sensorPoint, accData);
        }
    }

    if (got_gyro) {

        // Calibration data is applied in the coprocessor; just scale
        if (ScaledSensorDataFlag) {
            for(uint8_t i=0; i<3; i++) {
                gyroData[i] = ((float)gyroADC[i])*gyroDpsPerCount();
            }
        } else {       // Calibration data applied locally
            applyAdvCalibration(gyrocal, gyroADC, gyroDpsPerCount(), sensorPoint);
            USFSMAX::gyroOrientation(sensorPoint, gyroData);
        }
    }
}

void USFSMAX::applyAdvCalibration(full_adv_cal_t calibration, int16_t *raw, float sf, float *out)
{
    float x, y, z;

    x = ((float)raw[0] * sf) - calibration.V[0];
    y = ((float)raw[1] * sf) - calibration.V[1];
    z = ((float)raw[2] * sf) - calibration.V[2];
    out[0] = (x * calibration.invW[0][0] + y * calibration.invW[0][1] +
            z * calibration.invW[0][2]);
    out[1] = (x * calibration.invW[1][0] + y * calibration.invW[1][1] +
            z * calibration.invW[1][2]);
    out[2] = (x * calibration.invW[2][0] + y * calibration.invW[2][1] +
            z * calibration.invW[2][2]);
}

void USFSMAX::applyAdvCalibration(full_adv_cal_t calibration, float *raw, float sf, float *out)
{
    float x, y, z;

    x = (raw[0] * sf) - calibration.V[0];
    y = (raw[1] * sf) - calibration.V[1];
    z = (raw[2] * sf) - calibration.V[2];
    out[0] = (x * calibration.invW[0][0] + y * calibration.invW[0][1] +
            z * calibration.invW[0][2]);
    out[1] = (x * calibration.invW[1][0] + y * calibration.invW[1][1] +
            z * calibration.invW[1][2]);
    out[2] = (x * calibration.invW[2][0] + y * calibration.invW[2][1] +
            z * calibration.invW[2][2]);
}

void USFSMAX::readGyroAdc(int16_t gyroAdc[3])
{
    gyroAdc[0] = gyroADC[0];
    gyroAdc[1] = gyroADC[1];
    gyroAdc[2] = gyroADC[2];
}

void USFSMAX::readQuat(float quat[4])
{
    uint8_t bytes[16];

    i2cReadBytes(MAX32660_ADDR, Q0_BYTE0, 16, bytes);

    quat[0] = uint32_reg_to_float(&bytes[0]);
    quat[1] = uint32_reg_to_float(&bytes[4]);
    quat[2] = uint32_reg_to_float(&bytes[8]);
    quat[3] = uint32_reg_to_float(&bytes[12]);
}

bool USFSMAX::gotAccel(void)
{
    return got_accel;
}

bool USFSMAX::gotBaro(void)
{
    return got_baro;
}

bool USFSMAX::gotGyro(void)
{
    return got_gyro;
}

bool USFSMAX::gotMag(void)
{
    return got_mag;
}

bool USFSMAX::gotQuat(void)
{
    return got_quat;
}

float USFSMAX::gPerCount(void)
{
    float f = 0;

    switch (config.Ascale) {
        case ACC_SCALE_2:
            f = 2;
            break;
        case ACC_SCALE_4:
            f = 4;
            break;
        case ACC_SCALE_8:
            f = 8;
            break;
        case ACC_SCALE_16:
            f = 16;
            break;
    }

    return f * 3.05e-5;
}

void USFSMAX::gyroAccelGetAdc()
{
    uint8_t bytes[12];

    i2cReadBytes(MAX32660_ADDR, G_X_L, 12, bytes);
    gyroADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    gyroADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    gyroADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
    accADC[0]  = ((int16_t)bytes[7] << 8) | bytes[6];
    accADC[1]  = ((int16_t)bytes[9] << 8) | bytes[8];
    accADC[2]  = ((int16_t)bytes[11] << 8) | bytes[10];
}

void USFSMAX::gyroAccelMagBaroGetAdc()
{
    uint8_t bytes[21];

    i2cReadBytes(MAX32660_ADDR, G_X_L, 21, bytes);
    gyroADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    gyroADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    gyroADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
    accADC[0]  = ((int16_t)bytes[7] << 8) | bytes[6];
    accADC[1]  = ((int16_t)bytes[9] << 8) | bytes[8];
    accADC[2]  = ((int16_t)bytes[11] << 8) | bytes[10];
    magADC[0]  = ((int16_t)bytes[13] << 8) | bytes[12];
    magADC[1]  = ((int16_t)bytes[15] << 8) | bytes[14];
    magADC[2]  = ((int16_t)bytes[17] << 8) | bytes[16];
    baroADC    = (int32_t)bytes[20] << 16 | (int32_t)bytes[19] << 8 | bytes[18];
}

void USFSMAX::gyroGetAdc()
{
    uint8_t bytes[6];

    i2cReadBytes(MAX32660_ADDR, G_X_L, 6, bytes);
    gyroADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    gyroADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    gyroADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

void USFSMAX::gyroCal(void)
{
    // 0x01 - assert bit 0, start gyro cal
    i2cWriteByte(MAX32660_ADDR, USFSMAX::CALIBRATION_REQUEST, 0x01);  
}

// ENU rotation axes  PITCH (EAST) axis gyro (nose-up +)
//                    ROLL (NORTH) axis gyro (CW +)
//                    YAW (UP) axis gyro (E of N +)
void USFSMAX::gyroOrientation(float sensorValues[3], float gyroData[3])
{
    gyroData[USFSMAX::PITCH] = +sensorValues[0];     
    gyroData[USFSMAX::ROLL]  = +sensorValues[1];     
    gyroData[USFSMAX::YAW]   = -sensorValues[2];
}

void USFSMAX::linAccGetAdc()
{
    uint8_t bytes[12];

    i2cReadBytes(MAX32660_ADDR, LIN_X_L, 12, bytes);
    accLIN[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    accLIN[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    accLIN[2] = ((int16_t)bytes[5] << 8) | bytes[4];
    grav[0]   = ((int16_t)bytes[7] << 8) | bytes[6];
    grav[1]   = ((int16_t)bytes[9] << 8) | bytes[8];
    grav[2]   = ((int16_t)bytes[11] << 8) | bytes[10];
}

void USFSMAX::magBaroGetAdc()
{
    uint8_t bytes[9];

    i2cReadBytes(MAX32660_ADDR, M_X_L, 9, bytes);
    magADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    magADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    magADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
    baroADC   = (int32_t)bytes[8] << 16 | (int32_t)bytes[7] << 8 | bytes[6];
}

void USFSMAX::magGetAdc()
{
    uint8_t bytes[6];

    i2cReadBytes(MAX32660_ADDR, M_X_L, 6, bytes);
    magADC[0] = ((int16_t)bytes[1] << 8) | bytes[0];
    magADC[1] = ((int16_t)bytes[3] << 8) | bytes[2];
    magADC[2] = ((int16_t)bytes[5] << 8) | bytes[4];
}

// Mag follows NED convention  FWD RIGHT DOWN
// Check component sign with NOAA calculator
void USFSMAX::magOrientation(float sensorValues[3], float magData[3])
{
    magData[0] = -sensorValues[1];
    magData[1] = +sensorValues[0];
    magData[2] = -sensorValues[2];  // sensor is left-handed!
}

void USFSMAX::resetDhi()
{
    if (use2DDhiCorrector) {

        // Assert DHI Enable=true, DHI Reset=true and 2D Corrector=true
        // (0x40|0x20|0x10)
        i2cWriteByte(MAX32660_ADDR, CALIBRATION_REQUEST, 0x70);                                                                 
    } else {

        // Assert DHI Enable=true, DHI Reset=true and 2D Corrector=false
        // (0x00|0x20|0x10)
        i2cWriteByte(MAX32660_ADDR, CALIBRATION_REQUEST, 0x30);
    }
}

void USFSMAX::retrieveConfig()
{
    i2cReadBytes(MAX32660_ADDR, COPRO_CFG_DATA0, 30, &cfg_buff[0]);
    delay(100);
    i2cReadBytes(MAX32660_ADDR, COPRO_CFG_DATA1,
            (sizeof(CoProcessorConfig_t) - 30), &cfg_buff[30]);
    memcpy(&config, cfg_buff, sizeof(CoProcessorConfig_t));
}

void USFSMAX::retrieveEllipMagCal()
{
    i2cReadBytes(MAX32660_ADDR, ELLIP_MAG_CAL_DATA0, 30, &EllipMagCal_buff[0]);
    delay(100);
    i2cReadBytes(MAX32660_ADDR, ELLIP_MAG_CAL_DATA1, (sizeof(full_adv_cal_t) - 30),
            &EllipMagCal_buff[30]);
    memcpy(&ellipsoid_magcal, EllipMagCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::retrieveFinalMagCal()
{
    i2cReadBytes(MAX32660_ADDR, FINE_MAG_CAL_DATA0, 30, &FineMagCal_buff[0]);
    delay(100);
    i2cReadBytes(MAX32660_ADDR, FINE_MAG_CAL_DATA1, (sizeof(full_adv_cal_t) - 30),
            &FineMagCal_buff[30]);
    memcpy(&final_magcal, FineMagCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::retrieveFullAccelCal()
{
    i2cReadBytes(MAX32660_ADDR, ACCEL_CAL_DATA0, 30, &AccelCal_buff[0]);
    delay(100);
    i2cReadBytes(MAX32660_ADDR, ACCEL_CAL_DATA1, (sizeof(full_adv_cal_t) - 30),
            &AccelCal_buff[30]);
    memcpy(&accelcal, AccelCal_buff, sizeof(full_adv_cal_t));
}

void USFSMAX::retrieveFullGyroCal()
{
    i2cReadBytes(MAX32660_ADDR, GYRO_CAL_DATA0, 30, &gyroCal_buff[0]);
    delay(100);
    i2cReadBytes(MAX32660_ADDR, GYRO_CAL_DATA1, (sizeof(full_adv_cal_t) - 30),
            &gyroCal_buff[30]);
    memcpy(&gyrocal, gyroCal_buff, sizeof(full_adv_cal_t));
}

float USFSMAX::uint32_reg_to_float (uint8_t *buf)
{
    union
    {
        uint32_t ui32;
        float f;
    } u;

    u.ui32 = (((uint32_t)buf[0]) +
            (((uint32_t)buf[1]) <<  8) +
            (((uint32_t)buf[2]) << 16) +
            (((uint32_t)buf[3]) << 24));
    return u.f;
}

void USFSMAX::setAccLpfOdr(const lsm6dsm_acc_lpf_odr_e alpf)
{
    config.Alpf = alpf;
}

void USFSMAX::setAccOdr(const acc_gyro_odr_e aodr)
{
    config.AODR = aodr;
}

void USFSMAX::setAccScale(const acc_scale_e ascale)
{
    config.Ascale = ascale;
}

void USFSMAX::setBaroLpf(const lps22hb_baro_lpf_e blpf)
{
    config.Plpf = blpf;
}

void USFSMAX::setBaroOdr(const baro_odr_e bodr)
{
    config.PODR = bodr;
}

void USFSMAX::setCalPoints(const uint16_t calPoints)
{
    config.CalPoints = calPoints;
}

void USFSMAX::setGeoMag(
        const float geoMagMv,
        const float geoMagMh,
        const float geoMagDeclination)
{
    config.m_v   = geoMagMv;
    config.m_h   = geoMagMh;
    config.m_dec = geoMagDeclination;
}

void USFSMAX::setGyroLpf(const lsm6dsm_gyro_lpf_e glpf)
{
    config.Glpf = glpf;
}

void USFSMAX::setGyroOdr(const acc_gyro_odr_e godr)
{
    config.GODR = godr;
}

void USFSMAX::setGyroScale(const gyro_scale_e gscale)
{
    config.Gscale = gscale;
}

void USFSMAX::setMagLpf(const lis2mdl_mag_lpf_e mlpf)
{
    config.Mlpf = mlpf;
}

void USFSMAX::setMagOdr(const mag_odr_e modr)
{
    config.MODR = modr;
}

void USFSMAX::setQuatDiv(uint8_t qdiv)
{
    config.quat_div = qdiv - 1;
}
void USFSMAX::uploadConfig()
{
    // Clears bit0 to stop fusion an sets bit3 to specify configuration upload
    i2cWriteByte(MAX32660_ADDR, FUSION_START_STOP, 0x08);

    delay(1000);

    // Assign config structure to byte array upload
    memcpy(cfg_buff, &config, sizeof(CoProcessorConfig_t));

    // Upload configuration bytes
    i2cWriteBytes(MAX32660_ADDR, COPRO_CFG_DATA0, 30, &cfg_buff[0]);
    delay(100);
    i2cWriteBytes(MAX32660_ADDR, COPRO_CFG_DATA1,
            (sizeof(CoProcessorConfig_t) - 30), &cfg_buff[30]);
    delay(100);
}

bool USFSMAX::gyroCalIsActive(void)
{
    return (bool)(i2cReadByte(MAX32660_ADDR, CALIBRATION_STATUS) & 0x01);
}
