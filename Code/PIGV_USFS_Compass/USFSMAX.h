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

#pragma once

#include <stdbool.h>
#include <stdint.h>

class USFSMAX {

    public:

        typedef enum {

            ACC_GYRO_12_5_HZ = 1,
            ACC_GYRO_26_HZ, 
            ACC_GYRO_52_HZ, 
            ACC_GYRO_104_HZ, 
            ACC_GYRO_208_HZ, 
            ACC_GYRO_416_HZ, 
            ACC_GYRO_834_HZ, 
            ACC_GYRO_1660_HZ, 
            ACC_GYRO_3330_HZ, 
            ACC_GYRO_6660_HZ

        } acc_gyro_odr_e;

        // LSM6DSM Acc Output Scale
        typedef enum {

            ACC_SCALE_2  = 0,
            ACC_SCALE_4  = 2,
            ACC_SCALE_8  = 3,
            ACC_SCALE_16 = 1

        } acc_scale_e;

        typedef enum {

            BARO_1_HZ = 1,
            BARO_10_HZ = 0,
            BARO_25_HZ, 
            BARO_50_HZ, 
            BARO_75_HZ

        } baro_odr_e;

        // LSM6DSM gyro output scale
        typedef enum {

            GYRO_SCALE_125_DPS = 2,
            GYRO_SCALE_250_DPS = 0,
            GYRO_SCALE_500_DPS = 4,
            GYRO_SCALE_1000_DPS = 8,
            GYRO_SCALE_2000_DPS = 12

        } gyro_scale_e;

        // LIS2MDL Mag low pass filter setting
        typedef enum {

            LIS2MDL_MAG_2_HZ = 0,
            LIS2MDL_MAG_4_HZ

        } lis2mdl_mag_lpf_e;

        // LPS22HB Mag low pass filter setting
        typedef enum {

            LPS22HB_BARO_2_HZ = 0,
            LPS22HB_BARO_8_HZ = 8,
            LPS22HB_BARO_20_HZ = 12

        } lps22hb_baro_lpf_e;

        typedef enum {

            LSM6DSM_ACC_LPF_ODR_DIV_2 = 0,
            LSM6DSM_ACC_LPF_ODR_DIV_4,
            LSM6DSM_ACC_LPF_ODR_DIV_9,
            LSM6DSM_ACC_LPF_ODR_DIV_50,
            LSM6DSM_ACC_LPF_ODR_DIV_100,
            LSM6DSM_ACC_LPF_ODR_DIV_400,

        } lsm6dsm_acc_lpf_odr_e;

        // LSM6DSM Gyro low pass filter setting
        typedef enum {

            LSM6DSM_GYRO_LPF_167 = 2,
            LSM6DSM_GYRO_LPF_223 = 1,
            LSM6DSM_GYRO_LPF_314 = 0,
            LSM6DSM_GYRO_LPF_655 = 3

        } lsm6dsm_gyro_lpf_e;

        typedef enum {

            MAG_10_HZ = 0,
            MAG_20_HZ, 
            MAG_50_HZ, 
            MAG_100_HZ

        } mag_odr_e;

        typedef struct {
            float V[3];             // Offset vector components in physical units
            float invW[3][3];       // Inverse calibration matrix
            uint8_t cal_good;       // Byte to verify valid cal is in EEPROM
        } full_adv_cal_t;

        full_adv_cal_t accelcal;
        full_adv_cal_t ellipsoid_magcal;
        full_adv_cal_t final_magcal;
        full_adv_cal_t gyrocal;

        void begin(
                const bool enableDhiCorrector=true,
                const bool use2DDhiCorrector=true);

        float gyroDpsPerCount(void);

        void getDhiRsq(void);

        void getMxMy(void);

        bool gotAccel(void);
        bool gotBaro(void);
        bool gotMag(void);
        bool gotQuat(void);

        // Only needed when not using interrupts
        bool gotGyro(void);

        void readGyroAdc(int16_t gyroAdc[3]);

        void readQuat(float quat[4]);

        float gPerCount(void);

        void gyroCal(void);

        bool gyroCalIsActive(void);

        void resetDhi(void);

        void retrieveConfig();
        
        void retrieveCalibration(void);

        void setAccLpfOdr(const lsm6dsm_acc_lpf_odr_e alpf);
        void setAccOdr(const acc_gyro_odr_e aodr);
        void setAccScale(const acc_scale_e ascale);
        void setBaroLpf(const lps22hb_baro_lpf_e blpf);
        void setBaroOdr(const baro_odr_e bodr);
        void setCalPoints(const uint16_t calPoints);
        void setGyroLpf(const lsm6dsm_gyro_lpf_e glpf);
        void setGyroScale(const gyro_scale_e gscale);
        void setGyroOdr(const acc_gyro_odr_e godr);
        void setMagLpf(const lis2mdl_mag_lpf_e mlpf);
        void setMagOdr(const mag_odr_e modr);

        void setGeoMag(
                const float geoMagMv,
                const float geoMagMh,
                const float geoMagDeclination);

        // Actual quat_div will be set to this value minus 1, so user can start at 1.
        void setQuatDiv(uint8_t qdiv);

        // Call this on interrupt
        void update(void);

  int32_t             baroADC;



    private:

        static const uint8_t MAX32660_ADDR         = 0x57;

        // Status bytes -------------------------------------

        static const uint8_t SENS_ERR_STAT         = 0x00;
        static const uint8_t ACCEL_CAL_POS         = 0x02;
        static const uint8_t FUSION_STATUS         = 0x03;

        static const uint8_t CALIBRATION_STATUS    = 0x01;
        static const uint8_t COMBO_DRDY_STAT       = 0x04;
        static const uint8_t CALIBRATION_REQUEST   = 0x61;

        // Registers ----------------------------------------

        static const uint8_t G_X_L                 = 0x05;
        static const uint8_t G_X_H                 = 0x06;
        static const uint8_t G_Y_L                 = 0x07;
        static const uint8_t G_Y_H                 = 0x08;
        static const uint8_t G_Z_L                 = 0x09;
        static const uint8_t G_Z_H                 = 0x0A;
        static const uint8_t A_X_L                 = 0x0B;
        static const uint8_t A_X_H                 = 0x0C;
        static const uint8_t A_Y_L                 = 0x0D;
        static const uint8_t A_Y_H                 = 0x0E;
        static const uint8_t A_Z_L                 = 0x0F;
        static const uint8_t A_Z_H                 = 0x10;
        static const uint8_t M_X_L                 = 0x11;
        static const uint8_t M_X_H                 = 0x12;
        static const uint8_t M_Y_L                 = 0x13;
        static const uint8_t M_Y_H                 = 0x14;
        static const uint8_t M_Z_L                 = 0x15;
        static const uint8_t M_Z_H                 = 0x16;
        static const uint8_t BARO_XL               = 0x17;
        static const uint8_t BARO_L                = 0x18;
        static const uint8_t BARO_H                = 0x19;
        static const uint8_t Q0_BYTE0              = 0x1A;
        static const uint8_t Q0_BYTE1              = 0x1B;
        static const uint8_t Q0_BYTE2              = 0x1C;
        static const uint8_t Q0_BYTE3              = 0x1D;
        static const uint8_t Q1_BYTE0              = 0x1E;
        static const uint8_t Q1_BYTE1              = 0x1F;
        static const uint8_t Q1_BYTE2              = 0x20;
        static const uint8_t Q1_BYTE3              = 0x21;
        static const uint8_t Q2_BYTE0              = 0x22;
        static const uint8_t Q2_BYTE1              = 0x23;
        static const uint8_t Q2_BYTE2              = 0x24;
        static const uint8_t Q2_BYTE3              = 0x25;
        static const uint8_t Q3_BYTE0              = 0x26;
        static const uint8_t Q3_BYTE1              = 0x27;
        static const uint8_t Q3_BYTE2              = 0x28;
        static const uint8_t Q3_BYTE3              = 0x29;
        static const uint8_t LIN_X_L               = 0x2A;
        static const uint8_t LIN_X_H               = 0x2B;
        static const uint8_t LIN_Y_L               = 0x2C;
        static const uint8_t LIN_Y_H               = 0x2D;
        static const uint8_t LIN_Z_L               = 0x2E;
        static const uint8_t LIN_Z_H               = 0x2F;
        static const uint8_t GRAV_X_L              = 0x30;
        static const uint8_t GRAV_X_H              = 0x31;
        static const uint8_t GRAV_Y_L              = 0x32;
        static const uint8_t GRAV_Y_H              = 0x33;
        static const uint8_t GRAV_Z_L              = 0x34;
        static const uint8_t GRAV_Z_H              = 0x35;
        static const uint8_t YAW_BYTE0             = 0x36;
        static const uint8_t YAW_BYTE1             = 0x37;
        static const uint8_t YAW_BYTE2             = 0x38;
        static const uint8_t YAW_BYTE3             = 0x39;
        static const uint8_t PITCH_BYTE0           = 0x3A;
        static const uint8_t PITCH_BYTE1           = 0x3B;
        static const uint8_t PITCH_BYTE2           = 0x3C;
        static const uint8_t PITCH_BYTE3           = 0x3D;
        static const uint8_t ROLL_BYTE0            = 0x3E;
        static const uint8_t ROLL_BYTE1            = 0x3F;
        static const uint8_t ROLL_BYTE2            = 0x40;
        static const uint8_t ROLL_BYTE3            = 0x41;
        static const uint8_t AG_TEMP_L             = 0x42;
        static const uint8_t AG_TEMP_H             = 0x43;
        static const uint8_t M_TEMP_L              = 0x44;
        static const uint8_t M_TEMP_H              = 0x45;
        static const uint8_t B_TEMP_L              = 0x46;
        static const uint8_t B_TEMP_H              = 0x47;
        static const uint8_t AUX_1_X_L             = 0x48;
        static const uint8_t AUX_1_X_H             = 0x49;
        static const uint8_t AUX_1_Y_L             = 0x4A;
        static const uint8_t AUX_1_Y_H             = 0x4B;
        static const uint8_t AUX_1_Z_L             = 0x4C;
        static const uint8_t AUX_1_Z_H             = 0x4D;
        static const uint8_t AUX_2_X_L             = 0x4E;
        static const uint8_t AUX_2_X_H             = 0x4F;
        static const uint8_t AUX_2_Y_L             = 0x50;
        static const uint8_t AUX_2_Y_H             = 0x51;
        static const uint8_t AUX_2_Z_L             = 0x52;
        static const uint8_t AUX_2_Z_H             = 0x53;
        static const uint8_t AUX_3_X_L             = 0x54;
        static const uint8_t AUX_3_X_H             = 0x55;
        static const uint8_t AUX_3_Y_L             = 0x56;
        static const uint8_t AUX_3_Y_H             = 0x57;
        static const uint8_t AUX_3_Z_L             = 0x58;
        static const uint8_t AUX_3_Z_H             = 0x59;
        static const uint8_t MX_L                  = 0x5A;
        static const uint8_t MX_H                  = 0x5B;
        static const uint8_t MY_L                  = 0x5C;
        static const uint8_t MY_H                  = 0x5D;
        static const uint8_t DHI_RSQ_L             = 0x5E;
        static const uint8_t DHI_RSQ_H             = 0x5F;

        static const uint8_t FUSION_START_STOP     = 0x60;

        static const uint8_t COPRO_CFG_DATA0       = 0x62;
        static const uint8_t COPRO_CFG_DATA1       = 0x63;
        static const uint8_t GYRO_CAL_DATA0        = 0x64;
        static const uint8_t GYRO_CAL_DATA1        = 0x65;
        static const uint8_t ACCEL_CAL_DATA0       = 0x66;
        static const uint8_t ACCEL_CAL_DATA1       = 0x67;
        static const uint8_t ELLIP_MAG_CAL_DATA0   = 0x68;
        static const uint8_t ELLIP_MAG_CAL_DATA1   = 0x69;
        static const uint8_t FINE_MAG_CAL_DATA0    = 0x6A;
        static const uint8_t FINE_MAG_CAL_DATA1    = 0x6B;

        static const uint8_t FUSION_RUNNING_MASK   = 0x01;
        static const uint8_t HI_CORRECTOR_MASK     = 0x10;

        enum attitudes
        {
            PITCH = 0,
            ROLL,
            YAW
        };

        enum axes
        {
            EAST = 0,
            NORTH,
            UP
        };

        // Configuration block using reasonable defaults) ---------------------
        // Note that the types must be declared in this order, using standard sizes
        /// (uint8_t, float, ...) instead of enumerated types, to avoid the block 
        // size changing with different compilers.
        typedef struct {

            // Number or data points collected for gyro and accel/fine mag
            // calibrations
            uint16_t CalPoints = 2048;

            uint8_t Ascale    = ACC_SCALE_16;
            uint8_t AODR      = ACC_GYRO_834_HZ;
            uint8_t Alpf      = LSM6DSM_ACC_LPF_ODR_DIV_400;
            uint8_t Ahpf      = 0; // future option
            uint8_t Gscale    = GYRO_SCALE_2000_DPS;
            uint8_t GODR      = ACC_GYRO_834_HZ;
            uint8_t Glpf      = LSM6DSM_GYRO_LPF_167;
            uint8_t Ghpf      = 0; // future option
            uint8_t Mscale    = 0; // not adjustable
            uint8_t MODR      = MAG_100_HZ;
            uint8_t Mlpf      = LIS2MDL_MAG_4_HZ;
            uint8_t Mhpf      = 0; // future option
            uint8_t Pscale    = 0; // not adjustable
            uint8_t PODR      = BARO_50_HZ;
            uint8_t Plpf      = LPS22HB_BARO_20_HZ;
            uint8_t AUX1scale = 0; // future option
            uint8_t AUX1ODR   = 0; // future option
            uint8_t AUX2scale = 0; // future option
            uint8_t AUX2ODR   = 0; // future option
            uint8_t AUX3scale = 0; // future option
            uint8_t AUX3ODR   = 0; // future option

            // Geomagnetic settings
            float    m_v   = 0;
            float    m_h   = 0;
            float    m_dec = 0;

            // Quaternion rate divisor; rate is the gyro ODR (in Hz) divided by
            // divisor.
            uint8_t  quat_div = 4;

        } CoProcessorConfig_t;

        // --------------------------------------------------------------------:w

        int16_t             accADC[3];
        uint8_t             AccelCal_buff[sizeof(full_adv_cal_t)];
        int16_t             accLIN[3];
      
        CoProcessorConfig_t config;
        uint8_t             cfg_buff[sizeof(CoProcessorConfig_t)];
        uint8_t             EllipMagCal_buff[sizeof(full_adv_cal_t)];
        uint8_t             EulerQuatFlag;
        uint8_t             FineMagCal_buff[sizeof(full_adv_cal_t)];
        bool                got_accel;
        bool                got_baro;
        bool                got_gyro;
        bool                got_mag;
        bool                got_quat;
        int16_t             grav[3];
        int16_t             gyroADC[3];
        uint8_t             gyroCal_buff[sizeof(full_adv_cal_t)];
        int16_t             magADC[3];
        float               Mx;
        float               My;
        float               qt[4] = {1, 0, 0, 0};
        float               Rsq;
        uint8_t             ScaledSensorDataFlag;
        float               uint32_reg_to_float(uint8_t *buf);
        bool                use2DDhiCorrector;
        float               UT_per_Count;

        void applyAdvCalibration(full_adv_cal_t calibration,
                int16_t *raw, float sf, float *out);

        void applyAdvCalibration(full_adv_cal_t calibration,
                float *raw, float sf, float *out);

        void uploadConfig(void);

        void gyroAccelMagBaroGetAdc(void);
        void gyroAccelGetAdc(void);
        void magBaroGetAdc(void);
        void gyroGetAdc(void);
        void accGetAdc(void);
        void magGetAdc(void);
        void linAccGetAdc(void);
        void baroGetAdc(void);

        void retrieveFullAccelCal(void);
        void retrieveEllipMagCal(void);
        void retrieveFinalMagCal(void);
        void retrieveFullGyroCal(void);

        static void accOrientation(float sensorValues[3], float accData[3]);
        static void gyroOrientation(float sensorValues[3], float gyroData[3]);
        static void magOrientation(float sensorValues[3], float magData[3]);
};
