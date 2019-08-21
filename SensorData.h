/***********************************************************************
** Copyright (C) 2019 LP-Research
** All rights reserved.
** Contact: LP-Research (klaus@lp-research.com)
**
** This file is part of the Open Motion Analysis Toolkit (OpenMAT).
**
** Redistribution and use in source and binary forms, with
** or without modification, are permitted provided that the
** following conditions are met:
**
** Redistributions of source code must retain the above copyright
** notice, this list of conditions and the following disclaimer.
** Redistributions in binary form must reproduce the above copyright
** notice, this list of conditions and the following disclaimer in
** the documentation and/or other materials provided with the
** distribution.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
** FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
***********************************************************************/

#ifndef SENSOR_DATA
#define SENSOR_DATA
#include <string>
#include <sstream>
#include <cstdio>
#include "LpMatrix.h"
#include "LpmsIG1Registers.h"
#include "util.h"

#include "SensorDataI.h"

struct IG1ImuData :IG1ImuDataI
{
    IG1ImuData()
    {
        reset();
    }

    void setData(uint32_t transmitDataConfig, unsigned char *_data, int length) {

        dataSize = length / 4 -1;
        memcpy(data, _data+4, dataSize*4);

        uint2char i2c;
        floatArray2char f2c;

        int dataIdx = 0;

        memcpy(i2c.c, _data, 4);
        timestamp = i2c.int_val;
        dataIdx += 4;

            // 32-bit Floating Point
            if (transmitDataConfig & TDR_ACC_RAW_OUTPUT_ENABLED) {
            memcpy(accRaw.data, _data + dataIdx, 12);
            dataIdx += 12;
            }
            if (transmitDataConfig & TDR_ACC_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(accCalibrated.data, _data + dataIdx, 12);
            dataIdx += 12;
            }
            if (transmitDataConfig & TDR_GYR0_RAW_OUTPUT_ENABLED) {
            memcpy(gyroIRaw.data, _data + dataIdx, 12);
            dataIdx += 12;
            }
            if (transmitDataConfig & TDR_GYR1_RAW_OUTPUT_ENABLED) {
            memcpy(gyroIIRaw.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_GYR0_BIAS_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(gyroIBiasCalibrated.data, _data + dataIdx, 12);
            dataIdx += 12;
            }
            if (transmitDataConfig & TDR_GYR1_BIAS_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(gyroIIBiasCalibrated.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_GYR0_ALIGN_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(gyroIAlignmentCalibrated.data, _data + dataIdx, 12);
            dataIdx += 12;
            }
            if (transmitDataConfig & TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(gyroIIAlignmentCalibrated.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_MAG_RAW_OUTPUT_ENABLED) {
            memcpy(magRaw.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_MAG_CALIBRATED_OUTPUT_ENABLED) {
            memcpy(f2c.c, _data + dataIdx, 12);
            memcpy(magCalibrated.data, f2c.float_val, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_ANGULAR_VELOCITY_OUTPUT_ENABLED) {
            memcpy(angularVelocity.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_QUAT_OUTPUT_ENABLED) {
            memcpy(quaternion.data, _data + dataIdx, 16);
            dataIdx += 16;
            }

            if (transmitDataConfig & TDR_EULER_OUTPUT_ENABLED) {
            memcpy(euler.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_LINACC_OUTPUT_ENABLED) {
            memcpy(linAcc.data, _data + dataIdx, 12);
            dataIdx += 12;
            }

            if (transmitDataConfig & TDR_PRESSURE_OUTPUT_ENABLED) {
            memcpy(f2c.c, _data + dataIdx, 4);
            pressure = f2c.float_val[0];
            dataIdx += 4;
            }

            if (transmitDataConfig & TDR_ALTITUDE_OUTPUT_ENABLED) {
            memcpy(f2c.c, _data + dataIdx, 4);
            altitude = f2c.float_val[0];
            dataIdx += 4;
            }

            if (transmitDataConfig & TDR_TEMPERATURE_OUTPUT_ENABLED) {
            memcpy(f2c.c, _data + dataIdx, 4);
            temperature = f2c.float_val[0];
            dataIdx += 4;
            }
    }

    void setData16bit(uint32_t useRadianOutput, uint32_t transmitDataConfig, unsigned char *_data) {

        uint2char i2c;
        int dataIdx = 0;

        memcpy(i2c.c, _data, 4);
        timestamp = i2c.int_val;
        dataIdx += 4;

        float useRadianOutputFactor;

        if (useRadianOutput)
        {
           useRadianOutputFactor = 100.0f;
        }
        else {
           useRadianOutputFactor = 10.0f;
        }

        //16-bit fixed point
        if (transmitDataConfig & TDR_ACC_RAW_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                accRaw.data[i] = ((float)s) / 1000.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_ACC_CALIBRATED_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                accCalibrated.data[i] = ((float)s) / 1000.0f;
                dataIdx += 2;
            }
        }
        if (transmitDataConfig & TDR_GYR0_RAW_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIRaw.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }
        if (transmitDataConfig & TDR_GYR1_RAW_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIIRaw.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_GYR0_BIAS_CALIBRATED_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIBiasCalibrated.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }
        if (transmitDataConfig & TDR_GYR1_BIAS_CALIBRATED_OUTPUT_ENABLED) {
            //if (useRadianOutput) //100.0f
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIIBiasCalibrated.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_GYR0_ALIGN_CALIBRATED_OUTPUT_ENABLED) {
            //if (useRadianOutput) //100.0f
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIAlignmentCalibrated.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }
        if (transmitDataConfig & TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED) {
            //if (useRadianOutput) //100.0f
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                gyroIIAlignmentCalibrated.data[i] = ((float)s) / useRadianOutputFactor;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_MAG_RAW_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                magRaw.data[i] = ((float)s) / 100.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_MAG_CALIBRATED_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                magCalibrated.data[i] = ((float)s) / 100.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_ANGULAR_VELOCITY_OUTPUT_ENABLED) {
            //if (useRadianOutput)
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                angularVelocity.data[i] = ((float)s) / 100.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_QUAT_OUTPUT_ENABLED) {
            for (int i = 0; i < 4; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                quaternion.data[i] = ((float)s) / 10000.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_EULER_OUTPUT_ENABLED) {

            if (useRadianOutput) {
                for (int i = 0; i < 3; i++) {
                    int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                    euler.data[i] = ((float) s) / 10000.0f;
                    dataIdx += 2;
                }
            }
            else {
                for (int i = 0; i < 3; i++) {
                    int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                    euler.data[i] = ((float)s) / 100.0f;
                    dataIdx += 2;
                }
            }
        }

        if (transmitDataConfig & TDR_LINACC_OUTPUT_ENABLED) {
            for (int i = 0; i < 3; i++) {
                int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
                linAcc.data[i] = ((float)s) / 1000.0f;
                dataIdx += 2;
            }
        }

        if (transmitDataConfig & TDR_PRESSURE_OUTPUT_ENABLED) {
            int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
            pressure = ((float)s) / 100.0f;
            dataIdx += 2;
        }

        if (transmitDataConfig & TDR_ALTITUDE_OUTPUT_ENABLED) {
            int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
            altitude = ((float)s) / 10.0f;
            dataIdx += 2;
        }

        if (transmitDataConfig & TDR_TEMPERATURE_OUTPUT_ENABLED) {
            int16_t s = _data[dataIdx] + _data[dataIdx + 1] * 256;
            temperature = ((float)s) / 100.0f;
            dataIdx += 2;
        }
    }
};

struct IG1Info : IG1InfoI
{

    IG1Info()
    {
        reset();
    }

};


struct IG1AdvancedSettings : IG1SettingsI
{
    IG1AdvancedSettings()
    {
        reset();
    }
};

////////////////////////////////
// GPS Data
////////////////////////////////
struct IG1GpsData : IG1GpsDataI 
{
    IG1GpsData()
    {
        reset();
    }

        void setData(uint32_t gpsPVT_DataConfig, uint32_t gpsATT_DataConfig, unsigned char *_data)
        {
        int dataIdx = 0;

        uint2char i2c;
        memcpy(i2c.c, _data, 4);
        timestamp = i2c.int_val;
        dataIdx += 4;

        if (gpsPVT_DataConfig & GPS_NAV_PVT_ITOW_ENABLE) { memcpy(&pvt.iTOW, _data + dataIdx, sizeof(pvt.iTOW)); dataIdx += sizeof(pvt.iTOW); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_YEAR_ENABLE) { memcpy(&pvt.year, _data + dataIdx, sizeof(pvt.year)); dataIdx += sizeof(pvt.year); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_MONTH_ENABLE) { memcpy(&pvt.month, _data + dataIdx, sizeof(pvt.month)); dataIdx += sizeof(pvt.month); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_DAY_ENABLE) { memcpy(&pvt.day, _data + dataIdx, sizeof(pvt.day)); dataIdx += sizeof(pvt.day); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HOUR_ENABLE) { memcpy(&pvt.hour, _data + dataIdx, sizeof(pvt.hour)); dataIdx += sizeof(pvt.hour); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_MIN_ENABLE) { memcpy(&pvt.min, _data + dataIdx, sizeof(pvt.min)); dataIdx += sizeof(pvt.min); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_SEC_ENABLE) { memcpy(&pvt.sec, _data + dataIdx, sizeof(pvt.sec)); dataIdx += sizeof(pvt.sec); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_VALID_ENABLE) { memcpy(&pvt.valid, _data + dataIdx, sizeof(pvt.valid)); dataIdx += sizeof(pvt.valid); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_TACC_ENABLE) { memcpy(&pvt.tAcc, _data + dataIdx, sizeof(pvt.tAcc)); dataIdx += sizeof(pvt.tAcc); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_NANO_ENABLE) { memcpy(&pvt.nano, _data + dataIdx, sizeof(pvt.nano)); dataIdx += sizeof(pvt.nano); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_FIXTYPE_ENABLE) { memcpy(&pvt.fixType, _data + dataIdx, sizeof(pvt.fixType)); dataIdx += sizeof(pvt.fixType); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_FLAGS_ENABLE) { memcpy(&pvt.flags, _data + dataIdx, sizeof(pvt.flags)); dataIdx += sizeof(pvt.flags); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_FLAGS2_ENABLE) { memcpy(&pvt.flags2, _data + dataIdx, sizeof(pvt.flags2)); dataIdx += sizeof(pvt.flags2); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_NUMSV_ENABLE) { memcpy(&pvt.numSV, _data + dataIdx, sizeof(pvt.numSV)); dataIdx += sizeof(pvt.numSV); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_LONGITUDE_ENABLE) { memcpy(&pvt.longitude, _data + dataIdx, sizeof(pvt.longitude)); dataIdx += sizeof(pvt.longitude); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_LATITUDE_ENABLE) { memcpy(&pvt.latitude, _data + dataIdx, sizeof(pvt.latitude)); dataIdx += sizeof(pvt.latitude); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HEIGHT_ENABLE) { memcpy(&pvt.height, _data + dataIdx, sizeof(pvt.height)); dataIdx += sizeof(pvt.height); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HMSL_ENABLE) { memcpy(&pvt.hMSL, _data + dataIdx, sizeof(pvt.hMSL)); dataIdx += sizeof(pvt.hMSL); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HACC_ENABLE) { memcpy(&pvt.hAcc, _data + dataIdx, sizeof(pvt.hAcc)); dataIdx += sizeof(pvt.hAcc); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_VACC_ENABLE) { memcpy(&pvt.vAcc, _data + dataIdx, sizeof(pvt.vAcc)); dataIdx += sizeof(pvt.vAcc); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_VELN_ENABLE) { memcpy(&pvt.velN, _data + dataIdx, sizeof(pvt.velN)); dataIdx += sizeof(pvt.velN); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_VELE_ENABLE) { memcpy(&pvt.velE, _data + dataIdx, sizeof(pvt.velE)); dataIdx += sizeof(pvt.velE); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_VELD_ENABLE) { memcpy(&pvt.velD, _data + dataIdx, sizeof(pvt.velD)); dataIdx += sizeof(pvt.velD); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_GSPEED_ENABLE) { memcpy(&pvt.gSpeed, _data + dataIdx, sizeof(pvt.gSpeed)); dataIdx += sizeof(pvt.gSpeed); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HEADMOT_ENABLE) { memcpy(&pvt.headMot, _data + dataIdx, sizeof(pvt.headMot)); dataIdx += sizeof(pvt.headMot); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_SACC_ENABLE) { memcpy(&pvt.sAcc, _data + dataIdx, sizeof(pvt.sAcc)); dataIdx += sizeof(pvt.sAcc); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HEADACC_ENABLE) { memcpy(&pvt.headAcc, _data + dataIdx, sizeof(pvt.headAcc)); dataIdx += sizeof(pvt.headAcc); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_PDOP_ENABLE) { memcpy(&pvt.pDOP, _data + dataIdx, sizeof(pvt.pDOP)); dataIdx += sizeof(pvt.pDOP); }
        if (gpsPVT_DataConfig & GPS_NAV_PVT_HEADVEH_ENABLE) { memcpy(&pvt.headVeh, _data + dataIdx, sizeof(pvt.headVeh)); dataIdx += sizeof(pvt.headVeh); }

        if (gpsATT_DataConfig & GPS_NAV_ATT_ITOW_ENABLE) { memcpy(&att.iTOW, _data + dataIdx, sizeof(pvt.iTOW)); dataIdx += sizeof(pvt.iTOW); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_VERSION_ENABLE) { memcpy(&att.version, _data + dataIdx, sizeof(att.version)); dataIdx += sizeof(att.version); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_ROLL_ENABLE) { memcpy(&att.roll, _data + dataIdx, sizeof(att.roll)); dataIdx += sizeof(att.roll); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_PITCH_ENABLE) { memcpy(&att.pitch, _data + dataIdx, sizeof(att.pitch)); dataIdx += sizeof(att.pitch); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_HEADING_ENABLE) { memcpy(&att.heading, _data + dataIdx, sizeof(att.heading)); dataIdx += sizeof(att.heading); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_ACCROLL_ENABLE) { memcpy(&att.accRoll, _data + dataIdx, sizeof(att.accRoll)); dataIdx += sizeof(att.accRoll); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_ACCPITCH_ENABLE) { memcpy(&att.accPitch, _data + dataIdx, sizeof(att.accPitch)); dataIdx += sizeof(att.accPitch); }
        if (gpsATT_DataConfig & GPS_NAV_ATT_ACCHEADING_ENABLE) { memcpy(&att.accHeading, _data + dataIdx, sizeof(att.accHeading)); dataIdx += sizeof(att.accHeading); }

        if (gpsATT_DataConfig & GPS_ESF_STATUS_ITOW_ENABLE) { memcpy(&esf.iTOW, _data + dataIdx, sizeof(esf.iTOW)); dataIdx += sizeof(esf.iTOW); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_VERSION_ENABLE) { memcpy(&esf.version, _data + dataIdx, sizeof(esf.version)); dataIdx += sizeof(esf.version); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_INITSTATUS1_ENABLE) { memcpy(&esf.initStatus1, _data + dataIdx, sizeof(esf.initStatus1)); dataIdx += sizeof(esf.initStatus1); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_INITSTATUS2_ENABLE) { memcpy(&esf.initStatus2, _data + dataIdx, sizeof(esf.initStatus2)); dataIdx += sizeof(esf.initStatus2); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_FUSIONMODE_ENABLE) { memcpy(&esf.fusionMode, _data + dataIdx, sizeof(esf.fusionMode)); dataIdx += sizeof(esf.fusionMode); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_NUMSENS_ENABLE) { memcpy(&esf.numSens, _data + dataIdx, sizeof(esf.numSens)); dataIdx += sizeof(esf.numSens); }
        if (gpsATT_DataConfig & GPS_ESF_STATUS_SENSSTATUS_ENABLE) { memcpy(&esf.sensStatus, _data + dataIdx, sizeof(esf.sensStatus)); dataIdx += sizeof(esf.sensStatus); }
        
        if (gpsATT_DataConfig & GPS_UDR_STATUS_ENABLE) { memcpy(&udrStatus, _data + dataIdx, sizeof(udrStatus)); dataIdx += sizeof(udrStatus); }

    }

    std::string getStringCSV()
    {
        std::stringstream ss;
        // int precision = 12;
        // ss << std::setprecision(precision) << std::fixed;
        ss << timestamp << ",";
        ss << pvt.iTOW << ",";
        ss << pvt.year 
           << std::setw(2) << std::setfill('0') << (int)pvt.month 
           << std::setw(2) << std::setfill('0') << (int)pvt.day << ",";
        ss << std::setw(2) << std::setfill('0') << (int)pvt.hour 
           << std::setw(2) << std::setfill('0') << (int)pvt.min 
           << std::setw(2) << std::setfill('0') << (int)pvt.sec << ",";
        ss << (int)pvt.valid <<  ",";
        ss << pvt.tAcc <<  ",";
        ss << pvt.nano <<  ",";
        ss << (int)pvt.fixType <<  ",";
        ss << (int)pvt.flags <<  ",";
        ss << (int)pvt.flags2 <<  ",";
        ss << pvt.longitude<<  ",";
        ss << pvt.latitude <<  ",";
        ss << (int)pvt.numSV <<  ",";
        ss << pvt.height <<  ",";
        ss << pvt.hMSL <<  ",";
        ss << pvt.hAcc <<  ",";
        ss << pvt.vAcc <<  ",";
        ss << pvt.velN <<  ",";
        ss << pvt.velE <<  ",";
        ss << pvt.velD <<  ",";
        ss << pvt.gSpeed <<  ",";
        ss << pvt.headMot*1e-5 <<  ",";
        ss << pvt.sAcc <<  ",";
        ss << pvt.headAcc*1e-5 <<  ",";
        ss << pvt.pDOP*0.01 <<  ",";
        ss << pvt.headVeh*1e-5 <<  ",";

        ss << att.iTOW <<  ",";
        ss << (int)att.version <<  ",";
        ss << att.roll*1e-5 <<  ",";
        ss << att.pitch*1e-5 <<  ",";
        ss << att.heading*1e-5 <<  ",";
        ss << att.accRoll*1e-5 <<  ",";
        ss << att.accPitch*1e-5 <<  ",";
        ss << att.accHeading*1e-5 <<  ",";

        ss << esf.iTOW <<  ",";
        ss << (int)esf.version <<  ",";
        ss << (int)esf.initStatus1 <<  ",";
        ss << (int)esf.initStatus2 <<  ",";
        ss << (int)esf.fusionMode <<  ",";
        ss << (int)esf.numSens <<  ",";
        ss << esf.sensStatus[0] <<  ",";
        ss << esf.sensStatus[1] <<  ",";
        ss << esf.sensStatus[2] <<  ",";
        ss << esf.sensStatus[3] <<  ",";
        ss << esf.sensStatus[4] <<  ",";
        ss << esf.sensStatus[5] <<  ",";
        ss << esf.sensStatus[6] << ",";

        ss << udrStatus;

        return ss.str();
    }
};

#endif