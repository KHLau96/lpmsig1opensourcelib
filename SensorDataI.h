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

#ifndef SENSOR_DATA_I_H
#define SENSOR_DATA_I_H
#include <string>
#include <sstream>
#include <iomanip>
#include <cstdio>
#include "LpMatrix.h"

#if defined(_MSC_VER) && _MSC_VER < 1900
#define snprintf _snprintf
#else
#include <stdio.h> //sprintf
#endif

/* --- PRINTF_BYTE_TO_BINARY macro's --- */
//https://stackoverflow.com/questions/111928/is-there-a-printf-converter-to-print-in-binary-format
#define PRINTF_BINARY_PATTERN_INT8 "%c%c%c%c%c%c%c%c"
#define PRINTF_BYTE_TO_BINARY_INT8(i)    \
    (((i) & 0x80ll) ? '1' : '0'), \
    (((i) & 0x40ll) ? '1' : '0'), \
    (((i) & 0x20ll) ? '1' : '0'), \
    (((i) & 0x10ll) ? '1' : '0'), \
    (((i) & 0x08ll) ? '1' : '0'), \
    (((i) & 0x04ll) ? '1' : '0'), \
    (((i) & 0x02ll) ? '1' : '0'), \
    (((i) & 0x01ll) ? '1' : '0')

#define PRINTF_BYTE_TO_BINARY_INT16(i) \
    PRINTF_BYTE_TO_BINARY_INT8((i) >> 8),   PRINTF_BYTE_TO_BINARY_INT8(i)
#define PRINTF_BYTE_TO_BINARY_INT32(i) \
    PRINTF_BYTE_TO_BINARY_INT16((i) >> 16), PRINTF_BYTE_TO_BINARY_INT16(i)
#define PRINTF_BYTE_TO_BINARY_INT64(i) \
    PRINTF_BYTE_TO_BINARY_INT32((i) >> 32), PRINTF_BYTE_TO_BINARY_INT32(i)

#define PRINTF_BINARY_PATTERN_INT16_PP \
    PRINTF_BINARY_PATTERN_INT8 " " PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BINARY_PATTERN_INT32_PP \
    PRINTF_BINARY_PATTERN_INT16_PP " " PRINTF_BINARY_PATTERN_INT16_PP
#define PRINTF_BINARY_PATTERN_INT64_PP    \
    PRINTF_BINARY_PATTERN_INT32_PP " " PRINTF_BINARY_PATTERN_INT32_PP

#define PRINTF_BINARY_PATTERN_INT16 \
    PRINTF_BINARY_PATTERN_INT8  PRINTF_BINARY_PATTERN_INT8
#define PRINTF_BINARY_PATTERN_INT32 \
    PRINTF_BINARY_PATTERN_INT16  PRINTF_BINARY_PATTERN_INT16
/* --- end macros --- */


struct IG1ImuDataI
{
    static const int MAX_DATA_SIZE = 50;
    unsigned int timestamp;
    int dataSize;
    float data[MAX_DATA_SIZE];
    LpVector3f accRaw;
    LpVector3f accCalibrated;
    LpVector3f gyroIRaw;
    LpVector3f gyroIBiasCalibrated;
    LpVector3f gyroIAlignmentCalibrated;
    LpVector3f gyroIIRaw;
    LpVector3f gyroIIBiasCalibrated;
    LpVector3f gyroIIAlignmentCalibrated;
    LpVector3f magRaw;
    LpVector3f magCalibrated;
    LpVector3f angularVelocity;
    LpVector4f quaternion;
    LpVector3f euler;
    LpVector3f linAcc;
    float pressure;
    float altitude;
    float temperature;

    IG1ImuDataI()
    {
        reset();
    }

    void reset()
    {
        dataSize = 0;
        timestamp = 0;
        for (int i = 0; i < MAX_DATA_SIZE; ++i)
            data[i] = 0.0f;

        vectZero3x1(&accRaw);
        vectZero3x1(&accCalibrated);
        vectZero3x1(&gyroIRaw);
        vectZero3x1(&gyroIBiasCalibrated);
        vectZero3x1(&gyroIAlignmentCalibrated);
        vectZero3x1(&gyroIIRaw);
        vectZero3x1(&gyroIIBiasCalibrated);
        vectZero3x1(&gyroIIAlignmentCalibrated);

        vectZero3x1(&magRaw);
        vectZero3x1(&magCalibrated);

        vectZero3x1(&angularVelocity);
        vectZero4x1(&quaternion);
        vectZero3x1(&euler);
        vectZero3x1(&linAcc);
        pressure = 0.0f;
        altitude = 0.0f;
        temperature = 0.0f;
    }
};

struct IG1InfoI
{
    std::string firmwareInfo;
    std::string deviceName;
    std::string serialNumber;
    std::string filterVersion;
    bool iapCheckStatus;

    IG1InfoI()
    {
        reset();
    }


    void reset()
    {
        firmwareInfo = "";
        deviceName = "";
        serialNumber = "";
        filterVersion = "";
        iapCheckStatus = 0;
    }


    std::string toString()
    {
        std::stringstream ss;
        int w = 20;
        ss << std::setw(w) << std::left << "Device name: " << deviceName << "\n";
        ss << std::setw(w) << std::left << "Firmware info: " << firmwareInfo << "\n";
        ss << std::setw(w) << std::left << "Filter version: " << filterVersion << "\n";
        ss << std::setw(w) << std::left << "Serial no.: " << serialNumber << "\n";
        ss << std::setw(w) << std::left << "IAP check status: " << (iapCheckStatus ? "Ready" : "Not Ready") << "\n";
        return ss.str();
    }
};

struct IG1SettingsI
{
    uint32_t transmitDataConfig;
    uint32_t sensorId;
    uint32_t dataStreamFrequency;
    uint32_t useRadianOutput;
    uint32_t enableGyroAutocalibration;
    float gyroThreshold;

    // Acc
    uint32_t accRange;

    // Gyro
    uint32_t gyroRange;

    // Mag
    uint32_t magRange;
    float magCalibrationTimeout;

    // Filter 
    uint32_t filterMode;

    // Uart communication
    uint32_t uartBaudrate;
    uint32_t uartDataFormat;
    uint32_t uartDataPrecision;
    char uartAsciiStart;
    char uartAsciiStop;

    // CAN Communication
    uint32_t canStartId;
    uint32_t canBaudrate;
    uint32_t canDataPrecision;  // Fix point 16bit or Floating point
    uint32_t canMode;           // Sequential / CANOpen
    uint32_t canMapping[16];
    uint32_t canHeartbeatTime;

    //offset mode
    uint32_t offsetMode;

    //gps
    uint32_t gpsTransmitDataConfig[2];


    IG1SettingsI() 
    { 
        reset(); 
    }

    void reset() 
    {
        // general
        transmitDataConfig = 0;
        sensorId = 0;
        dataStreamFrequency = 0;
        useRadianOutput = 0;
        enableGyroAutocalibration = 0;
        gyroThreshold = 0;

        // Acc
        accRange = 0;

        // Gyro
        gyroRange = 0;

        // Mag
        magRange = 0;
        magCalibrationTimeout = 0;

        // Filter parameters
        filterMode = 0;

        // uart communication
        uartBaudrate = 0;
        uartDataFormat = 0;
        uartDataPrecision = 0;
        uartAsciiStart = '$';
        uartAsciiStop = '\n';

        // CAN Communication
        canStartId = 0;
        canBaudrate = 0;
        canDataPrecision = 0;  // Fix point 16bit or Floating point
        canMode = 0;           // Sequential / CANOpen
        memset(canMapping, 0, sizeof(canMapping[0]) * 16);
        canHeartbeatTime = 0;

        //offset mode
        offsetMode = 0;

        //gps
        memset(gpsTransmitDataConfig, 0, sizeof(gpsTransmitDataConfig[0]) * 2);
    }

    std::string toString()
     {
            int w = 38;
            std::stringstream ss;
            // general
            ss << "=== General === \n";
            ss << "Transmit Data: " << transmitDataConfig << " (" << uint32ToBinaryPP(transmitDataConfig) << ")\n";
            ss << std::setw(w) << std::left << "Sensor ID: " << sensorId << "\n";
            ss << std::setw(w) << std::left << "Data stream freq: " << dataStreamFrequency << "Hz\n";
            ss << std::setw(w) << std::left << "Gyro output unit: " << (useRadianOutput ? "radian" : "deg") << "\n";
            ss << std::setw(w) << std::left << "Gyro autocalibration: " << (enableGyroAutocalibration ? "Enable" : "Disable") << "\n";
            ss << std::setw(w) << std::left << "Gyro threshold: " << gyroThreshold << "dps\n";

            ss << "=== Acc === \n";
            ss << std::setw(w) << std::left << "Acc range: " << accRange << "G\n";

            ss << "=== Gyro === \n";
            ss << std::setw(w) << std::left << "Gyro range: " << gyroRange << "dps\n";

            ss << "=== Mag === \n";
            ss << std::setw(w) << std::left << "Mag range: " << magRange << "Gauss\n";
            ss << std::setw(w) << std::left << "Mag CalibrationTimeout: " << magCalibrationTimeout << " s\n";

            ss << "=== Filter === \n";
            ss << std::setw(w) << std::left << "Filter mode: " << filterMode << "\n";

            ss << "=== Uart === \n";
            ss << std::setw(w) << std::left << "Uart baudrate: " << uartBaudrate << "\n";
            ss << std::setw(w) << std::left << "Uart data format: " << (uartDataFormat ? "ASCII" : "LPBus") << "\n";
            ss << std::setw(w) << std::left << "Uart data precision: " << (uartDataPrecision ? "Floating point" : "Fixed point") << "\n";
            ss << std::setw(w) << std::left << "Uart ascii start: " << std::hex << "0x" << (int)uartAsciiStart << "\n";
            ss << std::setw(w) << std::left << "Uart ascii stop: " << std::hex << "0x" << (int)uartAsciiStop << "\n";
            ss << std::dec;

            ss << "=== CAN === \n";
            ss << std::setw(w) << std::left << "CAN Start ID: " << canStartId << std::hex << "(0x" << canStartId << std::dec << ")" << "\n";
            ss << std::setw(w) << std::left << "CAN baudrate: " << canBaudrate << "\n";
            ss << std::setw(w) << std::left << "CAN data precision: " << (canDataPrecision ? "Floating point" : "Fixed point") << "\n";
            ss << std::setw(w) << std::left << "CAN Mode: " << (canMode ? "Sequential" : "CANOpen") << "\n";

            ss << std::setw(w / 2) << std::left << "CAN Mapping:";
            for (int i = 0; i < 16; ++i)
            {
                ss << canMapping[i] << " ";
            }
            ss << "\n";
            ss << std::setw(w) << std::left << "CAN Heartbeat: " << canHeartbeatTime << "\n";

            ss << "=== Offset === \n";
            ss << std::setw(w) << std::left << "Offset Mode: " << offsetMode << "\n";

            ss << "=== GPS === \n";
            ss << "GPS Transmit Data0: " << gpsTransmitDataConfig[0] << " (" << uint32ToBinaryPP(gpsTransmitDataConfig[0]) << ")\n";
            ss << "GPS Transmit Data1: " << gpsTransmitDataConfig[1] << " (" << uint32ToBinaryPP(gpsTransmitDataConfig[1]) << ")\n";

            return ss.str();
    }

    std::string uint32ToBinaryPP(uint32_t data)
    {
        char binaryFormat[36] = { 0 };
        snprintf(binaryFormat, 36, PRINTF_BINARY_PATTERN_INT32_PP, PRINTF_BYTE_TO_BINARY_INT32(data));
        return std::string(binaryFormat);
    }

    std::string uint32ToBinary(uint32_t data)
    {
        char binaryFormat[36] = { 0 };
        snprintf(binaryFormat, 36, PRINTF_BINARY_PATTERN_INT32, PRINTF_BYTE_TO_BINARY_INT32(data));
        return std::string(binaryFormat);
    }
};

////////////////////////////////
// GPS Data
////////////////////////////////
typedef struct _NAV_PVT
{
    uint32_t iTOW;
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint8_t sec;
    uint8_t valid;        //0xF3 Means "Time(Mask:0x02) and Date(Mask:0x01) is valid"
    uint32_t tAcc;
    int32_t nano;         //unit:ns
    uint8_t fixType;      //enum fixType;
    uint8_t flags;        //differential correction; head of Vehicle is valid
    uint8_t flags2;
    uint8_t numSV;
    int32_t longitude;
    int32_t latitude;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    int32_t gSpeed;
    int32_t headMot;
    uint32_t sAcc;
    uint32_t headAcc;
    uint16_t pDOP;
    int32_t headVeh;

} NAV_PVT;

typedef struct _NAV_ATT
{
    uint32_t iTOW;
    uint8_t version;
    int32_t roll;
    int32_t pitch;
    int32_t heading;
    uint32_t accRoll;
    uint32_t accPitch;
    uint32_t accHeading;

} NAV_ATT;

typedef struct _ESF_STATUS
{
    uint32_t iTOW;
    uint8_t version;
    uint8_t initStatus1;
    uint8_t initStatus2;
    uint8_t fusionMode;
    uint8_t numSens;
    uint32_t sensStatus[7];

} ESF_STATUS;

struct IG1GpsDataI
{
    unsigned int timestamp;
    NAV_PVT pvt;
    NAV_ATT att;
    ESF_STATUS esf;
    unsigned int udrStatus;

    IG1GpsDataI()
    {
        reset();
    }

    void reset()
    {
        timestamp = 0;
        memset(&pvt, 0, sizeof(NAV_PVT));
        memset(&att, 0, sizeof(NAV_ATT));
        memset(&esf, 0, sizeof(ESF_STATUS));
        udrStatus = 0;
    }

};

#endif