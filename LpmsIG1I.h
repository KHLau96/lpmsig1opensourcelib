#ifndef LPMSIG1_I_H
#define LPMSIG1_I_H

#ifdef _WIN32
#include "windows.h"
#endif
#include "SensorDataI.h"

// Sensor status
#define STATUS_DISCONNECTED     0
#define STATUS_CONNECTING       1
#define STATUS_CONNECTED        2
#define STATUS_CONNECTION_ERROR 3
#define STATUS_DATA_TIMEOUT     4
#define STATUS_UPDATING         5

class IG1I
{
public:
    const std::string TAG = "IG1";
    /////////////////////////////////////////////
    // Constructor/Destructor
    /////////////////////////////////////////////
    ~IG1I(void) {};
    virtual void release() = 0;

    /////////////////////////////////////////////
    // Connection
    /////////////////////////////////////////////
    virtual void setPCBaudrate(int baud) =0;
#ifdef _WIN32
    virtual void setPCPort(int port) =0;
#else
    virtual void setPCPort(std::string port) = 0;
#endif

#ifdef _WIN32
    virtual int connect(int _portno, int _baudrate) = 0;
    virtual int connect(std::string sensorName, int _baudrate) = 0;
#else
    virtual int connect(std::string _portno, int _baudrate) = 0;
#endif

    virtual bool disconnect() = 0;

    /////////////////////////////////////////////
    // Commands:
    // - send command to sensor
    // - commandXXX functions will add appropriate commands to command queue
    //   and processed in the internal thread
    /////////////////////////////////////////////

    /*
    Function: set sensor to command mode
    Parameters: none
    Returns: none
    */
    virtual void commandGotoCommandMode(void) = 0;

    /*
    Function: set sensor to streaming mode
    Parameters: none
    Returns: none
    */
    virtual void commandGotoStreamingMode(void) = 0;

	/*
	Function:
	- send command to sensor to get sensor ID
    - hasSettings() will return true once sensor info is available
	Parameters: none
    Returns: none
	*/
	virtual void commandGetSensorID(void) = 0;

	/*
	Function:
	- send command to sensor to set sensor ID
	Parameters: int32
	Returns: none
	*/
	virtual void commandSetSensorID(uint32_t id) = 0;

	/*
	Function:
	- send command to sensor to get/set sensor Frenquency
	- hasInfo() will return true once sensor info is available
	Parameters: 
        #define DATA_STREAM_FREQ_5HZ                    5
        #define DATA_STREAM_FREQ_10HZ                   10
        #define DATA_STREAM_FREQ_50HZ                   50
        #define DATA_STREAM_FREQ_100HZ                  100
        #define DATA_STREAM_FREQ_250HZ                  250
        #define DATA_STREAM_FREQ_500HZ                  500
        #define DATA_STREAM_FREQ_1000HZ                 1000
	Returns: none
	*/
	virtual void commandGetSensorFrequency(void) = 0;
	virtual void commandSetSensorFrequency(uint32_t freq) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor gyro range
    - hasSettings() will return true once sensor info is available
    Parameters: 
        #define GYR_RANGE_400DPS                        400
        #define GYR_RANGE_1000DPS                       1000
        #define GYR_RANGE_2000DPS                       2000
    Returns: none
    */
	virtual void commandGetSensorGyroRange(void) = 0;
	virtual void commandSetSensorGyroRange(uint32_t range) = 0;

    /*
    Function:
    - send command to sensor to start calibration static gyro
        Please keep imu still 4 seconds.
    Parameters: none
    Returns: none
    */
    virtual void commandStartGyroCalibration(void) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor acc range
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define ACC_RANGE_2G                            2
        #define ACC_RANGE_4G                            4
        #define ACC_RANGE_8G                            8
        #define ACC_RANGE_16G                           16
    Returns: none
    */
	virtual void commandGetSensorAccRange(void) = 0;
	virtual void commandSetSensorAccRange(uint32_t range) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor mag range
    - hasSettings() will return true once sensor info is available
    Parameters:    
        #define MAG_RANGE_2GUASS                        2
        #define MAG_RANGE_8GUASS                        8
    Returns: none
    */
	virtual void commandGetSensorMagRange(void) = 0;
	virtual void commandSetSensorMagRange(uint32_t range) = 0;

    /*
    Function:
    - send command to sensor to start/stop sensor mag calibration
     You can set the time for the magnetometer calibration use commandSetSensorMagCalibrationTimeout(second) 
     [Default is 20 seconds]
    Parameters: none
    Returns: none
    */
    virtual void commandStartMagCalibration(void) = 0;
    virtual void commandStopMagCalibration(void) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor mag refrence
    - hasSettings() will return true once sensor info is available
    Parameters: none
    Returns: none
    */
    //virtual void commandGetMagRefrence(void) = 0;
    //virtual void commandSetMagRefrence(uint32_t reference) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor mag calibration time
    - hasSettings() will return true once sensor info is available
    Parameters: none
    Returns: none
    */
    virtual void commandGetSensorMagCalibrationTimeout(void) = 0;
    virtual void commandSetSensorMagCalibrationTimeout(float second) = 0;

    /*
    Function: 
        - send command to sensor to get sensor info
        - hasInfo() will return true once sensor info is available
    Parameters: none
    Returns: none
    */
    virtual void commandGetSensorInfo(void) = 0;

    /*
    Function:
    - send command to save sensor parameters to flash
    Parameters: none
    Returns: none
    */
    virtual void commandSaveParameters(void) = 0;

    /*
    Function:
    - send command to reset factory
    Parameters: none
    Returns: none
    */
    virtual void commandResetFactory(void) = 0;

    /*
    Function:
    - send command to set transmit data
    Parameters: as defined in transmit data register TDR
        #define TDR_XXX_ENABLED
    Returns: none
    */
    virtual void commandSetTransmitData(uint32_t config) = 0;

    // CAN
    /*
    Function:
    - send command to sensor to get/set sensor can id
    - hasSettings() will return true once sensor info is available
    Parameters:int32
    Returns: none
    */
    virtual void commandGetCanStartId(void) = 0;
    virtual void commandSetCanStartId(uint32_t data) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor can baudrate
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_CAN_BAUDRATE_125K                  125
        #define LPMS_CAN_BAUDRATE_250K                  250
        #define LPMS_CAN_BAUDRATE_500K                  500
        #define LPMS_CAN_BAUDRATE_800K                  800
        #define LPMS_CAN_BAUDRATE_1M                    1000
    Returns: none
    */
    virtual void commandGetCanBaudrate(void) = 0;
    virtual void commandSetCanBaudrate(uint32_t baudrate) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor can precision (16/32 bit)
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_CAN_DATA_PRECISION_FIXED_POINT     0
        #define LPMS_CAN_DATA_PRECISION_FLOATING_POINT  1   
    Returns: none
    */
    virtual void commandGetCanDataPrecision(void) = 0;
    virtual void commandSetCanDataPrecision(uint32_t data) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor can mode
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_CAN_MODE_CANOPEN                   0
        #define LPMS_CAN_MODE_SEQUENTIAL                1
    Returns: none
    */
    virtual void commandGetCanChannelMode(void) = 0;
    virtual void commandSetCanChannelMode(uint32_t data) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor can mapping
    - hasSettings() will return true once sensor info is available
    Parameters:
        each can channel is a (uint32_t)CanMapping.
    Returns: none
    */
    virtual void commandGetCanMapping(void) = 0;
    virtual void commandSetCanMapping(uint32_t map[16]) = 0;

    /*
    Function:
    - send command to sensor to get/set sensor can heartbeat freq
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_CAN_HEARTBEAT_005                  0
        #define LPMS_CAN_HEARTBEAT_010                  1
        #define LPMS_CAN_HEARTBEAT_020                  2
        #define LPMS_CAN_HEARTBEAT_050                  5
        #define LPMS_CAN_HEARTBEAT_100                  10
    Returns: none
    */
    virtual void commandGetCanHeartbeat(void) = 0;
    virtual void commandSetCanHeartbeat(uint32_t data) = 0;

    //Filter
    /*
    Function:
    - send command to sensor to get/set sensor filter mode
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_FILTER_GYR                 0
        #define LPMS_FILTER_KALMAN_GYR_ACC      1
        #define LPMS_FILTER_KALMAN_GYR_ACC_MAG  2
        #define LPMS_FILTER_DCM_GYR_ACC         3
        #define LPMS_FILTER_DCM_GYR_ACC_MAG     4
    Returns: none
    */
    virtual void commandGetFilterMode(void) = 0;
    virtual void commandSetFilterMode(uint32_t data) = 0;

    /*
    Function:
    - send command to sensor to enable/disable gyro auto calibraiton
    - hasSettings() will return true once sensor info is available
    Parameters: bool
    Returns: none
    */
    virtual void commandSetGyroAutoCalibration(bool enable) = 0;
    virtual void commandGetGyroAutoCalibration(void) = 0;

    //Offset Mode
    /*
    Function:
    - send command to sensor to set offset
    - hasSettings() will return true once sensor info is available
    Parameters: 
        #define LPMS_OFFSET_MODE_OBJECT         0
        #define LPMS_OFFSET_MODE_HEADING        1
        #define LPMS_OFFSET_MODE_ALIGNMENT      2
    Returns: none
    */
    virtual void commandSetOffsetMode(uint32_t data) = 0;
    virtual void commandResetOffsetMode(void) = 0;

    //GPS
    /*
    Function:
    - send command to sensor to save/clear gps flash
    - hasSettings() will return true once sensor info is available
    Parameters:
    Returns: none
    */
    virtual void commandSaveGPSState(void) = 0;
    virtual void commandClearGPSState(void) = 0;

    /*
    Function:
    - send command to sensor to get/set GPS transmit data.
    - hasSettings() will return true once sensor info is available
    Parameters:
    Returns: none
    */
    virtual void commandSetGpsTransmitData(uint32_t data, uint32_t data1) = 0;
    virtual void commandGetGpsTransmitData(void) = 0;

    // Uart
    /*
    Function:
    - send command to sensor to get/set uart baudrate
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_UART_BAUDRATE_xxxxx
    Returns: none
    */
    virtual void commandSetUartBaudRate(uint32_t data) = 0;
    virtual void commandGetUartBaudRate(void) = 0;
	
    /*
    Function:
    - send command to sensor to get/set uart data format
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_UART_DATA_FORMAT_LPBUS             0
        #define LPMS_UART_DATA_FORMAT_ASCII             1
    Returns: none
    */
    virtual void commandSetUartDataFormat(uint32_t data) = 0;
    virtual void commandGetUartDataFormat(void) = 0;

    /*
    Function:
    - send command to sensor to get/set uart data precision
    - hasSettings() will return true once sensor info is available
    Parameters:
        #define LPMS_UART_DATA_PRECISION_FIXED_POINT    0
        #define LPMS_UART_DATA_PRECISION_FLOATING_POINT 1
    Returns: none
    */
    virtual void commandSetUartDataPrecision(uint32_t data) = 0;
    virtual void commandGetUartDataPrecision(void) = 0;

    /*
    Function:
    - send command to sensor directly bypassing internal command queue
    Parameters: 
    - cmd: command register
    - length: length of data to send
    - data: pointer to data buffer
    Returns: none
    */
    virtual void sendCommand(uint16_t cmd, uint16_t length, uint8_t* data) = 0;


    /////////////////////////////////////////////
    // Sensor interface
    /////////////////////////////////////////////
    // General
    /*
    Function: enable auto reconnection
    Parameters:
    - true: enable
    - false: disable
    Returns: none
    */
    virtual void enableAutoReconnect(bool b) = 0;

    /*
    Function: get status of sensor
    Parameters:none
    Returns: as defined in STATUS_XXX
    #define STATUS_DISCONNECTED     0
    #define STATUS_CONNECTING       1
    #define STATUS_CONNECTED        2
    #define STATUS_CONNECTION_ERROR 3
    #define STATUS_DATA_TIMEOUT     4
    #define STATUS_UPDATING         5
    */
    virtual int getStatus(void) = 0;

    /*
    Function: get frequency of incoming data
    Parameters:none
    Returns: data frequency in Hz
    */
    virtual float getDataFrequency(void) = 0;

    // info
    /*
    Function: has new sensor info. consume once
    Parameters:none
    Returns: 
    - true: new sensor info available
    - false: no new sensor info available
    */
    virtual bool hasInfo(void) = 0;

    /*
    Function: get sensor info
    Parameters: reference IG1Info
    Returns: none
    */
    virtual void getInfo(IG1InfoI &info) = 0;

    // settings  
    /*
    Function: has new sensor settings. consume once
    Parameters:none
    Returns: 
    - true: new sensor settings available
    - false: no new sensor settings available
    */
    virtual bool hasSettings(void) = 0;

    /*
    Function: get sensor settings
    Parameters: reference IG1Settings
    Returns: none
    */
    virtual void getSettings(IG1SettingsI &settings) = 0;

    // response from sensor 
    /*
    Function: has feedback from sensor after command sent
    Parameters : none
    Returns : number of feedback in queue
    */
    virtual int hasResponse(void) = 0;

    /*
    Function: get feedback from internal response queue
    Parameters : string reference
    Returns :
    - true : feedback valid and available
    - false : no feedback available
    */
    virtual bool getResponse(std::string &s) = 0;

    // Imu data
    /*
    Function: has new imu data
    Parameters:none
    Returns: number of imu data available in queue
    */
    virtual int hasImuData(void) = 0;

    /*
    Function: get imu data from queue
    Parameters : IG1ImuData reference
    Returns :
    - true : data pop from queue
    - false :   queue is empty, return latest imu data
    */
    virtual bool getImuData(IG1ImuDataI &sd) = 0;

    // gps data
    /*
    Function: has new gps data
    Parameters:none
    Returns: number of gps data available in queue
    */
    virtual int hasGpsData() = 0;

    /*
    Function: get gps data from queue
    Parameters : IG1ImuData reference
    Returns :
    - true : data pop from queue
    - false : queue is empty, return latest gps data
    */
    virtual bool getGpsData(IG1GpsDataI &data) = 0;

    // Sensor data
    /*
    Function: is acc raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isAccRawEnabled(void) = 0;

    /*
    Function: is calibrated acc data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isAccCalibratedEnabled(void) = 0;

    /*
    Function: is gyro raw (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIRawEnabled(void) = 0;

    /*
    Function: is bias calibrated gyro (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIBiasCalibratedEnabled(void) = 0;

    /*
    Function: is alignment + bias calibrated gyro (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIAlignCalibratedEnabled(void) = 0;

    /*
    Function: is gyro raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIIRawEnabled(void) = 0;

    /*
    Function: is bias calibrated gyro data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIIBiasCalibratedEnabled(void) = 0;

    /*
    Function: is alignment + bias calibrated gyro data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isGyroIIAlignCalibratedEnabled(void) = 0;

    /*
    Function: is mag raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isMagRawEnabled(void) = 0;

    /*
    Function: is calibrated mag data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isMagCalibratedEnabled(void) = 0;

    /*
    Function: is angular velocity data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isAngularVelocityEnabled(void) = 0;

    /*
    Function: is quaternion data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isQuaternionEnabled(void) = 0;

    /*
    Function: is euler data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isEulerEnabled(void) = 0;

    /*
    Function: is linear acceleration data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isLinearAccelerationEnabled(void) = 0;

    /*
    Function: is pressure data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isPressureEnabled(void) = 0;

    /*
    Function: is altitude data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isAltitudeEnabled(void) = 0;

    /*
    Function: is temperature data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    virtual bool isTemperatureEnabled(void) = 0;

    /**
    * Get current data queue size (default: 10)
    */
    virtual int getSensorDataQueueSize(void) = 0;

    // Error 
    virtual std::string getLastErrMsg() = 0;
};

#ifdef _WIN32
    #ifdef DLL_EXPORT
        #define LPMS_IG1_API __declspec(dllexport)
    #else
        #define LPMS_IG1_API __declspec(dllimport)
    #endif

    extern "C" LPMS_IG1_API	IG1I* APIENTRY IG1Factory();

#else
    #define LPMS_IG1_API
    extern "C" LPMS_IG1_API IG1I* IG1Factory();
#endif

#endif