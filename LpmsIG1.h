#ifndef LPMSIG1_H
#define LPMSIG1_H

#include <string>
#include <cstring>
#include <thread>
#include <iostream>
#include <fstream>
#ifdef _WIN32
#include <comutil.h>
#endif
#include <mutex>
#include <queue>
#include <vector>
#include <cstdio>
#include <sstream>
#include <iomanip>
#include "util.h"

#include "SerialPort.h"
#include "MicroMeasure.h"
#include "SensorData.h"
#include "LpmsIG1Registers.h"

#include "LpmsIG1I.h"

///////////////////////////////////////
// LPMod bus
///////////////////////////////////////
#define LPPACKET_MAX_BUFFER 512
enum {
    PACKET_START,
    PACKET_ADDRESS0,
    PACKET_ADDRESS1,
    PACKET_FUNCTION0,
    PACKET_FUNCTION1,
    PACKET_LENGTH0,
    PACKET_LENGTH1,
    PACKET_RAW_DATA,
    PACKET_LRC_CHECK0,
    PACKET_LRC_CHECK1,
    PACKET_END0,
    PACKET_END1
};
struct LPPacket
{
    int rxState;
    uint16_t address;
    uint16_t length;
    uint16_t function;
    uint8_t data[LPPACKET_MAX_BUFFER];
    uint16_t rawDataIndex;
    uint16_t chksum;


    LPPacket()
    {
        rxState = 0;
        address = 0;
        length = 0;
        function = 0;
        rawDataIndex = 0;
        chksum = 0;
        memset(data, 0, LPPACKET_MAX_BUFFER);
    }

};

#define BYTE_START              0x3A
#define BYTE_END0               0x0D
#define BYTE_END1               0x0A

#define TDR_INVALID                                 0
#define TDR_VALID                                   1
#define TDR_UPDATING                                2
#define TDR_ERROR                                   3

// Others
#define SAVE_DATA_LIMIT             720000 //2 hours at 100hz  
#define SENSOR_DATA_QUEUE_SIZE      10
#define SENSOR_RESPONSE_QUEUE_SIZE  50
#define FIRMWARE_PACKET_LENGTH      256

// sensor response logic
#define WAIT_IGNORE                     0
#define WAIT_FOR_ACKNACK                1
#define WAIT_FOR_DATA                   2
#define WAIT_FOR_TRANSMIT_DATA_REGISTER 3
#define WAIT_FOR_CALIBRATION_DATA       4
#define WAIT_FOR_SENSOR_SETTINGS        5

struct IG1Command
{
    short command;
    union Data {
        uint32_t i[64];
        float f[64];
        unsigned char c[256];
    } data;
    int dataLength;

    int expectedResponse;
    int retryCount;
    bool sent;
    bool processed;

    IG1Command(short cmd = 0, int response = WAIT_FOR_ACKNACK)
    {
        command = cmd;
        dataLength = 0;
        memset(data.c, 0, 256);
        expectedResponse = response;
        retryCount = 0;
        processed = false;
        sent = false;
    }

    void setData(const LpVector3f &d)
    {
        dataLength = 12;
        memcpy(data.c, &d, dataLength);
    }

    void setData(const LpMatrix3x3f &d)
    {
        dataLength = 36;
        memcpy(data.c, &d, dataLength);
    }

};

class IG1 : public IG1I
{
    const static int INCOMING_DATA_MAX_LENGTH = 2048;
public:
    /////////////////////////////////////////////
    // Constructor/Destructor
    /////////////////////////////////////////////
    IG1(void);
    //IG1(int portno, int baudrate);
    IG1(const IG1 &obj);
    ~IG1(void);
    void init();
    void release() { delete this; };

    /////////////////////////////////////////////
    // Connection
    /////////////////////////////////////////////
    void setPCBaudrate(int baud);
#ifdef _WIN32
    void setPCPort(int port);
    int connect(int _portno, int _baudrate);
    int connect(std::string sensorName, int _baudrate);
#else
    void setPCPort(std::string port);
    int connect(std::string _portno, int _baudrate);
    //int connect(std::string sensorName);
#endif

    bool disconnect();
    void setConnectionMode(int);

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
    void commandGotoCommandMode(void);

    /*
    Function: set sensor to streaming mode
    Parameters: none
    Returns: none
    */
    void commandGotoStreamingMode(void);

	/*
	Function:
	- send command to sensor to get sensor ID
	Parameters: none
	Returns: int32
	*/
	void commandGetSensorID(void);

	/*
	Function:
	- send command to sensor to set sensor ID
	Parameters: int32
	Returns: none
	*/
	void commandSetSensorID(uint32_t id);

	/*
	Function:
	- send command to sensor to get sensor Frenquency
	- hasInfo() will return true once sensor info is available
	Parameters: none
	Returns: none
	*/
	void commandGetSensorFrequency(void);

	/*
	Function:
	- send command to sensor to set sensor Frenquency
	Parameters: int32
	Returns: none
	*/
	void commandSetSensorFrequency(uint32_t freq);


	void commandGetSensorGyroRange(void);
	void commandSetSensorGyroRange(uint32_t range);

    void commandStartGyroCalibration(void);

	void commandGetSensorAccRange(void);
	void commandSetSensorAccRange(uint32_t range);

	void commandGetSensorMagRange(void);
	void commandSetSensorMagRange(uint32_t range);

    void commandGetSensorUseRadianOutput(void);
    void commandSetSensorUseRadianOutput(bool b);

    void commandStartMagCalibration(void);
    void commandStopMagCalibration(void);

    void commandGetMagRefrence(void);
    void commandSetMagRefrence(uint32_t reference);

    void commandGetSensorMagCalibrationTimeout(void);
    void commandSetSensorMagCalibrationTimeout(float second);
    /*
    Function: 
        - send command to sensor to get sensor info
        - hasInfo() will return true once sensor info is available
    Parameters: none
    Returns: none
    */
    void commandGetSensorInfo(void);

    void commandSaveParameters(void);

    void commandResetFactory(void);

    /*
    Function:
    - send command to set transmit data
    Parameters: as defined in transmit data register TDR
        #define TDR_XXX_ENABLED
    Returns: none
    */
    void commandSetTransmitData(uint32_t config);

    // CAN
    void commandGetCanStartId(void);
    void commandSetCanStartId(uint32_t data);

    void commandGetCanBaudrate(void);
    void commandSetCanBaudrate(uint32_t baudrate);

    void commandGetCanDataPrecision(void);
    void commandSetCanDataPrecision(uint32_t data);

    void commandGetCanChannelMode(void);
    void commandSetCanChannelMode(uint32_t data);

    void commandGetCanMapping(void);
    void commandSetCanMapping(uint32_t map[16]);

    void commandGetCanHeartbeat(void);
    void commandSetCanHeartbeat(uint32_t data);

    //Filter
    void commandSetFilterMode(uint32_t data);
    void commandGetFilterMode(void);

    void commandSetGyroAutoCalibration(bool enable);
    void commandGetGyroAutoCalibration(void);

    void commandSetOffsetMode(uint32_t data);
    void commandResetOffsetMode(void);

    //GPS
    void commandSaveGPSState(void);
    void commandClearGPSState(void);

    void commandSetGpsTransmitData(uint32_t data, uint32_t data1);
    void commandGetGpsTransmitData(void);

    // Uart
    void commandSetUartBaudRate(uint32_t data);
    void commandGetUartBaudRate(void);
	
    void commandSetUartDataFormat(uint32_t data);
    void commandGetUartDataFormat(void);

    void commandSetUartDataPrecision(uint32_t data);
    void commandGetUartDataPrecision(void);

    /*
    Function:
    - send command to sensor directly bypassing internal command queue
    Parameters: 
    - cmd: command register
    - length: length of data to send
    - data: pointer to data buffer
    Returns: none
    */
    void sendCommand(uint16_t cmd, uint16_t length, uint8_t* data);


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
    void enableAutoReconnect(bool b);

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
    int getStatus(void);

    /*
    Function: get frequency of incoming data
    Parameters:none
    Returns: data frequency in Hz
    */
    float getDataFrequency(void);

    // info
    /*
    Function: has new sensor info. consume once
    Parameters:none
    Returns: 
    - true: new sensor info available
    - false: no new sensor info available
    */
    bool hasInfo(void);

    /*
    Function: get sensor info
    Parameters: reference IG1Info
    Returns: none
    */
    void getInfo(IG1InfoI &info);

    // settings  
    /*
    Function: has new sensor settings. consume once
    Parameters:none
    Returns: 
    - true: new sensor settings available
    - false: no new sensor settings available
    */
    bool hasSettings(void);

    /*
    Function: get sensor settings
    Parameters: reference IG1Settings
    Returns: none
    */
    void getSettings(IG1SettingsI &settings);

    // response from sensor 
    /*
    Function: has feedback from sensor after command sent
    Parameters : none
    Returns : number of feedback in queue
    */
    int hasResponse(void);

    /*
    Function: get feedback from internal response queue
    Parameters : string reference
    Returns :
    - true : feedback valid and available
    - false : no feedback available
    */
    bool getResponse(std::string &s);

    // Imu data
    /*
    Function: has new imu data
    Parameters:none
    Returns: number of imu data available in queue
    */
    int hasImuData(void);

    /*
    Function: get imu data from queue
    Parameters : IG1ImuData reference
    Returns :
    - true : data pop from queue
    - false :   queue is empty, return latest imu data
    */
    bool getImuData(IG1ImuDataI &sd);

    // gps data
    /*
    Function: has new gps data
    Parameters:none
    Returns: number of gps data available in queue
    */
    int hasGpsData();

    /*
    Function: get gps data from queue
    Parameters : IG1ImuData reference
    Returns :
    - true : data pop from queue
    - false : queue is empty, return latest gps data
    */
    bool getGpsData(IG1GpsDataI &data);

    // Sensor data
    /*
    Function: is acc raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isAccRawEnabled(void);

    /*
    Function: is calibrated acc data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isAccCalibratedEnabled(void);

    /*
    Function: is gyro raw (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIRawEnabled(void);

    /*
    Function: is bias calibrated gyro (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIBiasCalibratedEnabled(void);

    /*
    Function: is alignment + bias calibrated gyro (high accuracy) data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIAlignCalibratedEnabled(void);

    /*
    Function: is gyro raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIIRawEnabled(void);

    /*
    Function: is bias calibrated gyro data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIIBiasCalibratedEnabled(void);

    /*
    Function: is alignment + bias calibrated gyro data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isGyroIIAlignCalibratedEnabled(void);

    /*
    Function: is mag raw data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isMagRawEnabled(void);

    /*
    Function: is calibrated mag data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isMagCalibratedEnabled(void);

    /*
    Function: is angular velocity data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isAngularVelocityEnabled(void);

    /*
    Function: is quaternion data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isQuaternionEnabled(void);

    /*
    Function: is euler data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isEulerEnabled(void);

    /*
    Function: is linear acceleration data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isLinearAccelerationEnabled(void);

    /*
    Function: is pressure data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isPressureEnabled(void);

    /*
    Function: is altitude data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isAltitudeEnabled(void);

    /*
    Function: is temperature data enabled
    Parameters: void
    Returns:
    - true: enabled
    - false: disabled
    */
    bool isTemperatureEnabled(void);
  
    /*
    Function: is useRadianOutput
    Parameters: void
    Returns:
    - true: use radian output
    - false: use degree output
    */
    bool isUseRadianOutput(void);

    void setSensorDataQueueSize(unsigned int size);
    int getSensorDataQueueSize(void);

    int getFilePages() { return firmwarePages; };
    bool getIsUpdatingStatus() { return sensorUpdating; };

    // Error 
    std::string getLastErrMsg() { return errMsg; };

    ///////////////////////////////////////////
    //Data saving
    ///////////////////////////////////////////
    bool startDataSaving();
    bool stopDataSaving();
    
    // imu
    int getSavedImuDataCount();
    IG1ImuData getSavedImuData(int i);
    
    // gps
    int getSavedGpsDataCount();
    IG1GpsData getSavedGpsData(int i);

private:
    void clearSensorDataQueue();
    void updateData();
    bool parseModbusByte(int n);
    bool parseASCII(int n);
    bool parseSensorData(const LPPacket &p);
    void addSensorResponseToQueue(std::string);

    void addCommandQueue(IG1Command cmd); 
    void clearCommandQueue();

private:

    // serial uart
    Serial sp;
#ifdef _WIN32
    int portno;
#else
    std::string portno;
#endif
    int baudrate;
    std::string sensorId;
    int connectionMode;

    // LPBus
    LPPacket packet;

    // Internal thread
    std::thread *t;
    bool isThreadRunning;
    bool isStopThread;
    bool autoReconnect;
    bool debugOutput;
    MicroMeasure mmDataFreq;	
    MicroMeasure mmDataIdle;
    MicroMeasure mmUpdating;

    // Sensor
    int sensorStatus;
    std::string errMsg;
    long long timeoutThreshold;
    unsigned char incomingData[INCOMING_DATA_MAX_LENGTH];

    // Stats
    float incomingDataRate;
    unsigned int lastTimestamp;

    // command queue
    std::mutex mLockCommandQueue;
    std::queue<IG1Command> commandQueue; //TODO use custom object to accomodate command data
    MicroMeasure mmCommandTimer;
    long long lastSendCommandTime;

    // response
    std::mutex mLockSensorResponseQueue;
    std::queue<std::string> sensorResponseQueue;

    // info
    bool hasNewInfo;
    IG1Info sensorInfo;

    // settings
    bool hasNewSettings;
    IG1AdvancedSettings sensorSettings;
    int transmitDataRegisterStatus;   // TDR_INVALID: TDR_VALID : TDR_UPDATING
    MicroMeasure mmTransmitDataRegisterStatus;

    // imu data
    IG1ImuData latestImuData;
    std::mutex mLockImuDataQueue;
    std::queue<IG1ImuData> imuDataQueue;
    unsigned int sensorDataQueueSize;

    // Gps data
    IG1GpsData latestGpsData;
    std::mutex mLockGpsDataQueue;
    std::queue<IG1GpsData> gpsDataQueue;
    
    // firmware update
    std::ifstream ifs;
    bool sensorUpdating;
    bool ackReceived;
    bool nackReceived;
    bool dataReceived;
    long long firmwarePages;
    int firmwarePageSize;
    unsigned long firmwareRemainder;
    uint32_t fileCheckSum;
    uint16_t updateCommand;
    unsigned char cBuffer[1024];

    // Save data
    // Imu
    bool isDataSaving;
    std::mutex mLockSavedImuDataQueue;
    std::vector<IG1ImuData> savedImuDataBuffer;
    int savedImuDataCount;

    // GPS
    std::mutex mLockSavedGpsDataQueue;
    std::vector<IG1GpsData> savedGpsDataBuffer;
    int savedGpsDataCount;


    // debug
    MicroMeasure mmDebug;
    
};


#endif
