#include "LpmsIG1.h"

using namespace std;
#ifdef _WIN32
IG1I* APIENTRY IG1Factory()
{
    return new IG1();
}
#else
IG1I* IG1Factory()
{
    return new IG1();
}
#endif
/////////////////////////////////////////////
// Constructor/Destructor
/////////////////////////////////////////////
IG1::IG1()
{
#ifdef _WIN32
    portno = 0;
#else
    portno = "";
#endif
    baudrate = LPMS_UART_BAUDRATE_921600;

    connectionMode = Serial::MODE_VCP;
    init();
}

IG1::IG1(const IG1 &obj)
{
    //TODO
}

IG1::~IG1()
{
    //incomingDataRate = 0.01f;
    if (sensorStatus != STATUS_DISCONNECTED || 
        sensorStatus != STATUS_CONNECTION_ERROR)
        disconnect();
}

void IG1::init()
{
    // Internal thread
    isThreadRunning = false;
    isStopThread = true;
    autoReconnect = false;
    debugOutput = true;
    mmDataFreq.reset();
    mmDataIdle.reset();
    mmUpdating.reset();

    // Sensor
    sensorStatus = STATUS_DISCONNECTED;
    errMsg = "";
    timeoutThreshold = 5000000;
    memset(incomingData, 0, INCOMING_DATA_MAX_LENGTH);

    // stats
    incomingDataRate = 0;

    // command queue
    mLockCommandQueue.lock();
    while (!commandQueue.empty())
    {
        commandQueue.pop();
    }
    mLockCommandQueue.unlock();
    mmCommandTimer.reset();
    lastSendCommandTime = 0;

    // response
    mLockSensorResponseQueue.lock();
    while (!sensorResponseQueue.empty())
    {
        sensorResponseQueue.pop();
    }
    mLockSensorResponseQueue.unlock();

    // info
    hasNewInfo = false;
    sensorInfo.reset();


    // settings
    hasNewSettings = false;
    sensorSettings.reset();
    transmitDataRegisterStatus = TDR_INVALID;
    mmTransmitDataRegisterStatus.reset();

    // imu data
    sensorDataQueueSize = SENSOR_DATA_QUEUE_SIZE;
    mLockImuDataQueue.lock();
    while (!imuDataQueue.empty())
    {
        imuDataQueue.pop();
    }
    mLockImuDataQueue.unlock();
    latestImuData.reset();

    // gps data
    mLockGpsDataQueue.lock();
    while (!gpsDataQueue.empty())
    {
        gpsDataQueue.pop();
    }
    mLockGpsDataQueue.unlock();
    latestGpsData.reset();

    // firmware update
    sensorUpdating = false;
    ackReceived = false;
    nackReceived = false;
    dataReceived = false;
    firmwarePages = 0;
    firmwarePageSize = FIRMWARE_PACKET_LENGTH;
    firmwareRemainder = 0;
    updateCommand = 0;
    memset(cBuffer, 0, 1024);

    // // Data saving
    isDataSaving = false;
    savedImuDataBuffer.reserve(SAVE_DATA_LIMIT);
    savedImuDataCount = 0;
    savedImuDataBuffer.clear();

    savedGpsDataBuffer.reserve(SAVE_DATA_LIMIT);
    savedGpsDataCount = 0;
    savedGpsDataBuffer.clear();
}

/////////////////////////////////////
// Connection
/////////////////////////////////////
void IG1::setPCBaudrate(int baud)
{
    baudrate = baud;
    if (debugOutput) logd(TAG, "Baudrate: %d\n", baudrate);
}

#ifdef _WIN32
void IG1::setPCPort(int port)
#else
void IG1::setPCPort(string port)
#endif
{
    portno = port;
    if (debugOutput) logd(TAG, "COM: %d\n", baudrate);
}

#ifdef _WIN32
int IG1::connect(int _portno, int _baudrate)
#else
int IG1::connect(string _portno, int _baudrate)
#endif
{
    if (sensorStatus == STATUS_CONNECTING || 
        sensorStatus == STATUS_CONNECTED  ||
        sensorStatus == STATUS_UPDATING)
    {
        if (debugOutput)
            logd(TAG, "Another connection established\n");
    } 
    else 
    {
        init();
        connectionMode = Serial::MODE_VCP;
        portno = _portno;
        baudrate = _baudrate;
        t = new std::thread(&IG1::updateData, this);
        this_thread::sleep_for(chrono::milliseconds(100));
    }
    return sensorStatus;
}

#ifdef _WIN32
int IG1::connect(std::string sensorName ,int _baudrate)
{ 
    if (sensorStatus == STATUS_CONNECTING ||
        sensorStatus == STATUS_CONNECTED ||
        sensorStatus == STATUS_UPDATING)
    {
        if (debugOutput)
            logd(TAG, "Another connection established\n");
    }
    else
    {
        init();
        connectionMode = Serial::MODE_USBEXPRESS;
        sensorId = sensorName;
        baudrate = _baudrate;
        t = new std::thread(&IG1::updateData, this);
    }
    return sensorStatus;
}
#endif

bool IG1::disconnect()
{
    if (isStopThread)
        return true;

    //incomingDataRate = 0;
    isStopThread = true;
    if (t != NULL)
        t->join();
    t = NULL;
    if (sp.isConnected())
    {
        sp.close();
        if (debugOutput)
#ifdef _WIN32
            logd(TAG, "COM:%d disconnected\n", portno);
#else
            logd(TAG, "COM:%s disconnected\n", portno.c_str());
#endif
    }

    sensorStatus = STATUS_DISCONNECTED;
    return true;
}


/////////////////////////////////////
// Commands:
/////////////////////////////////////
void IG1::commandGotoCommandMode()
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_SENSOR_STATUS, WAIT_IGNORE));
}
void IG1::commandGotoStreamingMode()
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
    addCommandQueue(IG1Command(GET_SENSOR_STATUS, WAIT_IGNORE));
}

//Sensor ID
void IG1::commandGetSensorID(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_IMU_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorID(uint32_t id)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_IMU_ID;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = id;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_IMU_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

//Sensor Freq
void IG1::commandGetSensorFrequency(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_STREAM_FREQ, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorFrequency(uint32_t freq)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_STREAM_FREQ;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = freq;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_STREAM_FREQ, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorGyroRange(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_GYR_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorGyroRange(uint32_t range)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_GYR_RANGE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = range;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_GYR_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandStartGyroCalibration(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(START_GYR_CALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorAccRange(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_ACC_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorAccRange(uint32_t range)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_ACC_RANGE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = range;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_ACC_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorMagRange(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_MAG_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorMagRange(uint32_t range)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_MAG_RANGE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = range;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_MAG_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorUseRadianOutput(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_DEGRAD_OUTPUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorUseRadianOutput(bool b)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_DEGRAD_OUTPUT;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = b;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_DEGRAD_OUTPUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandStartMagCalibration(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(START_MAG_CALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandStopMagCalibration(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(STOP_MAG_CALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetMagRefrence(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_MAG_REFERENCE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetMagRefrence(uint32_t reference)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_MAG_REFERENCE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = reference;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_MAG_REFERENCE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorMagCalibrationTimeout(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_MAG_CALIBRATION_TIMEOUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetSensorMagCalibrationTimeout(float second)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_MAG_CALIBRATION_TIMEOUT;
    cmd1.dataLength = 4;
    cmd1.data.f[0] = second;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_MAG_CALIBRATION_TIMEOUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

//CAN bus
void IG1::commandGetCanStartId(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_START_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));

}
void IG1::commandSetCanStartId(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_CAN_START_ID;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_START_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetCanBaudrate(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));

}

void IG1::commandSetCanBaudrate(uint32_t baudrate)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_CAN_BAUDRATE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = baudrate;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}


void IG1::commandGetCanDataPrecision(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_DATA_PRECISION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetCanDataPrecision(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_CAN_DATA_PRECISION;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_DATA_PRECISION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetCanChannelMode(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_MODE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetCanChannelMode(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_CAN_MODE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_MODE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetCanMapping(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_MAPPING, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetCanMapping(uint32_t map[16])
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();

    IG1Command cmd1;
    cmd1.command = SET_CAN_MAPPING;
    cmd1.dataLength = 16*4;
    memcpy(cmd1.data.i, map, sizeof(map[0])*16);

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_MAPPING, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetCanHeartbeat(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_CAN_HEARTBEAT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetCanHeartbeat(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_CAN_HEARTBEAT;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_CAN_HEARTBEAT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetFilterMode(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_FILTER_MODE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_FILTER_MODE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetFilterMode(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_FILTER_MODE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetGyroAutoCalibration(bool enable)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_ENABLE_GYR_AUTOCALIBRATION;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = enable;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_ENABLE_GYR_AUTOCALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetGyroAutoCalibration(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_ENABLE_GYR_AUTOCALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetOffsetMode(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_ORIENTATION_OFFSET;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    //addCommandQueue(IG1Command(SET_ORIENTATION_OFFSET, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandResetOffsetMode(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(RESET_ORIENTATION_OFFSET, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSaveGPSState(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(SAVE_GPS_STATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandClearGPSState(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(CLEAR_GPS_STATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetGpsTransmitData(uint32_t data, uint32_t data1)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_GPS_TRANSMIT_DATA;
    cmd1.dataLength = 8;
    cmd1.data.i[0] = data;
    cmd1.data.i[1] = data1;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_GPS_TRANSMIT_DATA, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetGpsTransmitData(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_GPS_TRANSMIT_DATA, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetUartBaudRate(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_UART_BAUDRATE;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_UART_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetUartBaudRate(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_UART_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetUartDataFormat(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_UART_FORMAT;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_UART_FORMAT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetUartDataFormat(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_UART_FORMAT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetUartDataPrecision(uint32_t data)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_LPBUS_DATA_PRECISION;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = data;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_LPBUS_DATA_PRECISION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetUartDataPrecision(void)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_LPBUS_DATA_PRECISION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandGetSensorInfo()
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(GET_SERIAL_NUMBER, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_SENSOR_MODEL, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_FIRMWARE_INFO, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_FILTER_VERSION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_IAP_CHECKSTATUS, WAIT_IGNORE));

    addCommandQueue(IG1Command(GET_IMU_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_STREAM_FREQ, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_DEGRAD_OUTPUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_ACC_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_GYR_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_ENABLE_GYR_AUTOCALIBRATION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_GYR_THRESHOLD, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_MAG_RANGE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_MAG_CALIBRATION_TIMEOUT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_FILTER_MODE, WAIT_IGNORE));

    addCommandQueue(IG1Command(GET_CAN_START_ID, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_CAN_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_CAN_DATA_PRECISION, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_CAN_MODE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_CAN_MAPPING, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_CAN_HEARTBEAT, WAIT_IGNORE));

    addCommandQueue(IG1Command(GET_UART_BAUDRATE, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_UART_FORMAT, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_LPBUS_DATA_PRECISION, WAIT_IGNORE));

    addCommandQueue(IG1Command(GET_GPS_TRANSMIT_DATA, WAIT_IGNORE));
    addCommandQueue(IG1Command(GET_IMU_TRANSMIT_DATA, WAIT_FOR_TRANSMIT_DATA_REGISTER));

    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSaveParameters()
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(WRITE_REGISTERS));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandResetFactory()
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(IG1Command(RESTORE_FACTORY_VALUE));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::commandSetTransmitData(uint32_t config)
{
    if (sensorStatus != STATUS_CONNECTED)
        return;
    clearCommandQueue();
    IG1Command cmd1;
    cmd1.command = SET_IMU_TRANSMIT_DATA;
    cmd1.dataLength = 4;
    cmd1.data.i[0] = config;

    addCommandQueue(IG1Command(GOTO_COMMAND_MODE));
    addCommandQueue(cmd1);
    addCommandQueue(IG1Command(GET_IMU_TRANSMIT_DATA, WAIT_FOR_TRANSMIT_DATA_REGISTER));
    addCommandQueue(IG1Command(GOTO_STREAM_MODE));
}

void IG1::sendCommand(uint16_t cmd, uint16_t length, uint8_t* data)
{
    if (data == NULL)
        length = 0;

    uint8_t cmdBuffer[512];
    int idx = 0;
    cmdBuffer[idx++] = 0x3A;
    cmdBuffer[idx++] = 0x00;
    cmdBuffer[idx++] = 0x00;
    cmdBuffer[idx++] = cmd & 0xFF;
    cmdBuffer[idx++] = (cmd >> 8) & 0xFF;
    cmdBuffer[idx++] = length & 0xFF;
    cmdBuffer[idx++] = (length >> 8) & 0xFF;

    //checksum
    uint16_t txLrcCheck = cmd + length;

    if (data != NULL)
    {
        for (int i = 0; i < length; ++i) 
        {
            cmdBuffer[idx] = data[i];
            txLrcCheck += data[i];
            idx++;
        }
    }

    cmdBuffer[idx++] = txLrcCheck & 0xFF;
    cmdBuffer[idx++] = (txLrcCheck>>8) & 0xFF;
    cmdBuffer[idx++] = 0x0D;
    cmdBuffer[idx++] = 0x0A;

#ifdef _WIN32
    sp.writeData((const char*)cmdBuffer, idx);
#else
    sp.writeData(cmdBuffer, idx);
#endif
}

/////////////////////////////////////
// Sensor interface
/////////////////////////////////////
// General
void IG1::enableAutoReconnect(bool b)
{ 
    autoReconnect = b; 
};

int IG1::getStatus() 
{ 
    return sensorStatus; 
};

float IG1::getDataFrequency()
{
    if (incomingDataRate == 0.0f)
        return 0;
    return 1.0f / incomingDataRate;
}

// info
bool IG1::hasInfo()
{
    return hasNewInfo;
}

void IG1::getInfo(IG1InfoI &info)
{
    info = sensorInfo;
    hasNewInfo = false;
}

// settings
bool IG1::hasSettings()
{
    return hasNewSettings;
}

void IG1::getSettings(IG1SettingsI &settings)
{
    //memcpy(&settings.transmitDataConfig, &sensorSettings.transmitDataConfig, sizeof(settings));
    settings = sensorSettings;
    hasNewSettings = false;
}

// response from sensor
int IG1::hasResponse()
{
    return sensorResponseQueue.size();
}

bool IG1::getResponse(std::string &s)
{

    mLockSensorResponseQueue.lock();
    if (sensorResponseQueue.empty())
    {
        mLockSensorResponseQueue.unlock();
        return false;
    }
    s = sensorResponseQueue.front();
    sensorResponseQueue.pop();
    mLockSensorResponseQueue.unlock();

    return true;
}

// Imu data
int IG1::hasImuData()
{
    if (sensorStatus != STATUS_CONNECTED)
        return 0;
    return imuDataQueue.size();
}

bool IG1::getImuData(IG1ImuDataI &sd)
{
    mLockImuDataQueue.lock();
    if (imuDataQueue.empty())
    {
        sd = latestImuData;
        mLockImuDataQueue.unlock();
        return false;
    }
    sd = imuDataQueue.front();
    imuDataQueue.pop();
    mLockImuDataQueue.unlock();
    return true;
}

// gps data
int IG1::hasGpsData()
{
    if (sensorStatus != STATUS_CONNECTED)
        return 0;
    return gpsDataQueue.size();
}

bool IG1::getGpsData(IG1GpsDataI &data)
{

    mLockGpsDataQueue.lock();
    if (gpsDataQueue.empty())
    {
        data = latestGpsData;
        mLockGpsDataQueue.unlock();
        return false;
    }
    data = gpsDataQueue.front();
    gpsDataQueue.pop();
    mLockGpsDataQueue.unlock();
    return true;
}


// Sensor Data
bool IG1::isAccRawEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_ACC_RAW_OUTPUT_ENABLED)
        return true;
    return false;
}


bool IG1::isAccCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_ACC_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isGyroIRawEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR0_RAW_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isGyroIBiasCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR0_BIAS_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}


bool IG1::isGyroIAlignCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR0_ALIGN_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isGyroIIRawEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR1_RAW_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isGyroIIBiasCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR1_BIAS_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isGyroIIAlignCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}


bool IG1::isMagRawEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_MAG_RAW_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isMagCalibratedEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_MAG_CALIBRATED_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isAngularVelocityEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_ANGULAR_VELOCITY_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isQuaternionEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_QUAT_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isEulerEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_EULER_OUTPUT_ENABLED)
        return true;
    return false;
}


bool IG1::isLinearAccelerationEnabled() 
{
    if (sensorSettings.transmitDataConfig & TDR_LINACC_OUTPUT_ENABLED)
        return true;
    return false;
}
bool IG1::isPressureEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_PRESSURE_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isAltitudeEnabled()
{
    if (sensorSettings.transmitDataConfig & TDR_ALTITUDE_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isTemperatureEnabled() 
{
    if (sensorSettings.transmitDataConfig & TDR_TEMPERATURE_OUTPUT_ENABLED)
        return true;
    return false;
}

bool IG1::isUseRadianOutput()
{
    return sensorSettings.useRadianOutput;
}

void IG1::setSensorDataQueueSize(unsigned int size)
{
    sensorDataQueueSize = size;

    mLockImuDataQueue.lock();
    while (imuDataQueue.size() > sensorDataQueueSize)
    {
        imuDataQueue.pop();
    }
    mLockImuDataQueue.unlock();

    mLockGpsDataQueue.lock();
    while (gpsDataQueue.size() > sensorDataQueueSize)
    {
        gpsDataQueue.pop();
    }
    mLockGpsDataQueue.unlock();

}

int IG1::getSensorDataQueueSize()
{
    return sensorDataQueueSize;
}

/////////////////////////////////////////////
// Data saving
/////////////////////////////////////////////
bool IG1::startDataSaving()
{
    if (isDataSaving)
    {
        logd(TAG, "Data save action running\n");
        return false;
    }
    isDataSaving = true;

    // Clear imu data queue
    mLockSavedImuDataQueue.lock();
    savedImuDataBuffer.clear();
    mLockSavedImuDataQueue.unlock();
    savedImuDataCount = 0;

    // Clear gps data queue
    mLockSavedGpsDataQueue.lock();
    savedGpsDataBuffer.clear();
    mLockSavedGpsDataQueue.unlock();
    savedGpsDataCount = 0;
    return true;
}

bool IG1::stopDataSaving()
{
    isDataSaving = false;
    return true;
}

int IG1::getSavedImuDataCount()
{
    return savedImuDataCount;
}

IG1ImuData IG1::getSavedImuData(int i)
{
    IG1ImuData d;
    if (i > savedImuDataBuffer.size())
        return d;
    d = savedImuDataBuffer[i];
    return d;
}

int IG1::getSavedGpsDataCount()
{
    return savedGpsDataCount;
}

IG1GpsData IG1::getSavedGpsData(int i)
{
    IG1GpsData d;
    if (i > savedGpsDataBuffer.size())
        return d;
    d = savedGpsDataBuffer[i];
    return d;
}

/////////////////////////////////////////////////////
// Private
/////////////////////////////////////////////////////
void IG1::clearSensorDataQueue()
{
    mLockImuDataQueue.lock();
    while (!imuDataQueue.empty())
        imuDataQueue.pop();
    mLockImuDataQueue.unlock();
}

void IG1::updateData()
{
    int reconnectCount = 0;
    //checksumErrorCount = 0;
    do
    {

        transmitDataRegisterStatus = TDR_INVALID;
        sensorStatus = STATUS_CONNECTING;
        isStopThread = false;

        // Connect sensor
        if (connectionMode == Serial::MODE_VCP)
        {
            sp.setMode(Serial::MODE_VCP);
            if (sp.open(portno, baudrate))
            {
                if (debugOutput)
#ifdef _WIN32
                    logd(TAG, "COM:%d connection established\n", portno);
#else
                    logd(TAG, "COM:%s connection established\n", portno.c_str());
#endif
            }
            else
            {
                stringstream ss;
                errMsg = ss.str();
#ifdef _WIN32
                ss << "Error connecting to port: " << portno << "@" << baudrate;
#else
                ss << "Error connecting to port: " << portno.c_str() << "@" << baudrate;
#endif 
                errMsg = ss.str();
                sensorStatus = STATUS_CONNECTION_ERROR;
            }
        }
        else if (connectionMode == Serial::MODE_USB_EXPRESS)
        {
            sp.setMode(Serial::MODE_USB_EXPRESS);
            if (sp.open(sensorId, baudrate))
            {
                if (debugOutput)
                    logd(TAG, "Sensor:%s connection established\n", sensorId.c_str());
            }
            else
            {
                stringstream ss;
                ss << "Error connecting to sensor: " << sensorId << endl;
                errMsg = ss.str();
                sensorStatus = STATUS_CONNECTION_ERROR;
            }
        }
        else {
            sensorStatus = STATUS_CONNECTION_ERROR;
            cout << "Unknown connection mode" << endl;
        }

        if (sensorStatus != STATUS_CONNECTION_ERROR)
        {
            // Initialize 
            int count = 0;
            int readResult = 0;
            packet.rxState = PACKET_START;
            int TDRRetryCount = 0;
            clearSensorDataQueue();
            incomingDataRate = 0;
            isDataSaving = false;

            sensorStatus = STATUS_CONNECTED;
            mmDataFreq.reset();
            mmDataIdle.reset();
            mmCommandTimer.reset();

            commandGetSensorInfo();
            while (!isStopThread)
            {
                // read data
                if (!sp.isConnected())
                    break;

                // Process command queue
                // Send command
                mLockCommandQueue.lock();
                if (!commandQueue.empty() && mmCommandTimer.measure() > 100000)
                {
                    if (commandQueue.front().processed)
                        commandQueue.pop();
                    else
                    {
                        if (!commandQueue.front().sent) 
                        {
                            IG1Command cmd = commandQueue.front();
                            sendCommand(cmd.command, cmd.dataLength, cmd.data.c);
                            commandQueue.front().sent = true;
                            mmCommandTimer.reset();
                        } 
                        // Sent but no feedback
                        else if(commandQueue.front().expectedResponse == WAIT_IGNORE)
                        {
                            commandQueue.front().processed = true;
                        }
                    }
                }
                mLockCommandQueue.unlock();

                // Get transmit data config
                if (transmitDataRegisterStatus == TDR_INVALID) 
                {
                    logd(TAG, "Send get transmit data\n");
                    transmitDataRegisterStatus = TDR_UPDATING;
                    mmTransmitDataRegisterStatus.reset();
                    commandGetSensorInfo();
                }

                // Resend get transmit data if no response after 5 sec
                if (transmitDataRegisterStatus == TDR_UPDATING && 
                    mmTransmitDataRegisterStatus.measure() > 3000000)
                {
                    TDRRetryCount++;
                    logd(TAG, "Resend get transmit data: %d\n", TDRRetryCount);
                    transmitDataRegisterStatus = TDR_UPDATING;
                    mmTransmitDataRegisterStatus.reset();
                    commandGetSensorInfo();
                }

                if (TDRRetryCount > 5) 
                {
                    TDRRetryCount = 0;
                    transmitDataRegisterStatus = TDR_ERROR;
                    stringstream ss;
                    ss << "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] Error getting transmit data register\n";
                    addSensorResponseToQueue(ss.str());
                }



                // Read data
                readResult = sp.readData(incomingData, INCOMING_DATA_MAX_LENGTH);
                isThreadRunning = true;
                
                // parse data
                if (readResult > 0)
                {
                    if (readResult >= INCOMING_DATA_MAX_LENGTH)
                        logd(TAG, "data: %d\n", readResult);

                    mmDataIdle.reset();
                    //logd(TAG, "data: %d\n", readResult);
                    parseModbusByte(readResult);

                    if (sensorSettings.uartDataFormat == LPMS_UART_DATA_FORMAT_ASCII) {
                        parseASCII(readResult);
                    }
                }

                //
                if (sensorUpdating)
                {
                    if (nackReceived)
                    {
                        errMsg = "Error updating sensor\n";
                        logd(TAG, errMsg.c_str());
                        nackReceived = false;
                        sensorUpdating = false;
                    }
                    else if (ackReceived)
                    {
                        ackReceived = false;
                        mmUpdating.reset();
                        // handle firmwares

                        if (firmwarePages > 0)
                        {
                            if (ifs.is_open())
                            {
                                if (firmwarePages == 1 && fileCheckSum != 0)
                                {
                                    //Checksum Packet

                                    IG1Command checksumCmd;
                                    checksumCmd.command = updateCommand;
                                    checksumCmd.dataLength = 4;
                                    checksumCmd.data.i[0] = fileCheckSum;
                                    /*
                                    addCommandQueue(checksumCmd);
                                    */

                                    sendCommand(updateCommand, checksumCmd.dataLength, checksumCmd.data.c);
                                    logd(TAG, "Sensor firmware packet: %d\n", firmwarePages);
                                    logd(TAG, "Sensor firmware checksum: %d\n", fileCheckSum);
                                    firmwarePages--;
                                }
                                else
                                {

                                    for (int i = 0; i < firmwarePageSize; i++) cBuffer[i] = (char)0x00;
                                    try
                                    {
                                        ifs.read((char *)cBuffer, firmwarePageSize);
                                    }
                                    catch (std::ifstream::failure e) {
                                        std::cout << "error " << e.what() << "\n";
                                    }

                                    if (ifs.fail() == true)
                                    {
                                        //logd(TAG, "read failed\n");
                                        ifs.clear();
                                    }

                                    //IG1Command packetCommand;
                                    //packetCommand.command = updateCommand;
                                    //packetCommand.dataLength = firmwarePageSize;
                                    //memcpy(packetCommand.data.c, &cBuffer, firmwarePageSize);
                                    //addCommandQueue(packetCommand);

                                    sendCommand(updateCommand, firmwarePageSize, cBuffer);
                                    logd(TAG, "Sensor firmware packet: %d\n", firmwarePages);
                                    firmwarePages--;
                                }
                            }
                            else
                            {
                                errMsg = "Error updating sensor: error reading firmware/iap file\n";
                                logd(TAG, errMsg.c_str());
                                ackReceived = false;
                                sensorUpdating = false;
                                if (ifs.is_open())
                                    ifs.close();
                            }
                        }
                        else
                        {
                            if (firmwarePages < 0) {
                                logd(TAG, "File checksum done\n");
                                sensorUpdating = false;
                            }
                            if (firmwarePages == 0) {
                                logd(TAG, "Update file done\n");
                                ifs.close();
                                firmwarePages--;
                            }
                        }
                    }

                    else {
                        if (mmUpdating.measure() > timeoutThreshold - 2000000) // 3 secs no data
                        {
                            errMsg = "Updating timeout";
                            sensorUpdating = false;

                            if (ifs.is_open())
                                ifs.close();
                            //commandGotoStreamingMode();
                            sensorStatus = STATUS_DATA_TIMEOUT;
                        }
                    }
                }

                // Update sensor status
                if (mmDataIdle.measure() > timeoutThreshold) // 5 secs no data
                {
                    errMsg = "Data timeout";
                    //sensorStatus = STATUS_DATA_TIMEOUT;
                }
                else if (sensorUpdating)
                {
                    errMsg = "Updating";
                    sensorStatus = STATUS_UPDATING;
                }
                else
                {
                    //errMsg = "Connected";
                    sensorStatus = STATUS_CONNECTED;
                }


                //Sleep(1);
                this_thread::sleep_for(chrono::milliseconds(1));
            }
        }

        if (sp.isConnected())
        {
            sp.close();
        }
        if (debugOutput)
#ifdef _WIN32
            logd(TAG, "COM:%d disconnected\n", portno);
#else
            logd(TAG, "COM:%s disconnected\n", portno.c_str());
#endif
        // deinit
        sensorStatus = STATUS_CONNECTION_ERROR;
        isThreadRunning = false;
        //incomingDataRate = 0;

        if (autoReconnect && !isStopThread)
        {
            reconnectCount++;
            if (debugOutput)
                logd(TAG, "reconnecting %d\n",  reconnectCount);
            this_thread::sleep_for(chrono::milliseconds(1000));
        }
    } while (autoReconnect && !isStopThread);

    t = NULL;
}

bool IG1::parseModbusByte(int n)
{
    unsigned char b;
    for (int i = 0; i < n; ++i)
    {
        b = incomingData[i];
        //printf("%02X ", b);

        switch (packet.rxState) 
        {
        case PACKET_START:  
            if (b == BYTE_START)
            {
                packet.rxState = PACKET_ADDRESS0; 
                packet.rawDataIndex = 0;
            }
            break;

        case PACKET_ADDRESS0:
            packet.address = b;
            packet.rxState = PACKET_ADDRESS1;
            break;

        case PACKET_ADDRESS1:
            packet.address += ((unsigned)b * 256);
            packet.rxState = PACKET_FUNCTION0;
            break;

        case PACKET_FUNCTION0:
            packet.function = b;
            packet.rxState = PACKET_FUNCTION1;
            break;

        case PACKET_FUNCTION1:
            packet.function += ((unsigned)b * 256);
            packet.rxState = PACKET_LENGTH0;
            break;

        case PACKET_LENGTH0:
            packet.length = b;
            packet.rxState = PACKET_LENGTH1;
            break;

        case PACKET_LENGTH1:
            packet.length += ((unsigned)b * 256);
            if (packet.length > LPPACKET_MAX_BUFFER)
                packet.rxState = PACKET_START;
            else 
            {
                if (packet.length > 0)
                {
                    packet.rxState = PACKET_RAW_DATA;
                    packet.rawDataIndex = 0;
                }
                else
                {
                    packet.rxState = PACKET_LRC_CHECK0;
                }
            }
            break;

        case PACKET_RAW_DATA:
            if (packet.rawDataIndex < packet.length && packet.rawDataIndex < LPPACKET_MAX_BUFFER) 
            {
                packet.data[packet.rawDataIndex++] = b;
                if (packet.rawDataIndex == packet.length)
                    packet.rxState = PACKET_LRC_CHECK0;
            }
            else
                packet.rxState = PACKET_START;

            break;

        case PACKET_LRC_CHECK0:
            packet.chksum = b;
            packet.rxState = PACKET_LRC_CHECK1;
            break;

        case PACKET_LRC_CHECK1:
            packet.chksum += ((unsigned)b * 256);
            packet.rxState = PACKET_END0;
            break;

        case PACKET_END0:
            if (b == BYTE_END0)
                packet.rxState = PACKET_END1;
            else
                packet.rxState = PACKET_START;
            break;

        case PACKET_END1:
            if (b == BYTE_END1) 
            {
                uint16_t cs = packet.address + packet.length + packet.function;
                for (int j = 0; j < packet.length; ++j)
                    cs += packet.data[j];
                if (cs == packet.chksum)
                    parseSensorData(packet);
            }

            packet.rxState = PACKET_START;
            break;
        }
    }
    return true;
}

bool IG1::parseASCII(int n)
{
    unsigned char b;
    stringstream ss;
    ss.clear();
    for (int i = 0; i < n; ++i)
    {
        b = incomingData[i];
        ss << b;
    }
    addSensorResponseToQueue(ss.str());
    return false;
}

bool IG1::parseSensorData(const LPPacket &p)
{
    stringstream ss;
    string res("");
    string s("");
    //char binaryFormat[36] = { 0 };
    uint2char i2c;
    float2char f2c;

    switch (p.function)
    {
    /////////////////////////////////
    // Essentials
    /////////////////////////////////
    case REPLY_ACK:
        ackReceived = true;
        //logd(TAG, "received ack\n",);
        mLockCommandQueue.lock();
        if (!commandQueue.empty())
        {
            if (commandQueue.front().expectedResponse == WAIT_FOR_ACKNACK)
                commandQueue.front().processed = true;
        }
        mLockCommandQueue.unlock();
        res = "["+currentDateTime("%Y/%m/%d %H:%M:%S")+"] ACK";
        addSensorResponseToQueue(res);
        break;

    case REPLY_NACK:
        nackReceived = true;
        //logd(TAG, "received Nack\n");
        mLockCommandQueue.lock();
        if (!commandQueue.empty())
        {
            if (commandQueue.front().expectedResponse == WAIT_FOR_ACKNACK)
                commandQueue.front().processed = true;
        }
        mLockCommandQueue.unlock();
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] NACK";
        addSensorResponseToQueue(res);
        break;

    /////////////////////////////////
    // Sensor data
    /////////////////////////////////
    case GET_IMU_DATA:
    {
        if (transmitDataRegisterStatus != TDR_VALID) break;

        float dt = float(mmDataFreq.measure()) / 1000000.0f;
        //if (dt >0.0005)
        incomingDataRate = 0.995f*incomingDataRate + 0.005f * dt;
        mmDataFreq.reset();

        mLockImuDataQueue.lock();
        latestImuData.reset();

        if (sensorSettings.uartDataPrecision == LPMS_UART_DATA_PRECISION_FIXED_POINT) 
        {
           latestImuData.setData16bit(sensorSettings.useRadianOutput, sensorSettings.transmitDataConfig, packet.data);
        }
        else 
        {
           latestImuData.setData(sensorSettings.transmitDataConfig, packet.data, packet.length);
        }

        if (imuDataQueue.size() < sensorDataQueueSize) {
            imuDataQueue.push(latestImuData);
        }
        else
        {
            imuDataQueue.pop();
            imuDataQueue.push(latestImuData);
        }
        mLockImuDataQueue.unlock();

        // Data saving
        if (isDataSaving)
        {
            if (savedImuDataCount < SAVE_DATA_LIMIT)
            {
                mLockSavedImuDataQueue.lock();
                savedImuDataBuffer.push_back(latestImuData);
                mLockSavedImuDataQueue.unlock();
                savedImuDataCount++;
            }
        }

        break;
    }

    case GET_GPS_DATA:
    {
        
        if (transmitDataRegisterStatus != TDR_VALID) break;
        //memcpy(&latestGpsData, packet.data, packet.length);
        mLockGpsDataQueue.lock();
        latestGpsData.reset();

        latestGpsData.setData(sensorSettings.gpsTransmitDataConfig[0], sensorSettings.gpsTransmitDataConfig[1], packet.data);

        if (gpsDataQueue.size() < sensorDataQueueSize) {
            gpsDataQueue.push(latestGpsData);
        }
        else
        {
            gpsDataQueue.pop();
            gpsDataQueue.push(latestGpsData);
        }
        mLockGpsDataQueue.unlock();

        // Data saving
        if (isDataSaving)
        {
            if (savedGpsDataCount < SAVE_DATA_LIMIT)
            {
                mLockSavedGpsDataQueue.lock();
                savedGpsDataBuffer.push_back(latestGpsData);
                mLockSavedGpsDataQueue.unlock();
                savedGpsDataCount++;
            }
        }
        break;
    }

    /////////////////////////////////
    // Sensor info
    /////////////////////////////////
    case GET_SENSOR_MODEL:
        s.assign((char*)packet.data, packet.length);
        sensorInfo.deviceName = s;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET DEVICE NAME: " + sensorInfo.deviceName;
        addSensorResponseToQueue(res);
        hasNewInfo = true;
        break;

    case GET_FIRMWARE_INFO:
        s.assign((char*)packet.data, packet.length);
        sensorInfo.firmwareInfo = s;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET FIRMWARE INFO: " + sensorInfo.firmwareInfo;
        addSensorResponseToQueue(res);
        hasNewInfo = true;
        break;

    case GET_SERIAL_NUMBER:
        s.assign((char*)packet.data, packet.length);
        sensorInfo.serialNumber = s;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET SERIAL NUMBER: " + sensorInfo.serialNumber;
        addSensorResponseToQueue(res);
        hasNewInfo = true;
        break;

    case GET_FILTER_VERSION:
        s.assign((char*)packet.data, packet.length);
        sensorInfo.filterVersion = s;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET FILTER VERSION: " + sensorInfo.filterVersion;
        addSensorResponseToQueue(res);
        hasNewInfo = true;
        break;

    case GET_IAP_CHECKSTATUS:
        memcpy(i2c.c, packet.data, packet.length);
        sensorInfo.iapCheckStatus = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET IAP CHECKSTATUS: ";
        ss.clear();
        ss << res << sensorInfo.iapCheckStatus;
        addSensorResponseToQueue(res);
        hasNewInfo = true;
        break;

    /////////////////////////////////
    // OpenMAT ID
    /////////////////////////////////
    case GET_IMU_TRANSMIT_DATA:

        mLockCommandQueue.lock();
        if (!commandQueue.empty())
        {
            if (commandQueue.front().expectedResponse == WAIT_FOR_TRANSMIT_DATA_REGISTER)
                commandQueue.front().processed = true;
        }
        mLockCommandQueue.unlock();

        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.transmitDataConfig = i2c.int_val;

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET_IMU_TRANSMIT_DATA: ";
        ss.clear();
        ss << res << sensorSettings.uint32ToBinaryPP(sensorSettings.transmitDataConfig);
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        transmitDataRegisterStatus = TDR_VALID;
        break;

    case GET_IMU_ID:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.sensorId = i2c.int_val;

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET IMU ID: ";
        ss.clear();
        ss << res << sensorSettings.sensorId;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;


    case GET_STREAM_FREQ: {
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.dataStreamFrequency = i2c.int_val;

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET STREAM FREQ: ";
        ss.clear();
        ss << res << sensorSettings.dataStreamFrequency;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;
    }

    case GET_DEGRAD_OUTPUT:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.useRadianOutput = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET_DEGRAD_OUTPUT: ";
        ss.clear();
        ss << res << sensorSettings.useRadianOutput;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_GYR_THRESHOLD:
        memcpy(f2c.c, packet.data, packet.length);
        sensorSettings.gyroThreshold = f2c.float_val;

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET GYRO THRESHOLD: ";
        ss.clear();
        ss << res << sensorSettings.gyroThreshold;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    /////////////////////////////////
    // Filter parameters
    /////////////////////////////////
    case GET_FILTER_MODE:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.filterMode = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET FILTER MODE: ";
        ss.clear();
        ss << res << sensorSettings.filterMode;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_ENABLE_GYR_AUTOCALIBRATION:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.enableGyroAutocalibration = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET ENBALE GYRO AUTOCALIBRATION: ";
        ss.clear();
        ss << res << sensorSettings.enableGyroAutocalibration;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;
        
    /////////////////////////////////
    // Acc
    /////////////////////////////////
    case GET_ACC_RANGE:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.accRange = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET ACC RANGE: ";
        ss.clear();
        ss << res << sensorSettings.accRange;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    /////////////////////////////////
    // Gyro
    /////////////////////////////////
    case GET_GYR_RANGE:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.gyroRange = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET GYRO RANGE: ";
        ss.clear();
        ss << res << sensorSettings.gyroRange;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    /////////////////////////////////
    // Mag
    /////////////////////////////////
    case GET_MAG_RANGE:
        memcpy(i2c.c, packet.data, packet.length);
        sensorSettings.magRange = i2c.int_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET MAG RANGE: ";
        ss.clear();
        ss << res << sensorSettings.magRange;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_MAG_CALIBRATION_TIMEOUT:
        memcpy(f2c.c, packet.data, packet.length);
        sensorSettings.magCalibrationTimeout = f2c.float_val;
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET MAG CALIBRATION TIME OUT: ";
        ss.clear();
        ss << res << sensorSettings.magCalibrationTimeout;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;
    
    
    //////////////////////////////////////////
    // CAN
    //////////////////////////////////////////
    case GET_CAN_START_ID:
        memcpy(&sensorSettings.canStartId, packet.data, packet.length);
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN STARTID: ";
        ss.clear();
        ss << res << sensorSettings.canStartId;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_CAN_BAUDRATE:
        memcpy(&sensorSettings.canBaudrate, packet.data, packet.length);
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN BAUDRATE: ";
        ss.clear();
        ss << res << sensorSettings.canBaudrate;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_CAN_DATA_PRECISION:
        memcpy(&sensorSettings.canDataPrecision, packet.data, packet.length);
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN DATA PRECISION: ";
        ss.clear();
        ss << res << sensorSettings.canDataPrecision;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_CAN_MODE:
        memcpy(&sensorSettings.canMode, packet.data, packet.length);
        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN MODE: ";
        ss.clear();
        ss << res << sensorSettings.canMode;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_CAN_MAPPING:
        {
            memcpy(&sensorSettings.canMapping, packet.data, packet.length);

            res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN MAPPING: \n";
            ss.clear();
            ss << res;
            for (int i = 0; i < packet.length; ++i) {
                if ((i + 1) % 4 == 0)
                {
                    memcpy(i2c.c, packet.data + i - 3, 4);
                    ss << i / 4 + 1 << ": " << i2c.int_val;

                    if (packet.length - i > 1)
                    {
                        ss << "\n";
                    }
                }
            }

            addSensorResponseToQueue(ss.str());
            hasNewSettings = true;
            break;
        }

    case GET_CAN_HEARTBEAT:
        memcpy(&sensorSettings.canHeartbeatTime, packet.data, packet.length);

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET CAN HEARTBEAT: ";
        ss.clear();
        ss << res << sensorSettings.canHeartbeatTime;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;
    /////////////////////////////////
    // UART/RS232
    /////////////////////////////////
    case GET_UART_BAUDRATE:
        memcpy(&sensorSettings.uartBaudrate, packet.data, packet.length);

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET UART BAUDRATE: ";
        ss.clear();
        ss << res << sensorSettings.uartBaudrate;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    case GET_UART_FORMAT:
        memcpy(&sensorSettings.uartDataFormat, packet.data, packet.length);

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET UART FORMAT: ";
        ss.clear();
        ss << res << sensorSettings.uartDataFormat;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;


    case GET_LPBUS_DATA_PRECISION:
        memcpy(&sensorSettings.uartDataPrecision, packet.data, packet.length);

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET UART LPBUS DATA PRECISION: ";
        ss.clear();
        ss << res << sensorSettings.uartDataPrecision;
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;

    /////////////////////////////////
    // GPS parameters
    /////////////////////////////////
    case GET_GPS_TRANSMIT_DATA:
        memcpy(&sensorSettings.gpsTransmitDataConfig, packet.data, packet.length);

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] GET GPS TRANSMIT DATA: ";
        ss.clear();
        ss << res << sensorSettings.uint32ToBinaryPP(sensorSettings.gpsTransmitDataConfig[0]) << " || "
         << sensorSettings.uint32ToBinaryPP(sensorSettings.gpsTransmitDataConfig[1]);
        addSensorResponseToQueue(ss.str());
        hasNewSettings = true;
        break;


    default:
    {
        ss.clear();

        res = "[" + currentDateTime("%Y/%m/%d %H:%M:%S") + "] Got:\n";
        ss << res;
        ss << std::hex;
        for (int i = 0; i < packet.length; ++i)
        {
            ss << hex << uppercase << setfill('0') << setw(2) << (int)packet.data[i] << " ";

            if ((i + 1) % 4 == 0)
            {

                memcpy(i2c.c, packet.data + i - 3, 4);
                memcpy(f2c.c, packet.data + i - 3, 4);
                ss << dec << " (" << i2c.int_val << " | " << f2c.float_val << ")";

                if (packet.length - i > 1)
                {
                    ss << "\n";
                }
            }
        }
        addSensorResponseToQueue(ss.str());
        break;
    }

    }
    return true;
}

void IG1::addSensorResponseToQueue(string s)
{
    mLockSensorResponseQueue.lock();
    if (sensorResponseQueue.size() < SENSOR_RESPONSE_QUEUE_SIZE) {
        sensorResponseQueue.push(s);
    }
    else
    {
        sensorResponseQueue.pop();
        sensorResponseQueue.push(s);
    }
    mLockSensorResponseQueue.unlock();
}

void IG1::addCommandQueue(IG1Command cmd)
{
    mLockCommandQueue.lock();
    commandQueue.push(cmd);
    mLockCommandQueue.unlock();
}

void IG1::clearCommandQueue()
{
    mLockCommandQueue.lock();
    while (!commandQueue.empty())
        commandQueue.pop();
    mLockCommandQueue.unlock();
}