#include "SerialPort.h"
#include <algorithm>
using namespace std;

Serial::Serial():
    portNo(0),
    connected(false),
    usbMode(MODE_USB_EXPRESS)
{
}

Serial::~Serial()
{
    //Check if we are connected before trying to disconnect
    close();
}

bool Serial::open(int portno, int baudrate)
{
    if (isConnected())
        CloseHandle(this->hSerial);

    //if (usbMode == MODE_USB_EXPRESS)
    //    return open("0001");

    portNo = portno;
    stringstream ss;
    ss << "\\\\.\\COM" << portNo;
    string portName = ss.str();
    //We're not yet connected
    this->connected = false;

    //Try to connect to the given port throuh CreateFile
    this->hSerial = CreateFile(portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        0,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL);

    //Check if the connection was successfull
    if (this->hSerial == INVALID_HANDLE_VALUE)
    {
        printf("[Serial] ERROR: Invalid handle value\n");        
        CloseHandle(this->hSerial);
        return false;
    }
    else
    {
        char mode_str[128];
        switch (baudrate)
        {
        case BAUDRATE_4800:
            strcpy(mode_str, "baud=4800");
            break;
        case BAUDRATE_9600:
            strcpy(mode_str, "baud=9600");
            break;
        case BAUDRATE_19200:
            strcpy(mode_str, "baud=19200");
            break;
        case BAUDRATE_38400:
            strcpy(mode_str, "baud=38400");
            break;
        case BAUDRATE_57600:
            strcpy(mode_str, "baud=57600");
            break;
        case BAUDRATE_115200:
            strcpy(mode_str, "baud=115200");
            break;
        case BAUDRATE_230400:
            strcpy(mode_str, "baud=230400");
            break;
        case BAUDRATE_256000:
            strcpy(mode_str, "baud=256000");
            break;
        case BAUDRATE_460800:
            strcpy(mode_str, "baud=460800");
            break;
        case BAUDRATE_921600:
            strcpy(mode_str, "baud=921600");
            break;
        }
        strcat(mode_str, " data=8");
        strcat(mode_str, " parity=n");
        strcat(mode_str, " stop=1");
        strcat(mode_str, " dtr=on rts=on");
        DCB port_settings;
        memset(&port_settings, 0, sizeof(port_settings));  /* clear the new struct  */
        port_settings.DCBlength = sizeof(port_settings);

        if (!BuildCommDCBA(mode_str, &port_settings))
        {
            printf("unable to set comport dcb settings\n");
            CloseHandle(this->hSerial);
            return false;
        }
        if (!BuildCommDCBA(mode_str, &port_settings))
        {
            printf("unable to set comport dcb settings\n");
            CloseHandle(this->hSerial);
            return false;
        }

        if (!SetCommState(this->hSerial, &port_settings))
        {
            printf("unable to set comport cfg settings\n");
            CloseHandle(this->hSerial);
            return false;
        }

        COMMTIMEOUTS Cptimeouts;

        Cptimeouts.ReadIntervalTimeout = MAXDWORD;
        Cptimeouts.ReadTotalTimeoutMultiplier = 0;
        Cptimeouts.ReadTotalTimeoutConstant = 0;
        Cptimeouts.WriteTotalTimeoutMultiplier = 100;
        Cptimeouts.WriteTotalTimeoutConstant = 100;

        if (!SetCommTimeouts(this->hSerial, &Cptimeouts))
        {
            printf("unable to set comport time-out settings\n");
            CloseHandle(this->hSerial);
            return false;
        }
    }

    this->connected = true;

    return true;
}

bool Serial::open(std::string deviceId, int baudrate)
{
    bool f;

    transform(deviceId.begin(), deviceId.end(), deviceId.begin(), ::toupper);
    this->idNumber = deviceId;
    std::cout << "[LpmsU] Lpms::connect  deviceID is : " << deviceId << std::endl;
    f = false;

    SI_STATUS siStatus;
    DWORD devercNum = 0;
    SI_DEVICE_STRING	DeviceString;

    this->idNumber = deviceId;

    DWORD numDevs;
    if (SI_GetNumDevices(&numDevs) != SI_SUCCESS)
        return false;
    devercNum = numDevs;
    for (DWORD d = 0; d < numDevs; d++)
    {
        siStatus = SI_GetProductString(d, DeviceString, SI_RETURN_SERIAL_NUMBER);
        if (siStatus == SI_SUCCESS)
        {
            string deviceString = string(DeviceString);

            transform(deviceString.begin(), deviceString.end(), deviceString.begin(), ::toupper);
            if (deviceId == deviceString)
            {
                devercNum = d;
                break;
            }
        }
    }

    siStatus = SI_Open(devercNum, &cyHandle);
    siStatus = SI_SetBaudRate(cyHandle, baudrate);
    siStatus = SI_SetFlowControl(cyHandle, SI_HANDSHAKE_LINE, SI_FIRMWARE_CONTROLLED, SI_HELD_INACTIVE, SI_STATUS_INPUT, SI_STATUS_INPUT, 0);
    if (siStatus == SI_SUCCESS) {
        std::cout << "[LpmsU] SI_Open  device is successul : " << deviceId << std::endl;

        this->connected = true;
        SI_FlushBuffers(cyHandle, TRUE, TRUE);
    }
    else {
        std::cout << "[LpmsU] SI_Open  device is failed : " << deviceId << std::endl;

        this->connected = false;
    }

    return this->connected ;
}

void Serial::setMode(int mode)
{
    usbMode = mode;
}

int Serial::getMode(void)
{
    return usbMode;
}

bool Serial::close()
{
    if (this->connected)
    {
        //We're no longer connected
        this->connected = false;
        //Close the serial handler
        if (usbMode == MODE_USB_EXPRESS)
        {
            SI_Close(cyHandle);

            cout << "[LpmsU] Connection to " << idNumber << " closed." << endl;
        }
        else
        {
            return CloseHandle(this->hSerial);
        }

    }

    return true;
}

int Serial::readData(unsigned char *buffer, unsigned int nbChar)
{
    if (!isConnected())
        return 0;

    if (usbMode == MODE_USB_EXPRESS)
    {
        SI_STATUS siStatus;
        PBYTE ModemStatus;

#ifdef _WIN32
        unsigned long eventDWord;
        unsigned long txBytes;
        unsigned long rxBytes;
        unsigned long qStatus;
#else
        unsigned int eventDWord;
        unsigned int txBytes;
        unsigned int rxBytes;
#endif

        bool f = true;

        unsigned long bytesReceived = 0;

        if (this->connected == false) return 0;

        siStatus = SI_CheckRXQueue(cyHandle, &rxBytes, &qStatus);

        if (siStatus != SI_SUCCESS)
        {
            std::cout << "USB read error\n";
            return false;
        }

#ifdef _WIN32
        siStatus = SI_Read(cyHandle, buffer, rxBytes, &bytesReceived);
#else
        siStatus = SI_Read(cyHandle, buffer, rxBytes, (unsigned int *)bytesReceived);
#endif


        return bytesReceived;
    }
    else 
    {
        //Number of bytes we'll have read
        DWORD bytesRead;
        //Number of bytes we'll really ask to read
        unsigned int toRead;

        //int n;
        //ReadFile(this->hSerial, buffer, nbChar, (LPDWORD)((void *)&n), NULL);
        //return n;
        //Use the ClearCommError function to get status info on the Serial port
        ClearCommError(this->hSerial, &this->errors, &this->status);

        //Check if there is something to read
        if (this->status.cbInQue>0)
        {
            //If there is we check if there is enough data to read the required number
            //of characters, if not we'll read only the available characters to prevent
            //locking of the application.
            if (this->status.cbInQue>nbChar)
            {
                toRead = nbChar;
            }
            else
            {
                toRead = this->status.cbInQue;
            }

            //Try to read the require number of chars, and return the number of read bytes on success
            if (ReadFile(this->hSerial, buffer, toRead, &bytesRead, NULL))
            {
                return bytesRead;
            }

        }

        //If nothing has been read, or that an error was detected return -1
    }
    return 0;

}


bool Serial::writeData(const char *buffer, unsigned int nbChar)
{
    if (!isConnected())
    {
        if (usbMode == MODE_USB_EXPRESS)
            std::cout << "Sensor " << idNumber << " not connected\n";
        else
            std::cout << "COM "<< portNo << " not connected\n";
        return false;
    }

    if (usbMode == MODE_USB_EXPRESS)
    {
        SI_STATUS siStatus;

#ifdef _WIN32
        unsigned long bytesWritten;
#else
        unsigned int bytesWritten;
#endif

        bool f = false;

        if (this->connected == false) return false;

        siStatus = SI_Write(cyHandle, (unsigned char *)buffer, nbChar, &bytesWritten);

        if (siStatus == SI_SUCCESS) {
            f = true;
        }else{
            f = false;
            std::cout << "Write error\n";
        }
        return f;
    }
    else
    {
        DWORD bytesSend;
        //Try to write the buffer on the Serial port
        if (!WriteFile(this->hSerial, (void *)buffer, nbChar, &bytesSend, 0))
        {
            std::cout << "Write error\n";
            //In case it don't work get comm error and return false
            ClearCommError(this->hSerial, &this->errors, &this->status);

            return false;
        }
        else
        {
            return true;
        }
    }

}

bool Serial::isConnected()
{
    //Simply return the connection status
    return this->connected;
}