#ifndef LPMSIG1_REGISTERS_H
#define LPMSIG1_REGISTERS_H


/////////////////////////////////////////////////////////////////////
// Essentials - should not change over time
/////////////////////////////////////////////////////////////////////
// Commands start address
#define COMMAND_START_ADDRESS               0

// Acknowledged and Not-acknowledged identifier
#define REPLY_ACK                           (COMMAND_START_ADDRESS + 0)
#define REPLY_NACK                          (COMMAND_START_ADDRESS + 1)

// Firmware update and in-application-programmer upload
#define UPDATE_FIRMWARE                     (COMMAND_START_ADDRESS + 2)
#define UPDATE_IAP                          (COMMAND_START_ADDRESS + 3)

// Register value save and reset
#define WRITE_REGISTERS                     (COMMAND_START_ADDRESS + 4)
#define RESTORE_FACTORY_VALUE               (COMMAND_START_ADDRESS + 5)

// Mode switching
#define GOTO_COMMAND_MODE                   (COMMAND_START_ADDRESS + 6)
#define GOTO_STREAM_MODE                    (COMMAND_START_ADDRESS + 7)

// Sensor status
#define GET_SENSOR_STATUS                   (COMMAND_START_ADDRESS + 8)     // streaming/command

// Data
#define GET_IMU_DATA                        (COMMAND_START_ADDRESS + 9)     // Todo
#define GET_GPS_DATA                        (COMMAND_START_ADDRESS + 10)    // Todo

/////////////////////////////////////////////////////////////////////
// Sensor Info
/////////////////////////////////////////////////////////////////////
#define GET_SENSOR_MODEL                    (COMMAND_START_ADDRESS + 20)
#define GET_FIRMWARE_INFO                   (COMMAND_START_ADDRESS + 21)
#define GET_SERIAL_NUMBER                   (COMMAND_START_ADDRESS + 22) 
#define GET_FILTER_VERSION                  (COMMAND_START_ADDRESS + 23)
#define GET_IAP_CHECKSTATUS                 (COMMAND_START_ADDRESS + 24)

/////////////////////////////////////////////////////////////////////
// General
/////////////////////////////////////////////////////////////////////

// Transmit data
#define SET_IMU_TRANSMIT_DATA               (COMMAND_START_ADDRESS + 30)
#define GET_IMU_TRANSMIT_DATA               (COMMAND_START_ADDRESS + 31)

// IMU ID setting
#define SET_IMU_ID                          (COMMAND_START_ADDRESS + 32)
#define GET_IMU_ID                          (COMMAND_START_ADDRESS + 33)

// Stream frequency
#define SET_STREAM_FREQ                     (COMMAND_START_ADDRESS + 34)
#define GET_STREAM_FREQ                     (COMMAND_START_ADDRESS + 35)

// Deg/Rad output
#define SET_DEGRAD_OUTPUT                   (COMMAND_START_ADDRESS + 36)
#define GET_DEGRAD_OUTPUT                   (COMMAND_START_ADDRESS + 37)

// Reference setting and offset reset 
#define SET_ORIENTATION_OFFSET              (COMMAND_START_ADDRESS + 38)
#define RESET_ORIENTATION_OFFSET            (COMMAND_START_ADDRESS + 39)


/////////////////////////////////////////////////////////////////////
// Accelerometer
/////////////////////////////////////////////////////////////////////
// Accelerometer settings
#define SET_ACC_RANGE                       (COMMAND_START_ADDRESS + 50)
#define GET_ACC_RANGE                       (COMMAND_START_ADDRESS + 51)

/////////////////////////////////////////////////////////////////////
// Gyro
/////////////////////////////////////////////////////////////////////
// Gyroscope settings
#define SET_GYR_RANGE                       (COMMAND_START_ADDRESS + 60)
#define GET_GYR_RANGE                       (COMMAND_START_ADDRESS + 61)

#define START_GYR_CALIBRATION               (COMMAND_START_ADDRESS + 62)

#define SET_ENABLE_GYR_AUTOCALIBRATION      (COMMAND_START_ADDRESS + 64)
#define GET_ENABLE_GYR_AUTOCALIBRATION      (COMMAND_START_ADDRESS + 65)

#define SET_GYR_THRESHOLD                   (COMMAND_START_ADDRESS + 66)
#define GET_GYR_THRESHOLD                   (COMMAND_START_ADDRESS + 67)

/////////////////////////////////////////////////////////////////////
// Magnetometer
/////////////////////////////////////////////////////////////////////
// Magnetometer settings
#define SET_MAG_RANGE                       (COMMAND_START_ADDRESS + 70)
#define GET_MAG_RANGE                       (COMMAND_START_ADDRESS + 71)

#define SET_HARD_IRON_OFFSET                (COMMAND_START_ADDRESS + 72)
#define GET_HARD_IRON_OFFSET                (COMMAND_START_ADDRESS + 73)

#define SET_SOFT_IRON_MATRIX                (COMMAND_START_ADDRESS + 74)
#define GET_SOFT_IRON_MATRIX                (COMMAND_START_ADDRESS + 75)

#define SET_FIELD_ESTIMATE                  (COMMAND_START_ADDRESS + 76)
#define GET_FIELD_ESTIMATE                  (COMMAND_START_ADDRESS + 77)

#define SET_MAG_ALIGNMENT_MATRIX            (COMMAND_START_ADDRESS + 78)
#define GET_MAG_ALIGNMENT_MATRIX            (COMMAND_START_ADDRESS + 79)

#define SET_MAG_ALIGNMENT_BIAS              (COMMAND_START_ADDRESS + 80)
#define GET_MAG_ALIGNMENT_BIAS              (COMMAND_START_ADDRESS + 81)

#define SET_MAG_REFERENCE                   (COMMAND_START_ADDRESS + 82)
#define GET_MAG_REFERENCE                   (COMMAND_START_ADDRESS + 83)

#define START_MAG_CALIBRATION               (COMMAND_START_ADDRESS + 84)
#define STOP_MAG_CALIBRATION                (COMMAND_START_ADDRESS + 85)

#define SET_MAG_CALIBRATION_TIMEOUT         (COMMAND_START_ADDRESS + 86)
#define GET_MAG_CALIBRATION_TIMEOUT         (COMMAND_START_ADDRESS + 87)

#define SET_ENABLE_MAG_AUTOCALIBRATION      (COMMAND_START_ADDRESS + 88)    // Not implemented yet
#define GET_ENABLE_MAG_AUTOCALIBRATION      (COMMAND_START_ADDRESS + 89)    // Not implemented yet


/////////////////////////////////////////////////////////////////////
// Filter
/////////////////////////////////////////////////////////////////////
// Filter settings
#define SET_FILTER_MODE                     (COMMAND_START_ADDRESS + 90)
#define GET_FILTER_MODE                     (COMMAND_START_ADDRESS + 91)


/////////////////////////////////////////////////////////////////////
// CAN
/////////////////////////////////////////////////////////////////////
// CAN settings
#define SET_CAN_START_ID                    (COMMAND_START_ADDRESS + 110)
#define GET_CAN_START_ID                    (COMMAND_START_ADDRESS + 111)
#define SET_CAN_BAUDRATE                    (COMMAND_START_ADDRESS + 112)
#define GET_CAN_BAUDRATE                    (COMMAND_START_ADDRESS + 113)
#define SET_CAN_DATA_PRECISION              (COMMAND_START_ADDRESS + 114)
#define GET_CAN_DATA_PRECISION              (COMMAND_START_ADDRESS + 115)
#define SET_CAN_MODE                        (COMMAND_START_ADDRESS + 116)
#define GET_CAN_MODE                        (COMMAND_START_ADDRESS + 117)
#define SET_CAN_MAPPING                     (COMMAND_START_ADDRESS + 118)
#define GET_CAN_MAPPING                     (COMMAND_START_ADDRESS + 119)
#define SET_CAN_HEARTBEAT                   (COMMAND_START_ADDRESS + 120)
#define GET_CAN_HEARTBEAT                   (COMMAND_START_ADDRESS + 121)

/////////////////////////////////////////////////////////////////////
// UART/RS232
/////////////////////////////////////////////////////////////////////
// UART
#define SET_UART_BAUDRATE                   (COMMAND_START_ADDRESS + 130)
#define GET_UART_BAUDRATE                   (COMMAND_START_ADDRESS + 131)
#define SET_UART_FORMAT                     (COMMAND_START_ADDRESS + 132)
#define GET_UART_FORMAT                     (COMMAND_START_ADDRESS + 133)
#define SET_UART_ASCII_CHARACTER            (COMMAND_START_ADDRESS + 134)
#define GET_UART_ASCII_CHARACTER            (COMMAND_START_ADDRESS + 135)
#define SET_LPBUS_DATA_PRECISION            (COMMAND_START_ADDRESS + 136)
#define GET_LPBUS_DATA_PRECISION            (COMMAND_START_ADDRESS + 137)


/////////////////////////////////////////////////////////////////////
// Sync
/////////////////////////////////////////////////////////////////////
// Software Sync
#define START_SYNC                          (COMMAND_START_ADDRESS + 150)    // Todo
#define STOP_SYNC                           (COMMAND_START_ADDRESS + 151)    // Todo

// Timestamp manipulation
#define SET_TIMESTAMP                       (COMMAND_START_ADDRESS + 152)    // Todo


/////////////////////////////////////////////////////////////////////
// GPS
/////////////////////////////////////////////////////////////////////
#define SET_GPS_TRANSMIT_DATA               (COMMAND_START_ADDRESS + 160)
#define GET_GPS_TRANSMIT_DATA               (COMMAND_START_ADDRESS + 161)

#define SAVE_GPS_STATE                      (COMMAND_START_ADDRESS + 162)
#define CLEAR_GPS_STATE                     (COMMAND_START_ADDRESS + 163)

#define SET_GPS_UDR                         (COMMAND_START_ADDRESS + 164)
#define GET_GPS_UDR                         (COMMAND_START_ADDRESS + 165)


/////////////////////////////////////////////////////////////////////
//  Transmit data Register
/////////////////////////////////////////////////////////////////////
#define TDR_ACC_RAW_OUTPUT_ENABLED                  (uint32_t)(0x00000001 << 0)
#define TDR_ACC_CALIBRATED_OUTPUT_ENABLED           (uint32_t)(0x00000001 << 1)
#define TDR_GYR0_RAW_OUTPUT_ENABLED                 (uint32_t)(0x00000001 << 2)
#define TDR_GYR1_RAW_OUTPUT_ENABLED                 (uint32_t)(0x00000001 << 3)
#define TDR_GYR0_BIAS_CALIBRATED_OUTPUT_ENABLED     (uint32_t)(0x00000001 << 4)
#define TDR_GYR1_BIAS_CALIBRATED_OUTPUT_ENABLED     (uint32_t)(0x00000001 << 5)
#define TDR_GYR0_ALIGN_CALIBRATED_OUTPUT_ENABLED    (uint32_t)(0x00000001 << 6)
#define TDR_GYR1_ALIGN_CALIBRATED_OUTPUT_ENABLED    (uint32_t)(0x00000001 << 7)
#define TDR_MAG_RAW_OUTPUT_ENABLED                  (uint32_t)(0x00000001 << 8)
#define TDR_MAG_CALIBRATED_OUTPUT_ENABLED           (uint32_t)(0x00000001 << 9)
#define TDR_ANGULAR_VELOCITY_OUTPUT_ENABLED         (uint32_t)(0x00000001 << 10)
#define TDR_QUAT_OUTPUT_ENABLED                     (uint32_t)(0x00000001 << 11)
#define TDR_EULER_OUTPUT_ENABLED                    (uint32_t)(0x00000001 << 12)
#define TDR_LINACC_OUTPUT_ENABLED                   (uint32_t)(0x00000001 << 13)
#define TDR_PRESSURE_OUTPUT_ENABLED                 (uint32_t)(0x00000001 << 14)
#define TDR_ALTITUDE_OUTPUT_ENABLED                 (uint32_t)(0x00000001 << 15)
#define TDR_TEMPERATURE_OUTPUT_ENABLED              (uint32_t)(0x00000001 << 16)

/////////////////////////////////////////////////////////////////////
// GPS TRANSMIT_DATA_CONFIG Register
/////////////////////////////////////////////////////////////////////
// GPS transmit data register0 contents
#define GPS_NAV_PVT_ITOW_ENABLE                     (uint32_t)(0x00000001 << 0)
#define GPS_NAV_PVT_YEAR_ENABLE                     (uint32_t)(0x00000001 << 1)
#define GPS_NAV_PVT_MONTH_ENABLE                    (uint32_t)(0x00000001 << 2)
#define GPS_NAV_PVT_DAY_ENABLE                      (uint32_t)(0x00000001 << 3)
#define GPS_NAV_PVT_HOUR_ENABLE                     (uint32_t)(0x00000001 << 4)
#define GPS_NAV_PVT_MIN_ENABLE                      (uint32_t)(0x00000001 << 5)
#define GPS_NAV_PVT_SEC_ENABLE                      (uint32_t)(0x00000001 << 6)
#define GPS_NAV_PVT_VALID_ENABLE                    (uint32_t)(0x00000001 << 7)
#define GPS_NAV_PVT_TACC_ENABLE                     (uint32_t)(0x00000001 << 8)
#define GPS_NAV_PVT_NANO_ENABLE                     (uint32_t)(0x00000001 << 9)
#define GPS_NAV_PVT_FIXTYPE_ENABLE                  (uint32_t)(0x00000001 << 10)
#define GPS_NAV_PVT_FLAGS_ENABLE                    (uint32_t)(0x00000001 << 11)
#define GPS_NAV_PVT_FLAGS2_ENABLE                   (uint32_t)(0x00000001 << 12)
#define GPS_NAV_PVT_NUMSV_ENABLE                    (uint32_t)(0x00000001 << 13)
#define GPS_NAV_PVT_LONGITUDE_ENABLE                (uint32_t)(0x00000001 << 14)
#define GPS_NAV_PVT_LATITUDE_ENABLE                 (uint32_t)(0x00000001 << 15)
#define GPS_NAV_PVT_HEIGHT_ENABLE                   (uint32_t)(0x00000001 << 16)
#define GPS_NAV_PVT_HMSL_ENABLE                     (uint32_t)(0x00000001 << 17)
#define GPS_NAV_PVT_HACC_ENABLE                     (uint32_t)(0x00000001 << 18)
#define GPS_NAV_PVT_VACC_ENABLE                     (uint32_t)(0x00000001 << 19)
#define GPS_NAV_PVT_VELN_ENABLE                     (uint32_t)(0x00000001 << 20)
#define GPS_NAV_PVT_VELE_ENABLE                     (uint32_t)(0x00000001 << 21)
#define GPS_NAV_PVT_VELD_ENABLE                     (uint32_t)(0x00000001 << 22)
#define GPS_NAV_PVT_GSPEED_ENABLE                   (uint32_t)(0x00000001 << 23)
#define GPS_NAV_PVT_HEADMOT_ENABLE                  (uint32_t)(0x00000001 << 24)
#define GPS_NAV_PVT_SACC_ENABLE                     (uint32_t)(0x00000001 << 25)
#define GPS_NAV_PVT_HEADACC_ENABLE                  (uint32_t)(0x00000001 << 26)
#define GPS_NAV_PVT_PDOP_ENABLE                     (uint32_t)(0x00000001 << 27)
#define GPS_NAV_PVT_HEADVEH_ENABLE                  (uint32_t)(0x00000001 << 28)

// GPS transmit data register1 contents
#define GPS_NAV_ATT_ITOW_ENABLE                     (uint32_t)(0x00000001 << 0)
#define GPS_NAV_ATT_VERSION_ENABLE                  (uint32_t)(0x00000001 << 1)
#define GPS_NAV_ATT_ROLL_ENABLE                     (uint32_t)(0x00000001 << 2)
#define GPS_NAV_ATT_PITCH_ENABLE                    (uint32_t)(0x00000001 << 3)
#define GPS_NAV_ATT_HEADING_ENABLE                  (uint32_t)(0x00000001 << 4)
#define GPS_NAV_ATT_ACCROLL_ENABLE                  (uint32_t)(0x00000001 << 5)
#define GPS_NAV_ATT_ACCPITCH_ENABLE                 (uint32_t)(0x00000001 << 6)
#define GPS_NAV_ATT_ACCHEADING_ENABLE               (uint32_t)(0x00000001 << 7)

#define GPS_ESF_STATUS_ITOW_ENABLE                  (uint32_t)(0x00000001 << 8)
#define GPS_ESF_STATUS_VERSION_ENABLE               (uint32_t)(0x00000001 << 9)
#define GPS_ESF_STATUS_INITSTATUS1_ENABLE           (uint32_t)(0x00000001 << 10)
#define GPS_ESF_STATUS_INITSTATUS2_ENABLE           (uint32_t)(0x00000001 << 11)
#define GPS_ESF_STATUS_FUSIONMODE_ENABLE            (uint32_t)(0x00000001 << 12)
#define GPS_ESF_STATUS_NUMSENS_ENABLE               (uint32_t)(0x00000001 << 13)
#define GPS_ESF_STATUS_SENSSTATUS_ENABLE            (uint32_t)(0x00000001 << 14)

#define GPS_UDR_STATUS_ENABLE                       (uint32_t)(0x00000001 << 15)


// Data stream frequency
#define DATA_STREAM_FREQ_5HZ                    5
#define DATA_STREAM_FREQ_10HZ                   10
#define DATA_STREAM_FREQ_50HZ                   50
#define DATA_STREAM_FREQ_100HZ                  100
#define DATA_STREAM_FREQ_250HZ                  250
#define DATA_STREAM_FREQ_500HZ                  500
#define DATA_STREAM_FREQ_1000HZ                 1000

//Gyro Range
#define GYR_RANGE_400DPS                        400
#define GYR_RANGE_1000DPS                       1000
#define GYR_RANGE_2000DPS                       2000

//Acc Range
#define ACC_RANGE_2G                            2
#define ACC_RANGE_4G                            4
#define ACC_RANGE_8G                            8
#define ACC_RANGE_16G                           16

//Mag Range
#define MAG_RANGE_2GUASS                        2
#define MAG_RANGE_8GUASS                        8

//Filter Mode
#define LPMS_FILTER_GYR                 0
#define LPMS_FILTER_KALMAN_GYR_ACC      1
#define LPMS_FILTER_KALMAN_GYR_ACC_MAG  2
#define LPMS_FILTER_DCM_GYR_ACC         3
#define LPMS_FILTER_DCM_GYR_ACC_MAG     4

#define LPMS_OFFSET_MODE_OBJECT         0
#define LPMS_OFFSET_MODE_HEADING        1
#define LPMS_OFFSET_MODE_ALIGNMENT      2

// UART Settings
#define LPMS_UART_DATA_PRECISION_FIXED_POINT    0
#define LPMS_UART_DATA_PRECISION_FLOATING_POINT 1

#define LPMS_UART_BAUDRATE_9600                 9600
#define LPMS_UART_BAUDRATE_19200                19200
#define LPMS_UART_BAUDRATE_38400                38400
#define LPMS_UART_BAUDRATE_57600                57600
#define LPMS_UART_BAUDRATE_115200               115200
#define LPMS_UART_BAUDRATE_230400               230400
#define LPMS_UART_BAUDRATE_256000               256000
#define LPMS_UART_BAUDRATE_460800               460800
#define LPMS_UART_BAUDRATE_921600               921600

#define LPMS_UART_DATA_FORMAT_LPBUS             0
#define LPMS_UART_DATA_FORMAT_ASCII             1

// CAN bus baudrate values
#define LPMS_CAN_BAUDRATE_125K                  125
#define LPMS_CAN_BAUDRATE_250K                  250
#define LPMS_CAN_BAUDRATE_500K                  500
#define LPMS_CAN_BAUDRATE_800K                  800
#define LPMS_CAN_BAUDRATE_1M                    1000

#define LPMS_CAN_HEARTBEAT_005                  0
#define LPMS_CAN_HEARTBEAT_010                  1
#define LPMS_CAN_HEARTBEAT_020                  2
#define LPMS_CAN_HEARTBEAT_050                  5
#define LPMS_CAN_HEARTBEAT_100                  10

#define LPMS_CAN_MODE_CANOPEN                   0
#define LPMS_CAN_MODE_SEQUENTIAL                1

#define LPMS_CAN_DATA_PRECISION_FIXED_POINT     0
#define LPMS_CAN_DATA_PRECISION_FLOATING_POINT  1

// CAN Mapping
enum CanMapping {
    LPMS_CAN_MAPPING_DISABLE = 0,
    LPMS_CAN_MAPPING_ACC_RAW_X = 1,
    LPMS_CAN_MAPPING_ACC_RAW_Y,
    LPMS_CAN_MAPPING_ACC_RAW_Z,

    LPMS_CAN_MAPPING_ACC_CALIBRATED_X,
    LPMS_CAN_MAPPING_ACC_CALIBRATED_Y,
    LPMS_CAN_MAPPING_ACC_CALIBRATED_Z,

    LPMS_CAN_MAPPING_GYR0_RAW_X,
    LPMS_CAN_MAPPING_GYR0_RAW_Y,
    LPMS_CAN_MAPPING_GYR0_RAW_Z,

    LPMS_CAN_MAPPING_GYR1_RAW_X,
    LPMS_CAN_MAPPING_GYR1_RAW_Y,
    LPMS_CAN_MAPPING_GYR1_RAW_Z,

    LPMS_CAN_MAPPING_GYR0_BIAS_CALIBRATED_X,
    LPMS_CAN_MAPPING_GYR0_BIAS_CALIBRATED_Y,
    LPMS_CAN_MAPPING_GYR0_BIAS_CALIBRATED_Z,

    LPMS_CAN_MAPPING_GYR1_BIAS_CALIBRATED_X,
    LPMS_CAN_MAPPING_GYR1_BIAS_CALIBRATED_Y,
    LPMS_CAN_MAPPING_GYR1_BIAS_CALIBRATED_Z,

    LPMS_CAN_MAPPING_GYR0_ALIGN_CALIBRATED_X,
    LPMS_CAN_MAPPING_GYR0_ALIGN_CALIBRATED_Y,
    LPMS_CAN_MAPPING_GYR0_ALIGN_CALIBRATED_Z,

    LPMS_CAN_MAPPING_GYR1_ALIGN_CALIBRATED_X,
    LPMS_CAN_MAPPING_GYR1_ALIGN_CALIBRATED_Y,
    LPMS_CAN_MAPPING_GYR1_ALIGN_CALIBRATED_Z,

    LPMS_CAN_MAPPING_MAG_RAW_X,
    LPMS_CAN_MAPPING_MAG_RAW_Y,
    LPMS_CAN_MAPPING_MAG_RAW_Z,

    LPMS_CAN_MAPPING_MAG_CALIBRATED_X,
    LPMS_CAN_MAPPING_MAG_CALIBRATED_Y,
    LPMS_CAN_MAPPING_MAG_CALIBRATED_Z,

    LPMS_CAN_MAPPING_ANGULAR_VELOCITY_X,
    LPMS_CAN_MAPPING_ANGULAR_VELOCITY_Y,
    LPMS_CAN_MAPPING_ANGULAR_VELOCITY_Z,

    LPMS_CAN_MAPPING_QUAT_W,
    LPMS_CAN_MAPPING_QUAT_X,
    LPMS_CAN_MAPPING_QUAT_Y,
    LPMS_CAN_MAPPING_QUAT_Z,

    LPMS_CAN_MAPPING_EULER_X,
    LPMS_CAN_MAPPING_EULER_Y,
    LPMS_CAN_MAPPING_EULER_Z,

    LPMS_CAN_MAPPING_LINACC_X,
    LPMS_CAN_MAPPING_LINACC_Y,
    LPMS_CAN_MAPPING_LINACC_Z,

    LPMS_CAN_MAPPING_PRESSURE,
    LPMS_CAN_MAPPING_TEMPERATURE,
    LPMS_CAN_MAPPING_MAX

};

#endif
