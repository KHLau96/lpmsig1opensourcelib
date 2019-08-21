#ifndef LPMSIG1_REGISTERS_H
#define LPMSIG1_REGISTERS_H



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