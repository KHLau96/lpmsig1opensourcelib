
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Trigger.h"
#include "std_msgs/Bool.h"

#include "lpsensor/LpmsIG1I.h"
#include "lpsensor/SensorDataI.h"
#include "lpsensor/LpmsIG1Registers.h"

//! Manages connection with the sensor, publishes data
/*!
  \TODO: Make noncopyable!
 */

struct IG1Command
{
    short command;
    union Data {
        uint32_t i[64];
        float f[64];
        unsigned char c[256];
    } data;
    int dataLength;
};

class LpBE1Proxy
{
public:
    // Node handler
    ros::NodeHandle nh, private_nh;
    ros::Timer updateTimer;

    // Publisher
    ros::Publisher imu_pub;
    ros::Publisher mag_pub;
    ros::Publisher autocalibration_status_pub;

    // Service
    ros::ServiceServer autocalibration_serv;
    ros::ServiceServer gyrocalibration_serv;
    ros::ServiceServer resetHeading_serv;

    sensor_msgs::Imu imu_msg;
    sensor_msgs::MagneticField mag_msg;

    // Parameters
    std::string comportNo;
    int baudrate;
    std::string frame_id;
    int rate;

    LpBE1Proxy(ros::NodeHandle h) : 
        nh(h),
        private_nh("~")
    {
        // Get node parameters
        private_nh.param<std::string>("port", comportNo, "/dev/ttyUSB0");
        private_nh.param("baudrate", baudrate, 115200);
        private_nh.param<std::string>("frame_id", frame_id, "imu");
        private_nh.param("rate", rate, 200);

        // Create LpmsBE1 object 
        sensor1 = IG1Factory();

        imu_pub = nh.advertise<sensor_msgs::Imu>("data",1);
        mag_pub = nh.advertise<sensor_msgs::MagneticField>("mag",1);
        autocalibration_status_pub = nh.advertise<std_msgs::Bool>("is_autocalibration_active", 1, true);

        autocalibration_serv = nh.advertiseService("enable_gyro_autocalibration", &LpBE1Proxy::setAutocalibration, this);
        gyrocalibration_serv = nh.advertiseService("calibrate_gyroscope", &LpBE1Proxy::calibrateGyroscope, this);
        resetHeading_serv = nh.advertiseService("reset_heading", &LpBE1Proxy::resetHeading, this);

         // Connects to sensor
        if (!sensor1->connect(comportNo, baudrate))
        {
            //logd(TAG, "Error connecting to sensor\n");
            ROS_ERROR("Error connecting to sensor\n");
            sensor1->release();
            ros::Duration(3).sleep(); // sleep 3 s
        }
    }

    ~LpBE1Proxy(void)
    {
        sensor1->release();
    }

    void update(const ros::TimerEvent& te)
    {
        static bool runOnce = false;

        if (sensor1->getStatus() == STATUS_CONNECTED &&
                sensor1->hasImuData())
        {
            if (!runOnce)
            {
                publishIsAutocalibrationActive();
                runOnce = true;
            }
            IG1ImuDataI sd;
            sensor1->getImuData(sd);

            /* Fill the IMU message */

            // Fill the header
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = frame_id;

            // Fill orientation quaternion
            imu_msg.orientation.w = sd.quaternion.data[0];
            imu_msg.orientation.x = -sd.quaternion.data[1];
            imu_msg.orientation.y = -sd.quaternion.data[2];
            imu_msg.orientation.z = -sd.quaternion.data[3];

            // Fill angular velocity data
            // - scale from deg/s to rad/s
            imu_msg.angular_velocity.x = sd.gyroIIAlignmentCalibrated.data[0]*3.1415926/180;
            imu_msg.angular_velocity.y = sd.gyroIIAlignmentCalibrated.data[1]*3.1415926/180;
            imu_msg.angular_velocity.z = sd.gyroIIAlignmentCalibrated.data[2]*3.1415926/180;

            // Fill linear acceleration data
            imu_msg.linear_acceleration.x = sd.accCalibrated.data[0]*9.81;
            imu_msg.linear_acceleration.y = sd.accCalibrated.data[1]*9.81;
            imu_msg.linear_acceleration.z = sd.accCalibrated.data[2]*9.81;

            /* Fill the magnetometer message */
            mag_msg.header.stamp = imu_msg.header.stamp;
            mag_msg.header.frame_id = frame_id;

            // Units are microTesla in the LPMS library, Tesla in ROS.
            mag_msg.magnetic_field.x = sd.magRaw.data[0]*1e-6;
            mag_msg.magnetic_field.y = sd.magRaw.data[1]*1e-6;
            mag_msg.magnetic_field.z = sd.magRaw.data[2]*1e-6;

            // Publish the messages
            imu_pub.publish(imu_msg);
            mag_pub.publish(mag_msg);
        }
    }

    void run(void)
    {
        // The timer ensures periodic data publishing
        updateTimer = ros::Timer(nh.createTimer(ros::Duration(1.0f/rate),
                                                &LpBE1Proxy::update,
                                                this));
    }

    void publishIsAutocalibrationActive()
    {
        std_msgs::Bool msg;
        IG1SettingsI settings;
        sensor1->getSettings(settings);
        msg.data = settings.enableGyroAutocalibration;
        autocalibration_status_pub.publish(msg);
    }

    ///////////////////////////////////////////////////
    // Service Callbacks
    ///////////////////////////////////////////////////
    bool setAutocalibration (std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
    {
        ROS_INFO("set_autocalibration");


        // clear current settings
        IG1SettingsI settings;
        sensor1->getSettings(settings);

        // Send command
        IG1Command cmd;
        cmd.command = SET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 4;
        cmd.data.i[0] = req.data;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);

        ros::Duration(0.2).sleep();

        cmd.command = GET_ENABLE_GYR_AUTOCALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);


        while (!sensor1->hasSettings()) 
        {
            ros::Duration(0.1).sleep();
            ROS_INFO("set_autocalibration wait");
        }
        ROS_INFO("set_autocalibration done");


        // Get settings
        sensor1->getSettings(settings);

        std::string msg;
        if (settings.enableGyroAutocalibration == req.data) 
        {
            res.success = true;
            msg.append(std::string("[Success] autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }
        else 
        {
            res.success = false;
            msg.append(std::string("[Failed] current autocalibration status set to: ") + (settings.enableGyroAutocalibration?"True":"False"));
        }

        res.message = msg;

        publishIsAutocalibrationActive();
        return res.success;
    }

    
    // reset heading
    bool resetHeading (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("reset_heading");
        
        // Send command
        IG1Command cmd;
        cmd.command = SET_ORIENTATION_OFFSET;
        cmd.dataLength = 4;
        cmd.data.i[0] = LPMS_OFFSET_MODE_HEADING;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);

        res.success = true;
        res.message = "[Success] Heading reset";
        return true;
    }


    bool calibrateGyroscope (std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
    {
        ROS_INFO("calibrate_gyroscope: Please make sure the sensor is stationary for 4 seconds");

        IG1Command cmd;
        cmd.command = START_GYR_CALIBRATION;
        cmd.dataLength = 0;
        sensor1->sendCommand(cmd.command, cmd.dataLength, cmd.data.c);

        ros::Duration(4).sleep();
        res.success = true;
        res.message = "[Success] Gyroscope calibration procedure completed";
        ROS_INFO("calibrate_gyroscope: Gyroscope calibration procedure completed");
        return true;
    }


 private:

    // Access to LPMS data
    IG1I* sensor1;
};

int main(int argc, char *argv[])
{

    ros::init(argc, argv, "lpms_ig1_node");
    ros::NodeHandle nh("imu");

    ros::AsyncSpinner spinner(0);
    spinner.start();

    LpBE1Proxy lpBE1(nh);

    lpBE1.run();
    ros::waitForShutdown();

    return 0;
}
