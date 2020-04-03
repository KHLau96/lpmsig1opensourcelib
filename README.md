# LPMS-IG1 Series OpenSource Lib


## Usage
### 1.Compiling Library programs
```bash
    $ cd ~
    $ git clone https://bitbucket.org/lpresearch/lpmsig1opensourcelib
    $ cd lpmsig1opensourcelib
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make package
    $ sudo dpkg -i libLpmsIG1_OpenSource-0.x.x-Linux.deb
```
### 2.Compiling Sample programs
```bash
    $ cd linux_example
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo ./LpmsIG1SimpleExample
```
### 3.Compiling ROS Example programs
```bash
# Create ROS workspace.
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src

# Copy ros_example folder to your ROS src folder.
    $ cp -r ~/lpmsig1opensourcelib/ros_example ./

# Compiling ROS example programs
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ./devel/setup.bash
```

Open a new terminal window and run roscore
```bash
    $ roscore
```
Connect `LPMS-IG1` sensor to PC.
Now you can run lpms_ig1 node on your other terminal windows.

You should see the following output on successful connection:
```bash
    $ rosrun lpms_ig1 lpms_ig1_node
    #[IG1] COM:/dev/ttyUSB0 connection established
    #[IG1] Send get transmit data
```
 
Please refer to [Troubleshooting](#troubleshooting) section is error occurs.

You can print out the imu data by subscribing to `/imu/data` topic
```bash
#Show imu message.
    $ rostopic echo /imu/data
#Plot imu message.
    $ rosrun rqt_plot rqt_plot
```

Alternatively, you can use the sample launch file (lpmsig1.launch) start data acquisition and data plotting:

```
roslaunch lpms_ig1 lpmsig1.launch
```

## Troubleshooting

If prompted with the following information：
```bash
    $ rosrun lpms_ig1 lpms_ig1_node
    #Error opening2[IG1] COM:/dev/ttyUSB0 disconnected
```
this error might be due to the fact that the current user does not have sufficient permission to access the device. 
To allow access to sensors connected via USB, you need to ensure that the user running the ROS sensor node has access to the /dev/ttyUSB devices. You can do this by adding the user to the dialout group. After this call, you should logout and login with this user to ensure the changed permissions are in effect.

```bash
    $ sudo adduser <username> dialout
```



## ROS Package Summary

### 1. Supported Hardware
This driver interfaces with LPMS-IG1 IMU sensor from LP-Research Inc.


### 2.1 lpms_ig1_node
lpms_ig1_node is a driver for the LPMS-IG1 Inertial Measurement Unit. It publishes orientation, angular velocity, linear acceleration and magnetometer data (covariances are not yet supported), and complies with the [Sensor message](https://wiki.ros.org/sensor_msgs) for [IMU API](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html) and [MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html) API.

#### 2.1.1 Published Topics
/imu/data ([sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)) 
:   Inertial data from the IMU. Includes calibrated acceleration, calibrated angular rates and orientation. The orientation is always unit quaternion. 

/imu/mag ([sensor_msgs/MagneticField](http://docs.ros.org/melodic/api/sensor_msgs/html/msg/MagneticField.html))
:   Magnetometer reading from the sensor.

/imu/is_autocalibration_active ([std_msgs/Bool](http://docs.ros.org/api/std_msgs/html/msg/Bool.html), default: True)
:   Latched topic indicating if the gyro autocalibration feature is active

#### 2.1.2 Services
/imu/calibrate_gyroscope ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) 
:   This service activates the IMU internal gyro bias estimation function. Please make sure the IMU sensor is placed on a stable platform with minimal vibrations before calling the service. Please make sure the sensor is stationary for at least 4 seconds. The service call returns a success response once the calibration procedure is completed.

/imu/reset_heading ([std_srvs/Empty](http://docs.ros.org/api/std_srvs/html/srv/Empty.html)) 
:   This service will reset the heading (yaw) angle of the sensor to zero. 

/imu/enable_gyro_autocalibration ([std_srvs/SetBool](http://docs.ros.org/melodic/api/std_srvs/html/srv/SetBool.html))
:   Turn on/off autocalibration function in the IMU. The status of autocalibration can be obtained by subscribing to the /imu/is_autocalibration_active topic. A message will published to /imu/is_autocalibration_active for each call to /imu/autocalibrate. 

#### 2.1.3 Parameters


&#126;port (string, default: /dev/ttyUSB0) 
:   The port the IMU is connected to.

&#126;baudrate (int, default: 921600)
:   Baudrate for the IMU sensor.

&#126;frame_id (string, default: imu) 
:   The frame in which imu readings will be returned.

&#126;data_process_rate (int, default: 200) 
:   Data processing rate of the internal loop. This rate has to be equal or larger than the data streaming frequency of the sensor to prevent internal data queue overflow.


For further information on `LP-Research`, please visit our website:

* http://www.lp-research.com

* http://www.alubi.cn
