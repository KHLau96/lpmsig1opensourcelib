#LPMS-IG1 Series OpenSource Lib

##Usage
###1.Compiling Library programs
```bash
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ make package
    $ sudo dpkg -i libLpmsIG1_OpenSource-0.x.x-Linux.deb
```
###2.Compiling Sample programs
```bash
    $ cd linux_example
    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ sudo ./LpmsIG1SimpleExample
```
###3.Compiling ROS Example programs
```bash
# Create ROS workspace.
    $ mkdir -p ~/catkin_ws/src
    $ cd ~/catkin_ws/src
# Copy ros_example folder to your ROS src folder.

    $ cp -r /ros_example ./
# Or
    $ git clone https://xxx/lpresearch/lpmsig1opensourcelib.git

# Compiling ROS example programs
    $ cd ~/catkin_ws
    $ catkin_make
    $ source ./devel/setup.bash
```

Open a new terminal window  & run roscore
```bash
    $ roscore
```
Connect `LPMS-IG1` sensor to PC.
Now you can run lpms_ig1 node on your other terminal windows.

If there are no other prompts indicating that the connection has been successful, or [See Troubleshooting](#troubleshooting).
```bash
    $ rosrun lpms_ig1 lpms_ig1_node
    #[IG1] COM:/dev/ttyUSB0 connection established
    #[IG1] Send get transmit data
```


If sensor connected you can open a new terminal window.
```bash
#Show imu message.
    $ rostopic echo /imu
#Plot imu message.
    $ rosrun rqt_plot rqt_plot
```

##Troubleshooting

* If prompted with the following information：
```bash
    $ rosrun lpms_ig1 lpms_ig1_node
    #Error opening2[IG1] COM:/dev/ttyUSB0 disconnected
```
Please modify the permissions.
```bash
    $ sudo chmod 777 /dev/ttyUSB0
```


For further information on `LP-Research`, please visit our website:

* http://www.lp-research.com

* http://www.alubi.cn
