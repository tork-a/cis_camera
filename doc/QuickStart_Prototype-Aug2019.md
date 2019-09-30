# Quick Start

For CIS ToF Camera Sensor - Prototype Aug 2019

## System Configurations

- OS / ROS
    - Ubuntu 16.04
        - ROS Kinetic
    - Ubuntu 18.04
        - ROS Melodic
- USB 3.0 Port
- CIS ToF Camera Sensor
    - Prototype of Aug 2019

## Installation

### Installing ROS

Install "ROS Desktop Full" on Ubuntu PC.

- ROS Kinetic for Ubuntu 16.04
    - http://wiki.ros.org/kinetic/Installation/Ubuntu
- ROS Melodic for Ubuntu 18.04
    - http://wiki.ros.org/melodic/Installation/Ubuntu

### Catkin Workspace Preparation

```
$ source /opt/ros/$ROS_DISTRO/setup.bash
$ mkdir -p ~/camera_ws/src
$ cd ~/camera_ws/src
$ catkin_init_workspace
```

- **NOTE:** Replase `$ROS_DISTRO` to the ROS distribution of your system, `kinetic` or `melodic`.

<div style="page-break-before:always"></div>

### ToF Camera ROS Driver Software Codes

Unzip "cis_camera.zip" in Ubuntu and put `cis_camera` folder in `~/camera_ws/src`.

- **NOTE:** Please DO NOT uncompress the ZIP file in MS Windows 
  because file permissions are lost. 

### Build

```
$ cd ~/camera_ws
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source ~/camera_ws/devel/setup.bash
```

## Connecting Camera

1. Connect the camera to the USB 3.0 port of your Ubuntu PC
1. Connect the external power source to the camera and turn it on

<div style="page-break-before:always"></div>

## Launching Software

### PointCloud

To see the pointcloud with RViz.

```
$ source ~/camera_ws/devel/setup.bash
$ roslaunch cis_camera pointcloud.launch
```

This file launches windows of RViz and `rqt_reconfigure`.

When you do not need to launch `rqt_reconfigure`, 
please set a launch option as below.

```
$ roslaunch cis_camera pointcloud.launch reconfigure:=false
```

#### NOTICE

At the first launch, you may get a device permission error like below.

```
$ roslaunch cis_camera pointcloud.launch
...

[ERROR] [1553240805.160155192]: Permission denied opening /dev/bus/usb/002/018
...
```

Change the permission of the port displayed in the error by the following method,
and execute the launch file again. (The port number of the device is different every time,
please replace it each time.)

```
$ sudo chmod o+w /dev/bus/usb/002/018
```

![RViz PointCloud.launch](images/ros_cis_camera_rviz-pointcloud_20190923.png)

<img src="images/ros_cis_camera_rqt_reconfigure_20190923.png" style="width: 50%;" />

### Publishing Images Only

When you publish only Depth, IR and RGB images, launch `tof.launch`.

```
$ source ~/camera_ws/devel/setup.bash
$ roslaunch cis_camera tof.launch
```

If you show the images, run `rqt` and open Plugins -> Visualization -> Image View.

```
$ source ~/camera_ws/devel/setup.bash
$ rqt
```

### Dynamic Reconfigure

After you launched `pointcloud.launch reconfigure:=false` or `tof.launch`, 
you can also reconfigure Depth/IR configurations dynamically with launching `rqt_reconfigure`.

```
$ source ~/camera_ws/devel/setup.bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

When you reconfigure Depth/IR camera distortion correction parameters,
check `ir_dist_reconfig` to effect parameters `ir_fx`, `ir_fy` and so on.

To set back the parameters to `config/camera_ir.yaml` data, 
uncheck `ir_dist_reconfig`.

<img src="images/ros_cis_camera_rqt_reconfigure_check-ir_dist_reconfig_20190923.png" style="width: 50%;" />

<div style="page-break-before:always"></div>

### Frame Rate

When you want to know a frame rate of ROS topic, please run `rostopic hz` as below.

In the case of a topic `/camera/depth/points`,

```
$ source ~/camera_ws/devel/setup.bash
$ rostopic hz /camera/depth/points
```

To find out what topics exits,

```
$ source ~/camera_ws/devel/setup.bash
$ rostopic list
```


### Quit Software

Enter `Ctrl-C` on the running terminal.

<!-- EOF  -->

