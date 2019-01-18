# Quick start

## Clone the source code

```
$ cd catkin_ws/src
$ git clone https://github.com/tork-a/cis_camera.git
$ git clone https://github.com/7675t/libuvc_ros.git
$ cd libuvc_ros
$ git checkout cis_tof_camera
$ catkin build
```

## Connect camera

Please connect to the USB 3.0 port.

## Launch

When you want only depth image, 

```
$ roslaunch cis_camera tof.launch 
```

To see the pointcloud,

```
$ roslaunch cis_camera pointcloud.launch 
```

# What I changed

My change is cis_tof_camera branch from master.

https://github.com/7675t/libuvc_ros/commit/10f577caa6f08254926952e6001e25c1c627ce83
