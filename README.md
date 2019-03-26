# Quick Start

## System Configurations

- Ubuntu 16.04
- ROS Kinetic
- USB 3.0 Port

## Installation

```
$ source /opt/ros/kinetic/setup.bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ catkin_init_workspace
$ git clone https://github.com/tork-a/cis_camera.git
$ cd ~/catkin_ws/src/cis_camera
$ git checkout mod_test
$ cd ~/catkin_ws
$ rosdep install -y -r --from-paths src --ignore-src
$ catkin_make
$ source devel/setup.bash
```

## Connecting Camera

Please connect the camera to the USB 3.0 port of your Ubuntu PC.

## Launch Software

To see the pointcloud with RViz,

```
$ roslaunch cis_camera pointcloud.launch
```

When you need only depth image,

```
$ roslaunch cis_camera tof.launch
```

> #### NOTICE
>
> At the first launch, you will get a device permission error like below.
>
> ```
> $ roslaunch cis_camera pointcloud.launch
>
> ...
>
> [ERROR] [1553240805.160155192]: Permission denied opening /dev/bus/usb/002/018
>
> ...
>
> ```
>
> The port number of the device is different every time,
> please replace it each time.
>
> Change the permission of the port displayed in the error by the following method,
> and execute the launch file again.
>
> ```
> $ sudo chmod o+w /dev/bus/usb/002/018
> ```

### Quit Software

Enter Ctrl-C on the running terminal.


# Reference

## CIS TOF Camera Specific Topics

#### Point Cloud Topic

- /camera/depth/points
  - Type: sensor_msgs/PointCloud2
  - Publisher : /camera/camera_nodelet_manager
  -

#### Depth Image Topic

- /camera/depth/image_raw
  - Type : sensor_msgs/Image
  - Publisher : /camera/cistof
  - Default
    - Width      : 640 px
    - Height     : 480 px
    - Video Mode : gray16
    - Frame Rate : 30 Hz
  - Raw image topic with gray16 scale for depth or IR

#### Camera Temperature Topics

- /camera/cistof/t1
  - Type : sensor_msgs/Temperature
  - Publisher : /camera/cistof
  - Temperature [deg C] on LT board
- /camera/cistof/t2
  - Type : sensor_msgs/Temperature
  - Publisher : /camera/cistof
  - Temperature [deg C] on IM board

```
$ rostopic echo /camera/cistof/t1
header:
  seq: 3128
  stamp:
    secs: 1553252925
    nsecs:  22929619
  frame_id: "camera"
temperature: 41.0625
variance: 0.0
---

...

```

## CIS TOF Camera Specific Parameters

#### depth_ir
  - Depth Image Mode or IR Image Mode
  - Value
    - Default : 0
    - Depth Mode : 0
    - IR Mode : 1

#### depth_range
  - Depth Range
    - Range 0 : XNear / around 100mm - 500mm
    - Range 1 : Near / around 400mm - 15000mm
  - Value
    - Default : 0
    - Range 0 : 0
    - Range 1 : 1

#### threshold
  - Coring Threshold
    - Increasing the value will lower the background threshold.
  - Value
    - Default : 0
    - Maximum : 0x3FFF
    - Minimum : 0

#### nr_filter
  - Noise Reduction Filter ON/OFF
  - Value
    - Default : 1
    - NR Filter ON  : 1
    - NR Filter OFF : 0

#### pulse_count
  - Number of light emitting pulses per frame
    - Increasing the value improves the distance measurement accuracy.
  - Value
    - Default : 1000
    - Maximum : 2000
    - Minimum : 1

#### ld_enable
  - Enable LEDs
    - LD1 ON : 0x0001
    - LD2 ON : 0x0002
    - LD3 ON : 0x0004
    - LD4 ON : 0x0008
  - Value
    - Default : 15
    - Maximum : 15
    - Minimum : 0

#### ir_gain
  - IR Gain
  - Value
    - Default : 1024
    - Maximum : 2047
    - Minimum : 0

#### error_stop
  - Operation at errors occur
  - Value
    - Default : 1
    - Continue : 1
    - Stop : 0

### How to Change Parameters

To change the parameters, add options descriptions like below
when you execute the launch file.

```
$ roslaunch cis_camera pointcloud.launch nr_filter:=0 pulse_count:=100
```

If you want to display the informations about parameters when launch files extecuted, use `--screen` option as below.

```
$ roslaunch cis_camera pointcloud.launch --screen
```


## Launch Files

### tof.launch

#### Nodes

```
$ rosnode list
/camera/cistof
/rosout
```

#### Topics

```
$ rostopic list
/camera/cistof/parameter_descriptions
/camera/cistof/parameter_updates
/camera/cistof/t1
/camera/cistof/t2
/camera/depth/camera_info
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressed/parameter_descriptions
/camera/depth/image_raw/compressed/parameter_updates
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/compressedDepth/parameter_descriptions
/camera/depth/image_raw/compressedDepth/parameter_updates
/camera/depth/image_raw/theora
/camera/depth/image_raw/theora/parameter_descriptions
/camera/depth/image_raw/theora/parameter_updates
/rosout
/rosout_agg
```

#### Parameters

```
$ rosparam list
/camera/cistof/camera_info_url
/camera/cistof/depth_ir
/camera/cistof/depth_range
/camera/cistof/error_stop
/camera/cistof/frame_id
/camera/cistof/frame_rate
/camera/cistof/height
/camera/cistof/index
/camera/cistof/ir_gain
/camera/cistof/ld_enable
/camera/cistof/nr_filter
/camera/cistof/product
/camera/cistof/pulse_count
/camera/cistof/serial
/camera/cistof/threshold
/camera/cistof/timestamp_method
/camera/cistof/vendor
/camera/cistof/video_mode
/camera/cistof/width
/camera/depth/image_raw/compressed/format
/camera/depth/image_raw/compressed/jpeg_quality
/camera/depth/image_raw/compressed/png_level
/camera/depth/image_raw/compressedDepth/depth_max
/camera/depth/image_raw/compressedDepth/depth_quantization
/camera/depth/image_raw/compressedDepth/png_level
/camera/depth/image_raw/theora/keyframe_frequency
/camera/depth/image_raw/theora/optimize_for
/camera/depth/image_raw/theora/quality
/camera/depth/image_raw/theora/target_bitrate
/rosdistro
/roslaunch/uris/host_robotuser_pc__36069
/rosversion
/run_id
```

### pointcloud.launch

#### Nodes

```
$ rosnode list
/camera/camera_nodelet_manager
/camera/cistof
/camera/depth_metric
/camera/depth_metric_rect
/camera/depth_points
/camera/depth_rectify_depth
/map_to_camera
/rosout
/rviz
```

#### Topics

```
robotuser@robotuser-PC:~/camera_ws$ rostopic list
/camera/cistof/parameter_descriptions
/camera/cistof/parameter_updates
/camera/cistof/t1
/camera/cistof/t2
/camera/depth/camera_info
/camera/depth/image
/camera/depth/image/compressed
/camera/depth/image/compressed/parameter_descriptions
/camera/depth/image/compressed/parameter_updates
/camera/depth/image/compressedDepth
/camera/depth/image/compressedDepth/parameter_descriptions
/camera/depth/image/compressedDepth/parameter_updates
/camera/depth/image/theora
/camera/depth/image/theora/parameter_descriptions
/camera/depth/image/theora/parameter_updates
/camera/depth/image_raw
/camera/depth/image_raw/compressed
/camera/depth/image_raw/compressed/parameter_descriptions
/camera/depth/image_raw/compressed/parameter_updates
/camera/depth/image_raw/compressedDepth
/camera/depth/image_raw/compressedDepth/parameter_descriptions
/camera/depth/image_raw/compressedDepth/parameter_updates
/camera/depth/image_raw/theora
/camera/depth/image_raw/theora/parameter_descriptions
/camera/depth/image_raw/theora/parameter_updates
/camera/depth/image_rect
/camera/depth/image_rect/compressed
/camera/depth/image_rect/compressed/parameter_descriptions
/camera/depth/image_rect/compressed/parameter_updates
/camera/depth/image_rect/compressedDepth
/camera/depth/image_rect/compressedDepth/parameter_descriptions
/camera/depth/image_rect/compressedDepth/parameter_updates
/camera/depth/image_rect/theora
/camera/depth/image_rect/theora/parameter_descriptions
/camera/depth/image_rect/theora/parameter_updates
/camera/depth/image_rect_raw
/camera/depth/image_rect_raw/compressed
/camera/depth/image_rect_raw/compressed/parameter_descriptions
/camera/depth/image_rect_raw/compressed/parameter_updates
/camera/depth/image_rect_raw/compressedDepth
/camera/depth/image_rect_raw/compressedDepth/parameter_descriptions
/camera/depth/image_rect_raw/compressedDepth/parameter_updates
/camera/depth/image_rect_raw/theora
/camera/depth/image_rect_raw/theora/parameter_descriptions
/camera/depth/image_rect_raw/theora/parameter_updates
/camera/depth/points
/camera/depth_rectify_depth/parameter_descriptions
/camera/depth_rectify_depth/parameter_updates
/clicked_point
/initialpose
/move_base_simple/goal
/rosout
/rosout_agg
/tf
/tf_static
```

#### Parameters

```
$ rosparam list
/camera/camera_nodelet_manager/num_worker_threads
/camera/cistof/camera_info_url
/camera/cistof/depth_ir
/camera/cistof/depth_range
/camera/cistof/error_stop
/camera/cistof/frame_id
/camera/cistof/frame_rate
/camera/cistof/height
/camera/cistof/index
/camera/cistof/ir_gain
/camera/cistof/ld_enable
/camera/cistof/nr_filter
/camera/cistof/product
/camera/cistof/pulse_count
/camera/cistof/serial
/camera/cistof/threshold
/camera/cistof/timestamp_method
/camera/cistof/vendor
/camera/cistof/video_mode
/camera/cistof/width
/camera/depth/image/compressed/format
/camera/depth/image/compressed/jpeg_quality
/camera/depth/image/compressed/png_level
/camera/depth/image/compressedDepth/depth_max
/camera/depth/image/compressedDepth/depth_quantization
/camera/depth/image/compressedDepth/png_level
/camera/depth/image/theora/keyframe_frequency
/camera/depth/image/theora/optimize_for
/camera/depth/image/theora/quality
/camera/depth/image/theora/target_bitrate
/camera/depth/image_raw/compressed/format
/camera/depth/image_raw/compressed/jpeg_quality
/camera/depth/image_raw/compressed/png_level
/camera/depth/image_raw/compressedDepth/depth_max
/camera/depth/image_raw/compressedDepth/depth_quantization
/camera/depth/image_raw/compressedDepth/png_level
/camera/depth/image_raw/theora/keyframe_frequency
/camera/depth/image_raw/theora/optimize_for
/camera/depth/image_raw/theora/quality
/camera/depth/image_raw/theora/target_bitrate
/camera/depth/image_rect/compressed/format
/camera/depth/image_rect/compressed/jpeg_quality
/camera/depth/image_rect/compressed/png_level
/camera/depth/image_rect/compressedDepth/depth_max
/camera/depth/image_rect/compressedDepth/depth_quantization
/camera/depth/image_rect/compressedDepth/png_level
/camera/depth/image_rect/theora/keyframe_frequency
/camera/depth/image_rect/theora/optimize_for
/camera/depth/image_rect/theora/quality
/camera/depth/image_rect/theora/target_bitrate
/camera/depth/image_rect_raw/compressed/format
/camera/depth/image_rect_raw/compressed/jpeg_quality
/camera/depth/image_rect_raw/compressed/png_level
/camera/depth/image_rect_raw/compressedDepth/depth_max
/camera/depth/image_rect_raw/compressedDepth/depth_quantization
/camera/depth/image_rect_raw/compressedDepth/png_level
/camera/depth/image_rect_raw/theora/keyframe_frequency
/camera/depth/image_rect_raw/theora/optimize_for
/camera/depth/image_rect_raw/theora/quality
/camera/depth/image_rect_raw/theora/target_bitrate
/camera/depth_rectify_depth/interpolation
/rosdistro
/roslaunch/uris/host_robotuser_pc__46447
/rosversion
/run_id
```

<!-- EOF  -->
