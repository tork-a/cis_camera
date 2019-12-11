cis_camera
##########

Overview
##########

This cis_camera package contains ROS driver nodes for CIS Corporation ToF Camera
Sensor DCC-RGBD1 with USB 3.0 interface.

System Configurations
=====================

-  OS / ROS

   -  Ubuntu 16.04 - 64bit / ROS Kinetic
   -  Ubuntu 18.04 - 64bit / ROS Melodic

-  USB 3.0 Port
-  CIS ToF Camera Sensor DCC-RGBD1

CIS ToF Camera Sensor DCC-RGBD1
===============================

-  Camera system

   -  Dimensions : H:50mm x W:55mm x D:35mm ( Protruding parts are not included )
   -  Weight : 110 g
   -  Frame rate : 30 fps

-  RGB camera

   -  1/3” CMOS image sensor
   -  Global shutter operation
   -  QVGA : 1280 x 960
   -  M12 lens

-  Depth / IR camera

   -  1/4” CCD image sensor
   -  VGA : 640 x 480
   -  M12 lens
   -  Output : Depth and IR images
   -  Depth sensing type: ToF ( Time-of-Flight )
   -  Depth range

      -  mode 0 : 300 - 5000 mm
      -  mode 1 : 150 - 700 mm

-  NIR light source

   -  2 Laser Diodes : 850nm / Class 1

-  USB output

   -  USB 3.0 micro B
   -  UVC interface
   -  Images

      -  RGB : YUV422 - 1920 × 960
      -  IR : Gray 16bit - 640 × 480
      -  Depth : Gray 16bit - 640 × 480

-  Power source

   -  DC 12V, 3A


.. figure:: images/cis-tof-camera_dcc-rgbd1.jpg
   :alt: CIS ToF Camera
   :width: 50%
   
   CIS ToF Camera - DCC-RGBD1


Installations
###############

Install ROS
=============

Install "ROS Desktop Full" on Ubuntu PC.

-  Installing ROS Kinetic for Ubuntu 16.04

   -  http://wiki.ros.org/kinetic/Installation/Ubuntu

-  Installing ROS Melodic for Ubuntu 18.04

   -  http://wiki.ros.org/melodic/Installation/Ubuntu

Install cis\_camera Package
=============================

-  **NOTE** : Replase ``$ROS_DISTRO`` to the ROS distribution of your
   system, ``kinetic`` or ``melodic``.

Install cis\_camera package from a debian package
-------------------------------------------------

- **NOTE** : The debian package is in preparation and will be available around January 2020.

::

    $ sudo apt-get update
    $ sudo apt-get install ros-$ROS_DISTRO-cis-camera

Install cis\_camera package from source codes
---------------------------------------------

::

    $ source /opt/ros/$ROS_DISTRO/setup.bash
    $ mkdir -p ~/camera_ws/src
    $ cd ~/camera_ws/src
    $ catkin_init_workspace
    $ git clone https://github.com/tork-a/cis_camera.git
    $ cd ~/camera_ws
    $ rosdep install -y -r --from-paths src --ignore-src
    $ catkin_make
    $ source ~/camera_ws/devel/setup.bash

Device Permission Configuration
-------------------------------

For the first time you start using CIS ToF camera, run
``set_udev_rules`` to set CIS ToF camera device permission configuration
with entering sudo password in responce to program input.

::

    $ rosrun cis_camera set_udev_rules

-  **NOTE** : This process is needed only once after the installations on your Ubuntu PC.

Launching CIS ToF Camera
##########################

Connecting Camera
===================

1. Connect the camera to the USB 3.0 port of your Ubuntu PC
2. Connect the external power source to the camera and turn it on

-  **NOTE** : It takes about 4 seconds for the camera to start up
   normally after the external power is turned on.

Launching Software
====================

Set up ROS Environment
------------------------

Execute ROS environment setup every time you launch a new terminal.

::

    $ source /opt/ros/$ROS_DISTRO/setup.bash


Or run setup.bash as below when you installed cis\_camera package from source codes.

::

    $ source ~/camera_ws/devel/setup.bash

It is convenient to add the settings to the .bashrc file as shown below 
so that setup.bash runs automatically when a terminal is started.

::

    $ echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc

Or set .bashrc as below when you installed cis\_camera package from source codes.

::

    $ echo "source ~/camera_ws/devel/setup.bash" >> ~/.bashrc

- **NOTE** : DO NOT set ``>>`` to ``>`` in the above command! If you set ``>``, all the settings in the original .bashrc will be lost.

PointCloud
------------

To see the pointcloud with RViz.

::

    $ roslaunch cis_camera pointcloud.launch

This file launches windows of RViz and ``rqt_reconfigure``.

When you do not need to launch ``rqt_reconfigure``, please set a launch
option as below.

::

    $ roslaunch cis_camera pointcloud.launch reconfigure:=false

.. figure:: images/cis_camera_pointcloud_rviz.png
   :alt: RViz PointCloud.launch

   RViz PointCloud.launch

.. figure:: images/cis_camera_dynamic_reconfigure.png
   :alt: Dynamic Reconfigure
   :width: 50%
   
   Dynamic Reconfigure

Launch Options and Default Values of pointcloud.launch
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

-  ``rviz:=true``

   -  Launching RViz

-  ``reconfigure:=true``

   -  Launching Dynamic Reconfigure

-  ``camera:=camera``

   -  Name of cis\_camera for ROS nodes and topics

-  ``num_worker_threads:=4``

   -  Number of threads

-  ``vendor:=0x2af2``

   -  Vendor ID of CIS ToF Camera

-  ``product:=0x1001``

   -  Product ID of CIS ToF Camera

-  ``pointcloud_rgb:=false``

   -  Projecting RGB colors on the pointcloud

-  ``flying_pixel_filter:=false``

   -  Applying flying pixel filter with PCL ``VoxelGrid`` and
      ``StatisticalOutlierRemoval`` filters

.. figure:: images/cis_camera_pointcloud_rgb.png
   :alt: RViz PointCloud.launch
   :width: 50%

   PointCloud with RGB Color Projection

Point Cloud Library (PCL) Sample Program
------------------------------------------

**Terminal 1**

::

  $ roslaunch cis_camera pointcloud.launch

**Terminal 2**

::

  $ rosrun cis_camera pcl_example

This PCL example code extracts a target object by filtering the point
cloud, calculates the centroid of the extracted point cloud and
publishes a TF on the centroid.

.. figure:: images/cis-camera_pcl-example_object-tf_clipped.png
   :alt: PCL Example
   :width: 50%

   PCL Example

This example is based on "Building a Perception Pipleline" of ROS
Industrial Training.

-  https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/Building-a-Perception-Pipeline.html
-  https://industrial-training-master.readthedocs.io/en/kinetic/_source/session5/Building-a-Perception-Pipeline.html
-  https://industrial-training-jp.readthedocs.io/ja/latest/_source/session5_JP/Building-a-Perception-Pipeline_JP.html

Quit Software
-------------

Enter ``Ctrl-C`` on the running terminal.

Launch and Run Tips
===================

Publishing Images Only
----------------------

When you publish only Depth, IR and RGB images, launch ``tof.launch`` .

::

    $ roslaunch cis_camera tof.launch

If you show the images, run ``rqt`` and open Plugins -> Visualization -> Image View.

::

    $ rqt


Dynamic Reconfigure
-------------------

After you launched ``pointcloud.launch reconfigure:=false`` or
``tof.launch``, you can also reconfigure Depth/IR configurations
dynamically with launching ``rqt_reconfigure``.

::

    $ rosrun rqt_reconfigure rqt_reconfigure

When you reconfigure Depth/IR camera distortion correction parameters,
check ``ir_dist_reconfig`` to effect parameters ``ir_fx``, ``ir_fy`` and
so on.

To set back the parameters to ``config/camera_ir.yaml`` data, uncheck
``ir_dist_reconfig``.

.. figure:: images/cis_camera_rqt_reconfigure_check-ir_dist_reconfig.png
   :alt: ir_dist_redonfig
   :width: 50%

   IR Distortion Reconfigure

Frame Rate
----------

When you want to know a frame rate of ROS topic, please run
``rostopic hz`` as below.

In the case of a topic ``/camera/depth/points``,

::

    $ rostopic hz /camera/depth/points

To find out what topics exits,

::

    $ rostopic list

Trouble Shooting
##################

Errors at Launching
=====================

Errors about set_camera_info
------------------------------

You may get errors about `set_camera_info` as shown below every time you execute ``tof.launch`` or ``pointcloud.launch``.
These errors are cause of mulutiple camera_info data and you can safely ignore them.

::

    [ERROR] [1575989725.084830797]: Tried to advertise a service that is already advertised in this node [/camera/set_camera_info]
    [ERROR] [1575989725.084902986]: Tried to advertise a service that is already advertised in this node [/camera/set_camera_info]
    [ERROR] [1575989725.084928007]: Tried to advertise a service that is already advertised in this node [/camera/set_camera_info]

Device Permission Error 
-------------------------

When you get an error that says ``Permission denied opening /dev/bus/usb/.../...`` like shown below,
it is a device permission error about your CIS ToF camera. 

::

    [ERROR] [1575991779.884027635]: Permission denied opening /dev/bus/usb/002/003
    [ERROR] [1575991779.884053757]: Please quit by 'Ctrl-C' and change the permission with 'sudo chmod o+x /dev/bus/usb/002/003'

1. Turn off your CIS camera
2. Set udev rules for CIS ToF Camera with ``rrosrun cis_camera set_udev_rules``
3. Turn on your CIS camera again

Error for a Unconnected ToF Camera
------------------------------------

When your CIS ToF camera is not connected or not turned on, you get a error as shown below. 


::

    [ERROR] [1575989725.283170878]: uvc_find_device : Error Num = -4

Please connect and turn on your camera.

Reference
###########

CIS TOF Camera ROS Driver Specific Topics
===========================================

Depth Image Topic
-------------------

-  /camera/depth/image\_raw
-  Type : sensor\_msgs/Image
-  Publisher : /camera/cistof
-  Default

   -  Width : 640 px
   -  Height : 480 px
   -  Video Mode : gray16

-  Raw image topic with gray16 scale for depth data
-  /camera/depth/image\_rect
-  /camera/depth/image\_rect\_raw

IR Image Topic
----------------

-  /camera/ir/image\_raw
-  Type : sensor\_msgs/Image
-  Publisher : /camera/cistof
-  Default

   -  Width : 640 px
   -  Height : 480 px
   -  Video Mode : gray16

-  Raw image topic with gray16 scale for IR image
-  /camera/ir/image\_rect

RGB Image Topic
-----------------

-  /camera/rgb/image\_raw
-  Type : sensor\_msgs/Image
-  Publisher : /camera/cistof
-  Default

   -  Width : 1280 px
   -  Height : 960 px
   -  Video Mode : bgr8

-  Raw image topic with bgr8 for RGB image
-  /camera/rgb/image\_rect\_color

Point Cloud Topic
-------------------

-  /camera/depth/points
-  Type: sensor\_msgs/PointCloud2
-  Publisher : /camera/camera\_nodelet\_manager

CIS TOF Camera Specific Parameters
====================================

Depth / IR Parameters
-----------------------

*depth\_range*

-  Depth Range

   -  Range 0 : Distance min: 300 [mm] - Max: 5000 [mm]
   -  Range 1 : Distance min: 150 [mm] - Max: 1500 [mm]

-  Value

   -  Range 0 : 0
   -  Range 1 : 1

-  Default : 0

*threshold*

-  Coring Threshold

   -  Increasing the value will lower the background threshold.

-  Value

   -  Maximum : 0x3FFF
   -  Minimum : 0

-  Default : 0

*nr\_filter*

-  Noise Reduction Filter ON/OFF
-  Value

   -  NR Filter ON : 1
   -  NR Filter OFF : 0

-  Default : 1

*pulse\_count*

-  Number of light emitting pulses per frame

   -  Increasing the value improves the distance measurement accuracy.

-  Value

   -  Maximum : 2000
   -  Minimum : 1

-  Default : 2000

*ld\_enable*

-  Enable LEDs

   -  LD1 ON : 0x0001
   -  LD2 ON : 0x0002

-  Value

   -  Maximum : 3
   -  Minimum : 0

-  Default : 3

*ir\_gain*

-  IR Gain
-  Value

   -  Maximum : 2047
   -  Minimum : 0

-  Default : 256

*ae\_mode*

-  Auto exposure mode
-  Value

   -  Manual : 0
   -  Auto\_Gain : 1
   -  Auto\_Shutter : 2
   -  Auto\_Full : 3

-  Default : 3

RGB Camera Prameters
----------------------

*color\_correction*

-  Color correction Mode
-  Value

   -  Off : 0
   -  Standard : 1

-  Default : 0

*brightness\_gain*

-  RGB brightness gain
-  Value

   -  min : 1.0
   -  Max : 10.67

-  Default : 1.0

*exposure\_time*

-  RGB exposure time (shutter control)
-  Value

   -  min : 0.00001
   -  MAX : 0.01

-  Default : 0.01

How to Change Parameters
--------------------------

To change the parameters,

-  run ``rqt_reconfigure`` as mentioned above.

   -  ``pointcloud.launch`` runs ``rqt_reconfigure`` by defalut.

-  add options descriptions like below when you execute a launch file.

::

    $ roslaunch cis_camera pointcloud.launch nr_filter:=0 pulse_count:=1000

If you want to display the informations about parameters when launch
files extecuted, use ``--screen`` option as below.

::

    $ roslaunch cis_camera pointcloud.launch --screen

Launch Files
============

tof.launch
----------

*Nodes*

::

    $ rosnode list
    /camera/camera_base_to_camera
    /camera/camera_ir_to_camera_depth
    /camera/camera_to_camera_color
    /camera/camera_to_camera_ir
    /camera/cistof
    /rosout


*Topics*

::

    $ rostopic list
    /camera/camera_info
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
    /camera/image_raw
    /camera/image_raw/compressed
    /camera/image_raw/compressed/parameter_descriptions
    /camera/image_raw/compressed/parameter_updates
    /camera/image_raw/compressedDepth
    /camera/image_raw/compressedDepth/parameter_descriptions
    /camera/image_raw/compressedDepth/parameter_updates
    /camera/image_raw/theora
    /camera/image_raw/theora/parameter_descriptions
    /camera/image_raw/theora/parameter_updates
    /camera/ir/camera_info
    /camera/ir/image_raw
    /camera/ir/image_raw/compressed
    /camera/ir/image_raw/compressed/parameter_descriptions
    /camera/ir/image_raw/compressed/parameter_updates
    /camera/ir/image_raw/compressedDepth
    /camera/ir/image_raw/compressedDepth/parameter_descriptions
    /camera/ir/image_raw/compressedDepth/parameter_updates
    /camera/ir/image_raw/theora
    /camera/ir/image_raw/theora/parameter_descriptions
    /camera/ir/image_raw/theora/parameter_updates
    /camera/rgb/camera_info
    /camera/rgb/image_raw
    /camera/rgb/image_raw/compressed
    /camera/rgb/image_raw/compressed/parameter_descriptions
    /camera/rgb/image_raw/compressed/parameter_updates
    /camera/rgb/image_raw/compressedDepth
    /camera/rgb/image_raw/compressedDepth/parameter_descriptions
    /camera/rgb/image_raw/compressedDepth/parameter_updates
    /camera/rgb/image_raw/theora
    /camera/rgb/image_raw/theora/parameter_descriptions
    /camera/rgb/image_raw/theora/parameter_updates
    /rosout
    /rosout_agg
    /tf

*Parameters*

::

    $ rosparam list
    /camera/cistof/ae_mode
    /camera/cistof/b_gain
    /camera/cistof/brightness_gain
    /camera/cistof/camera_info_url
    /camera/cistof/camera_info_url_color
    /camera/cistof/camera_info_url_depth
    /camera/cistof/camera_info_url_ir
    /camera/cistof/color_correction
    /camera/cistof/color_width
    /camera/cistof/depth_range
    /camera/cistof/exposure_time
    /camera/cistof/frame_id
    /camera/cistof/frame_id_color
    /camera/cistof/frame_id_depth
    /camera/cistof/frame_id_ir
    /camera/cistof/frame_rate
    /camera/cistof/g_gain
    /camera/cistof/height
    /camera/cistof/index
    /camera/cistof/ir_cx
    /camera/cistof/ir_cy
    /camera/cistof/ir_dist_reconfig
    /camera/cistof/ir_fx
    /camera/cistof/ir_fy
    /camera/cistof/ir_gain
    /camera/cistof/ir_k1
    /camera/cistof/ir_k2
    /camera/cistof/ir_k3
    /camera/cistof/ir_p1
    /camera/cistof/ir_p2
    /camera/cistof/ld_enable
    /camera/cistof/nr_filter
    /camera/cistof/product
    /camera/cistof/pulse_count
    /camera/cistof/r_gain
    /camera/cistof/rgb_cx
    /camera/cistof/rgb_cy
    /camera/cistof/rgb_dist_reconfig
    /camera/cistof/rgb_fx
    /camera/cistof/rgb_fy
    /camera/cistof/rgb_k1
    /camera/cistof/rgb_k2
    /camera/cistof/rgb_k3
    /camera/cistof/rgb_p1
    /camera/cistof/rgb_p2
    /camera/cistof/serial
    /camera/cistof/temp_time
    /camera/cistof/threshold
    /camera/cistof/timestamp_method
    /camera/cistof/vendor
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
    /camera/image_raw/compressed/format
    /camera/image_raw/compressed/jpeg_quality
    /camera/image_raw/compressed/png_level
    /camera/image_raw/compressedDepth/depth_max
    /camera/image_raw/compressedDepth/depth_quantization
    /camera/image_raw/compressedDepth/png_level
    /camera/image_raw/theora/keyframe_frequency
    /camera/image_raw/theora/optimize_for
    /camera/image_raw/theora/quality
    /camera/image_raw/theora/target_bitrate
    /camera/ir/image_raw/compressed/format
    /camera/ir/image_raw/compressed/jpeg_quality
    /camera/ir/image_raw/compressed/png_level
    /camera/ir/image_raw/compressedDepth/depth_max
    /camera/ir/image_raw/compressedDepth/depth_quantization
    /camera/ir/image_raw/compressedDepth/png_level
    /camera/ir/image_raw/theora/keyframe_frequency
    /camera/ir/image_raw/theora/optimize_for
    /camera/ir/image_raw/theora/quality
    /camera/ir/image_raw/theora/target_bitrate
    /camera/rgb/image_raw/compressed/format
    /camera/rgb/image_raw/compressed/jpeg_quality
    /camera/rgb/image_raw/compressed/png_level
    /camera/rgb/image_raw/compressedDepth/depth_max
    /camera/rgb/image_raw/compressedDepth/depth_quantization
    /camera/rgb/image_raw/compressedDepth/png_level
    /camera/rgb/image_raw/theora/keyframe_frequency
    /camera/rgb/image_raw/theora/optimize_for
    /camera/rgb/image_raw/theora/quality
    /camera/rgb/image_raw/theora/target_bitrate
    /rosdistro
    /roslaunch/uris/host_robotuser_pc__41709
    /rosversion
    /run_id

pointcloud.launch
-----------------

*Nodes*

::

    $ rosnode list
    /camera/camera_base_to_camera
    /camera/camera_ir_to_camera_depth
    /camera/camera_nodelet_manager
    /camera/camera_to_camera_color
    /camera/camera_to_camera_ir
    /camera/cistof
    /camera/depth_metric
    /camera/depth_metric_rect
    /camera/depth_points
    /camera/depth_rectify_depth
    /camera/ir_rectify_ir
    /camera/rgb_debayer
    /camera/rgb_rectify_color
    /camera/rgb_rectify_mono
    /map_to_camera_base
    /rosout
    /rqt_reconfigure
    /rviz

*Topics*

::

    $ rostopic list
    /camera/camera_info
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
    /camera/image_raw
    /camera/image_raw/compressed
    /camera/image_raw/compressed/parameter_descriptions
    /camera/image_raw/compressed/parameter_updates
    /camera/image_raw/compressedDepth
    /camera/image_raw/compressedDepth/parameter_descriptions
    /camera/image_raw/compressedDepth/parameter_updates
    /camera/image_raw/theora
    /camera/image_raw/theora/parameter_descriptions
    /camera/image_raw/theora/parameter_updates
    /camera/ir/camera_info
    /camera/ir/image_raw
    /camera/ir/image_raw/compressed
    /camera/ir/image_raw/compressed/parameter_descriptions
    /camera/ir/image_raw/compressed/parameter_updates
    /camera/ir/image_raw/compressedDepth
    /camera/ir/image_raw/compressedDepth/parameter_descriptions
    /camera/ir/image_raw/compressedDepth/parameter_updates
    /camera/ir/image_raw/theora
    /camera/ir/image_raw/theora/parameter_descriptions
    /camera/ir/image_raw/theora/parameter_updates
    /camera/ir/image_rect_ir
    /camera/ir/image_rect_ir/compressed
    /camera/ir/image_rect_ir/compressed/parameter_descriptions
    /camera/ir/image_rect_ir/compressed/parameter_updates
    /camera/ir/image_rect_ir/compressedDepth
    /camera/ir/image_rect_ir/compressedDepth/parameter_descriptions
    /camera/ir/image_rect_ir/compressedDepth/parameter_updates
    /camera/ir/image_rect_ir/theora
    /camera/ir/image_rect_ir/theora/parameter_descriptions
    /camera/ir/image_rect_ir/theora/parameter_updates
    /camera/ir_rectify_ir/parameter_descriptions
    /camera/ir_rectify_ir/parameter_updates
    /camera/rgb/camera_info
    /camera/rgb/image_color
    /camera/rgb/image_color/compressed
    /camera/rgb/image_color/compressed/parameter_descriptions
    /camera/rgb/image_color/compressed/parameter_updates
    /camera/rgb/image_color/compressedDepth
    /camera/rgb/image_color/compressedDepth/parameter_descriptions
    /camera/rgb/image_color/compressedDepth/parameter_updates
    /camera/rgb/image_color/theora
    /camera/rgb/image_color/theora/parameter_descriptions
    /camera/rgb/image_color/theora/parameter_updates
    /camera/rgb/image_mono
    /camera/rgb/image_mono/compressed
    /camera/rgb/image_mono/compressed/parameter_descriptions
    /camera/rgb/image_mono/compressed/parameter_updates
    /camera/rgb/image_mono/compressedDepth
    /camera/rgb/image_mono/compressedDepth/parameter_descriptions
    /camera/rgb/image_mono/compressedDepth/parameter_updates
    /camera/rgb/image_mono/theora
    /camera/rgb/image_mono/theora/parameter_descriptions
    /camera/rgb/image_mono/theora/parameter_updates
    /camera/rgb/image_raw
    /camera/rgb/image_raw/compressed
    /camera/rgb/image_raw/compressed/parameter_descriptions
    /camera/rgb/image_raw/compressed/parameter_updates
    /camera/rgb/image_raw/compressedDepth
    /camera/rgb/image_raw/compressedDepth/parameter_descriptions
    /camera/rgb/image_raw/compressedDepth/parameter_updates
    /camera/rgb/image_raw/theora
    /camera/rgb/image_raw/theora/parameter_descriptions
    /camera/rgb/image_raw/theora/parameter_updates
    /camera/rgb/image_rect_color
    /camera/rgb/image_rect_color/compressed
    /camera/rgb/image_rect_color/compressed/parameter_descriptions
    /camera/rgb/image_rect_color/compressed/parameter_updates
    /camera/rgb/image_rect_color/compressedDepth
    /camera/rgb/image_rect_color/compressedDepth/parameter_descriptions
    /camera/rgb/image_rect_color/compressedDepth/parameter_updates
    /camera/rgb/image_rect_color/theora
    /camera/rgb/image_rect_color/theora/parameter_descriptions
    /camera/rgb/image_rect_color/theora/parameter_updates
    /camera/rgb/image_rect_mono
    /camera/rgb/image_rect_mono/compressed
    /camera/rgb/image_rect_mono/compressed/parameter_descriptions
    /camera/rgb/image_rect_mono/compressed/parameter_updates
    /camera/rgb/image_rect_mono/compressedDepth
    /camera/rgb/image_rect_mono/compressedDepth/parameter_descriptions
    /camera/rgb/image_rect_mono/compressedDepth/parameter_updates
    /camera/rgb/image_rect_mono/theora
    /camera/rgb/image_rect_mono/theora/parameter_descriptions
    /camera/rgb/image_rect_mono/theora/parameter_updates
    /camera/rgb_debayer/parameter_descriptions
    /camera/rgb_debayer/parameter_updates
    /camera/rgb_rectify_color/parameter_descriptions
    /camera/rgb_rectify_color/parameter_updates
    /camera/rgb_rectify_mono/parameter_descriptions
    /camera/rgb_rectify_mono/parameter_updates
    /clicked_point
    /initialpose
    /move_base_simple/goal
    /object_cluster
    /rosout
    /rosout_agg
    /tf
    /tf_static

*Parameters*

::

    $ rosparam list
    /camera/camera_nodelet_manager/num_worker_threads
    /camera/cistof/ae_mode
    /camera/cistof/b_gain
    /camera/cistof/brightness_gain
    /camera/cistof/camera_info_url
    /camera/cistof/camera_info_url_color
    /camera/cistof/camera_info_url_depth
    /camera/cistof/camera_info_url_ir
    /camera/cistof/color_correction
    /camera/cistof/color_width
    /camera/cistof/depth_range
    /camera/cistof/exposure_time
    /camera/cistof/frame_id
    /camera/cistof/frame_id_color
    /camera/cistof/frame_id_depth
    /camera/cistof/frame_id_ir
    /camera/cistof/frame_rate
    /camera/cistof/g_gain
    /camera/cistof/height
    /camera/cistof/index
    /camera/cistof/ir_cx
    /camera/cistof/ir_cy
    /camera/cistof/ir_dist_reconfig
    /camera/cistof/ir_fx
    /camera/cistof/ir_fy
    /camera/cistof/ir_gain
    /camera/cistof/ir_k1
    /camera/cistof/ir_k2
    /camera/cistof/ir_k3
    /camera/cistof/ir_p1
    /camera/cistof/ir_p2
    /camera/cistof/ld_enable
    /camera/cistof/nr_filter
    /camera/cistof/product
    /camera/cistof/pulse_count
    /camera/cistof/r_gain
    /camera/cistof/rgb_cx
    /camera/cistof/rgb_cy
    /camera/cistof/rgb_dist_reconfig
    /camera/cistof/rgb_fx
    /camera/cistof/rgb_fy
    /camera/cistof/rgb_k1
    /camera/cistof/rgb_k2
    /camera/cistof/rgb_k3
    /camera/cistof/rgb_p1
    /camera/cistof/rgb_p2
    /camera/cistof/serial
    /camera/cistof/temp_time
    /camera/cistof/threshold
    /camera/cistof/timestamp_method
    /camera/cistof/vendor
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
    /camera/image_raw/compressed/format
    /camera/image_raw/compressed/jpeg_quality
    /camera/image_raw/compressed/png_level
    /camera/image_raw/compressedDepth/depth_max
    /camera/image_raw/compressedDepth/depth_quantization
    /camera/image_raw/compressedDepth/png_level
    /camera/image_raw/theora/keyframe_frequency
    /camera/image_raw/theora/optimize_for
    /camera/image_raw/theora/quality
    /camera/image_raw/theora/target_bitrate
    /camera/ir/image_raw/compressed/format
    /camera/ir/image_raw/compressed/jpeg_quality
    /camera/ir/image_raw/compressed/png_level
    /camera/ir/image_raw/compressedDepth/depth_max
    /camera/ir/image_raw/compressedDepth/depth_quantization
    /camera/ir/image_raw/compressedDepth/png_level
    /camera/ir/image_raw/theora/keyframe_frequency
    /camera/ir/image_raw/theora/optimize_for
    /camera/ir/image_raw/theora/quality
    /camera/ir/image_raw/theora/target_bitrate
    /camera/ir/image_rect_ir/compressed/format
    /camera/ir/image_rect_ir/compressed/jpeg_quality
    /camera/ir/image_rect_ir/compressed/png_level
    /camera/ir/image_rect_ir/compressedDepth/depth_max
    /camera/ir/image_rect_ir/compressedDepth/depth_quantization
    /camera/ir/image_rect_ir/compressedDepth/png_level
    /camera/ir/image_rect_ir/theora/keyframe_frequency
    /camera/ir/image_rect_ir/theora/optimize_for
    /camera/ir/image_rect_ir/theora/quality
    /camera/ir/image_rect_ir/theora/target_bitrate
    /camera/ir_rectify_ir/interpolation
    /camera/rgb/image_color/compressed/format
    /camera/rgb/image_color/compressed/jpeg_quality
    /camera/rgb/image_color/compressed/png_level
    /camera/rgb/image_color/compressedDepth/depth_max
    /camera/rgb/image_color/compressedDepth/depth_quantization
    /camera/rgb/image_color/compressedDepth/png_level
    /camera/rgb/image_color/theora/keyframe_frequency
    /camera/rgb/image_color/theora/optimize_for
    /camera/rgb/image_color/theora/quality
    /camera/rgb/image_color/theora/target_bitrate
    /camera/rgb/image_mono/compressed/format
    /camera/rgb/image_mono/compressed/jpeg_quality
    /camera/rgb/image_mono/compressed/png_level
    /camera/rgb/image_mono/compressedDepth/depth_max
    /camera/rgb/image_mono/compressedDepth/depth_quantization
    /camera/rgb/image_mono/compressedDepth/png_level
    /camera/rgb/image_mono/theora/keyframe_frequency
    /camera/rgb/image_mono/theora/optimize_for
    /camera/rgb/image_mono/theora/quality
    /camera/rgb/image_mono/theora/target_bitrate
    /camera/rgb/image_raw/compressed/format
    /camera/rgb/image_raw/compressed/jpeg_quality
    /camera/rgb/image_raw/compressed/png_level
    /camera/rgb/image_raw/compressedDepth/depth_max
    /camera/rgb/image_raw/compressedDepth/depth_quantization
    /camera/rgb/image_raw/compressedDepth/png_level
    /camera/rgb/image_raw/theora/keyframe_frequency
    /camera/rgb/image_raw/theora/optimize_for
    /camera/rgb/image_raw/theora/quality
    /camera/rgb/image_raw/theora/target_bitrate
    /camera/rgb/image_rect_color/compressed/format
    /camera/rgb/image_rect_color/compressed/jpeg_quality
    /camera/rgb/image_rect_color/compressed/png_level
    /camera/rgb/image_rect_color/compressedDepth/depth_max
    /camera/rgb/image_rect_color/compressedDepth/depth_quantization
    /camera/rgb/image_rect_color/compressedDepth/png_level
    /camera/rgb/image_rect_color/theora/keyframe_frequency
    /camera/rgb/image_rect_color/theora/optimize_for
    /camera/rgb/image_rect_color/theora/quality
    /camera/rgb/image_rect_color/theora/target_bitrate
    /camera/rgb/image_rect_mono/compressed/format
    /camera/rgb/image_rect_mono/compressed/jpeg_quality
    /camera/rgb/image_rect_mono/compressed/png_level
    /camera/rgb/image_rect_mono/compressedDepth/depth_max
    /camera/rgb/image_rect_mono/compressedDepth/depth_quantization
    /camera/rgb/image_rect_mono/compressedDepth/png_level
    /camera/rgb/image_rect_mono/theora/keyframe_frequency
    /camera/rgb/image_rect_mono/theora/optimize_for
    /camera/rgb/image_rect_mono/theora/quality
    /camera/rgb/image_rect_mono/theora/target_bitrate
    /camera/rgb_debayer/debayer
    /camera/rgb_rectify_color/interpolation
    /camera/rgb_rectify_mono/interpolation
    /rosdistro
    /roslaunch/uris/host_robotuser_pc__34749
    /rosversion
    /run_id

