// Copyright (c) 2019, Analog Devices Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
// * Redistributions of source code must retain the above copyright
//   notice, this list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in
//   the documentation and/or other materials provided with the
//   distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
// INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
// HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
// STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
// OF THE POSSIBILITY OF SUCH DAMAGE.

// Based on original source from Ken Tossell's libuvc_camera. License copied below.
//
// * fix code style for ROS coding guidelines
// * change namespace from libuvc_camera to cis_camera
// * add functions and variables for CIS ToF camera sensor controls

/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (C) 2012 Ken Tossell
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the author nor other contributors may be
*     used to endorse or promote products derived from this software
*     without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/


#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>

#include <cis_camera/CISCameraConfig.h>


namespace cis_camera
{

/**
 * @brief The CameraDriver class is a ROS device driver of CIS ToF Camera Sensor.
 * The CameraDriver class gets/sets configurations of the ToF camera sensor, 
 * aquires RGB, IR and Depth Images from the camera sensor and publishes these images.
 */
class CameraDriver
{
public:
  
  CameraDriver( ros::NodeHandle nh, ros::NodeHandle priv_nh );
  ~CameraDriver();
  
  bool Start();
  void Stop();
  
  int  setToFMode_All();
  void getToFInfo_All();
  void getRGBInfo_All();
  
  
private:
  
  enum State
  {
    Initial = 0,
    Stopped = 1,
    Running = 2,
  };
  
  // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
  static const int ReconfigureClose   = 3; // Need to close and reopen sensor to change this setting
  static const int ReconfigureStop    = 1; // Need to stop the stream before changing this setting
  static const int ReconfigureRunning = 0; // We can change this setting without stopping the stream
  
  void readConfigFromParameterServer();
  void advertiseROSTopics();
  void OpenCamera();
  void CloseCamera();
  
  // Accept a reconfigure request from a client
  void ReconfigureCallback( CISCameraConfig &config, uint32_t level );
  
  // Accept a new image frame from the camera
  void ImageCallback( uvc_frame_t *frame );
  static void ImageCallbackAdapter( uvc_frame_t *frame, void *ptr );
  
  enum uvc_extention_unit_control_number
  {
    UVC_XU_CTRL_TOF = 3,
    UVC_XU_CTRL_RGB = 9,
  };
  
  enum tof_process_num
  {
    TOF_SET_EEPROM          = 0x0000,
    TOF_SET_DEPTH_RANGE     = 0x0002,
    TOF_SET_THRESHOLD       = 0x0003,
    TOF_SET_NR_FILTER       = 0x0004,
    TOF_SET_PULSE_COUNT     = 0x0005,
    TOF_SET_LD_ENABLE       = 0x0006,
    TOF_SET_IR_GAIN         = 0x0009,
    TOF_SET_ERROR_STOP      = 0x0010,
    TOF_SET_ERROR_CLEAR     = 0x7F01,
    TOF_GET_DEPTH_RANGE     = 0x8002,
    TOF_GET_THRESHOLD       = 0x8003,
    TOF_GET_NR_FILTER       = 0x8004,
    TOF_GET_PULSE_COUNT     = 0x8005,
    TOF_GET_LD_ENABLE       = 0x8006,
    TOF_GET_DEPTH_CNV_GAIN  = 0x8007,
    TOF_GET_DEPTH_INFO      = 0x8008,
    TOF_GET_IR_GAIN         = 0x8009,
    TOF_GET_TEMPERATURE     = 0x800A,
    TOF_GET_LD_PULSE_WIDTH  = 0x800B,
    TOF_GET_VERSION         = 0xFF00,
    TOF_GET_ERROR_INFO      = 0xFF01,
  };
  
  enum tof_eeprom_mode
  {
    TOF_EEPROM_FACTORY_DEFAULT  = 0x0000,
    TOF_EEPROM_UPDATE_CURRENT   = 0x0001,
  };
  
  enum rgb_process_num
  {
    RGB_SET_WHITE_BALANCE    = 0x0000,
    RGB_SET_AE_MODE          = 0x0001,
    RGB_SET_BRIGHTNESS_GAIN  = 0x0002,
    RGB_SET_SHUTTER_CONTROL  = 0x0003,
    RGB_SET_COLOR_CORRECTION = 0x0005,
    RGB_GET_AE_MODE          = 0x8001,
    RGB_GET_BRIGHTNESS_GAIN  = 0x8002,
    RGB_GET_SHUTTER_CONTROL  = 0x8003,
    RGB_GET_COLOR_CORRECTION = 0x8005,
  };
  
  int setCameraCtrl( uint8_t unit, uint16_t *data, int len );
  int getCameraCtrl( uint8_t unit, uint16_t *data, int len );
  
  int setToFMode_ROSParameter( std::string param_name, double param );
  int setToFMode_ROSParameter( std::string param_name, int param );
  int setToFMode_ROSParameter( std::string param_name, int param, int param_2 );
  
  int setToFEEPROMMode( uint16_t mode );
  int clearToFError();
  
  int getToFDepthRange( uint16_t& depth_range, uint16_t& dr_index );
  int getToFThreshold( uint16_t& threshold );
  int getToFNRFilter( uint16_t& nr_filter );
  int getToFPulseCount( uint16_t& pulse_count );
  int getToFLDEnable( uint16_t& ld_enable_near, uint16_t& ld_enable_wide );
  int getToFDepthCnvGain( double& depth_cnv_gain );
  int getToFDepthInfo( short&      depth_offset,
                       unsigned short& max_data,
                       unsigned short& min_dist,
                       unsigned short& max_dist );
  int getToFIRGain( uint16_t& ir_gain );
  int getToFTemperature( double& t1, double& t2 );
  int getToFLDPulseWidth( int& time_near, int& time_wide );
  int getToFVersion( uint16_t& version_n,
                     uint16_t& build_n,
                     uint16_t& build_y,
                     uint16_t& build_d );
  int getToFErrorInfo( uint16_t& common_err,
                       uint16_t& eeprom_err_factory,
                       uint16_t& eeprom_err,
                       uint16_t& mipi_temp_err );
  
  int setRGBAEMode();
  int setRGBColorCorrection();
  
  int getRGBAEMode( uint16_t& ae_mode );
  int getRGBBrightnessGain( double& brightness_gain, double& brightness_maxg );
  int getRGBShutterControl( double& exposure_time, double& exposure_maxt );
  int getRGBColorCorrection( uint16_t& color_correction );
  
  ros::Publisher pub_tof_t1_;
  ros::Publisher pub_tof_t2_;
  
  ros::Timer temp_timer_;
  
  static void TemperatureCallback( void* ptr );
  
  void publishToFTemperature();
  
  double depth_cnv_gain_;
  short  depth_offset_;
  
  ros::NodeHandle nh_, priv_nh_;
  
  State                  state_;
  boost::recursive_mutex mutex_;
  
  uvc_context_t       *ctx_;
  uvc_device_t        *dev_;
  uvc_device_handle_t *devh_;
  uvc_frame_t         *rgb_frame_;
  
  image_transport::ImageTransport  it_;
  image_transport::CameraPublisher pub_camera_;
  image_transport::CameraPublisher pub_color_;
  image_transport::CameraPublisher pub_depth_;
  image_transport::CameraPublisher pub_ir_;
  
  dynamic_reconfigure::Server<CISCameraConfig> config_server_;
  
  CISCameraConfig config_;
  bool            config_changed_;
  
  camera_info_manager::CameraInfoManager cinfo_manager_;
  camera_info_manager::CameraInfoManager cinfo_manager_ir_;
  camera_info_manager::CameraInfoManager cinfo_manager_depth_;
  camera_info_manager::CameraInfoManager cinfo_manager_color_;
  
  std::string camera_info_url_;
  std::string camera_info_url_ir_;
  std::string camera_info_url_depth_;
  std::string camera_info_url_color_;
  
  double r_gain_, g_gain_, b_gain_;
  
};

};
