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
// * add and modify functions for CIS ToF camera sensor controls

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


#include "cis_camera/camera_driver.h"

#include <unistd.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Temperature.h>
#include <std_msgs/Header.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <libuvc/libuvc.h>
#include <math.h>

namespace cis_camera
{

/**
 * @brief CameraDriver is a constructor of the CameraDriver class.
 * @param nh is a ROS node handler.
 * @param priv_nh is a ROS private node handler.
 */
CameraDriver::CameraDriver( ros::NodeHandle nh, ros::NodeHandle priv_nh ) :
    nh_(nh),
    priv_nh_(priv_nh),
    state_(Initial),
    ctx_(NULL), 
    dev_(NULL),
    devh_(NULL), 
    rgb_frame_(NULL),
    it_(nh_),
    config_server_(mutex_, priv_nh_),
    config_changed_(false),
    cinfo_manager_(nh),
    cinfo_manager_ir_(nh),
    cinfo_manager_depth_(nh),
    cinfo_manager_color_(nh)
{
  readConfigFromParameterServer();
  advertiseROSTopics();
}


/**
 * @brief ~CameraDriver destroys dev_ devh_ and so on for destructiion of the CameraDriver class.
 */
CameraDriver::~CameraDriver()
{
  if ( rgb_frame_ )
    uvc_free_frame( rgb_frame_ );
  
  if ( ctx_ )
    uvc_exit( ctx_ );  // Destroys dev_, devh_, etc.
}


/**
 * @brief readConfigFromParameterServer reads camera infomations from the ROS parameter server.
 */
void CameraDriver::readConfigFromParameterServer()
{
  int err;
  
  err = priv_nh_.getParam( "camera_info_url"      , camera_info_url_       );
  err = priv_nh_.getParam( "camera_info_url_ir"   , camera_info_url_ir_    );
  err = priv_nh_.getParam( "camera_info_url_depth", camera_info_url_depth_ );
  err = priv_nh_.getParam( "camera_info_url_color", camera_info_url_color_ );
}


/**
 * @brief advertiseROSTopics advertises topic publishers for images and temperatures.
 */
void CameraDriver::advertiseROSTopics()
{
  // Remapping namespaces
  ros::NodeHandle color_nh( nh_, "rgb" );
  image_transport::ImageTransport color_it( color_nh );
  
  ros::NodeHandle depth_nh( nh_, "depth" );
  image_transport::ImageTransport depth_it( depth_nh );
  
  ros::NodeHandle ir_nh( nh_, "ir" );
  image_transport::ImageTransport ir_it( ir_nh );
  
  // Advertise Camera Pubishers
  pub_camera_ = it_.advertiseCamera( "image_raw", 1, false );
  pub_color_  = color_it.advertiseCamera( "image_raw", 1, false );
  pub_depth_  = depth_it.advertiseCamera( "image_raw", 1, false );
  pub_ir_     = ir_it.advertiseCamera( "image_raw", 1, false );
  
  // Set Publishers for TOF Camera Temperature
  std::string node_name = ros::this_node::getName();
  pub_tof_t1_ = nh_.advertise<sensor_msgs::Temperature>( node_name + "/t1", 1000 );
  pub_tof_t2_ = nh_.advertise<sensor_msgs::Temperature>( node_name + "/t2", 1000 );
  
  return;
}


/**
 * @brief Start initialize the CIS Tof Camera Sensora as a uvc
 * and set a callback function for dynamic reconfigure.
 * @return State of the uvc status.
 */
bool CameraDriver::Start()
{
  uvc_error_t err;
  
  err = uvc_init( &ctx_, NULL );
  
  if ( err != UVC_SUCCESS )
  {
    ROS_ERROR( "ERROR: uvc_init" );
    return false;
  }
  
  state_ = Stopped;
  
  config_server_.setCallback( boost::bind( &CameraDriver::ReconfigureCallback, this, _1, _2 ) );
  
  return state_ == Running;
}


/**
 * @brief Stop performs as the camera driver stops.
 * @return State of the uvc status.
 */
void CameraDriver::Stop()
{
  boost::recursive_mutex::scoped_lock( mutex_ );
  
  if ( state_ == Running )
    CloseCamera();
  
  uvc_exit( ctx_ );
  ctx_ = NULL;
  
  state_ = Initial;
}


/**
 * @brief ReconfiureCallback is a callback method for a dynamic reconfigure.
 * This method sets camera paremeters with the setToFMode_ROSParameter method.
 */
void CameraDriver::ReconfigureCallback( CISCameraConfig &new_config, uint32_t level )
{
  boost::recursive_mutex::scoped_lock( mutex_ );
  
  if ( (level & ReconfigureClose) == ReconfigureClose )
  {
    if ( state_ == Running )
      CloseCamera();
  }
  
  if ( state_ == Stopped )
  {
    OpenCamera();
  }
  
  if ( state_ == Running )
  {
    if ( new_config.depth_range != config_.depth_range )
    {
      setToFMode_ROSParameter( "depth_range", new_config.depth_range );
    }
    
    if ( new_config.threshold != config_.threshold )
    {
      setToFMode_ROSParameter( "threshold", new_config.threshold );
    }
    
    if ( new_config.nr_filter != config_.nr_filter )
    {
      setToFMode_ROSParameter( "nr_filter", new_config.nr_filter );
    }
    
    if ( new_config.pulse_count != config_.pulse_count )
    {
      setToFMode_ROSParameter( "pulse_count", new_config.pulse_count );
    }
    
    if ( new_config.ld_enable != config_.ld_enable )
    {
      setToFMode_ROSParameter( "ld_enable", new_config.ld_enable );
    }
    
    if ( new_config.ir_gain != config_.ir_gain )
    {
      setToFMode_ROSParameter( "ir_gain", new_config.ir_gain );
    }
    
    if ( new_config.ae_mode != config_.ae_mode )
    {
      setToFMode_ROSParameter( "ae_mode", new_config.ae_mode );
    }
    
    if ( new_config.brightness_gain != config_.brightness_gain )
    {
      setToFMode_ROSParameter( "brightness_gain", new_config.brightness_gain );
    }
    
    if ( new_config.exposure_time != config_.exposure_time )
    {
      setToFMode_ROSParameter( "exposure_time", new_config.exposure_time );
    }
    
    if ( new_config.color_correction != config_.color_correction )
    {
      setToFMode_ROSParameter( "color_correction", new_config.color_correction );
    }
    
    if ( new_config.r_gain != config_.r_gain )
    {
      r_gain_ = new_config.r_gain;
    }
    if ( new_config.g_gain != config_.g_gain )
    {
      g_gain_ = new_config.g_gain;
    }
    if ( new_config.b_gain != config_.b_gain )
    {
      b_gain_ = new_config.b_gain;
    }
  }
  
  config_changed_ = true;
  config_ = new_config;
  
  return;
}


/**
 * @brief cvtDoubleToByte converts double type value to 0-255 limited integer.
 * @param x double value to convert
 * @return uint8_t of the limited integer
 */
uint8_t cvtDoubleToByte( double x )
{
  if ( x < 0 )        return 0;
  else if ( 255 < x ) return 255;
  
  return static_cast<uint8_t>( x );
}

/**
 * @brief ImageCallback is a method to process a camera image.
 * This method disassembles the whole one image in *frame to a color image, 
 * an IR image and a depth image. The color image is converted from yuv422 data
 * to bgr8 data. The depth data is converted from the distances from the camera
 * element to the distances from the camera plane.
 * The images are published as ROS sensor_msgs::Image topics.
 * @param *frame uvc_frame_t image frame pointer of RGB/IR/Depth combined data
 */
void CameraDriver::ImageCallback( uvc_frame_t *frame )
{
  ros::Time timestamp = ros::Time( frame->capture_time.tv_sec, frame->capture_time.tv_usec * 1000 );
  if ( timestamp == ros::Time(0) )
  {
    timestamp = ros::Time::now();
  }
  
  boost::recursive_mutex::scoped_lock(mutex_);
  
  if ( state_ != Running || not rgb_frame_ )
  {
    return;
  }
  
  // Checking Depth Conversion Gain
  if ( depth_cnv_gain_ <= 0.000001 )
  {
    double dcg = depth_cnv_gain_;
    getToFDepthCnvGain( depth_cnv_gain_ );
    ROS_WARN( "Wrong Depth Cnv Gain: %lf -> Re-get Depth Cnv Gain: %lf", dcg, depth_cnv_gain_ );
    
    unsigned short max_data;
    unsigned short min_dist;
    unsigned short max_dist;
    getToFDepthInfo( depth_offset_, max_data, min_dist, max_dist );
    ROS_INFO( "Get Depth Info - Offset: %d / Max Data : %d / min Distance : %d [mm] MAX Distance :%d [mm]",
                depth_offset_, max_data, min_dist, max_dist );
  }
  
  int    err;
  int    frame_width  = 1920;
  int    frame_height = 960;
  int    color_width  = 1280;
  double frame_rate   = 30.0;
  
  err = priv_nh_.getParam( "width"      , frame_width  );
  err = priv_nh_.getParam( "height"     , frame_height );
  err = priv_nh_.getParam( "color_width", color_width  );
  
  sensor_msgs::Image::Ptr image( new sensor_msgs::Image() );
  image->width  = frame_width;
  image->height = frame_height;
  image->step   = image->width * 2;
  image->data.resize( image->step * image->height );
  
  sensor_msgs::Image::Ptr image_bgr8( new sensor_msgs::Image() );
  sensor_msgs::Image::Ptr image_depth( new sensor_msgs::Image() );
  sensor_msgs::Image::Ptr image_ir( new sensor_msgs::Image() );
  
  sensor_msgs::CameraInfo::Ptr cinfo( new sensor_msgs::CameraInfo( cinfo_manager_.getCameraInfo() ) );
  sensor_msgs::CameraInfo::Ptr cinfo_ir( new sensor_msgs::CameraInfo( cinfo_manager_ir_.getCameraInfo() ) );
  sensor_msgs::CameraInfo::Ptr cinfo_depth( new sensor_msgs::CameraInfo( cinfo_manager_depth_.getCameraInfo() ) );
  sensor_msgs::CameraInfo::Ptr cinfo_color( new sensor_msgs::CameraInfo( cinfo_manager_color_.getCameraInfo() ) );
  
  std::string frame_id;
  std::string frame_id_ir;
  std::string frame_id_depth;
  std::string frame_id_color;
  err = priv_nh_.getParam( "frame_id"      , frame_id       );
  err = priv_nh_.getParam( "frame_id_ir"   , frame_id_ir    );
  err = priv_nh_.getParam( "frame_id_depth", frame_id_depth );
  err = priv_nh_.getParam( "frame_id_color", frame_id_color );
  
  if ( frame->frame_format == UVC_FRAME_FORMAT_GRAY16 )
  {
    if ( frame->data_bytes != ( frame_width * frame_height * sizeof(uint16_t) ) )
    {
      ROS_WARN( "Image Frame: Unexpected Data Size (%ld Bytes) - Skip this frame."
                , frame->data_bytes );
      return;
    }
    
    image->encoding = "16UC1";
    image->step     = image->width * 2;
    image->data.resize( image->step * image->height );
    memcpy( &(image->data[0]), frame->data, frame->data_bytes );
    
    uint16_t* data = reinterpret_cast<uint16_t*>( &(image->data[0]) );
    
    // Cropping Color Image Frame
    int color_height = frame_height;
    
    uint16_t color_data[ color_width * color_height ];
    
    int m = 0;
    int n = 0;
    for ( int i=0; i < color_height; i++ )
    {
      m = i * frame_width;
      n = i * color_width;
      memcpy( &(color_data[n]), &(data[m]), color_width * sizeof(uint16_t) );
    }
    
    // Converting YUV422 to BGR8
    image_bgr8->encoding = "bgr8";
    image_bgr8->width  = color_width;
    image_bgr8->height = color_height;
    image_bgr8->step   = image_bgr8->width * 3;
    image_bgr8->data.resize( image_bgr8->step * image_bgr8->height );
    
    uint8_t *uyvy_ptr;
    uint8_t *bgr8_ptr;
    
    uyvy_ptr = reinterpret_cast<uint8_t*>( &(color_data[0]) );
    bgr8_ptr = reinterpret_cast<uint8_t*>( &(image_bgr8->data[0]) );
    
    double u0, y0, v0, y1;
    double r0, g0, b0;
    
    int half_pixels = color_width * color_height / 2;
    for ( int i = 0; i < half_pixels; i++ )
    {
      u0 = static_cast<double>( *(uyvy_ptr)   );
      y0 = static_cast<double>( *(uyvy_ptr+1) );
      v0 = static_cast<double>( *(uyvy_ptr+2) );
      y1 = static_cast<double>( *(uyvy_ptr+3) );
      
      r0 = 1.574800 * ( v0 - 128 );
      g0 = 0.187324 * ( u0 - 128 ) - 0.468124 * ( v0 - 128 );
      b0 = 1.855600 * ( u0 - 128 );
      
      *(bgr8_ptr)   = cvtDoubleToByte( ( y0 + b0 ) * b_gain_ );
      *(bgr8_ptr+1) = cvtDoubleToByte( ( y0 + g0 ) * g_gain_ );
      *(bgr8_ptr+2) = cvtDoubleToByte( ( y0 + r0 ) * r_gain_ );
      
      *(bgr8_ptr+3) = cvtDoubleToByte( ( y1 + b0 ) * b_gain_ );
      *(bgr8_ptr+4) = cvtDoubleToByte( ( y1 + g0 ) * g_gain_ );
      *(bgr8_ptr+5) = cvtDoubleToByte( ( y1 + r0 ) * r_gain_ );
      
      uyvy_ptr += 4;
      bgr8_ptr += 6;
    }
    
    // Camera Info. Dynamic Reconfigure
    bool rgb_dist_reconfig;
    priv_nh_.getParam( "rgb_dist_reconfig", rgb_dist_reconfig );
    if( rgb_dist_reconfig )
    {
      double rgb_fx, rgb_fy, rgb_cx, rgb_cy;
      double rgb_k1, rgb_k2, rgb_k3, rgb_p1, rgb_p2;
      
      priv_nh_.getParam( "rgb_fx", rgb_fx );
      priv_nh_.getParam( "rgb_fy", rgb_fy );
      priv_nh_.getParam( "rgb_cx", rgb_cx );
      priv_nh_.getParam( "rgb_cy", rgb_cy );
      priv_nh_.getParam( "rgb_k1", rgb_k1 );
      priv_nh_.getParam( "rgb_k2", rgb_k2 );
      priv_nh_.getParam( "rgb_k3", rgb_k3 );
      priv_nh_.getParam( "rgb_p1", rgb_p1 );
      priv_nh_.getParam( "rgb_p2", rgb_p2 );
      
      cinfo_color->K[0] = rgb_fx;
      cinfo_color->K[4] = rgb_fy;
      cinfo_color->K[2] = rgb_cx;
      cinfo_color->K[5] = rgb_cy;
      cinfo_color->D[0] = rgb_k1;
      cinfo_color->D[1] = rgb_k2;
      cinfo_color->D[2] = rgb_p1;
      cinfo_color->D[3] = rgb_p2;
      cinfo_color->D[4] = rgb_k3;
    }

    // Cropping Depth and IR Image Frame
    int depth_width  = frame_width - color_width;
    int depth_height = frame_height / 2;
    
    image_depth->encoding = "16UC1";
    image_depth->width  = depth_width;
    image_depth->height = depth_height;
    image_depth->step   = image_depth->width * 2;
    image_depth->data.resize( image_depth->step * image_depth->height );
    
    uint16_t depth_data[ depth_width * depth_height ];
    
    image_ir->encoding = "16UC1";
    image_ir->width  = depth_width;
    image_ir->height = depth_height;
    image_ir->step   = image_ir->width * 2;
    image_ir->data.resize( image_ir->step * image_ir->height );
    
    uint16_t ir_data[ depth_width * depth_height ];
    
    int offset_x = color_width;
    int offset_y = 0;
    m = 0;
    n = 0;
    for ( int i=0; i < depth_height; i++ )
    {
      m = ( 2 * i + offset_y ) * frame_width + offset_x; // Interlace
      n = i * depth_width;
      memcpy( &(depth_data[n]), &(data[m]), depth_width * sizeof(uint16_t) );
      
      m += frame_width;
      memcpy( &(ir_data[n]), &(data[m]), depth_width * sizeof(uint16_t) );
    }
    
    // Depth Data Modification for Cartesian Coordinate System
    double fx, fy, cx, cy;
    double k1, k2, k3, p1, p2;
    
    bool ir_dist_reconfig;
    priv_nh_.getParam( "ir_dist_reconfig", ir_dist_reconfig );
    if( ir_dist_reconfig )
    {
      priv_nh_.getParam( "ir_fx", fx );
      priv_nh_.getParam( "ir_fy", fy );
      priv_nh_.getParam( "ir_cx", cx );
      priv_nh_.getParam( "ir_cy", cy );
      priv_nh_.getParam( "ir_k1", k1 );
      priv_nh_.getParam( "ir_k2", k2 );
      priv_nh_.getParam( "ir_k3", k3 );
      priv_nh_.getParam( "ir_p1", p1 );
      priv_nh_.getParam( "ir_p2", p2 );
      
      cinfo_ir->K[0] = fx;
      cinfo_ir->K[4] = fy;
      cinfo_ir->K[2] = cx;
      cinfo_ir->K[5] = cy;
      cinfo_ir->D[0] = k1;
      cinfo_ir->D[1] = k2;
      cinfo_ir->D[2] = p1;
      cinfo_ir->D[3] = p2;
      cinfo_ir->D[4] = k3;
      
      cinfo_depth->K[0] = fx;
      cinfo_depth->K[4] = fy;
      cinfo_depth->K[2] = cx;
      cinfo_depth->K[5] = cy;
      cinfo_depth->D[0] = k1;
      cinfo_depth->D[1] = k2;
      cinfo_depth->D[2] = p1;
      cinfo_depth->D[3] = p2;
      cinfo_depth->D[4] = k3;
    }
    else
    {
      fx = cinfo_depth->K[0];
      fy = cinfo_depth->K[4];
      cx = cinfo_depth->K[2];
      cy = cinfo_depth->K[5];
      k1 = cinfo_depth->D[0];
      k2 = cinfo_depth->D[1];
      k3 = cinfo_depth->D[4];
      p1 = cinfo_depth->D[2];
      p2 = cinfo_depth->D[3];
    }
    
    double xp, yp, x2, y2, r2, r4, r6, k0, s0;
    double xp_mod, yp_mod;
    
    if ( fx <= 0 ) fx = depth_width / 2;
    if ( fy <= 0 ) fy = depth_height / 2;
    
    for ( int i=0; i< depth_height; i++ )
    {
      yp = ( i - cy ) / fy;
      y2 = yp * yp;
      
      for ( int j=0; j < depth_width; j++ )
      {
        xp = ( j - cx ) / fx;
        x2 = xp * xp;
        
        // Lens Distortion Correction
        r2  = x2 + y2;
        r4  = r2 * r2;
        r6  = r2 * r4;
        k0  = 1.0 + k1 * r2 + k2 * r4 + k3 * r6;
        xp_mod = xp * k0 + 2.0 * p1 * xp * yp + p2 * ( r2 + 2.0 * x2 );
        yp_mod = yp * k0 + 2.0 * p2 * xp * yp + p1 * ( r2 + 2.0 * y2 );
        
        s0 = sqrt( fabs( xp_mod * xp_mod + yp_mod * yp_mod + 1.0 ) );
        
        if ( s0 <= 0 ) s0 = 1.0;
        
        depth_data[ i*depth_width + j ] = (uint16_t)( floor( ( depth_data[ i*depth_width + j ] * depth_cnv_gain_ * 4.0 + depth_offset_ ) / s0 + 0.5 ) );
        
      }
    }
    
    memcpy( &(image_depth->data[0]), depth_data, depth_width * depth_height * sizeof(uint16_t) );
    memcpy( &(image_ir->data[0]), ir_data, depth_width * depth_height * sizeof(uint16_t) );
    
  }
  else
  {
    return;
  }
  
  image->header.frame_id = frame_id;
  image->header.stamp    = timestamp;
  
  image_ir->header.frame_id = frame_id_ir;
  image_ir->header.stamp    = timestamp;
  
  image_depth->header.frame_id = frame_id_depth;
  image_depth->header.stamp    = timestamp;
  
  image_bgr8->header.frame_id = frame_id_color;
  image_bgr8->header.stamp    = timestamp;
  
  cinfo->header.frame_id = frame_id;
  cinfo->header.stamp    = timestamp;
  
  cinfo_ir->header.frame_id = frame_id_ir;
  cinfo_ir->header.stamp    = timestamp;
  
  cinfo_depth->header.frame_id = frame_id_depth;
  cinfo_depth->header.stamp    = timestamp;
  
  cinfo_color->header.frame_id = frame_id_color;
  cinfo_color->header.stamp    = timestamp;
  
  pub_camera_.publish( image, cinfo );
  pub_ir_.publish( image_ir, cinfo_ir );
  pub_depth_.publish( image_depth, cinfo_depth );
  pub_color_.publish( image_bgr8, cinfo_color );
  
}


/**
 * @brief ImageCallbackAdapter is a callback method for the camera image.
 * This method calls ImageCallback method to process a camera image.
 * @param *frame uvc_frame_t pointer of image frame
 * @param *ptr void pointer of this camera driver
 */
void CameraDriver::ImageCallbackAdapter( uvc_frame_t *frame, void *ptr )
{
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);
  
  driver->ImageCallback( frame );
}


/**
 * @breif Opening and setting up a Tof camera.
 */
void CameraDriver::OpenCamera()
{
  int err;
  int vendor_id  = 0;
  int product_id = 0;
  int index_id   = 0;
  
  std::string param_st;
  
  std::string serial_id  = "0";
  
  err = priv_nh_.getParam( "vendor" , param_st );
  vendor_id  = strtol( param_st.c_str(), NULL, 0 );
  
  err = priv_nh_.getParam( "product", param_st );
  product_id = strtol( param_st.c_str(), NULL, 0 );
  
  err = priv_nh_.getParam( "serial" , param_st );
  serial_id  = param_st;
  
  err = priv_nh_.getParam( "index"  , param_st );
  index_id   = strtol( param_st.c_str(), NULL, 0 );
  
  ROS_INFO( "Opening camera with vendor=0x%x, product=0x%x, serial=\"%s\", index=%d",
            vendor_id, product_id, serial_id.c_str(), index_id );
  
  uvc_device_t **devs;
  
  uvc_error_t find_err = uvc_find_devices(
      ctx_, &devs,
      vendor_id,
      product_id,
      serial_id.empty() ? NULL : serial_id.c_str() );
  
  if ( find_err != UVC_SUCCESS )
  {
    ROS_ERROR( "uvc_find_device : Error Num = %d", find_err );
    return;
  }
  
  // select device by index
  dev_ = NULL;
  int dev_idx = 0;
  while ( devs[dev_idx] != NULL )
  {
    if ( dev_idx == index_id )
    {
      dev_ = devs[dev_idx];
    }
    else
    {
      uvc_unref_device( devs[dev_idx] );
    }
    dev_idx++;
  }
  
  if ( dev_ == NULL )
  {
    ROS_ERROR( "Unable to find device at index %d", index_id );
    return;
  }
  
  uvc_error_t open_err = uvc_open( dev_, &devh_ );
  
  if ( open_err != UVC_SUCCESS )
  {
    int bus_num = uvc_get_bus_number( dev_ );
    int dev_adr = uvc_get_device_address( dev_ );
    
    switch ( open_err )
    {
      case UVC_ERROR_ACCESS:
        ROS_ERROR( "Permission denied opening /dev/bus/usb/%03d/%03d" , bus_num, dev_adr );
        ROS_ERROR( "Please quit by 'Ctrl-C' and change the permission with 'sudo chmod o+x /dev/bus/usb/%03d/%03d'", bus_num, dev_adr );
        break;
      default:
        ROS_ERROR( "Can't open /dev/bus/usb/%03d/%03d: %s (%d)", bus_num, dev_adr, 
                    uvc_strerror( open_err ), open_err );
        break;
    }
    
    uvc_unref_device( dev_ );
    return;
  }
  
  int         frame_width  = 1920;
  int         frame_height = 960;
  double      frame_rate   = 30.0;
  std::string video_mode   = "uncompressed";
  
  err = priv_nh_.getParam( "width"  , frame_width  );
  err = priv_nh_.getParam( "height" , frame_height );
  err = priv_nh_.getParam( "frame_rate" , frame_rate  );
  err = priv_nh_.getParam( "video_mode" , video_mode  );
  
  uvc_stream_ctrl_t ctrl;
  
  uvc_error_t mode_err;
  mode_err = uvc_get_stream_ctrl_format_size(
      devh_, &ctrl,
      UVC_COLOR_FORMAT_GRAY16,
      frame_width, frame_height,
      frame_rate );
  
  if ( mode_err != UVC_SUCCESS )
  {
    ROS_ERROR( "uvc_get_stream_ctrl_format_size" );
    uvc_close( devh_ );
    uvc_unref_device( dev_ );
    ROS_ERROR( "check video_mode/width/height/frame_rate are available" );
    uvc_print_diag( devh_, NULL );
    return;
  }
  
  uvc_error_t stream_err = uvc_start_streaming( devh_, &ctrl,
                                                &CameraDriver::ImageCallbackAdapter,
                                                this, 0 );
  
  if ( stream_err != UVC_SUCCESS )
  {
    ROS_ERROR( "uvc_start_streaming" );
    uvc_close( devh_ );
    uvc_unref_device( dev_ );
    return;
  }
  
  if ( rgb_frame_ )
    uvc_free_frame( rgb_frame_ );
  
  rgb_frame_   = uvc_allocate_frame( frame_width * frame_height * 3 );

  cinfo_manager_.loadCameraInfo( camera_info_url_ );
  cinfo_manager_ir_.loadCameraInfo( camera_info_url_ir_ );
  cinfo_manager_depth_.loadCameraInfo( camera_info_url_depth_ );
  cinfo_manager_color_.loadCameraInfo( camera_info_url_color_ );
  
  // TOF Camera Settigns
  int tof_err;
//  tof_err = setToFEEPROMMode( TOF_EEPROM_FACTORY_DEFAULT );
//  tof_err = clearToFError();
  setToFMode_All();
  
  // Get TOF & RGB Camera Informations
  getToFInfo_All();
  getRGBInfo_All();
  
  // Set Timer for Publishing Temperatures 
  double temp_time = 0.0;
  err = priv_nh_.getParam( "temp_time", temp_time );
  if ( 0.0 < temp_time )
  {
    ROS_INFO( "Set Timer for Publishing Temperatures as %.3f [sec]", temp_time );
    temp_timer_ = nh_.createTimer( ros::Duration( temp_time ), 
                                   boost::bind( &CameraDriver::TemperatureCallback, this ) );
  }
  
  tof_err = clearToFError();
  
  err = priv_nh_.getParam( "r_gain", r_gain_ );
  err = priv_nh_.getParam( "g_gain", g_gain_ );
  err = priv_nh_.getParam( "b_gain", b_gain_ );
  
  state_ = Running;
}


/**
 * @brief setCameraCtrl sets a camera control using uvc control.
 * @param ctrl uint8_t a control number
 * @param *data uint16_t pointer for data to set
 * @param size int a size of the data to set
 * @return int of ther result of uvc_set_ctrl
 */
int CameraDriver::setCameraCtrl( uint8_t ctrl, uint16_t *data ,int size )
{
  int err;
  
  err = uvc_set_ctrl( devh_, 3, ctrl, data, size );
  if ( err != size )
  {
    ROS_ERROR( "Set Ctrl failed. Error: %d", err );
  }
  return err;
}


/**
 * @brief getCameraCtrl gets a camera control using uvc control.
 * @param ctrl uint8_t a control number
 * @param *data uint16_t pointer for data to get
 * @param size int a size of the data to get
 * @return int of the result of uvc_get_ctrl
 */
int CameraDriver::getCameraCtrl( uint8_t ctrl, uint16_t *data, int size )
{
  int err;
  
  err = setCameraCtrl( ctrl, data, size );
  if ( err != size )
  {
    ROS_ERROR( "Set Ctrl to Get failed : Error: %d", err );
    return err;
  }
  else
  {
    err = uvc_get_ctrl( devh_, 3, ctrl, data, size, UVC_GET_CUR );
    if ( err != size )
    {
      ROS_ERROR( "Get Ctrl failed. Error: %d", err );
    }
  }
  return err;
}

/**
 * @brief setToFMode_All sets all ToF parameters with getting ROS parameters
 * @return int of the result
 */
int CameraDriver::setToFMode_All()
{
  int err;
  
  // Set RGB White Balance
  err = setToFMode_ROSParameter( "white_balance", 0 );
  
  // Set Depth/IR & RGB Parameters
  std::string rosparam_names[10] =
  {
    "depth_range",
    "threshold",
    "nr_filter",
    "pulse_count",
    "ld_enable",
    "ir_gain",
    "ae_mode",
    "brightness_gain",
    "exposure_time",
    "color_correction"
  };
  std::string param_name;
  
  int param;
  int name_num = sizeof( rosparam_names ) / sizeof( rosparam_names[0] );
  
  for ( int i = 0; i < name_num ; i++ )
  {
    param_name = rosparam_names[i];

    int err   = 0;
    
    // Get ROS Parameter and Set Data
    if ( priv_nh_.getParam( param_name, param ) )
    {
      err = setToFMode_ROSParameter( param_name, param );
    }
    else
    {
      ROS_ERROR( "Parameter Acquisition Error : %s", param_name.c_str() );
      return err;
    }
  }
  
  return err;
}

/**
 * @brief setToFMode_ROSParameter sets a integer Tof parameter converted from double parameter.
 * @param param_name std::string Parameter name
 * @param param double Parameter datum
 * @return int of the result
 */
int CameraDriver::setToFMode_ROSParameter( std::string param_name, double param )
{
  int err      = 0;
  
  uint32_t param_ui0 = 0;
  uint16_t param_ui1 = 0;
  uint16_t param_ui2 = 0;
  
  if ( param_name == "brightness_gain" )
  {
    param_ui0 = (uint32_t)( fabs( param ) * 100.0 );
  }
  else if ( param_name == "exposure_time" )
  {
    param_ui0 = (uint32_t)( fabs( param ) * 0x00100000 );
  }
  else
  {
    return err;
  }
  
  param_ui1 = (uint16_t)(   param_ui0         & 0xFFFF );
  param_ui2 = (uint16_t)( ( param_ui0 >> 16 ) & 0xFFFF );
  err = setToFMode_ROSParameter( param_name, param_ui1, param_ui2 );
  
  return err;
}

/**
 * @brief setToFMode_ROSParameter sets 1 integer Tof parameter with setting 0 for the 2nd datum.
 * @param param_name std::string Parameter name
 * @param param int Parameter datum
 * @return int of the result
 */
int CameraDriver::setToFMode_ROSParameter( std::string param_name, int param )
{
  int err;
  
  err = setToFMode_ROSParameter( param_name, param, 0 );
  return err;
}

/**
 * @brief setToFMode_ROSParameter sets 2 integer Tof parameters.
 * @param param_name std::string Parameter name
 * @param param int Parameter datum
 * @param param2 int Parameter datum
 * @return int of the result
 */
int CameraDriver::setToFMode_ROSParameter( std::string param_name, int param, int param_2 )
{
  uint8_t ctrl = UVC_XU_CTRL_TOF;
  
  uint16_t send[5] = { TOF_SET_DEPTH_RANGE, 0, 0, 0, 0 };
  uint16_t recv[5] = { TOF_GET_DEPTH_RANGE, 0, 0, 0, 0 };
  
  int param_set[5] = { TOF_SET_DEPTH_RANGE, 0, 0, 0, 0 };
  int param_min[5] = { 0x0000, 0, 0, 0, 0 };
  int param_max[5] = { 0xFFFF, 1, 1, 1, 1 };
  
  int err   = 0;
  
  // Set Parameter MAX/min and ToF Process Number
  if ( param_name == "depth_range" )
  {
    param_set[0] = TOF_SET_DEPTH_RANGE;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 2;
    
    recv[0] = TOF_GET_DEPTH_RANGE;
  }
  else if ( param_name == "threshold" )
  {
    param_set[0] = TOF_SET_THRESHOLD;
    param_set[1] = param;
    
    param_min[1] = 0x0000;
    param_max[1] = 0x3FFF;
    
    recv[0] = TOF_GET_THRESHOLD;
  }
  else if ( param_name == "nr_filter" )
  {
    param_set[0] = TOF_SET_NR_FILTER;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 1;
    
    recv[0] = TOF_GET_NR_FILTER;
  }
  else if ( param_name == "pulse_count" )
  {
    param_set[0] = TOF_SET_PULSE_COUNT;
    param_set[1] = param;
    
    param_min[1] = 1;
    param_max[1] = 2000;
    
    recv[0] = TOF_GET_PULSE_COUNT;
  }
  else if ( param_name == "ld_enable" )
  {
    param_set[0] = TOF_SET_LD_ENABLE;
    param_set[1] = param;
    param_set[2] = param;
    
    param_min[1] = 0;
    param_max[1] = 3;
    param_min[2] = 0;
    param_max[2] = 3;
    
    recv[0] = TOF_GET_LD_ENABLE;
  }
  else if ( param_name == "ir_gain" )
  {
    param_set[0] = TOF_SET_IR_GAIN;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 0x07FF;
    
    recv[0] = TOF_GET_IR_GAIN;
  }
  else if ( param_name == "white_balance" )
  {
    ctrl = UVC_XU_CTRL_RGB;
    param_set[0] = RGB_SET_WHITE_BALANCE;
  }
  else if ( param_name == "ae_mode" )
  {
    ctrl = UVC_XU_CTRL_RGB;
    param_set[0] = RGB_SET_AE_MODE;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 3;
  
    recv[0] = RGB_GET_AE_MODE;
  }
  else if ( param_name == "brightness_gain" )
  {
    ctrl = UVC_XU_CTRL_RGB;
    param_set[0] = RGB_SET_BRIGHTNESS_GAIN;
    param_set[1] = param;
    param_set[2] = param_2;
    
    param_min[1] = 100;
    param_max[1] = 1067;
    param_min[2] = 0;
    param_max[2] = 0xFFFF;
    
    recv[0] = RGB_GET_BRIGHTNESS_GAIN;
  }
  else if ( param_name == "exposure_time" )
  {
    ctrl = UVC_XU_CTRL_RGB;
    param_set[0] = RGB_SET_SHUTTER_CONTROL;
    param_set[1] = param;
    param_set[2] = param_2;
    
    param_min[1] = 0x0069;
    param_max[1] = 0x28F6;
    param_min[2] = 0;
    param_max[2] = 0xFFFF;
    
    recv[0] = RGB_GET_SHUTTER_CONTROL;
  }
  else if ( param_name == "color_correction" )
  {
    ctrl = UVC_XU_CTRL_RGB;
    param_set[0] = RGB_SET_COLOR_CORRECTION;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 1;
  
    recv[0] = RGB_GET_COLOR_CORRECTION;
  }
  else
  {
    ROS_WARN( "Unmatch Parameter Name : %s", param_name.c_str() );
    err = 0;
    return err;
  }
  
  // Max/min Value Check
  for ( int i = 0; i < 5; i++ )
  {
    if ( param_set[i] < param_min[i] )      send[i] = param_min[i];
    else if ( param_max[i] < param_set[i] ) send[i] = param_max[i];
    else                                    send[i] = param_set[i];
  }
  
  // Set Parameter on TOF Camera
  err = setCameraCtrl( ctrl, send, sizeof(send) );
  if ( err == sizeof(send) )
  {
    ROS_INFO( "Set Parameter %s as { %d, %d, %d, %d } on TOF Camera",
                param_name.c_str(), send[1], send[2], send[3], send[4] );
  }
  else
  {
    ROS_ERROR( "Set Parameter %s failed. Error: %d", param_name.c_str(), err );
    return err;
  }
  
  if ( param_name == "depth_range" )
  {
    getToFDepthCnvGain( depth_cnv_gain_ );
    ROS_INFO( "Get Depth Cnv Gain : %f", depth_cnv_gain_ );
    
    unsigned short max_data;
    unsigned short min_dist;
    unsigned short max_dist;
    getToFDepthInfo( depth_offset_, max_data, min_dist, max_dist );
    ROS_INFO( "Get Depth Info - Offset: %d / Max Data : %d / min Distance : %d [mm] MAX Distance :%d [mm]",
                depth_offset_, max_data, min_dist, max_dist );
  }
  
  // Check the valid values on ToF Camera
  if ( param_name == "pulse_count" || param_name == "depth_range" )
  {
    uint16_t pulse_count;
    getToFPulseCount( pulse_count );
  }
  
  return err;
}

/**
 * @brief setToFEEPROMMode sets EEPROM mode of the Tof camera sensor.
 * @param mode uint16_t EEPROM mode
 * @return int of the result
 */
int CameraDriver::setToFEEPROMMode( uint16_t mode = TOF_EEPROM_FACTORY_DEFAULT )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t send[5] = { TOF_SET_EEPROM, 0, 0, 0, 0 };
  int err;
  
  uint16_t val_min = 0x0000;
  uint16_t val_max = 0x0001;
  
  // Value Check
  if ( mode < val_min )      send[1] = val_min;
  else if ( val_max < mode ) send[1] = val_max;
  else                       send[1] = mode;
  
  err = setCameraCtrl( ctrl, send, sizeof(send) );
  if ( err == sizeof(send) )
  {
    ROS_INFO( "Set EEPROM Mode : %d", send[1] );
  }
  else
  {
    ROS_ERROR( "Set EEPROM Mode failed. Error: %d", err );
    return err;
  }
  
  return err;
}


/**
 * @brief clearToFError clears errors of the Tof camera sensor.
 * @return int of the result
 */
int CameraDriver::clearToFError()
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t send[5] = { TOF_SET_ERROR_CLEAR, 0, 0, 0, 0 };
  int err;
  
  err = setCameraCtrl( ctrl, send, sizeof(send) );
  if ( err == sizeof(send) )
  {
    ROS_INFO( "Clear TOF Camera Errors" );
  }
  else
  {
    ROS_ERROR( "Clear TOF Camera Errors failed. Error: %d", err );
    return err;
  }
  
  return err;
}


/**
 * @brief getToFInfo_All gets all ToF informations from the camera.
 */
void CameraDriver::getToFInfo_All()
{
  int tof_err;
  
  // Get TOF Camera Informations
  
  uint16_t version_n;
  uint16_t build_n;
  uint16_t build_y;
  uint16_t build_d;
  tof_err = getToFVersion( version_n, build_n, build_y, build_d );
  
  uint16_t depth_range;
  uint16_t dr_index;
  tof_err = getToFDepthRange( depth_range, dr_index );
  
  uint16_t threshold;
  tof_err = getToFThreshold( threshold );
  
  uint16_t nr_filter;
  tof_err = getToFNRFilter( nr_filter );
  
  uint16_t pulse_count;
  tof_err = getToFPulseCount( pulse_count );
  
  uint16_t ld_enable_near;
  uint16_t ld_enable_wide;
  tof_err = getToFLDEnable( ld_enable_near, ld_enable_wide );
  
  depth_cnv_gain_ = 0.5;
  tof_err = getToFDepthCnvGain( depth_cnv_gain_ );
  ROS_INFO( "Get Depth Cnv Gain : %f", depth_cnv_gain_ );
  
  unsigned short max_data;
  unsigned short min_dist;
  unsigned short max_dist;
  tof_err = getToFDepthInfo( depth_offset_, max_data, min_dist, max_dist );
  ROS_INFO( "Get Depth Info - Offset: %d / Max Data : %d / min Distance : %d [mm] MAX Distance :%d [mm]",
              depth_offset_, max_data, min_dist, max_dist );
  
  uint16_t ir_gain;
  tof_err = getToFIRGain( ir_gain );
  
  double t1;
  double t2;
  tof_err = getToFTemperature( t1, t2 );
  ROS_INFO( "Get Temperature T1 : %.1f / T2 : %.1f [deg C]", t1, t2 );
  
  int time_near, time_wide;
  tof_err = getToFLDPulseWidth( time_near, time_wide );
  ROS_INFO( "Get LD Pulse Width - Near: %d / Wide: %d [ns]", time_near, time_near );
  
  uint16_t common_err;
  uint16_t eeprom_err_factory;
  uint16_t eeprom_err;
  uint16_t mipi_temp_err;
  tof_err = getToFErrorInfo( common_err, eeprom_err_factory, eeprom_err, mipi_temp_err );
  
  return;
}


/**
 * @brief getToFDepthRange gets the depth range of the ToF camera sensor.
 * @param depth_range uint16_t& Depth Range
 * @param dr_index uint16_t& Depth Index
 * @return int of the result
 */
int CameraDriver::getToFDepthRange( uint16_t& depth_range, uint16_t& dr_index )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_DEPTH_RANGE, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    depth_range = data[1];
    dr_index    = data[2];
    ROS_INFO( "Get Depth Range Mode : %d / Index : %d", depth_range, dr_index );
  }
  else
  {
    ROS_ERROR( "Get Depth  Range Mode failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFThreshold gets the threshold of the ToF camera sensor.
 * @param threshold uint16_t& Threshold
 * @return int of the result
 */
int CameraDriver::getToFThreshold( uint16_t& threshold )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_THRESHOLD, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    threshold = data[1];
    ROS_INFO( "Get Threshold : %d", threshold );
  }
  else
  {
    ROS_ERROR( "Get Threshold failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFNRFilter gets the NR filter of the Depth/IR camera.
 * @param nr_filter uint16_t& NR Filter ON/OFF
 * @return int of the result
 */
int CameraDriver::getToFNRFilter( uint16_t& nr_filter )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_NR_FILTER, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    nr_filter = data[1];
    ROS_INFO( "Get NR Filter : %d", nr_filter );
  }
  else
  {
    ROS_ERROR( "Get NR Filter failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFPulseCount gets the pulse count of the ToF camera sensor.
 * @param pluse_count uint16_t& Pulse count
 * @return int of the result
 */
int CameraDriver::getToFPulseCount( uint16_t& pulse_count )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_PULSE_COUNT, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    pulse_count = data[1];
    ROS_INFO( "Get Pulse Count : %d", pulse_count );
  }
  else
  {
    ROS_ERROR( "Get Pulse Count failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFLDEnable gets the LD enable configurations of the ToF camera sensor.
 * @param pluse_count uint16_t& LD enable configurations
 * @return int of the result
 */
int CameraDriver::getToFLDEnable( uint16_t& ld_enable_near, uint16_t& ld_enable_wide )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_LD_ENABLE, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    ld_enable_near = data[1];
    ld_enable_wide = data[2];
    ROS_INFO( "Get LD Enable - Near: %d Wide: %d", ld_enable_near, ld_enable_wide );
  }
  else
  {
    ROS_ERROR( "Get LD Enable failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFDepthCnvGain gets the depth conversion gain of ToF camera sensor.
 * @param depth_cnv_gain uint16_t& Depth conversion gain
 * @return int of the result
 */
int CameraDriver::getToFDepthCnvGain( double& depth_cnv_gain )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_DEPTH_CNV_GAIN, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    depth_cnv_gain = *(double*)(&data[1]);
  }
  else
  {
    ROS_ERROR( "Get Depth Cnv Gain failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFDepthInfo gets the depth informations of the ToF camera sensor.
 * @param depth_offset short& Depth offset value data
 * @param max_data unsigned short& Maximum data
 * @param min_dist unsigned short& Minimum distance [mm]
 * @param max_dist unsigned short& Maximum distance [mm]
 * @return int of the result
 */
int CameraDriver::getToFDepthInfo( short&          depth_offset,
                                   unsigned short& max_data,
                                   unsigned short& min_dist,
                                   unsigned short& max_dist )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_DEPTH_INFO, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    depth_offset = *(short*)(&data[1]);
    max_data     = (unsigned short)(data[2]);
    min_dist     = (unsigned short)(data[3]);
    max_dist     = (unsigned short)(data[4]);
  }
  else
  {
    ROS_ERROR( "Get Depth Info failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFIRGain gets the IR gain of the Depth/IR camera.
 * @param ir_gain uint16_t& IR gain
 * @return int of the result
 */
int CameraDriver::getToFIRGain( uint16_t& ir_gain )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_IR_GAIN, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    ir_gain = data[1];
    ROS_INFO( "Get IR Gain : %d", ir_gain );
  }
  else
  {
    ROS_ERROR( "Get IR Gain failed. Error : %d", err );
  }
  
  return err;
}


/**
 * @brief getToFTemperature gets the temperature data of the ToF camera sensor.
 * @param t1 double& Temperature data [deg C]
 * @param t2 double& Temperature data [deg C]
 * @return int of the result
 */
int CameraDriver::getToFTemperature( double& t1, double& t2 )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_TEMPERATURE, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    t1 = data[1] / 256.0;
    t2 = data[2] / 256.0;
  }
  else
  {
    ROS_ERROR( "Get Temperature failed. Error: %d", err );
  }
  
  return err;
}


/**
 * @brief getToFLDPulseWidth gets the pulse width of the ToF camera sensor.
 * @param time_near int& Pulse width time for the near mode [ns]
 * @param time_wide int& Pulse width time for the wide mode [ns]
 * @return int of the result
 */
int CameraDriver::getToFLDPulseWidth( int& time_near, int& time_wide )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_LD_PULSE_WIDTH, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    time_near = data[1];
    time_wide = data[2];
  }
  else
  {
    ROS_ERROR( "Get Temperature failed. Error: %d", err );
  }
  
  return err;
}


/**
 * @brief getToFVersion gets the version informations of the ToF camera sensor.
 * @param version_n uint16_t& Major and minor version data
 * @param build_n uint16_t& Build number
 * @param build_y uint16_t& Build year
 * @param build_d uint16_t& Build date
 * @return int of the result
 */
int CameraDriver::getToFVersion( uint16_t& version_n,
                                 uint16_t& build_n,
                                 uint16_t& build_y,
                                 uint16_t& build_d )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_VERSION, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    version_n = data[1];
    build_n   = data[2];
    build_y   = data[3];
    build_d   = data[4];
    ROS_INFO( "Get Version : %x / Build : %x / Build Date : %x%x (YYYYMMDD)",
                version_n, build_n, build_y, build_d );
  }
  else
  {
    ROS_ERROR( "Get Version failed. Error: %d", err );
  }
  
  return err;
}


/**
 * @brief getToFErrorInfo gets the error informations of the ToF camera sensor.
 * @param common_err uint16_t& Errors
 * @param eeprom_err_factory uint16_t& Not used now, used in old versions
 * @param eeprom_err uint16_t& Not used now, used in old versions
 * @param mipi_temp_err uint16_t& Not used now, used in old versions
 * @return int of the result
 */
int CameraDriver::getToFErrorInfo( uint16_t& common_err,
                                   uint16_t& eeprom_err_factory,
                                   uint16_t& eeprom_err,
                                   uint16_t& mipi_temp_err )
{
  uint8_t  ctrl    = UVC_XU_CTRL_TOF;
  uint16_t data[5] = { TOF_GET_ERROR_INFO, 0, 0, 0, 0 };
  int err;
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    common_err          = data[1];
    eeprom_err_factory  = data[2];
    eeprom_err          = data[3];
    mipi_temp_err       = data[4];
    ROS_INFO( "Get Error Info - Common : 0x%02x / EEPROM Factory : 0x%02x / EEPROM : 0x%02x / Misc-Temperature : 0x%02x",
                common_err, eeprom_err_factory, eeprom_err, mipi_temp_err );
  }
  else
  {
    ROS_ERROR( "Get Error Info failed. Error: %d", err );
  }
  
  return err;
}


/**
 * @brief getRGBInfo_All gets all informations of the RGB camera.
 */
void CameraDriver::getRGBInfo_All()
{
  int err;
  
  uint16_t ae_mode;
  err = getRGBAEMode( ae_mode );
  
  double brightness_gain;
  double brightness_maxg;
  err = getRGBBrightnessGain( brightness_gain, brightness_maxg );
  
  double exposure_time;
  double exposure_maxt;
  err = getRGBShutterControl( exposure_time, exposure_maxt );
  
  uint16_t color_correction;
  err = getRGBColorCorrection( color_correction );
  
  return;
}


/**
 * @brief getRGBAEMode gets the Auto Exposure mode of the RGB camera.
 * @param ae_mode uint16_t& AE mode
 * @return int of the result
 */
int CameraDriver::getRGBAEMode( uint16_t& ae_mode )
{
  int err;
  
  uint8_t  ctrl    = UVC_XU_CTRL_RGB;
  uint16_t data[5] = { RGB_GET_AE_MODE, 0, 0, 0, 0 };
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    ae_mode = data[1];
    ROS_INFO( "Get RGB AE Mode: %d ( 0:Manual / 1: Gain Auto / 2: Shutter Auto / 3: Full Auto )", ae_mode );
  }
  else
  {
    ROS_ERROR( "Get Error Info failed. Error: %d", err );
  }
  
  return err;
}

/**
 * @brief getRGBBrightnessGain gets the brightness gain of the RGB camera.
 * @param brightness_gain double& Brightness gain
 * @param brightness_maxg double& Brightness maximum gain
 * @return int of the result
 */
int CameraDriver::getRGBBrightnessGain( double& brightness_gain,  double& brightness_maxg )
{
  int err;
  
  uint8_t  ctrl    = UVC_XU_CTRL_RGB;
  uint16_t data[5] = { RGB_GET_BRIGHTNESS_GAIN, 0, 0, 0, 0 };
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    uint32_t gain = ( data[2] << 16 ) + data[1];
    uint32_t maxg = ( data[4] << 16 ) + data[3];
    
    brightness_gain = (double)(gain) / 100.0;
    brightness_maxg = (double)(maxg) / 100.0;
    
    ROS_INFO( "Get RGB Brightness Gain: %f ( MAX: %f )", brightness_gain, brightness_maxg );
  }
  else
  {
    ROS_ERROR( "Get Error Info failed. Error: %d", err );
  }
  
  return err;
}

/**
 * @brief getRGBShutterControl gets the shutter control time of the RGB camera.
 * @param exposure_time double& Shutter exposure time [s]
 * @param exposure_maxt double& Shutter exposure maximum time [s]
 * @return int of the result
 */
int CameraDriver::getRGBShutterControl( double& exposure_time, double& exposure_maxt )
{
  int err;
  
  uint8_t  ctrl    = UVC_XU_CTRL_RGB;
  uint16_t data[5] = { RGB_GET_SHUTTER_CONTROL, 0, 0, 0, 0 };
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    uint32_t time = ( data[2] << 16 ) + data[1];
    uint32_t maxt = ( data[4] << 16 ) + data[3];
    
    exposure_time = (double)(time) / 0x00100000;
    exposure_maxt = (double)(maxt) / 0x00100000;
    
    ROS_INFO( "Get RGB Exposure Time: %f (MAX: %f) [sec]", exposure_time, exposure_maxt );
  }
  else
  {
    ROS_ERROR( "Get Error Info failed. Error: %d", err );
  }
  
  return err;
}

/**
 * @brief getRGBColorCorrection gets the color correction mode of the RGB camera.
 * @param color_correction uint16_t& Color correction mode Off: 0 / Standard: 1
 * @return int of the result
 */
int CameraDriver::getRGBColorCorrection( uint16_t& color_correction )
{
  int err;
  
  uint8_t  ctrl    = UVC_XU_CTRL_RGB;
  uint16_t data[5] = { RGB_GET_COLOR_CORRECTION, 0, 0, 0, 0 };
  
  err = getCameraCtrl( ctrl, data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    color_correction = data[1];
    ROS_INFO( "Get RGB Color Correction: %d ( 0: OFF / 1: Standard )", color_correction );
  }
  else
  {
    ROS_ERROR( "Get Error Info failed. Error: %d", err );
  }
  
  return err;
}

/**
 * @brief TemperatureCallback is a callback method to get and publish temperature data.
 * @param ptr void* pointer for this driver
 */
void CameraDriver::TemperatureCallback( void* ptr )
{
  boost::recursive_mutex::scoped_lock( mutex_ );
  
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);
  
  driver->publishToFTemperature();
  
  return;
}


/**
 * @brief publishToFTemperature gets and publishes temperature data of the Tof camera sensor.
 */
void CameraDriver::publishToFTemperature()
{
  std::string frame_id;
  priv_nh_.getParam( "frame_id" , frame_id );
  
  sensor_msgs::Temperature t_msg;
  
  double t1;
  double t2;
  
  getToFTemperature( t1, t2 );
  
  t_msg.header.frame_id = frame_id;
  t_msg.header.stamp    = ros::Time::now();
  
  t_msg.temperature = t1;
  pub_tof_t1_.publish( t_msg );
  
  t_msg.temperature = t2;
  pub_tof_t2_.publish( t_msg );
  
  return;
}


/**
 * @brief CloseCamera closes the ToF camera sensor.
 */
void CameraDriver::CloseCamera()
{
  uvc_close( devh_ );
  devh_ = NULL;
  
  uvc_unref_device( dev_ );
  dev_ = NULL;
  
  temp_timer_.stop();
  
  state_ = Stopped;
}

};
