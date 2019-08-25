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
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>

#include <math.h>


namespace cis_camera
{

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
    cinfo_manager_color_(nh),
    cinfo_manager_ir_(nh)
{
  readConfigFromParameterServer();
  advertiseROSTopics();
}


CameraDriver::~CameraDriver()
{
  if ( rgb_frame_ )
    uvc_free_frame( rgb_frame_ );
  
  if ( ctx_ )
    uvc_exit( ctx_ );  // Destroys dev_, devh_, etc.
}


void CameraDriver::readConfigFromParameterServer()
{
  int err;
  
  err = priv_nh_.getParam( "camera_info_url"      , camera_info_url_       );
  err = priv_nh_.getParam( "camera_info_url_ir"   , camera_info_url_ir_    );
  err = priv_nh_.getParam( "camera_info_url_color", camera_info_url_color_ );
}


void CameraDriver::advertiseROSTopics()
{
  // Remapping namespaces
  ros::NodeHandle color_nh( nh_, "rgb" );
  image_transport::ImageTransport color_it( color_nh );
  
//  ros::NodeHandle bgr8_nh( nh_, "bgr8" );
//  image_transport::ImageTransport bgr8_it( bgr8_nh );
  
  ros::NodeHandle depth_nh( nh_, "depth" );
  image_transport::ImageTransport depth_it( depth_nh );
  
  ros::NodeHandle ir_nh( nh_, "ir" );
  image_transport::ImageTransport ir_it( ir_nh );
  
  // Advertise Camera Pubishers
  pub_camera_ = it_.advertiseCamera( "image_raw", 1, false );
  pub_color_  = color_it.advertiseCamera( "image_raw", 1, false );
//  pub_bgr8_   = bgr8_it.advertiseCamera( "image_raw", 1, false );
  pub_depth_  = depth_it.advertiseCamera( "image_raw", 1, false );
  pub_ir_     = ir_it.advertiseCamera( "image_raw", 1, false );
  
}


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


void CameraDriver::Stop()
{
  boost::recursive_mutex::scoped_lock( mutex_ );
  
  if ( state_ == Running )
    CloseCamera();
  
  uvc_exit( ctx_ );
  ctx_ = NULL;
  
  state_ = Initial;
}


void CameraDriver::ReconfigureCallback( CISCameraConfig &new_config, uint32_t level )
{
  boost::recursive_mutex::scoped_lock( mutex_ );
  
  ROS_INFO( "Reconfigure Request" );
  
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
  }
  
  config_ = new_config;
  return;
}


uint8_t cvtDoubleToByte( double x )
{
  if ( x < 0 )        return 0;
  else if ( 255 < x ) return 255;
  
  return static_cast<uint8_t>( x );
}

void CameraDriver::ImageCallback( uvc_frame_t *frame )
{
  ros::Time timestamp = ros::Time( frame->capture_time.tv_sec, frame->capture_time.tv_usec );
  if ( timestamp == ros::Time(0) )
  {
    timestamp = ros::Time::now();
  }
  
  boost::recursive_mutex::scoped_lock(mutex_);
  
  if ( state_ != Running || not rgb_frame_ )
  {
    return;
  }
  
  int         err;
  int         frame_width;
  int         frame_height;
  double      frame_rate;
  std::string video_mode;
  
  err = priv_nh_.getParam( "width",  frame_width  );
  err = priv_nh_.getParam( "height", frame_height );
  
  sensor_msgs::Image::Ptr image( new sensor_msgs::Image() );
  image->width  = frame_width;
  image->height = frame_height;
  image->step   = image->width * 3;
  image->data.resize( image->step * image->height );
  
  sensor_msgs::Image::Ptr image_bgr8( new sensor_msgs::Image() );
  sensor_msgs::Image::Ptr image_color( new sensor_msgs::Image() );
  sensor_msgs::Image::Ptr image_depth( new sensor_msgs::Image() );
  sensor_msgs::Image::Ptr image_ir( new sensor_msgs::Image() );
  
  cv_bridge::CvImage image_rgb;
  
  std::string frame_id;
  err = priv_nh_.getParam( "frame_id",  frame_id );
  
  if ( frame->frame_format == UVC_FRAME_FORMAT_GRAY16 )
  {
    image->encoding = "16UC1";
    image->step     = image->width * 2;
    image->data.resize( image->step * image->height );
    memcpy( &(image->data[0]), frame->data, frame->data_bytes );
    
    uint16_t* data = reinterpret_cast<uint16_t*>( &image->data[0] );
    
    // Cropping Color Image Frame
    int color_width  = 1280;
    int color_height = 960;
    
    image_color->encoding = "yuv422";
    image_color->width  = color_width;
    image_color->height = color_height;
    image_color->step   = image_color->width * 2;
    image_color->data.resize( image_color->step * image_color->height );
    
    uint16_t color_data[ color_width * color_height ];
    
    int m = 0;
    int n = 0;
    for ( int i=0; i < color_height; i++ )
    {
      m = i * frame_width;
      n = i * color_width;
      memcpy( &(color_data[n]), &(data[m]), color_width * sizeof(uint16_t) );
    }
    
    memcpy( &(image_color->data[0]), color_data, color_width * color_height * sizeof(uint16_t) );
    
    // Converting YUV422 to BGR8 with OpenCV
    cv_bridge::CvImagePtr cv_ptr_yuv;
    try
    {
      cv_ptr_yuv = cv_bridge::toCvCopy( image_color, sensor_msgs::image_encodings::YUV422 );
    }
    catch ( cv_bridge::Exception& e )
    {
      ROS_ERROR( "cv_bridge exception: %s", e.what() );
    }
    cv::Mat mat_rgb;
//    cv::cvtColor( cv_ptr_yuv->image, mat_rgb, cv::COLOR_YUV2RGB_UYVY );
    cv::cvtColor( cv_ptr_yuv->image, mat_rgb, cv::COLOR_YUV2BGR_UYVY );
    
    image_rgb.image    = mat_rgb;
    image_rgb.encoding = "bgr8";
    
    image_rgb.header          = image_color->header;
    image_rgb.header.frame_id = frame_id;
    image_rgb.header.stamp    = timestamp;
    
    
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
      u0 = static_cast<double>( *(uyvy_ptr) );
      y0 = static_cast<double>( *(uyvy_ptr+1) );
      v0 = static_cast<double>( *(uyvy_ptr+2) );
      y1 = static_cast<double>( *(uyvy_ptr+3) );
      
      r0 = 1.547800 * ( v0 - 128 );
      g0 = 0.187324 * ( u0 - 128 ) - 0.468124 * ( v0 - 128 );
      b0 = 1.855600 * ( u0 - 128 );
      
      *(bgr8_ptr)   = cvtDoubleToByte( y0 + b0 );
      *(bgr8_ptr+1) = cvtDoubleToByte( y0 + g0 );
      *(bgr8_ptr+2) = cvtDoubleToByte( y0 + r0 );
      
      *(bgr8_ptr+3) = cvtDoubleToByte( y1 + b0 );
      *(bgr8_ptr+4) = cvtDoubleToByte( y1 + g0 );
      *(bgr8_ptr+5) = cvtDoubleToByte( y1 + r0 );
      
      uyvy_ptr += 4;
      bgr8_ptr += 6;
    }
    
//    uint8_t* color_ptr = reinterpret_cast<uint8_t*>( &(color_data[0]) );
//    memcpy( &(uyvy_data[0]), color_ptr, color_width * color_height * 2 * sizeof(uint8_t) );
    
//    memcpy( &(image_bgr8->data[0]), bgr8_data, color_width * color_height * 3 );
    
    
    // Cropping Depth and IR Image Frame
    int depth_width  = 640;
    int depth_height = 480;
    
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
    image_ir->data.resize( image_depth->step * image_depth->height );
    
    uint16_t ir_data[ depth_width * depth_height ];
    
    int offset_x = 1280;
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
    
    // Ad-hoc: Change metric to mm
//    uint16_t* data = reinterpret_cast<uint16_t*>(&image->data[0]);
//    for ( int i=0; i<image->height*image->width; i++ )
//    {
//      data[i] = (uint16_t)(data[i]*0.406615*4.0);
//    }
    
    // Convert Depth Data to [mm]
//    double depth_cnv_gain = 0.406615;
    double depth_cnv_gain = 0.203308;
    int tof_err = getToFDepthCnvGain( depth_cnv_gain );
    
    short  offset_val     = 0;
    
//    for ( int i=0; i< depth_height * depth_width; i++ )
//    {
//      depth_data[i] = (uint16_t)( depth_data[i] * depth_cnv_gain * 4.0 + offset_val );
//    }
    
    double depth_angle_width, p_1, i_d, j_d;
    err = priv_nh_.getParam( "depth_angle_width", depth_angle_width );
    double l_1 = depth_width / 2.0 / tan( M_PI / 180.0 * depth_angle_width );
    
    for ( int i=0; i< depth_height; i++ )
    {
      i_d = i - depth_height / 2.0;
      for ( int j=0; j < depth_width; j++ ) {
        j_d = j - depth_width / 2.0;
        p_1 = l_1 / sqrt( l_1 * l_1 + i_d * i_d + j_d * j_d );
        depth_data[ i*depth_width + j ] = (uint16_t)( ( depth_data[ i*depth_width + j ] * depth_cnv_gain * 4.0 + offset_val ) * p_1 );
      }
    }
    
    memcpy( &(image_depth->data[0]), depth_data, depth_width * depth_height * sizeof(uint16_t) );
    memcpy( &(image_ir->data[0]), ir_data, depth_width * depth_height * sizeof(uint16_t) );
    
  }
  else
  {
    uvc_error_t conv_ret = uvc_any2bgr( frame, rgb_frame_ );
    if ( conv_ret != UVC_SUCCESS )
    {
      ROS_ERROR( "Couldn't convert frame to RGB : Error.%d", conv_ret );
      return;
    }
    image->encoding = "bgr8";
    memcpy( &(image->data[0]), rgb_frame_->data, rgb_frame_->data_bytes );
  }
  
  image->header.frame_id = frame_id;
  image->header.stamp    = timestamp;
  
  image_color->header.frame_id = frame_id;
  image_color->header.stamp    = timestamp;
  
  image_bgr8->header.frame_id = frame_id;
  image_bgr8->header.stamp    = timestamp;
  
  image_depth->header.frame_id = frame_id;
  image_depth->header.stamp    = timestamp;
  
  image_ir->header.frame_id = frame_id;
  image_ir->header.stamp    = timestamp;
  
  sensor_msgs::CameraInfo::Ptr cinfo( new sensor_msgs::CameraInfo( cinfo_manager_.getCameraInfo() ) );
  sensor_msgs::CameraInfo::Ptr cinfo_ir( new sensor_msgs::CameraInfo( cinfo_manager_ir_.getCameraInfo() ) );
  sensor_msgs::CameraInfo::Ptr cinfo_color( new sensor_msgs::CameraInfo( cinfo_manager_color_.getCameraInfo() ) );
  
  cinfo->header.frame_id = frame_id;
  cinfo->header.stamp    = timestamp;
  
  cinfo_ir->header.frame_id = frame_id;
  cinfo_ir->header.stamp    = timestamp;
  
  cinfo_color->header.frame_id = frame_id;
  cinfo_color->header.stamp    = timestamp;
  
  pub_camera_.publish( image, cinfo );
//  pub_color_.publish( image_color, cinfo_color );
//  pub_bgr8_.publish( image_bgr8, cinfo_color );
  pub_color_.publish( image_bgr8, cinfo_color );
//  pub_color_.publish( image_rgb.toImageMsg(), cinfo_color );
  pub_depth_.publish( image_depth, cinfo_ir );
  pub_ir_.publish( image_ir, cinfo_ir );
  
  publishToFTemperature( frame_id );
  
  if ( config_changed_ )
  {
    config_server_.updateConfig( config_ );
    config_changed_ = false;
  }
}


void CameraDriver::ImageCallbackAdapter( uvc_frame_t *frame, void *ptr )
{
  CameraDriver *driver = static_cast<CameraDriver*>(ptr);
  
  driver->ImageCallback( frame );
}


enum uvc_frame_format CameraDriver::GetVideoMode( std::string vmode )
{
  if( vmode == "uncompressed" )
  {
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
  else if ( vmode == "compressed" )
  {
    return UVC_COLOR_FORMAT_COMPRESSED;
  }
  else if ( vmode == "yuyv" )
  {
    return UVC_COLOR_FORMAT_YUYV;
  }
  else if ( vmode == "uyvy" )
  {
    return UVC_COLOR_FORMAT_UYVY;
  }
  else if ( vmode == "rgb" )
  {
    return UVC_COLOR_FORMAT_RGB;
  }
  else if ( vmode == "bgr" )
  {
    return UVC_COLOR_FORMAT_BGR;
  }
  else if ( vmode == "mjpeg" )
  {
    return UVC_COLOR_FORMAT_MJPEG;
  }
  else if ( vmode == "gray8" )
  {
    return UVC_COLOR_FORMAT_GRAY8;
  }
  else if ( vmode == "gray16" )
  {
    return UVC_COLOR_FORMAT_GRAY16;
  }
  else
  {
    ROS_ERROR_STREAM( "Invalid Video Mode: " << vmode );
    ROS_WARN_STREAM( "Continue using video mode: uncompressed" );
    return UVC_COLOR_FORMAT_UNCOMPRESSED;
  }
};


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
    ROS_ERROR( "uvc_find_device" );
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
    switch ( open_err )
    {
      case UVC_ERROR_ACCESS:
#ifdef __linux__
        ROS_ERROR( "Permission denied opening /dev/bus/usb/%03d/%03d",
                    uvc_get_bus_number( dev_ ), uvc_get_device_address( dev_ ) );
#else
        ROS_ERROR( "Permission denied opening device %d on bus %d",
                    uvc_get_device_address( dev_ ), uvc_get_bus_number( dev_ ) );
#endif
        break;
      default:
#ifdef __linux__
        ROS_ERROR( "Can't open /dev/bus/usb/%03d/%03d: %s (%d)",
                    uvc_get_bus_number( dev_ ), uvc_get_device_address( dev_ ),
                    uvc_strerror( open_err ), open_err );
#else
        ROS_ERROR( "Can't open device %d on bus %d: %s (%d)",
                    uvc_get_device_address( dev_ ), uvc_get_bus_number( dev_ ),
                    uvc_strerror( open_err ), open_err );
#endif
        break;
    }
  
    uvc_unref_device( dev_ );
    return;
  }
  
  int         frame_width  = 1920;
  int         frame_height = 960;
  double      frame_rate   = 30.0;
  std::string video_mode   = "uncompressed";
  
  int         color_width  = 1280;
  int         color_height = 960;
  
  err = priv_nh_.getParam( "width"  , frame_width  );
  err = priv_nh_.getParam( "height" , frame_height );
  err = priv_nh_.getParam( "frame_rate" , frame_rate  );
  err = priv_nh_.getParam( "video_mode" , video_mode  );
  
  uvc_stream_ctrl_t ctrl;
  
  uvc_error_t mode_err;
  mode_err = uvc_get_stream_ctrl_format_size(
      devh_, &ctrl,
      GetVideoMode( video_mode ),
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
  cinfo_manager_color_.loadCameraInfo( camera_info_url_color_ );
  
  // TOF Camera Settigns
  int tof_err;
//  tof_err = setToFEEPROMMode( TOF_EEPROM_FACTORY_DEFAULT );
//  tof_err = clearToFError();
  setToFMode_All();
  
  // Get TOF Camera Informations
  getToFInfo_All();
  
  // Set Publishers for TOF Camera Temperature
  std::string node_name = ros::this_node::getName();
  pub_tof_t1_ = nh_.advertise<sensor_msgs::Temperature>( node_name + "/t1", 1000 );
  pub_tof_t2_ = nh_.advertise<sensor_msgs::Temperature>( node_name + "/t2", 1000 );
  
  tof_err = clearToFError();
  
  state_ = Running;
}


int CameraDriver::setToFCtrl( uint16_t *data ,int size )
{
  int err;
  
  err = uvc_set_ctrl( devh_, 3, 0x03, data, size );
  if ( err != size )
  {
    ROS_ERROR( "Set Ctrl failed. Error: %d", err );
  }
  return err;
}


int CameraDriver::getToFCtrl( uint16_t *data, int size )
{
  int err;
  
  err = setToFCtrl( data, size );
  if ( err != size )
  {
    ROS_ERROR( "Set Ctrl to Get failed : Error: %d", err );
    return err;
  }
  else
  {
    err = uvc_get_ctrl( devh_, 3, 0x03, data, size, UVC_GET_CUR );
    if ( err != size )
    {
      ROS_ERROR( "Get Ctrl failed. Error: %d", err );
    }
  }
  return err;
}


int CameraDriver::setToFMode_All()
{
  int err;
  
//  std::string rosparam_names[8] =
  std::string rosparam_names[6] =
  {
//    "depth_ir",
    "depth_range",
    "threshold",
    "nr_filter",
    "pulse_count",
    "ld_enable",
    "ir_gain",
//    "error_stop"
  };
  std::string param_name;
  
  int param;
  int name_num = sizeof( rosparam_names ) / sizeof( rosparam_names[0] );
  
  for ( int i = 0; i < name_num ; i++ )
  {
    param_name = rosparam_names[i];
    ROS_INFO( "%d. ROS Param : %s", i, param_name.c_str() );

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


int CameraDriver::setToFMode_ROSParameter( std::string param_name, int param = 0 )
{
  uint16_t send[5] = { TOF_SET_DEPTH_IR, 0, 0, 0, 0 };
  uint16_t recv[5] = { TOF_GET_DEPTH_IR, 0, 0, 0, 0 };
  
  int param_set[5] = { TOF_SET_DEPTH_IR, 0, 0, 0, 0 };
  int param_min[5] = { 0x0000, 0, 0, 0, 0 };
  int param_max[5] = { 0xFFFF, 1, 1, 1, 1 };
  
//  int param = 0;
  int err   = 0;
  
  // Set Parameter MAX/min and ToF Process Number
  if ( param_name == "depth_ir" )
  {
    param_set[0] = TOF_SET_DEPTH_IR;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 1;
    
    recv[0] = TOF_GET_DEPTH_IR;
  }
  else if ( param_name == "depth_range" )
  {
    param_set[0] = TOF_SET_DEPTH_RANGE;
    param_set[1] = param;
    
    param_min[1] = 0;
//      param_max[1] = 1;
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
//      param_max[1] = 2000;
    param_max[1] = 0xFFFF;
    
    recv[0] = TOF_GET_PULSE_COUNT;
  }
  else if ( param_name == "ld_enable" )
  {
    param_set[0] = TOF_SET_LD_ENABLE;
    param_set[1] = param;

    param_min[1] = 0;
//      param_max[1] = 15;
    param_max[1] = 3;
    
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
  else if ( param_name == "error_stop" )
  {
    param_set[0] = TOF_SET_ERROR_STOP;
    param_set[1] = param;
    
    param_min[1] = 0;
    param_max[1] = 1;
    
    recv[0] = TOF_GET_ERROR_STOP;
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
  err = setToFCtrl( send, sizeof(send) );
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
  
  // Get Parameter on TOF Camera for Check
//  err = getToFCtrl( recv, sizeof(recv) );
//  if ( err == sizeof(recv) )
//  {
//    ROS_INFO( "Get Parameter %s as { %d, %d, %d, %d } on TOF Camera",
//                param_name.c_str(), recv[1], recv[2], recv[3], recv[4] );
//  }
//  else
//  {
//    ROS_ERROR( "Get Parameter of %s for Check Failed. Error : %d", param_name.c_str(), err );
//    return err;
//  }
  
  return err;
}


int CameraDriver::setToFEEPROMMode( uint16_t mode = TOF_EEPROM_FACTORY_DEFAULT )
{
  uint16_t send[5] = { TOF_SET_EEPROM, 0, 0, 0, 0 };
  int err;
  
  uint16_t val_min = 0x0000;
  uint16_t val_max = 0x0001;
  
  // Value Check
  if ( mode < val_min )      send[1] = val_min;
  else if ( val_max < mode ) send[1] = val_max;
  else                       send[1] = mode;
  
  err = setToFCtrl( send, sizeof(send) );
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


int CameraDriver::clearToFError()
{
  uint16_t send[5] = { TOF_SET_ERROR_CLEAR, 0, 0, 0, 0 };
  int err;
  
  err = setToFCtrl( send, sizeof(send) );
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


void CameraDriver::getToFInfo_All()
{
  int tof_err;
  
  // Get TOF Camera Informations
  
  uint16_t version_n;
  uint16_t build_n;
  uint16_t build_y;
  uint16_t build_d;
  tof_err = getToFVersion( version_n, build_n, build_y, build_d );
  
//  uint16_t depth_ir;
//  tof_err = getToFDepthIR( depth_ir );
  
  uint16_t depth_range;
  uint16_t dr_index;
  tof_err = getToFDepthRange( depth_range, dr_index );
  
  uint16_t threshold;
  tof_err = getToFThreshold( threshold );
  
  uint16_t nr_filter;
  tof_err = getToFNRFilter( nr_filter );
  
  uint16_t pulse_count;
  tof_err = getToFPulseCount( pulse_count );
  
  uint16_t ld_enable;
  tof_err = getToFLDEnable( ld_enable );
  
  double depth_cnv_gain;
  tof_err = getToFDepthCnvGain( depth_cnv_gain );
  ROS_INFO( "Get Depth Cnv Gain : %f", depth_cnv_gain );

  
  short          offset_val;
  unsigned short max_data;
  unsigned short min_dist;
  unsigned short max_dist;
  tof_err = getToFDepthInfo( offset_val, max_data, min_dist, max_dist );
  ROS_INFO( "Get Depth Info - Offset: %d / Max Data : %d / min Distance : %d [mm] MAX Distance :%d [mm]",
              offset_val, max_data, min_dist, max_dist );

  uint16_t ir_gain;
  tof_err = getToFIRGain( ir_gain );
  
  double t1;
  double t2;
  tof_err = getToFTemperature( t1, t2 );
  ROS_INFO( "Get Temperature T1 : %.1f / T2 : %.1f [deg C]", t1, t2 );
  
//  uint16_t error_stop;
//  tof_err = getToFErrorStop( error_stop );
  
  uint16_t common_err;
  uint16_t eeprom_err_factory;
  uint16_t eeprom_err;
  uint16_t mipi_temp_err;
  tof_err = getToFErrorInfo( common_err, eeprom_err_factory, eeprom_err, mipi_temp_err );
  
  return;
}


int CameraDriver::getToFDepthIR( uint16_t& depth_ir )
{
  uint16_t data[5] = { TOF_GET_DEPTH_IR, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    depth_ir = data[1];
    ROS_INFO( "Get Depth/IR Mode : %d", depth_ir );
  }
  else
  {
    ROS_ERROR( "Get Depth IR Mode failed. Error : %d", err );
  }
  
  return err;
}


int CameraDriver::getToFDepthRange( uint16_t& depth_range, uint16_t& dr_index )
{
  uint16_t data[5] = { TOF_GET_DEPTH_RANGE, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFThreshold( uint16_t& threshold )
{
  uint16_t data[5] = { TOF_GET_THRESHOLD, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFNRFilter( uint16_t& nr_filter )
{
  uint16_t data[5] = { TOF_GET_NR_FILTER, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFPulseCount( uint16_t& pulse_count )
{
  uint16_t data[5] = { TOF_GET_PULSE_COUNT, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFLDEnable( uint16_t& ld_enable )
{
  uint16_t data[5] = { TOF_GET_LD_ENABLE, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    ld_enable = data[1];
    ROS_INFO( "Get LD Enable : %d", ld_enable );
  }
  else
  {
    ROS_ERROR( "Get LD Enable failed. Error : %d", err );
  }
  
  return err;
}


int CameraDriver::getToFDepthCnvGain( double& depth_cnv_gain )
{
  uint16_t data[5] = { TOF_GET_DEPTH_CNV_GAIN, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    depth_cnv_gain = *(double*)(&data[1]);
//    ROS_INFO( "Get Depth Cnv Gain : %f", depth_cnv_gain );
  }
  else
  {
    ROS_ERROR( "Get Depth Cnv Gain failed. Error : %d", err );
  }
  
  return err;
}


int CameraDriver::getToFDepthInfo( short&          offset_val,
                                   unsigned short& max_data,
                                   unsigned short& min_dist,
                                   unsigned short& max_dist )
{
  uint16_t data[5] = { TOF_GET_DEPTH_INFO, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    offset_val = *(short*)(&data[1]);
    max_data   = *(unsigned short*)(&data[2]);
    min_dist   = *(unsigned short*)(&data[3]);
    max_dist   = *(unsigned short*)(&data[4]);
//    ROS_INFO( "Get Depth Info - Offset: %d / Max Data : %d / min Distance : %d [mm] MAX Distance :%d [mm]",
//                offset_val, max_data, min_dist, max_dist );
  }
  else
  {
    ROS_ERROR( "Get Depth Info failed. Error : %d", err );
  }
  
  return err;
}


int CameraDriver::getToFIRGain( uint16_t& ir_gain )
{
  uint16_t data[5] = { TOF_GET_IR_GAIN, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFTemperature( double& t1, double& t2 )
{
  uint16_t data[5] = { TOF_GET_TEMPERATURE, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    t1 = data[1] / 256.0;
    t2 = data[2] / 256.0;
    // ROS_INFO( "Get Temperature T1 : %.1f / T2 : %.1f [deg C]", t1, t2 );
  }
  else
  {
    ROS_ERROR( "Get Temperature failed. Error: %d", err );
  }
  
  return err;
}


int CameraDriver::getToFErrorStop( uint16_t& error_stop )
{
  uint16_t data[5] = { TOF_GET_ERROR_STOP, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
  if ( err == sizeof(data) )
  {
    error_stop = data[1];
    ROS_INFO( "Get Error Stop : %d", error_stop );
  }
  else
  {
    ROS_ERROR( "Get Error Stop failed. Error : %d", err );
  }
  
  return err;
}


int CameraDriver::getToFVersion( uint16_t& version_n,
                                 uint16_t& build_n,
                                 uint16_t& build_y,
                                 uint16_t& build_d )
{
  uint16_t data[5] = { TOF_GET_VERSION, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


int CameraDriver::getToFErrorInfo( uint16_t& common_err,
                                   uint16_t& eeprom_err_factory,
                                   uint16_t& eeprom_err,
                                   uint16_t& mipi_temp_err )
{
  uint16_t data[5] = { TOF_GET_ERROR_INFO, 0, 0, 0, 0 };
  int err;
  
  err = getToFCtrl( data, sizeof(data) );
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


void CameraDriver::publishToFTemperature( std::string frame_id )
{
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


void CameraDriver::CloseCamera()
{
  uvc_close( devh_ );
  devh_ = NULL;
  
  uvc_unref_device( dev_ );
  dev_ = NULL;
  
  state_ = Stopped;
}

};
