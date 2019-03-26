#pragma once

#include <libuvc/libuvc.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/camera_publisher.h>
#include <dynamic_reconfigure/server.h>
#include <camera_info_manager/camera_info_manager.h>
#include <boost/thread/mutex.hpp>

#include <cis_camera/CISCameraConfig.h>


namespace cis_camera {

class CameraDriver {

public:
    
    CameraDriver( ros::NodeHandle nh, ros::NodeHandle priv_nh );
    ~CameraDriver();
    
    bool Start();
    void Stop();
    
    void TOF_SetMode_All();
    void TOF_GetInfo_All();
    
    
private:
    
    enum State {
        kInitial = 0,
        kStopped = 1,
        kRunning = 2,
    };
    
    // Flags controlling whether the sensor needs to be stopped (or reopened) when changing settings
    static const int kReconfigureClose   = 3; // Need to close and reopen sensor to change this setting
    static const int kReconfigureStop    = 1; // Need to stop the stream before changing this setting
    static const int kReconfigureRunning = 0; // We can change this setting without stopping the stream
    
    void OpenCamera();
    void CloseCamera();
    
    // Accept a reconfigure request from a client
    void ReconfigureCallback( CISCameraConfig &config, uint32_t level );
    enum uvc_frame_format GetVideoMode( std::string vmode );
    
    // Accept a new image frame from the camera
    void ImageCallback(uvc_frame_t *frame);
    static void ImageCallbackAdapter( uvc_frame_t *frame, void *ptr );
    
    
    // TOF Camera
    
    enum tof_process_num {
        TOF_SET_EEPROM          = 0x0000,
        TOF_SET_DEPTH_IR        = 0x0001,
        TOF_SET_DEPTH_RANGE     = 0x0002,
        TOF_SET_THRESHOLD       = 0x0003,
        TOF_SET_NR_FILTER       = 0x0004,
        TOF_SET_PULSE_COUNT     = 0x0005,
        TOF_SET_LD_ENABLE       = 0x0006,
        TOF_SET_IR_GAIN         = 0x0009,
        TOF_SET_ERROR_STOP      = 0x0010,
        TOF_SET_ERROR_CLEAR     = 0x7F01,
        TOF_GET_DEPTH_IR        = 0x8001,
        TOF_GET_DEPTH_RANGE     = 0x8002,
        TOF_GET_THRESHOLD       = 0x8003,
        TOF_GET_NR_FILTER       = 0x8004,
        TOF_GET_PULSE_COUNT     = 0x8005,
        TOF_GET_LD_ENABLE       = 0x8006,
        TOF_GET_DEPTH_CNV_GAIN  = 0x8007,
        TOF_GET_DEPTH_INFO      = 0x8008,
        TOF_GET_IR_GAIN         = 0x8009,
        TOF_GET_TEMPERATURE     = 0x800A,
        TOF_GET_ERROR_STOP      = 0x8010,
        TOF_GET_VERSION         = 0xFF00,
        TOF_GET_ERROR_INFO      = 0xFF01,
    };
    
    enum tof_eeprom_mode {
        TOF_EEPROM_FACTORY_DEFAULT  = 0x0000,
        TOF_EEPROM_UPDATE_CURRENT   = 0x0001,
    };
    
    int TOF_SetCtrl( uint16_t *data ,int len );
    int TOF_GetCtrl( uint16_t *data ,int len );
    
    int TOF_SetMode_ROSParameter( std::string param_name );
    int TOF_SetEEPROMMode( uint16_t mode );
    int TOF_ClearError();
    
    int TOF_GetDepthIR( uint16_t& depth_ir );
    int TOF_GetDepthRange( uint16_t& depth_range, uint16_t& dr_index );
    int TOF_GetThreshold( uint16_t& threshold );
    int TOF_GetNRFilter( uint16_t& nr_filter );
    int TOF_GetPulseCount( uint16_t& pulse_count );
    int TOF_GetLDEnable( uint16_t& ld_enable );
    int TOF_GetDepthCnvGain( double& depth_cnv_gain );
    int TOF_GetDepthInfo( short&          offset_val, 
                          unsigned short& max_data, 
                          unsigned short& min_dist, 
                          unsigned short& max_dist  );
    int TOF_GetIRGain( uint16_t& ir_gain );
    int TOF_GetTemperature( double& t1, double& t2 );
    int TOF_GetErrorStop( uint16_t& error_stop );
    int TOF_GetVersion( uint16_t& version_n, 
                        uint16_t& build_n, 
                        uint16_t& build_y, 
                        uint16_t& build_d );
    int TOF_GetErrorInfo( uint16_t& common_err, 
                          uint16_t& eeprom_err_factory, 
                          uint16_t& eeprom_err, 
                          uint16_t& mipi_temp_err );
    
    void TOF_PublishTemperature( std::string frame_id );
    
    ros::Publisher tof_t1_pub_;
    ros::Publisher tof_t2_pub_;
    
    // END TOF Camera
    
    ros::NodeHandle nh_, priv_nh_;
    
    State                  state_;
    boost::recursive_mutex mutex_;
    
    uvc_context_t       *ctx_;
    uvc_device_t        *dev_;
    uvc_device_handle_t *devh_;
    uvc_frame_t         *rgb_frame_;
    
    image_transport::ImageTransport  it_;
    image_transport::CameraPublisher cam_pub_;
    
    dynamic_reconfigure::Server<CISCameraConfig> config_server_;
    
    CISCameraConfig config_;
    bool            config_changed_;
    
    camera_info_manager::CameraInfoManager cinfo_manager_;
};

};
