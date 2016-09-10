#ifndef MAKER_BINOCULAR_H
#define MAKER_BINOCULAR_H
#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>

typedef unsigned char u8;

/**
 * @brief Driver for maker binocular 
 * Stereo images and 3 axis accelemeters and 3 axis gyroscope data are transfered
 * 
 */
class makerbinocular {
public:
    /**
     * @brief Constructor of class makerbinocular
     * 
     */
    makerbinocular();
    
    /**
     * @brief Deconstructor of the makerbinocular
     * 
     */
    ~makerbinocular();
    
    /**
     * @brief Init the maker binocular driver
     * 
     * @return void
     */
    void init();
    
    /**
     * @brief Get the transfered image
     * 
     * @param left_image left image of the stereo camera
     * @param right_image right image of the stereo camera
     * @return bool True: get new full image,  false: doesn't get new full image
     */
    bool get_frame(cv::Mat & left_image,  cv::Mat & right_image, float acc[12],  float gyro[12], float camera_interval, float imu_interval[4]);
    
    /**
     * @brief Get the flag weather the driver has been initialized
     * 
     * @return bool
     */
    bool is_initialized() {return initialized;}
    
    /**
     * @brief Get the flag weather a new frame has been received succesfully
     * 
     * @return bool
     */
    bool new_frame_arrived() {return has_new_frame;}

    /**
     * @brief The time elapsed between the current frame and last frame
     * 
     * @return long
     */
    long get_camera_time_elapsed() {return time_elapsed;}
    
    long get_imu_time_elapsed() {return imu_time_elpased;}
    /**
     * @brief Total time elapsed since start for camera data
     * 
     * @return long int
     */
    long get_current_camera_time() {return current_image_time;}
    
    /**
     * @brief Total time elapsed since start for imu data
     * 
     * @return long int
     */
    long get_current_imu_time() {return current_imu_time;}
    
    /**
     * @brief Get the imu data,  three axis acc and three axis gyro
     * 
     * @param acc acc data for x axis(acc[0),  y axis(acc[1]),  z axis(acc[2])
     * @param gyro gyro data for x axis(acc[0],  y axis(acc[1]),  z axis(acc[2])
     * @return void
     */
    void get_imu_data(float acc[3],  float gyro[3]);

private:

    /// libusb device pointer
    libusb_device ** devs;
    /// libusb context
    libusb_context *contex = NULL;

    /// libusb device descriptor
    struct libusb_device_descriptor desc;
    /// 
    struct libusb_config_descriptor * config;

    libusb_device *device;
    libusb_device_handle *dev_handle;

    u8 * image_buff;
    
    /// time elapsed since last camera frame,  unit is us
    long time_elapsed;
    /// time elapsed since last imu frame,  unit is us
    long imu_time_elpased;
    /// the time elapsed since start of camera,  unit is us
    long current_image_time;
    ///  the time elapsed since the start of imu,  unit is us
    long current_imu_time;
    
    /// builk in end point address
    u8 bulk_ep_in;
    
    // sample 1000 imu datas to estimate tht zero bias
    int imu_init_state;
    double acc_zero_bias[3];
    double gyro_zero_bias[3];
    
    // 
    bool imu_initialized;
    // 
    float gyro_raw[3];
    float acc_raw[3];
    
    // left and right image
    cv::Mat left_image_;
    cv::Mat right_image_;
    
    // quater part
    int quarter_part_;
    
    int buffer_size;

    bool initialized;
    bool has_new_frame;
  };

#endif