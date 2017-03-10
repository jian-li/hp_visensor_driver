#ifndef MAKER_BINOCULAR_H
#define MAKER_BINOCULAR_H
#include <opencv2/opencv.hpp>
#include <mutex>
#include <thread>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>

using namespace cv;
using namespace std;

#define IMAGE_PART 10
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define IMU_BUFFER_SIZE 300
#define IMAGE_BUFFER_SIZE 30

class imu_msg
{
public:
    imu_msg();
    ~imu_msg();

    void set_ts(double ts);
    // set acc data
    void set_acc_data(double acc_x, double acc_y, double acc_z);
    // set gyro data
    void set_gyro_data(double gyro_x, double gyro_y, double gyro_z);
    // print imu msg conten
    void print_content();

private:
    double ts_;                                               // time elapsed from the start of the driver
    double acc_x_, acc_y_, acc_z_;
    double gyro_x_, gyro_y_, gyro_z_;
};

//  the left image and right image are hardware synchronized
class image_msg
{
public:
    image_msg();
    ~image_msg();

    void set_image_msg(double ts, Mat& left_image, Mat& right_image);
    
    inline Mat left_image() {return left_image_;}
    inline Mat right_image() {return right_image_;}

private:
    // time elapsed from the start of the driver
    double ts_;
    Mat left_image_;
    Mat right_image_;
    bool is_color_;
};

typedef unsigned char u8;

/**
 * @brief Camera Driver for maker binocular 
 * Stereo images and 3 axis accelemeters and 3 axis gyroscope data are transfered
 * 
 */
class CameraDriver{
public:
    /**
     * @brief Constructor of class CameraDriver 
     * 
     */
    CameraDriver();
    
    /**
     * @brief Deconstructor of the makerbinocular
     * 
     */
    ~CameraDriver();
    
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
    bool get_frame(cv::Mat & left_image,  cv::Mat & right_image, float acc[12],  float gyro[12], float &camera_interval, float imu_interval[4]);
    
    /**
     * @brief Get the flag weather the driver has been initialized
     * 
     * @return bool
     */
    bool is_initialized() {return initialized_;}
    
    /**
     * @brief Get the imu data,  three axis acc and three axis gyro
     * 
     * @param acc acc data for x axis(acc[0),  y axis(acc[1]),  z axis(acc[2])
     * @param gyro gyro data for x axis(acc[0],  y axis(acc[1]),  z axis(acc[2])
     * @return void
     */
    void get_imu_data(float acc[3],  float gyro[3]);
    
    /**
     * @brief Produce imu and image message
     */
    void produce();
    
    void consume(vector<image_msg>& image_msgs, vector<imu_msg>& imu_msgs);

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

    u8 *data_buff;
    u8 *image_buff;
    int buffer_size;

    /// builk in end point address
    u8 bulk_ep_in;
    
    // hardware initialize
    bool initialized_;

    // initial time stamp, unit is us
    double initial_ts_;
    double current_ts_;

    //##################image transfer related variable####

    // the header 32 byte is cofig bytes and imu bytes
    int usb_packet_header_size_;
    // the continues bytes are image content bytes
    int usb_packet_image_size_;

    //#################hardware setting####################
    bool is_color_;

    // ring buffer store imu and image message
    std::deque<imu_msg> imu_deque_;
    int imu_buf_size_;
    int imu_received_num_;
    std::mutex imu_mutex_;
    std::deque<image_msg> img_deque_;
    int img_buf_size_;
    int img_received_num_;
    std::mutex img_mutex_;
    
    std::thread producer_thread;
};

#endif
