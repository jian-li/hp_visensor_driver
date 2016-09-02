#ifndef MAKER_BINOCULAR_H
#define MAKER_BINOCULAR_H
#include <opencv2/opencv.hpp>
#include <libusb-1.0/libusb.h>


typedef unsigned char u8;

class makerbinocular {
public:
    makerbinocular();
    
    ~makerbinocular();
    
    void init();
    
    void get_frame(cv::Mat & left_image,  cv::Mat & right_image);
    
    bool is_initialized() {return initialized;}
    
private:
    
    libusb_device ** devs;
    libusb_context *contex = NULL;
    
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor * config;
    
    libusb_device *device;
    libusb_device_handle *dev_handle;

    u8 * datain;
    
    u8 bulk_ep_in;
    int buffer_size;
    
    bool initialized;
  };

#endif