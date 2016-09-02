#include <iostream>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <boost/concept_check.hpp>

#include <opencv2/opencv.hpp>

#define bulk_ep_in 0x86
libusb_device *device;
libusb_device_handle *dev_handle;

typedef unsigned char u8;

int main()
{
    libusb_device ** devs;
    libusb_device * device;
    libusb_context *contex = NULL;
    struct libusb_device_descriptor desc;
    struct libusb_config_descriptor * config;

    int err;

    int r;
    ssize_t cnt;

    // initialize a library session
    r = libusb_init(&contex);

    if (r < 0)
    {
        std::cout <<  "Init error!" <<  std::endl;
    }

    // set verbosity level to 3
    libusb_set_debug(contex,  3);

    cnt = libusb_get_device_list(contex,  &devs);

    if (cnt < 0)
    {
        std::cout <<  "Get devices error" <<  std::endl;
    }
    else
    {
        std::cout <<  "Get " <<  cnt <<  " devices in total" <<  std::endl;
    }

    // found cypress usb device
    for (int i = 0; i < cnt; i++)
    {
        device = devs[i];
        err = libusb_get_device_descriptor(device, &desc);

        if (err < 0)
        {
            std::cout <<  "failed to get desc" <<  std::endl;
            continue;
        }

        if (desc.idVendor ==  0x04b4 && desc.idProduct ==  0x1005)
        {
            std::cout <<  "found cypress usb" <<  std::endl;
            break;
        }
    }

    // get cypress usb config desc
    libusb_get_config_descriptor(device,  0,  &config);

    // 

    if ((int)config->bNumInterfaces > 1)
    std::cout <<  "too many interfaces" <<  std::endl;

    const struct libusb_interface * inter;
    const struct libusb_interface_descriptor * interdesc;
    const struct libusb_endpoint_descriptor * epdesc;

    inter = &config->interface[0];
    interdesc = &inter->altsetting[0];

    if ((int) interdesc->bNumEndpoints > 2)
    std::cout <<  "too many endpoints" <<  std::endl;

    for (int j = 0; j < interdesc->bNumEndpoints; j++)
    {
        epdesc = &interdesc->endpoint[j];

        if ((epdesc->bEndpointAddress) & 0x80)
        {
            printf("Bulk in endpoint 0x%02x\n" ,  epdesc->bEndpointAddress);
            printf("Max packetsize is %d \n",  epdesc->wMaxPacketSize);
        }
    }

    err = libusb_open(device, &dev_handle);

    if (err < 0)
    {
        printf("open device failed\n");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return 0;
    }

    if (libusb_kernel_driver_active(dev_handle,  0) ==  1)
    {
        printf("Kernel Driver Active\n" );
        if (libusb_detach_kernel_driver(dev_handle,  0 ) ==  0)
        printf("Kernal Driver Detached\n");
    }

    err = libusb_claim_interface(dev_handle,  0);
    if (err < 0)
    {
        printf("can not claim interface");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return 0;
    }


    int size;
    unsigned char * datain = new u8[640 * 480 *2 + 2048];
    int transferd;
    int buffer_size  = 640 * 480 * 2 + 2048;

    unsigned char * image = new u8[640 * 480 * 6];

    unsigned char * pcS = (u8*) (datain + 32);
    unsigned char * pcD = image; 
     
    cv::Mat left_image_raw(640, 480, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image_raw(640, 480, CV_8UC1, cv::Scalar(0));
    
    cv::Mat left_image_rgb(640, 480, CV_8UC3, cv::Scalar(0));
    cv::Mat right_image_rgb(640, 480, CV_8UC3, cv::Scalar(0));
    
    while (1)
    {
        int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, datain, buffer_size, &transferd, 1000);

        std::cout <<  transferd <<  std::endl;

        if (error == 0) 
        {
            std::cout << "received " <<  std::endl;

            // frame header
            if (((*datain) ==  0x01) & (*(datain+1) ==  0xfe) & (*(datain+2) ==  0x01) & (*(datain + 3) ==  0xfe))
            {
                std::cout <<  "get frame header" <<  std::endl;
            }
            int cnt_y,  cnt_x;
            for (cnt_y  = 0; cnt_y < 480; cnt_y++)
            {
                for (cnt_x = 0; cnt_x < 1280; cnt_x++)
                {
                    
                    
                    // left image
                    left_image_raw.at<uchar>(cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);
                 
                    
                    // right image
                    right_image_raw.at<uchar>(cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
                 
                }
            }
        }
        
        cv::cvtColor(left_image_raw,  left_image_rgb,  CV_BayerBG2RGB);
        cv::cvtColor(right_image_raw, right_image_rgb,  CV_BayerBG2BGR);
        
        cv::imshow("left image", left_image_rgb);
        cv::waitKey(1);
        
        cv::imshow("right image" ,  right_image_rgb);
        cv::waitKey(1);
        
        std::cout <<  "get on frame" <<  std::endl;
        usleep(30000);

    }
    
    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);

    return 0;
}