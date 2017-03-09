#include "CameraDriver.h"
#include <iostream>
#include <stdio.h>

imu_msg::imu_msg()
{
    ts_ = 0;
    acc_x_ = 0;
    acc_y_ = 0;
    acc_z_ = 0;
    gyro_x_ = 0;
    gyro_y_ = 0;
    gyro_z_ = 0;
}

imu_msg::~imu_msg()
{

}

image_msg::image_msg()
{
    ts_ = 0;
    
    if (is_color_ == false)
    {
        left_image_ = Mat::zeros(640, 480, CV_8UC1);
        right_image_ = Mat::zeros(640, 480, CV_8UC1);
    }
    else
    {
        left_image_ = Mat::zeros(640, 480, CV_8UC3);
        right_image_ = Mat::zeros(640, 480, CV_8UC3);
    }
}

image_msg::~image_msg()
{

}

CameraDriver::CameraDriver()
{
    init();
}

CameraDriver::~CameraDriver()
{
    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);
    
    delete image_buff;
}


void CameraDriver::init()
{
    initialized = false;
    imu_initialized = false;
    // init required data
    buffer_size = 640 * 480 * 2 + 2048; 
    image_buff = new u8[buffer_size];
    has_new_frame = false;
    imu_init_state = 0;
    
    
    current_image_time = 0;
    current_imu_time = 0;
    time_elapsed =  0;
    
    int r;
    int err;
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
    bool idVendorAndProductfound =  false;
    for (int i = 0; i < cnt; i++)
    {
        device = devs[i];
        err = libusb_get_device_descriptor(device, &desc);

        if (err < 0)
        {
            std::cout <<  "failed to get desc" <<  std::endl;
            return; 
        }

        if (desc.idVendor ==  0x04b4 && desc.idProduct ==  0x1005)
        {
            std::cout <<  "============================================" << std::endl;
            printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n", desc.idVendor,desc.idProduct);
            std::cout <<  "============================================" <<  std::endl;
            idVendorAndProductfound = true;
            break;
        }
    }

    if (idVendorAndProductfound ==  false)
    {
        std::cout <<  "============================================" <<  std::endl;
        std::cout <<  "Error: Can not found the device, please check the idVendor and idProduct!" <<  std::endl;
        std::cout <<  "============================================" <<  std::endl;
        return ;
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
            bulk_ep_in = epdesc->bEndpointAddress;
            printf("Hints: Get Built in endpoint: 0x%02x\n" ,  epdesc->bEndpointAddress);
            printf("Max packetsize is %d \n",  epdesc->wMaxPacketSize);
        }
    }

    err = libusb_open(device, &dev_handle);

    if (err < 0)
    {
        printf("open device failed\n");
        libusb_free_device_list(devs, 1);
        libusb_close(dev_handle);
        return ;
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
        return; 
    }
    
    initialized = true;
}

void CameraDriver::get_imu_data(float acc[3], float gyro[3])
{
    for (int i = 0; i < 3; i++)
    {
        acc[i] = acc_raw[i];
        gyro[i] = gyro_raw[i]; 
    }
}

// produce image message and imu message
void CameraDriver::produce()
{
    while (true)
    {
        for (int  i = 0; i < IMAGE_PART; i++)
        {
            int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, data_buff, buffer_size, &transferd, 1000);

            if (*(data_buff + 3) !=  i)
            return false;

            if (transferd ==  0)
            {
                std::cout <<  "============================================" <<  std::endl;
                std::cout <<  "Warning: No data received ! Please check the buld endpoint address" <<  std::endl;
                std::cout <<  "============================================" <<  std::endl;
                return false;
            }

            memcpy(image_buff + i * data_incremental, data_buff + 32, data_incremental);

            if (error == 0) 
            {
                // frame header
                if (((*data_buff) ==  0x01) & (*(data_buff + 1) ==  0xfe) & (*(data_buff + 2) ==  0x01) & (*(data_buff + 3) ==  0xfe))
                {

                }
                printf("%2x %2x %2x %2x\n",  *(data_buff),  *(data_buff + 1),  *(data_buff + 2),  *(data_buff + 3));

                int time_stamp = *(data_buff + 8) | *(data_buff + 9) <<  8;

                time_elapsed = 1.0 * time_stamp * 256 / 108;
                std::cout <<  time_elapsed <<  std::endl;

                // time elapsed 
                imu_interval[i] = time_elapsed;
                image_interval = image_interval + time_elapsed;

                current_image_time = current_image_time + time_elapsed;
                current_imu_time = current_imu_time + time_elapsed;

                int raw_acc_x = (short) (data_buff[12] | data_buff[13] << 8);
                int raw_acc_y = (short) (data_buff[14] | data_buff[15] << 8);
                int raw_acc_z = (short) (data_buff[16] | data_buff[17] << 8);

                int raw_gyro_x = (short) (data_buff[18] | data_buff[19] << 8);
                int raw_gyro_y = (short) (data_buff[20] | data_buff[21] << 8);
                int raw_gyro_z = (short) (data_buff[22] | data_buff[23] << 8);

                int raw_temprature = (short) (data_buff[24] | data_buff[25] << 8);

                acc_raw[0 + 3 * i] = raw_acc_x * 1.0 / 16384 * 9.8;
                acc_raw[1 + 3 * i] = raw_acc_y * 1.0 / 16384 * 9.8;
                acc_raw[2 + 3 * i] = raw_acc_z * 1.0 / 16384 * 9.8;

                gyro_raw[0 + 3 * i] = raw_gyro_x * 1.0 / 16.4;
                gyro_raw[1 + 3 * i] = raw_gyro_y * 1.0 / 16.4;
                gyro_raw[2 + 3 * i] = raw_gyro_z * 1.0 / 16.4;

                if (imu_init_state <= 100)
                {
                    acc_zero_bias[0] = (acc_zero_bias[0] * imu_init_state + acc_raw[0 + 3 * i]) / (imu_init_state + 1);
                    acc_zero_bias[1] = (acc_zero_bias[1] * imu_init_state + acc_raw[1 + 3 * i]) / (imu_init_state + 1);
                    acc_zero_bias[2] = (acc_zero_bias[2] * imu_init_state + acc_raw[2 + 3 * i]) / (imu_init_state + 1);

                    gyro_zero_bias[0] = (gyro_zero_bias[0] * imu_init_state + gyro_raw[0 + 3 * i]) / (imu_init_state + 1);
                    gyro_zero_bias[1] = (gyro_zero_bias[1] * imu_init_state + gyro_raw[1 + 3 * i]) / (imu_init_state + 1);
                    gyro_zero_bias[2] = (gyro_zero_bias[2] * imu_init_state + gyro_raw[2 + 3 * i]) / (imu_init_state + 1);
                }
                else
                {
                    imu_initialized = true;

                    gyro_raw[0 + 3 * i] -=  gyro_zero_bias[0];
                    gyro_raw[1 + 3 * i] -=  gyro_zero_bias[1];
                    gyro_raw[2 + 3 * i] -=  gyro_zero_bias[2];
                }

                imu_init_state++;

                if (imu_initialized)
                {
                    std::cout <<  acc_raw[0 + 3 * i] << " " <<  acc_raw[1 + 3 * i] <<  " "<< acc_raw[2 + 3 * i] <<  std::endl;
                    std::cout <<  gyro_raw[0 + 3 * i] << " " <<  gyro_raw[1 + 3 * i] <<  " "<< gyro_raw[2 + 3 * i] <<  std::endl;
                }
            }

            int cnt_y,  cnt_x;
            for (cnt_y  = 0; cnt_y < 480; cnt_y++)
            {
                for (cnt_x = 0; cnt_x < 640; cnt_x++)
                {
                    // left image
                    left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);

                    // right image
                    right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
                }
            }
        }
    }
}

// 
bool CameraDriver::get_frame(cv::Mat &left_image, cv::Mat &right_image, float acc[12],  float gyro[12],float& image_interval, float imu_interval[4])
{
    time_elapsed = 0;
    has_new_frame = false;
    
    if (left_image.rows !=  480 |  left_image.cols !=  640 |  right_image.rows !=  480 |  right_image.cols !=  640)
    {
        std::cout <<  left_image.rows <<  left_image.cols <<  std::endl;
        std::cout <<  "Error: the image size should be: 640 x 480" <<  std::endl;
    }
    
    int transferd;
    unsigned char * pcS = (u8*) (image_buff);
    const int picture_part = 10;

    const int part_frame_size = 640 * 480 * 2 / picture_part + 32;
    const int data_incremental = 640 * 480 * 2 / picture_part;
    
    u8 data_buff[part_frame_size];

    for (int  i = 0; i < picture_part; i++)
    {
        int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, data_buff, buffer_size, &transferd, 1000);
        
        if (*(data_buff + 3) !=  i)
            return false;
        
        if (transferd ==  0)
        {
            std::cout <<  "============================================" <<  std::endl;
            std::cout <<  "Warning: No data received ! Please check the buld endpoint address" <<  std::endl;
            std::cout <<  "============================================" <<  std::endl;
            return false;
        }
        
        memcpy(image_buff + i * data_incremental, data_buff + 32, data_incremental);

        if (error == 0) 
        {
            // frame header
            if (((*data_buff) ==  0x01) & (*(data_buff + 1) ==  0xfe) & (*(data_buff + 2) ==  0x01) & (*(data_buff + 3) ==  0xfe))
            {
                
            }
            printf("%2x %2x %2x %2x\n",  *(data_buff),  *(data_buff + 1),  *(data_buff + 2),  *(data_buff + 3));

            int time_stamp = *(data_buff + 8) | *(data_buff + 9) <<  8;

            time_elapsed = 1.0 * time_stamp * 256 / 108;
            std::cout <<  time_elapsed <<  std::endl;
            
            // time elapsed 
            imu_interval[i] = time_elapsed;
            image_interval = image_interval + time_elapsed;

            current_image_time = current_image_time + time_elapsed;
            current_imu_time = current_imu_time + time_elapsed;

            int raw_acc_x = (short) (data_buff[12] | data_buff[13] << 8);
            int raw_acc_y = (short) (data_buff[14] | data_buff[15] << 8);
            int raw_acc_z = (short) (data_buff[16] | data_buff[17] << 8);

            int raw_gyro_x = (short) (data_buff[18] | data_buff[19] << 8);
            int raw_gyro_y = (short) (data_buff[20] | data_buff[21] << 8);
            int raw_gyro_z = (short) (data_buff[22] | data_buff[23] << 8);

            int raw_temprature = (short) (data_buff[24] | data_buff[25] << 8);

            acc_raw[0 + 3 * i] = raw_acc_x * 1.0 / 16384 * 9.8;
            acc_raw[1 + 3 * i] = raw_acc_y * 1.0 / 16384 * 9.8;
            acc_raw[2 + 3 * i] = raw_acc_z * 1.0 / 16384 * 9.8;

            gyro_raw[0 + 3 * i] = raw_gyro_x * 1.0 / 16.4;
            gyro_raw[1 + 3 * i] = raw_gyro_y * 1.0 / 16.4;
            gyro_raw[2 + 3 * i] = raw_gyro_z * 1.0 / 16.4;

            if (imu_init_state <= 100)
            {
                acc_zero_bias[0] = (acc_zero_bias[0] * imu_init_state + acc_raw[0 + 3 * i]) / (imu_init_state + 1);
                acc_zero_bias[1] = (acc_zero_bias[1] * imu_init_state + acc_raw[1 + 3 * i]) / (imu_init_state + 1);
                acc_zero_bias[2] = (acc_zero_bias[2] * imu_init_state + acc_raw[2 + 3 * i]) / (imu_init_state + 1);

                gyro_zero_bias[0] = (gyro_zero_bias[0] * imu_init_state + gyro_raw[0 + 3 * i]) / (imu_init_state + 1);
                gyro_zero_bias[1] = (gyro_zero_bias[1] * imu_init_state + gyro_raw[1 + 3 * i]) / (imu_init_state + 1);
                gyro_zero_bias[2] = (gyro_zero_bias[2] * imu_init_state + gyro_raw[2 + 3 * i]) / (imu_init_state + 1);
            }
            else
            {
                imu_initialized = true;
                
                gyro_raw[0 + 3 * i] -=  gyro_zero_bias[0];
                gyro_raw[1 + 3 * i] -=  gyro_zero_bias[1];
                gyro_raw[2 + 3 * i] -=  gyro_zero_bias[2];
            }
            
            imu_init_state++;

            if (imu_initialized)
            {
                std::cout <<  acc_raw[0 + 3 * i] << " " <<  acc_raw[1 + 3 * i] <<  " "<< acc_raw[2 + 3 * i] <<  std::endl;
                std::cout <<  gyro_raw[0 + 3 * i] << " " <<  gyro_raw[1 + 3 * i] <<  " "<< gyro_raw[2 + 3 * i] <<  std::endl;
            }
        }

        int cnt_y,  cnt_x;
        for (cnt_y  = 0; cnt_y < 480; cnt_y++)
        {
            for (cnt_x = 0; cnt_x < 640; cnt_x++)
            {
                // left image
                left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);

                // right image
                right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
            }
        }
    }

    has_new_frame = true;

    return true;
}
