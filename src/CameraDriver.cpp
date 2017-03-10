#include "CameraDriver.h"
#include <iostream>
#include <stdio.h>
#include <thread>

using namespace std;

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

void imu_msg::set_ts(double ts)
{
    ts_ = ts;
}

void imu_msg::set_acc_data(double acc_x, double acc_y, double acc_z)
{
    acc_x_ = acc_x;
    acc_y_ = acc_y;
    acc_z_ = acc_z;
}

void imu_msg::set_gyro_data(double gyro_x, double gyro_y, double gyro_z)
{
    gyro_x_ = gyro_x;
    gyro_y_ = gyro_y;
    gyro_z_ = gyro_z;
}

void imu_msg::print_content()
{
    cout <<"acc value is:" << acc_x_ << " " << acc_y_ << " " << acc_z_ << endl;
    cout <<"gyro value is:" << gyro_x_ << " " << gyro_y_ << " " << gyro_z_ << endl;
}

imu_msg::~imu_msg()
{

}

image_msg::image_msg()
{
    ts_ = 0;

}

void image_msg::set_image_msg(double ts, Mat &left_image, Mat &right_image)
{
    ts_ = ts;
    left_image_ = left_image.clone();
    right_image_ = right_image.clone();
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
    
    producer_thread.join();
}

void CameraDriver::init()
{
    // receive image and imu number
    imu_received_num_ = 0;
    img_received_num_ = 0;

    //#####set hardware related variables##########
    is_color_ = false;
    initial_ts_ = 0;

    //#####set image usb transfer related variables
    usb_packet_header_size_ = 32;
    if(is_color_)
    {
        usb_packet_image_size_ = (int)IMAGE_WIDTH*IMAGE_HEIGHT*3*2/IMAGE_PART;
    }
    else
    {
        usb_packet_image_size_ = (int)IMAGE_WIDTH*IMAGE_HEIGHT*2/IMAGE_PART;
    }
    
    //
    buffer_size = IMAGE_WIDTH*IMAGE_HEIGHT*2+2048;
    data_buff = new u8[buffer_size];

    img_buf_size_ = 0;
    imu_buf_size_ = 0;

    //
    if(is_color_)
    {
        image_buff = new u8[IMAGE_WIDTH*IMAGE_HEIGHT*3*2];
    }
    else
    {
        image_buff = new u8[IMAGE_WIDTH*IMAGE_HEIGHT*2];
    }

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
    
    initialized_ = true;
    
    // multi thread 
    producer_thread = thread(&CameraDriver::produce, this);
}

// produce image message and imu message
void CameraDriver::produce()
{
    while (true)
    {
        int processed_image_part = 0;
        double image_ts;
        for (int  i = 0; i < IMAGE_PART; i++)
        {
            processed_image_part++;

            int transfered = 0;
            int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, data_buff, buffer_size, &transfered, 1000);

            // libusb transfer error
            if(error != 0)
            {
                processed_image_part = 0;
                continue;
            }

            // synchronize the image part
            if (*(data_buff + 3) !=  i)
            {
                processed_image_part = 0;
                continue;
            }

            if (transfered ==  0)
            {
                std::cout <<  "============================================" <<  std::endl;
                std::cout <<  "Warning: No data received ! Please check the buld endpoint address" <<  std::endl;
                std::cout <<  "============================================" <<  std::endl;
                processed_image_part = 0;
                continue;
            }

            //#######First process the 32 bytes config part####
            // the first four byte is frame header
//            if(data_buff[0] != 0x01|| data_buff[1]!=0xfe||data_buff[2]!=0x01||data_buff[3]!=0xfe)
//                continue;

            //#####byte 8 and byte 9 stand for two imu ssampling interval time
            int time_interval = *(data_buff + 8) | *(data_buff + 9) <<  8;
            //time stamp one stand for 256 108M clock duty
            double time_elapsed = 1.0 * time_interval * 256 / 108;
//            std::cout <<  time_elapsed <<  std::endl;
            current_ts_ += time_elapsed;
            if(i == 0)
                image_ts = current_ts_;

            //######Then process the succeding image part
            memcpy(image_buff+i*usb_packet_image_size_, data_buff + 32, usb_packet_image_size_);

            //#####process the imu message
            int raw_acc_x = (short) (data_buff[12] | data_buff[13] << 8);
            int raw_acc_y = (short) (data_buff[14] | data_buff[15] << 8);
            int raw_acc_z = (short) (data_buff[16] | data_buff[17] << 8);
            int raw_gyro_x = (short) (data_buff[18] | data_buff[19] << 8);
            int raw_gyro_y = (short) (data_buff[20] | data_buff[21] << 8);
            int raw_gyro_z = (short) (data_buff[22] | data_buff[23] << 8);

            double acc_x_measure = raw_acc_x * 1.0f / 16384 * 9.8;
            double acc_y_measure = raw_acc_y * 1.0f / 16384 * 9.8;
            double acc_z_measure = raw_acc_z * 1.0f / 16384 * 9.8;

            double gyro_x_measure = raw_gyro_x * 1.0f / 16.4;
            double gyro_y_measure = raw_gyro_y * 1.0f / 16.4;
            double gyro_z_measure = raw_gyro_z * 1.0f / 16.4;

//            cout << "IMU gyro measurement:" << gyro_x_measure << " " << gyro_y_measure << " "
//                 << gyro_z_measure << endl;

//            cout << "IMU acc measurement:" << acc_x_measure << " " << acc_y_measure << " "
//                 << acc_z_measure << endl;

            imu_msg new_imu_msg;
            new_imu_msg.set_ts(current_ts_);
            new_imu_msg.set_acc_data(acc_x_measure, acc_y_measure, acc_z_measure);
            new_imu_msg.set_gyro_data(gyro_x_measure, gyro_y_measure, gyro_z_measure);

            {
//                cout << "Get imu message" << endl;
                imu_mutex_.lock();
                if(imu_buf_size_ >= IMU_BUFFER_SIZE)
                {
                    imu_deque_.pop_front();
                    imu_buf_size_--;
                }

                imu_deque_.push_back(new_imu_msg);
                imu_buf_size_++;

                imu_mutex_.unlock();
            }

            int raw_temprature = (short) (data_buff[24] | data_buff[25] << 8);
        }

        // change image data to
        if(processed_image_part == IMAGE_PART)
        {
            int cnt_y,  cnt_x;

            Mat left_image, right_image;
            if(is_color_)
            {
                left_image = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
                right_image = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);
            }
            else
            {
                left_image = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
                right_image = Mat::zeros(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC1);
            }

            u8* pcS = image_buff;
            for (cnt_y  = 0; cnt_y < 480; cnt_y++)
            {
                for (cnt_x = 0; cnt_x < 640; cnt_x++)
                {
                    // left image, the image is flipped
                    left_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2);

                    // right image
                    right_image.at<uchar>(479 - cnt_y, cnt_x) = *(pcS + cnt_y * 1280 + cnt_x * 2 + 1);
                }
            }

//            imshow("thread left image", left_image);
//            cvWaitKey(1);

            image_msg new_image_msg;
            new_image_msg.set_image_msg(image_ts, left_image, right_image);

            //
            {
//                cout <<  "Get image message" <<  endl;
                img_mutex_.lock();

                if(img_buf_size_ >= IMAGE_BUFFER_SIZE)
                {
                    img_deque_.push_front(image_msg());
                    img_buf_size_--;
                }

                img_deque_.push_back(new_image_msg);
                img_buf_size_++;

                img_mutex_.unlock();
            }
        }

        processed_image_part = 0;
        
        usleep(20000);
        //
    }
}

void CameraDriver::consume(vector<image_msg>& image_msgs, vector<imu_msg>& imu_msgs)
{
    image_msgs.clear();
    imu_msgs.clear();

    // consume image messages
    {
        img_mutex_.lock();
        while(img_buf_size_>0)
        {
            image_msgs.push_back(img_deque_.front());
            img_deque_.pop_front();
            img_buf_size_--;
        }
        img_mutex_.unlock();
    }

    // consume imu messages
    {
        imu_mutex_.lock();
        while(imu_buf_size_>0)
        {
            imu_msgs.push_back(imu_deque_.front());
            imu_deque_.pop_front();
            imu_buf_size_--;
        }
        imu_mutex_.unlock();
    }
}

