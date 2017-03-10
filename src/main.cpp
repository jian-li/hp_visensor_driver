#include <iostream>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <chrono>

#include <opencv2/opencv.hpp>
#include <fstream>
#include "CameraDriver.h"

typedef std::chrono::high_resolution_clock Clock;

int main()
{
    CameraDriver stereo_driver;

    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

    float image_interval; // image interval,
    float imu_interval[4];// imu timestamp, time interval after last sample
    float acc[12];
    float gyro[12];
    	
    vector<image_msg> image_msgs;
    vector<imu_msg> imu_msgs;	
    	
    auto t1 = Clock::now();

    int receive_image_num = 0;

    while (1)
    {
        stereo_driver.consume(image_msgs, imu_msgs);

        cout << "Consum image size:" << image_msgs.size() << endl;

        if (image_msgs.size() >0)
        {
            imshow("left image", image_msgs[0].left_image());
            cvWaitKey(1);

            imshow("right image", image_msgs[0].right_image());
            cvWaitKey(1);

            receive_image_num++;

            auto t2 = Clock::now();

            double time_from_start = 1e-9 * std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

            imu_msgs[0].print_content();
            cout << "Fps is: " << 1.0f / (time_from_start/receive_image_num) << endl;
        }
        
        usleep(30000);
    }

    return 0;
}
