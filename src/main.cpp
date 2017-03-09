#include <iostream>
#include <stdio.h>
#include <libusb-1.0/libusb.h>
#include <ctime>

#include <opencv2/opencv.hpp>
#include <fstream>
#include "maker_binocular.h"


// producer
// only store the last 30 image frame
class CameraDriver
{
public:
    CameraDriver();
    ~CameraDriver();
    
    void Run();  
  
private:
    std::vector<cv::Mat> left_imgs_;
    std::vector<cv::Mat> right_imgs_;    
    std::vector<imu_msg> imu_msgs_;

    int img_idx_ = 0;
    int imu_idx_ = 0;    
};

int main()
{
    makerbinocular m_makerbinocular;

    cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
    cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

    float image_interval; // image interval,
    float imu_interval[4];// imu timestamp, time interval after last sample
    float acc[12];
    float gyro[12];

    //std::ofstream myfile("imu_data.txt");
    	
    {
        while (1)
        {
            std::clock_t begin = clock();
            if (!m_makerbinocular.get_frame(left_image, right_image, acc,  gyro, image_interval,  imu_interval))
                continue;

            if (m_makerbinocular.new_frame_arrived())
            {
                cv::imshow("left image", left_image);
                cv::waitKey(1);
                cv::imshow("right image" ,  right_image);
                cv::waitKey(1);
            }
            
            std::clock_t end = clock();
            double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
            std::cout <<  "Time per frame:" << elapsed_secs << std::endl;
            
            std::cout <<  "Framerate is:" << 1.0f/elapsed_secs << std::endl;
        }
    }

    return 0;
}
