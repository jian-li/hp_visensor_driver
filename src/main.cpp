#include <chrono>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <stdio.h>

#include "ViSensorDriver.h"
#include <fstream>
#include <opencv2/opencv.hpp>

typedef std::chrono::high_resolution_clock Clock;

int main() {
  ViSensorDriver visensor_driver;

  cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
  cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

  float image_interval;  // image interval,
  float imu_interval[4]; // imu timestamp, time interval after last sample
  float acc[12];
  float gyro[12];

  queue<image_msg> image_msgs;
  queue<imu_msg> imu_msgs;

  int receive_image_num = 0;

  while (1) {
    visensor_driver.get_frame(imu_msgs, image_msgs);

    // std::cout << "IMU msgs size is: " << imu_msgs.size() << std::endl;
    // std::cout << "Image msgs size is: " << image_msgs.size() << std::endl;

    while (image_msgs.size() > 0) {
      image_msg &img_msg = image_msgs.front();
      memcpy(left_image.data, img_msg.left_image, 640 * 480 * sizeof(u8));
      memcpy(right_image.data, img_msg.right_image, 640 * 480 * sizeof(u8));
      image_msgs.pop();
      imshow("left image", left_image);
      imshow("right image", right_image);
      cvWaitKey(1);
    }

    // cout << "Consum image size:" << image_msgs.size() << endl;

    // if (image_msgs.size() >0)
    // {
    //     imshow("left image", image_msgs[0].left_image);
    //     imshow("right image", image_msgs[0].right_image);
    //     cvWaitKey(1);
    // }

    usleep(30000);
  }

  return 0;
}
