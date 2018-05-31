#include <chrono>
#include <iostream>
#include <libusb-1.0/libusb.h>
#include <stdio.h>

#include "ViSensorDriver.h"
#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <signal.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/image_encodings.h>

typedef std::chrono::high_resolution_clock Clock;

sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
void mySigIntHandler(int sig) { g_request_shutdown = 1; }

int main(int argc, char **argv) {

  ros::init(argc, argv, "ViSensorDriver", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;

  ros::Publisher left_image_pub =
      nh.advertise<sensor_msgs::Image>("camera/left/image_raw", 10);
  ros::Publisher right_image_pub =
      nh.advertise<sensor_msgs::Image>("camera/right/image_raw", 10);

  ros::Publisher imu_msg_pub = nh.advertise<sensor_msgs::Imu>("/imu", 100);

  cv::Mat left_image(480, 640, CV_8UC1, cv::Scalar(0));
  cv::Mat right_image(480, 640, CV_8UC1, cv::Scalar(0));

  double init_timestamp = -1, current_timestamp = -1;
  double ts_from_start = -1;
  ros::Time init_ros_ts;

  queue<image_msg> image_msgs;
  queue<imu_msg> imu_msgs;

  int receive_image_num = 0;

  ViSensorDriver visensor_driver;

  while (1) {
    if (g_request_shutdown == 1) {
      visensor_driver.shutdown();
      break;
    }

    visensor_driver.get_frame(imu_msgs, image_msgs);

    // printf("get frame\r\n");
    // publish imu data
    if (init_timestamp > 0) {
        // printf("IMU Buffer size: %d\r\n", imu_msgs.size());
      while (imu_msgs.size() > 0) {
        imu_msg &tmp_imu = imu_msgs.front();
        sensor_msgs::Imu imu_ros_msg;
        imu_ros_msg.header.stamp =
            init_ros_ts + ros::Duration(1e-4 * (tmp_imu.timestamp - init_timestamp));
        imu_ros_msg.linear_acceleration.x = tmp_imu.acc_x;
        imu_ros_msg.linear_acceleration.y = tmp_imu.acc_y;
        imu_ros_msg.linear_acceleration.z = tmp_imu.acc_z;
        imu_ros_msg.angular_velocity.x = tmp_imu.gyro_x;
        imu_ros_msg.angular_velocity.y = tmp_imu.gyro_y;
        imu_ros_msg.angular_velocity.z = tmp_imu.gyro_z;
        imu_msg_pub.publish(imu_ros_msg);

        imu_msgs.pop();
      }
    }

    // publish image data
    while (image_msgs.size() > 0) {
        //  printf("Image Buffer size: %d\r\n", image_msgs.size());
      image_msg &img_msg = image_msgs.front();
      memcpy(left_image.data, img_msg.left_image, 640 * 480 * sizeof(u8));
      memcpy(right_image.data, img_msg.right_image, 640 * 480 * sizeof(u8));

      if (init_timestamp == -1) {
        init_ros_ts = ros::Time::now();
        init_timestamp = img_msg.timestamp;
      }

      //   imshow("left image", left_image);
      //   imshow("right image", right_image);
      //   cvWaitKey(1);

      // publish image message
      cv_bridge::CvImage img_bridge;
      sensor_msgs::Image ros_left_image, ros_right_image;
      std_msgs::Header image_header;
    //   printf("pass time: %8.f \r\n", 1e-4 * (img_msg.timestamp - init_timestamp));
      image_header.stamp =
          init_ros_ts + ros::Duration((img_msg.timestamp - init_timestamp)*1e-4);

      // left image
      img_bridge = cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::MONO8, left_image);
      img_bridge.toImageMsg(ros_left_image);
      left_image_pub.publish(ros_left_image);

      // right image
      img_bridge = cv_bridge::CvImage(
          image_header, sensor_msgs::image_encodings::MONO8, right_image);
      img_bridge.toImageMsg(ros_right_image);
      right_image_pub.publish(ros_right_image);

      image_msgs.pop();
    }

    usleep(50000);

    // ros::spinOnce();
  }

  return 0;
}
