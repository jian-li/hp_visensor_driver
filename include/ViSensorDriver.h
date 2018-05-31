#ifndef VISENSOR_DRIVER_H
#define VISENSOR_DRIVER_H
#include <boost/concept_check.hpp>
#include <libusb-1.0/libusb.h>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <thread>

using namespace cv;
using namespace std;

#define IMAGE_PART 10
#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480

#define FRAMELEN 614600
#define BUFFER_SIZE 1229560
#define PACKETSIZE 614400

#define IMAGE_BUFFER_SIZE 10
#define IMU_BUFFER_SIZE 100

#define GValue 9.805

typedef unsigned char u8;

struct imu_msg {
  double timestamp;

  double acc_x, acc_y, acc_z;
  double gyro_x, gyro_y, gyro_z;
};

struct image_msg {
  double timestamp;
  u8 left_image[IMAGE_WIDTH * IMAGE_HEIGHT];
  u8 right_image[IMAGE_WIDTH * IMAGE_HEIGHT];
};

union TsUnionType {
  unsigned char char_value[4];
  unsigned int int_value;
};

/**
 * @brief Camera Driver for maker binocular
 * Stereo images and 3 axis accelemeters and 3 axis gyroscope data are
 * transfered
 *
 */
class ViSensorDriver {
public:
  ViSensorDriver();
  ~ViSensorDriver();

  void init();

  void init_usb_device();

  bool get_frame(std::queue<imu_msg>& out_imus, std::queue<image_msg> &out_images);

  void synchronize(int transfered_num);

  void get_imu_data(float acc[3], float gyro[3]);

  void parse_frame(const u8 *data_ptr, const int &data_size);

  void produce();

private:

  /// libusb device pointer
  libusb_device **devs;
  libusb_context *contex = NULL;
  struct libusb_device_descriptor desc;
  struct libusb_config_descriptor *config;
  libusb_device *device;
  libusb_device_handle *dev_handle;
  u8 bulk_ep_in;

  u8 buffer1[BUFFER_SIZE], buffer2[BUFFER_SIZE],
      receive_buffer[BUFFER_SIZE]; // dual buffer

  int buffer1_len, buffer2_len;
  bool use_buffer1;

  // hardware initialize
  bool synchronized;

  // initial time stamp, unit is us
  int curr_img_ts, last_img_ts;
  double initial_ts;
  double current_ts;

  std::queue<imu_msg> imu_queue;
  std::mutex imu_mutex;

  std::queue<image_msg> img_queue;
  std::mutex img_mutex;

  std::thread producer_thread;
};

#endif
