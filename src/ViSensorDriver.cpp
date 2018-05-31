#include "ViSensorDriver.h"
#include <iostream>
#include <stdio.h>
#include <thread>

#include <glog/logging.h>

using namespace std;

ViSensorDriver::ViSensorDriver()
    : synchronized(false), use_buffer1(true), last_img_ts(0), last_imu_ts(0) {
  init();
}

ViSensorDriver::~ViSensorDriver() {}

void ViSensorDriver::init() {

  init_usb_device();

  // multi thread
  producer_thread = thread(&ViSensorDriver::produce, this);
}

void ViSensorDriver::shutdown() {
  libusb_free_device_list(devs, 1);
  libusb_close(dev_handle);

  producer_thread.join();
}

void ViSensorDriver::init_usb_device() {

  int err = 0;

  if (libusb_init(&contex) < 0) {
    LOG(ERROR) << "Init error!" << std::endl;
  }

  // set verbosity level to 3
  libusb_set_debug(contex, 3);

  ssize_t cnt = libusb_get_device_list(contex, &devs);

  if (cnt < 0) {
    LOG(ERROR) << "Get devices error" << std::endl;
  } else {
    LOG(INFO) << "Get " << cnt << " devices in total" << std::endl;
  }

  // found cypress usb device
  bool idVendorAndProductfound = false;
  for (int i = 0; i < cnt; i++) {
    device = devs[i];
    err = libusb_get_device_descriptor(device, &desc);

    if (err < 0) {
      LOG(ERROR) << "failed to get desc" << std::endl;
      return;
    }

    if (desc.idVendor == 0x2014 && desc.idProduct == 0x0117) {
      LOG(INFO) << "============================================" << std::endl;
      printf("Found cypress usb device: idVendor 0x%04x idProduct: 0x%04x\r\n",
             desc.idVendor, desc.idProduct);
      LOG(INFO) << "============================================" << std::endl;
      idVendorAndProductfound = true;
      break;
    }
  }

  if (idVendorAndProductfound == false) {
    LOG(INFO) << "============================================" << std::endl;
    LOG(INFO) << "Error: Can not found the device, please check the idVendor "
                 "and idProduct!"
              << std::endl;
    LOG(INFO) << "============================================" << std::endl;
    return;
  }
  // get cypress usb config desc
  libusb_get_config_descriptor(device, 0, &config);

  //

  if ((int)config->bNumInterfaces > 1)
    LOG(INFO) << "too many interfaces" << std::endl;

  const struct libusb_interface *inter;
  const struct libusb_interface_descriptor *interdesc;
  const struct libusb_endpoint_descriptor *epdesc;

  inter = &config->interface[0];
  interdesc = &inter->altsetting[0];

  if ((int)interdesc->bNumEndpoints > 2)
    LOG(INFO) << "too many endpoints" << std::endl;

  for (int j = 0; j < interdesc->bNumEndpoints; j++) {
    epdesc = &interdesc->endpoint[j];

    if ((epdesc->bEndpointAddress) & 0x80) {
      bulk_ep_in = epdesc->bEndpointAddress;
      printf("Hints: Get Built in endpoint: 0x%02x\n",
             epdesc->bEndpointAddress);
      printf("Max packetsize is %d \n", epdesc->wMaxPacketSize);
    }
  }

  err = libusb_open(device, &dev_handle);

  if (err < 0) {
    printf("open device failed\n");
    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);
    return;
  }

  if (libusb_kernel_driver_active(dev_handle, 0) == 1) {
    printf("Kernel Driver Active\n");
    if (libusb_detach_kernel_driver(dev_handle, 0) == 0)
      printf("Kernal Driver Detached\n");
  }

  err = libusb_claim_interface(dev_handle, 0);
  if (err < 0) {
    printf("can not claim interface");
    libusb_free_device_list(devs, 1);
    libusb_close(dev_handle);
    return;
  }
}

void ViSensorDriver::synchronize(int transfered_num) {
  // printf("Synchronize Data!!!!\r\n");
  int i = 0;
  for (i = 0; i < transfered_num; i++) {
    if ((i + 4 <= transfered_num) && ((receive_buffer[i]) == 0x33) &&
        ((receive_buffer[i + 1]) == 0xcc) &&
        ((receive_buffer[i + 14]) == 0x22) &&
        ((receive_buffer[i + 15]) == 0xdd))
      synchronized = true;
    else
      synchronized = false;

    // imu
    if (i + 200 <= transfered_num) {
      if (((receive_buffer[i + 16]) == 0x66) &&
          ((receive_buffer[i + 17]) == 0xdd) &&
          ((receive_buffer[i + 198]) == 0x44) &&
          ((receive_buffer[i + 199]) == 0xbb)) {
        synchronized = true;
        break;
      } else
        synchronized = false;
    } else {
      synchronized = false;
      break;
    }
  }

  // copy the synchronized data to buffer
  if (synchronized == true) {
    buffer1_len = 0;
    // printf("Data Sychronized!!!!\r\n");
    memcpy(buffer1, receive_buffer + i, transfered_num - i);
    buffer1_len += transfered_num - i;
    buffer2_len = 0;
    // printf("Buffer1 Size %d !!\r\n", buffer1_len);
  } else {
    buffer1_len = 0;
    buffer2_len = 0;
  }
}

// produce image message and imu message
void ViSensorDriver::produce() {

  bool buffer1_full = false, buffer2_full = false;
  while (true) {
    // printf("Produce Data!!!!\r\n");
    int transfered = 0;

    int error = libusb_bulk_transfer(dev_handle, bulk_ep_in, receive_buffer,
                                     PACKETSIZE, &transfered, 100);

    if (synchronized == false) {
      synchronize(transfered);
    } else {

      assert(buffer1_len + transfered < BUFFER_SIZE);
      assert(buffer2_len + transfered < BUFFER_SIZE);
      // printf("Parse Data!!!!\r\n");
      // printf("buffer1 size is: %d, buffer2 size is: %d\r\n", buffer1_len,
      //  buffer2_len);

      if ((buffer1_len == 0 && buffer2_len == 0) || buffer1_len != 0)
        use_buffer1 = true;
      else
        use_buffer1 = false;

      // printf("buffer1 size is: %d, buffer2 size is: %d\r\n", buffer1_len,
      // buffer2_len);

      // if the first buffer is used
      if (use_buffer1) {
        if (buffer1_len + transfered >= FRAMELEN) {
          buffer1_full = true;
          buffer2_full = false;
          memcpy(buffer1 + buffer1_len, receive_buffer, FRAMELEN - buffer1_len);
          buffer2_len = 0;
          memcpy(buffer2 + buffer2_len, receive_buffer + FRAMELEN - buffer1_len,
                 transfered - FRAMELEN + buffer1_len);
          buffer2_len += transfered - FRAMELEN + buffer1_len;

          buffer1_len += FRAMELEN - buffer1_len;
        } else {
          memcpy(buffer1 + buffer1_len, receive_buffer, transfered);
          buffer1_len += transfered;
          buffer1_full = false;
          buffer2_full = false;
        }
      } else {
        if (buffer2_len + transfered >= FRAMELEN) {
          buffer1_full = false;
          buffer2_full = true;

          memcpy(buffer2 + buffer2_len, receive_buffer, FRAMELEN - buffer2_len);

          buffer1_len = 0;
          memcpy(buffer1 + buffer1_len, receive_buffer + FRAMELEN - buffer2_len,
                 transfered - FRAMELEN + buffer2_len);
          buffer1_len += transfered - FRAMELEN + buffer2_len;

          buffer2_len += FRAMELEN - buffer2_len;
        } else {
          memcpy(buffer2 + buffer2_len, receive_buffer, transfered);
          buffer2_len += transfered;
          buffer1_full = false;
          buffer2_full = false;
        }
      }

      // printf("buffer1 size is: %d, buffer2 size is: %d\r\n", buffer1_len,
      //  buffer2_len);

      if (buffer1_full || buffer2_full) {
        unsigned char *buffer_ptr = NULL;
        if (buffer1_full) {
          // printf("use buffer1\r\n");
          buffer_ptr = buffer1;
        }

        if (buffer2_full) {
          // printf("use buffer2\r\n");
          buffer_ptr = buffer2;
        }

        // check synchronized state
        bool resynchronize = false;
        if ((buffer_ptr[0] != 0x33) || (buffer_ptr[1] != 0xcc) ||
            (buffer_ptr[14] != 0x22) || (buffer_ptr[15] != 0xdd))
          resynchronize = true;

        // if (resynchronize == false)
        //   printf("Frame Header Sychronized!!!!\r\n");
        // else
        //   printf("Frame Header Unsynchronized!!!\r\n");

        // check imu synchronized state
        int imu_i = 16;
        int idx = 0;

        // printf("IMU data header: %x %x\r\n", msg_data_temp_last[imu_i],
        // msg_data_temp_last[imu_i+1]); printf("IMU data tail: %x %x\r\n",
        // msg_data_temp_last[imu_i+182], msg_data_temp_last[imu_i+183]);

        if ((buffer_ptr[imu_i] != 0x66) || (buffer_ptr[imu_i + 1] != 0xdd) ||
            (buffer_ptr[imu_i + 182] != 0x44) ||
            (buffer_ptr[imu_i + 183] != 0xbb))
          resynchronize = true;

        if (resynchronize == true) {
          synchronized = false;
          // printf("Data Unsynchronized, resynchronize!!!\r\n");
        }
        // else
        //   printf("Data Sychronized, ready to show!!!!\r\n");

        if (synchronized == false)
          continue;

        // assert();
        // data frame second
        // printf("Show Captured Data!!!!\r\n");
        parse_frame(buffer_ptr, FRAMELEN);

        if (buffer1_full == true) {
          buffer1_full = false;
          buffer1_len = 0;
        } else {
          buffer2_full = false;
          buffer2_len = 0;
        }
      }
    }

    usleep(30000);
  }
  //
}

void ViSensorDriver::parse_frame(const u8 *data_ptr, const int &data_size) {
  TsUnionType img_ts_data;
  img_ts_data.char_value[3] = data_ptr[10];
  img_ts_data.char_value[2] = data_ptr[11];
  img_ts_data.char_value[1] = data_ptr[12];
  img_ts_data.char_value[0] = data_ptr[13];

  if (last_img_ts != 0) {
    if (img_ts_data.int_value - last_img_ts >= 1000) {
      cout << "error lost frame!!!! img_TimeStamp is " << img_ts_data.int_value
           << "   " << img_ts_data.int_value - last_img_ts << endl;
    } else
      // printf("image timestamp is: %.8f \r\n", (float)img_ts_data.int_value);
      printf("Delta IMG Time is: %.8f \r\n",
             1e-4 * (img_ts_data.int_value - last_img_ts));
  }

  last_img_ts = img_ts_data.int_value;

  int i = 18;
  imu_msg imu_msgs[10];

  // if ((data_ptr[16] != 0x66) || (data_ptr[16 + 1] != 0xdd) ||
  //     (data_ptr[16 + 182] != 0x44) || (data_ptr[16 + 183] != 0xbb))
  //   std::cout << "IMU data error" << std::endl;
  // else
  //   std::cout << "IMU data right" << std::endl;

  for (int j = 0; j < 10; j++) {

    TsUnionType imu_ts_data;

    imu_ts_data.char_value[3] = (data_ptr[i + 18 * j]);
    imu_ts_data.char_value[2] = (data_ptr[i + 1 + 18 * j]);
    imu_ts_data.char_value[1] = (data_ptr[i + 2 + 18 * j]);
    imu_ts_data.char_value[0] = (data_ptr[i + 3 + 18 * j]);

    if (last_imu_ts != 0) {
      if (imu_ts_data.int_value - last_imu_ts >= 70) {
        cout << "error lost imu frame!!!! imu_TimeStamp is "
             << imu_ts_data.int_value << "   "
             << imu_ts_data.int_value - last_imu_ts << endl;
      } else
        // printf("image timestamp is: %.8f \r\n",
        // (float)img_ts_data.int_value);
        printf("Delta IMU Time is: %.8f \r\n",
               1e-4 * (imu_ts_data.int_value - last_imu_ts));
    }

    last_imu_ts = imu_ts_data.int_value;

    imu_msgs[j].timestamp = imu_ts_data.int_value;

    imu_msgs[j].acc_x = (short)(((data_ptr[i + 18 * j + 4]) << 8) +
                                (data_ptr[i + 18 * j + 5])) /
                        8192.0 * GValue;
    imu_msgs[j].acc_y = (short)(((data_ptr[i + 18 * j + 6]) << 8) +
                                (data_ptr[i + 18 * j + 7])) /
                        8192.0 * GValue;
    imu_msgs[j].acc_z = (short)(((data_ptr[i + 18 * j + 8]) << 8) +
                                (data_ptr[i + 18 * j + 9])) /
                        8192.0 * GValue;

    imu_msgs[j].gyro_x = (short)(((data_ptr[i + 18 * j + 12]) << 8) +
                                 (data_ptr[i + 18 * j + 13])) *
                         3.14159 / 5904;
    imu_msgs[j].gyro_y = (short)(((data_ptr[i + 18 * j + 14]) << 8) +
                                 (data_ptr[i + 18 * j + 15])) *
                         3.14159 / 5904;
    imu_msgs[j].gyro_z = (short)(((data_ptr[i + 18 * j + 16]) << 8) +
                                 (data_ptr[i + 18 * j + 17])) *
                         3.14159 / 5904;

    // printf("imu timestamp is: %.8f \r\n", imu_msgs[j].timestamp);
    // cout << "imu timestamp " << imu_msgs[j].timestamp << std::endl;
    // printf("Acce Data is: %f %f %f\r\n", imu_msgs[j].acc_x,
    // imu_msgs[j].acc_y,
    //        imu_msgs[j].acc_z);
    // printf("Gyro Data is: %f %f %f\r\n", imu_msgs[j].gyro_x,
    // imu_msgs[j].gyro_y,
    //        imu_msgs[j].gyro_z);
  }

  i = 184 + 16; //图像数据的起始位置

  image_msg image_msgs;
  image_msgs.timestamp = img_ts_data.int_value;
  memcpy(image_msgs.right_image, data_ptr + i, 307200);
  memcpy(image_msgs.left_image, data_ptr + i + 307200, 307200);

  //
  {
    data_mutex.lock();

    if (img_queue.size() >= IMAGE_BUFFER_SIZE) {
      img_queue.pop();
    }

    img_queue.push(image_msgs);

    for (int i = 0; i < 10; i++) {
      if (imu_queue.size() >= IMU_BUFFER_SIZE) {
        imu_queue.pop();
      }

      imu_queue.push(imu_msgs[i]);
    }

    data_mutex.unlock();
  }
}

bool ViSensorDriver::get_frame(std::queue<imu_msg> &out_imus,
                               std::queue<image_msg> &out_images) {

  {
    data_mutex.lock();
    out_images = img_queue;
    img_queue = std::queue<image_msg>();

    out_imus = imu_queue;

    imu_queue = std::queue<imu_msg>();
    data_mutex.unlock();
  }
}
