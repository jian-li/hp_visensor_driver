#binocular_driver
###安装环境配置,安装opencv,libusb
```
sudo apt-get install cmake # 安装cmake
sudo apt-get install libusb-dev #libusb支持
sudo apt-get install libopencv-dev #opencv的支持
```

###idVendor和idProduct
插拔双目相机,在终端输入dmesg,找到Product为DUAL-CAM GLOBAL M9V024的usb设备对应的idVendor和idProduct.i

###编译
```
mkdir build
cd build
cmake ..
make -j4
```