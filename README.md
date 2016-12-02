#binocular_driver
###安装环境配置,安装opencv,libusb
```
sudo apt-get install build-essential #install gcc, g++
sudo apt-get install libboost-dev
sudo apt-get install cmake # 安装cmake
sudo apt-get install libusb-1.0-0-dev #libusb支持
sudo apt-get install libopencv-dev #opencv的支持
```

###idVendor和idProduc
插拔双目相机,在终端输入
```
dmesg
```
找到Product为DUAL-CAM GLOBAL M9V024的usb设备对应的idVendor和idProduct
```
13472.009858] usb 3-2: new high-speed USB device number 24 using xhci_hcd
[13472.146367] usb 3-2: New USB device found, idVendor=04b4, idProduct=1005
[13472.146370] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[13472.146372] usb 3-2: Product: DUAL-CAM GLOBAL MT9V024 
[13472.146373] usb 3-2: Manufacturer: CypTest
```


###编译
```
mkdir build
cd build
cmake ..
make -j4
```
###权限设置
如果不设置rules,那么用普通用户启动应用程序的时候,会出现Pemission Denied.解决方法为在`/etc/udev/rules.d`目录中创建rules.例如
cypress.rules,内容如下（idVendor和idProduct在上面用dmesg得到的信息里面）:
```
 ATTRS{idVendor}=="04b4", ATTRS{idProduct}=="1234", MODE="0666"
```
重启即可.

###IMU模块
Maker binocular集成了mpu6050 IMU模块,加速度计的量程为2g,陀螺仪的量程为2000deg/s.

###固件版本
Maker binocular的固件暂时有两种，旧的版本的固件IMU采集频率为30Hz，新的版本频率为120Hz，新旧版本的固件图像采集不一样。如果采集的图像只有下面1/4有图像，那么需要切换到120HzIMU采集频率的版本，分支为`highimufreq`。Master分支的IMU采集频率为30Hz，图像为一次传输一整幅图像。
