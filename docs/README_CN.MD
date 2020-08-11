## **[English](../README.MD) | 中文**

# 使用Arduino IDE 
1. 安装正确的串口驱动程序，默认出货为[CH340](http://www.wch-ic.com/search?q=ch340&t=downloads)
1. 将`src/main.cpp` 更改为`src.ino`
2. 将lib目录内文件拷贝到`~/Arduino/libraries`,windows 用户拷贝到 `Documents/Arduino/libraries`
3. 双击打开`src/src.ino`
4. 更改端口为正确的端口，然后选择上传

# 使用PlatformIO
1. 安装正确的串口驱动程序，默认出货为[CH340](http://www.wch-ic.com/search?q=ch340&t=downloads)
1. 直接打开在`platformio.ini`中更改你的串口端口，点击编译即可

- 注意：使用TFT_eSPI需要选择正确的型号，需要选择`Setup26_TTGO_T_Wristband.h`，它与T_Wristband拥有相同的定义,如果拷贝的是lib文件夹中的，则可以忽略

## Datasheet
- [MPU6050 Sensor](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [ST7735](http://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf)
