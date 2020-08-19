## **English | [中文](docs/README_CN.MD)**

# Use Arduino IDE
1. Install the correct serial port driver, the default shipment is [CP210X Driver](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)
2. Change `src/main.cpp` to `src.ino`
3. Copy the files in the lib directory to `~/Arduino/libraries`,
Windows users copy to `Documents/Arduino/libraries`
1. Double-click to open `src/src.ino` 
2. Change the port to the correct port and select upload

# Use PlatformIO
1. Install the correct serial port driver, the default shipment is [CP210X Driver](https://www.silabs.com/products/development-tools/software/usb-to-uart-bridge-vcp-drivers)
2. Open directly to change your serial port in `platformio.ini`, click compile

- Note: To use `TFT_eSPI`, you need to select the correct model, you need to select `Setup26_TTGO_T_Wristband.h`, which has the same definition as `T-Wristband`, If the copy is in the lib folder, you can ignore it

# Examples Description
In the examples directory contains two sample programs, `FactoryTest` is the factory test file,
`HeartRateMeter` is a simple heart rate meter. You need to change the program. 
You only need to copy the files in the directory to the src directory. 
`Arduino IDE` needs to be renamed to `src.ino`, and `PlatformIO` can be opened and compiled directly



## Datasheet
- [MPU6050 Sensor](https://invensense.tdk.com/products/motion-tracking/6-axis/mpu-6050/)
- [ST7735](http://www.displayfuture.com/Display/datasheet/controller/ST7735.pdf)






