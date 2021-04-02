# MASK Code
Repo for MASK project


![License: MIT](https://img.shields.io/badge/License-MIT-blue.svg)
![Build Status](https://github.com/jmBSU/maskTest/actions/workflows/main.yml/badge.svg)

~~This code is written for the Arduino Pro Micro and its derivatives. This probably works with the Arduino Leonardo as well due to the similarity of their schematics.
Anyways we don't expect it to work outside the Pro Micro. [Pro Micro Pinout](https://cdn.sparkfun.com/assets/7/6/c/a/c/ProMicro8MHzv2.pdf)~~

Due to memory constraints, this project has migrated to the [Seeeduino Xiao](https://wiki.seeedstudio.com/Seeeduino-XIAO/).



The code is licensed to cover our included libraries.

Libraries include:
* Standard Arduino
* MAX30105 - [Library Link](https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library)
* Adafruit MLX90614 - [Library Link](https://github.com/adafruit/Adafruit-MLX90614-Library)

---
## Building/ Coding via Platformio (Native)

* Should be able to clone and compile
* Platformio should be able to recognize the project

---
## Building/ Coding in Arduino IDE

* Comment out Arduino.h
     * Arduino IDE might not recognize it and fail to compile.
* Add MAX30105 and MLX90614 libraries to Arduino IDE

It should be able to compile now.

---
## Contributing

This style guide will be used: [Style Guide](https://drive.google.com/file/d/1PkwYtBZ81AC5tD2T1TDWFzXf3yGa9XaX/view?usp=sharing)

* Create a new branch
* Code
* Open new pull request
* Merge when reviewed

---
## References

* https://learn.adafruit.com/adafruit-micro-sd-breakout-board-card-tutorial/arduino-library
* https://create.arduino.cc/projecthub/SurtrTech/measure-heart-rate-and-spo2-with-max30102-c2b4d8
* https://learn.adafruit.com/using-melexis-mlx90614-non-contact-sensors/wiring-and-test
* https://makersportal.com/blog/2017/9/20/hm-10-bluetooth-module
* https://electronics.stackexchange.com/questions/412525/when-i-write-data-to-a-ble-characteristic-ffe1-to-send-data-over-bluetooth-is
* https://www.arduino.cc/en/Reference/Libraries
* https://arduino.stackexchange.com/questions/1471/arduino-pro-micro-get-data-out-of-tx-pin
