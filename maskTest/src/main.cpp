#include <Arduino.h>
#include <Wire.h> //For I2C
#include <SD.h>
#include <SPI.h> //SD CARD
#include <math.h> 
#include <MAX30105.h>
#include <Adafruit_MLX90614.h>

#define bluetooth Serial1 //Define bluetooth to serial1
#define usb Serial //usb to serial


void setup() {
  bluetooth.begin(9600);
  usb.begin(9600);
}

void loop() {

  if(bluetooth.available()){ //Read and write from USB and Bluetooth
    usb.write(bluetooth.read());
  }

  if(usb.available()){
    bluetooth.write(usb.read());
  }
}