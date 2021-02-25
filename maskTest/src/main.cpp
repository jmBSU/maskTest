#include <Arduino.h>
#include <Wire.h> //For I2C
#include <SD.h>
#include <SPI.h> //SD CARD
#include <math.h> 
#include <MAX30105.h>
#include <Adafruit_MLX90614.h>
//#include <EEPROM.h> to have incremental log number

#define bluetooth Serial1 //Define bluetooth to serial1
#define usb Serial //usb to serial

const int CHIP_SELECT = 10;
const int MAX_SD_RETRY = 5;

bool SD_FOUND = false;
File logFile;

void setup() {
  bluetooth.begin(9600);
  usb.begin(9600);

  //Tries to start SD card
  for(int i = 0; i < MAX_SD_RETRY; i++){
    if(!SD.begin(CHIP_SELECT)){
      delay(1000);
    }else{
      SD_FOUND = true;
      break;
    }
  }

  logFile = SD.open(strcpy("1_","log.txt"), FILE_WRITE); 
  //TODO: Will probs change prefix with dynamic number via EEPROM
  //TODO: Needs char* and not string

  logFile.close();

}

void loop() {

  if(bluetooth.available()){ //Read and write from USB and Bluetooth
    usb.write(bluetooth.read());
  }

  logFile = SD.open(strcpy("1_","log.txt"), FILE_WRITE);
  logFile.println("HELLO THERE");
  logFile.close();

  if(usb.available()){
    bluetooth.write(usb.read());
  }


}