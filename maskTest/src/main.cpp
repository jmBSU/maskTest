//For Seeeduino Xiao
#include <spo2_algorithm.h>

#include <Arduino.h>
#include <Wire.h> //For I2C
#include <SD.h>
#include <SPI.h> //SD CARD
#include <MAX30105.h>
#include <Adafruit_MLX90614.h>

#define bluetooth Serial1 //Define bluetooth to serial1
#define usb Serial //usb to serial

const int CHIP_SELECT = 3;
const int MAX_SD_RETRY = 5;

bool SD_FOUND = false;
File logFile;
Adafruit_MLX90614 tempSensor = Adafruit_MLX90614();
MAX30105 spSensor;

const int32_t bufferLength = 100; //data length
const int quarterSize = floor(bufferLength/4.0); //Double to int calc
const int threeQuarterSize = floor(quarterSize * 3.0);

uint32_t irBuffer[bufferLength]; //infrared LED sensor data
uint32_t redBuffer[bufferLength];

int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

int32_t tempspo2 = 0; //SPO2 value - temp
int32_t tempheartRate = 0; //heart rate value - temp
int32_t tempTemperature = 0; //heart rate value - temp


void PrintData(void);
void PrintDataBluetooth(void);
void PrintDataUSB(void);


void setup() {
  bluetooth.begin(9600);
  //usb.begin(9600);

  
  //Tries to start SD card
  for(int i = 0; i < MAX_SD_RETRY; i++){
    if(SD.begin(CHIP_SELECT)){
      SD_FOUND = true;
      break;
    }
  }

  //TODO: Will probs change prefix with dynamic number via EEPROM
  //TODO: Needs char* and not string

  if(SD_FOUND){
    logFile = SD.open(F("LOG.TXT"), FILE_WRITE);
    logFile.println(F("NEW START"));
    logFile.close();
  }
  

  tempSensor.begin();
  spSensor.begin();
  spSensor.setup(60,4,2,100,411,4096);
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    redBuffer[i] = spSensor.getRed();
    irBuffer[i] = spSensor.getIR();
    spSensor.nextSample(); //We're finished with this sample so move to next sample

  }
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation( irBuffer, bufferLength,redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

}

void loop() {

//  if(bluetooth.available()){ //Read and write from USB and Bluetooth
//    usb.write(bluetooth.read());
//  }

    //dumping the first quarter of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = quarterSize; i < bufferLength; i++)
    {
      redBuffer[i - quarterSize] = redBuffer[i];
      irBuffer[i - quarterSize] = irBuffer[i];
    }
    //take some sets of samples before calculating the heart rate.
    for (byte i = threeQuarterSize; i < bufferLength; i++)
    {
      redBuffer[i] = spSensor.getRed();
      irBuffer[i] = spSensor.getIR();
      spSensor.nextSample(); //We're finished with this sample so move to next sample
    }
    //After gathering new samples recalculate HR and SP02
  maxim_heart_rate_and_oxygen_saturation( irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

  if(validSPO2 && validHeartRate){ //Prints new data when valid data
      tempspo2 = spo2;
      tempheartRate = heartRate;
      tempTemperature = tempSensor.readObjectTempF();

      if(SD_FOUND){
        logFile = SD.open(F("LOG.TXT"), FILE_WRITE);
        PrintData();
        logFile.close();
      }
      

      PrintDataBluetooth();

  }else{ //Prints old data with new temp
      tempTemperature = tempSensor.readObjectTempF();
//
      if(SD_FOUND){
        logFile = SD.open(F("LOG.TXT"), FILE_WRITE);
        PrintData();
        logFile.close();
      }

      PrintDataBluetooth();
  }


//  if(usb.available()){
//    bluetooth.write(usb.read());
//  }


}

//Will not open nor close
// Prints data
// logFile - File class
void PrintData(void){
  logFile.print(F("SPO2:"));
  logFile.print(tempspo2);
  logFile.println();
  logFile.print(F("HR: "));
  logFile.print(tempheartRate);
  logFile.println();
  logFile.print(F("TEMP:"));
  logFile.print(tempTemperature);
  logFile.println();
}

void PrintDataBluetooth(void){
  bluetooth.print(F("SPO2:"));
  bluetooth.print(tempspo2);
  bluetooth.println();
  bluetooth.print(F("HR: "));
  bluetooth.print(tempheartRate);
  bluetooth.println();
  bluetooth.print(F("TEMP:"));
  bluetooth.print(tempTemperature);
  bluetooth.println();
  
}

void PrintDataUSB(void){
  usb.print(F("SPO2:"));
  usb.print(tempspo2);
  usb.println();
  usb.print(F("HR: "));
  usb.print(tempheartRate);
  usb.println();
  usb.print(F("TEMP:"));
  usb.print(tempTemperature);
  usb.println();
}
