/* For Seeeduino Xiao w/RTC w/BT
* Purpose: Data collection code for two sensors
*           and outputs data to SD card and/or Bluetooth
* Author: Joe Ma (MASK Team)
*/
#include <spo2_algorithm.h>
#include <RTCZero.h> //for rtc
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
const int wakeTime = 1; //In minutes
const int sleepTime = wakeTime * 2;
const int btconnectPin = 1;
const int btdisconnectPin = 2;


uint32_t irBuffer[bufferLength]; //infrared LED sensor data
uint32_t redBuffer[bufferLength];

int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

int32_t tempspo2 = 0; //SPO2 value - temp
int32_t tempheartRate = 0; //heart rate value - temp
int32_t tempTemperature = 0; //heart rate value - temp

char strToPrint[50]; //String to hold data to print
RTCZero rtc;
bool rtcSleep = false; 
bool btCon = false;
bool firstSleep = true;

void PrintData(void);
void PrintDataBluetooth(void);
void PrintDataUSB(void);
void standbyone(void); //sleep function
void btCNISR(void);//Bt connect
void btDSISR(void);//Bt disconnect
void rtcSleepISR(void);//Sleep to wake transition
void rtcWakeISR(void); //Wake to sleep transition
static void configGCLK6(void);


void setup() {

  //Saves roughly 4mA total
  PM->CPUSEL.reg |= PM_CPUSEL_CPUDIV_DIV2; //48MHz to 24MHz
  PM->APBCMASK.reg &= ~PM_APBCMASK_ADC; //Shuts off ADC 
  
  bluetooth.begin(9600);
  usb.begin(9600);

  
  //Tries to start SD card
  for(int i = 0; i < MAX_SD_RETRY; i++){
    if(SD.begin(CHIP_SELECT)){
      SD_FOUND = true;
      break;
    }
  }

  //If there is an SD card, it will run. Otherwise this will crash the system
  if(SD_FOUND){
    logFile = SD.open(F("LOG.TXT"), FILE_WRITE);
    logFile.println(F("NEW START"));
    logFile.close();
  }

  //Begins sleep
  rtc.begin(true); //Resets clock
  rtc.setAlarmMinutes(sleepTime); //Sets alarm to sleepTime min
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(rtcSleepISR);
  rtcSleep = true;
  
  //Creates external interrupts for Bluetooth
  pinMode(btconnectPin, INPUT_PULLUP);
  attachInterrupt(btconnectPin, btCNISR, RISING );
  pinMode(btdisconnectPin, INPUT_PULLUP);
  attachInterrupt(btdisconnectPin, btDSISR, FALLING );

  //Latches EIC to a generic clock - makes it work in sleep
  configGCLK6();
  //Makes one of the interrupts from bluetooth to wake up the device
  EIC->WAKEUP.reg |= (1 << g_APinDescription[btconnectPin].ulExtInt);
 
  //Sets up sensors
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
  if(firstSleep){ //Enters first sleep cycle
    firstSleep = false;
    standbyone();
  }

  //Checks if either it is scheduled or bluetooth start
  if(!rtcSleep || btCon){
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
        sprintf(strToPrint, "Temp: %d \n HR: %d \n SPO2: %d \n", tempTemperature, tempheartRate, tempspo2);
        
    }else{ //Prints old data with new temp
        tempTemperature = tempSensor.readObjectTempF();
        sprintf(strToPrint, "Temp: %d \n HR: %d \n SPO2: %d \n", tempTemperature, tempheartRate, tempspo2);
        
    }
    
    if(SD_FOUND){
      logFile = SD.open(F("LOG.TXT"), FILE_WRITE);
      PrintData();
      logFile.close();
    }

    if(btCon){ //Prints to bluetooth if bluetooth is connected
      PrintDataBluetooth();
    }
    
    if(rtcSleep && !btCon){//Sleeps if timer ends or bluetooth disconnects
      standbyone();
    }
  }
}

// Prints data
void PrintData(void){
  logFile.print(strToPrint);
}

void PrintDataBluetooth(void){
  bluetooth.print(strToPrint);
}

void PrintDataUSB(void){
  usb.print(strToPrint);
}

void standbyone(void){ //Needs interrupt
  //From rocketscream/Low-Power with some tweaks
  spSensor.shutDown();
  SysTick->CTRL &= ~SysTick_CTRL_TICKINT_Msk;  
  SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
  __DSB();
  __WFI();
  __WFE();
  // Enable systick interrupt
  SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;  
}

void btCNISR(void){//Kills timer when bluetooth is connected
  spSensor.wakeUp();
  btCon = true;
  rtcSleep = false;
  rtc.disableAlarm();
  rtc.detachInterrupt();
}

void btDSISR(void){//changes state so Bluetooth is disconnected
  btCon = false;
  rtcWakeISR();
}

void rtcSleepISR(void){//If sleep, now woke
  //Being awake stops the sleep timer
  //and starts the wake timer
  spSensor.wakeUp();
  rtcSleep = false;
  rtc.disableAlarm();
  rtc.detachInterrupt();
  rtc.begin(true); //Resets time

  rtc.setAlarmMinutes(wakeTime);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(rtcWakeISR);
  
}

void rtcWakeISR(void){//If woke, now sleep
  rtcSleep = true;
  rtc.disableAlarm();
  rtc.detachInterrupt();
  rtc.begin(true); //Resets time

  rtc.setAlarmMinutes(sleepTime);
  rtc.enableAlarm(rtc.MATCH_MMSS);
  rtc.attachInterrupt(rtcSleepISR);
 
}

//Straight from
//https://github.com/arduino-libraries/ArduinoLowPower
//This code allows EIC to run off of a generic clock
//due to the fact that sleep will shut off all clocks except generics.
static void configGCLK6(){
  // enable EIC clock
  GCLK->CLKCTRL.bit.CLKEN = 0; //disable GCLK module
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID( GCM_EIC )) ;  //EIC clock switched on GCLK6
  while (GCLK->STATUS.bit.SYNCBUSY);

  GCLK->GENCTRL.reg = (GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_ID(6));  //source for GCLK6 is OSCULP32K
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);

  GCLK->GENCTRL.bit.RUNSTDBY = 1;  //GCLK6 run standby
  while (GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY);
}
