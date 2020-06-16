/*
 * Copyright (c) 2020 Tlera Corp.  All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal with the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimers.
 *  2. Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimers in the
 *     documentation and/or other materials provided with the distribution.
 *  3. Neither the name of Tlera Corp, nor the names of its contributors
 *     may be used to endorse or promote products derived from this Software
 *     without specific prior written permission.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * CONTRIBUTORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * WITH THE SOFTWARE.
 * 
 * The IIS3DWB is a system-in-package featuring a 3-axis digital vibration sensor with low noise over an ultra-wide and 
 * flat frequency range. The wide bandwidth, low noise, very stable and repeatable sensitivity, together with the capability 
 * of operating over an extended temperature range (up to +105 °C), make the device particularly suitable for vibration 
 * monitoring in industrial applications.
 * 
 * Uses the SPI inmterface for fast data transfer.
 * 
 * Intended to be run on a Dragonfly development board:
 * 
 * https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
 *  
 * 
 * Library may be used freely and without limit with attribution.
 */
#include "IIS3DWB.h"
#include <RTC.h>
#include "SPI.h"

#define SerialDebug true  // set to true to get Serial output for debugging
#define myLed    38
#define CSPIN    10
#define GROUND   31 // Use Dragonfly GPIO to power sensor breakout board
#define POWER    30 // Use Dragonfly GPIO to power sensor breakout board

const char        *build_date = __DATE__;   // 11 characters MMM DD YYYY
const char        *build_time = __TIME__;   // 8 characters HH:MM:SS

//IIS3DWB definitions
#define IIS3DWB_intPin1 8  // interrupt1 pin definitions, data ready
#define IIS3DWB_intPin2 9  // interrupt2 pin definitions, activity detection

/* Specify sensor parameters (sample rate is same as the bandwidth 6.3 kHz by default)
 * choices are:  AFS_2G, AFS_4G, AFS_8G, AFS_16G  
*/ 
uint8_t Ascale = AFS_2G;

float aRes;                              // scale resolutions per LSB for the accel 
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel 
int16_t IIS3DWBData[4];                  // Stores the 16-bit signed sensor output
float ax, ay, az, accelTemp;             // variables to hold latest accel data values 
uint8_t IIS3DWBstatus;

volatile bool IIS3DWB_DataReady = false, IIS3DWB_Wakeup = false;

IIS3DWB IIS3DWB(CSPIN); // instantiate IIS3DWB class


// RTC set time using STM32L4 native RTC class
/* Change these values to set the current initial time */
uint8_t seconds = 0, minutes = 15, hours = 17, day = 15, month = 6, year = 20;
uint8_t Seconds, Minutes, Hours, Day, Month, Year;

volatile bool alarmFlag = false; // for RTC alarm interrupt


void setup() 
{
  Serial.begin(115200);
  Serial.blockOnOverrun(false);
  delay(4000);

  SPI.begin(); // Start SPI serial peripheral

  // Configure led
  pinMode(myLed, OUTPUT);
  digitalWrite(myLed, HIGH); // start with led off
  
  // Set up power pins to power sensor breakout
  pinMode(GROUND, OUTPUT);
  digitalWrite(GROUND, LOW);
  pinMode(POWER, OUTPUT);
  digitalWrite(POWER, HIGH);
  delay(20); // wait at least 10 ms for IIS3DWB boot procedure to complete

  // Configure SPI ship select for sensor breakout
  pinMode(CSPIN, OUTPUT);
  pinMode(CSPIN, HIGH); // disable SPI at start
 
  // Configure interrupt pins
  pinMode(IIS3DWB_intPin1, INPUT); // enable IIS3DWB interrupt1
  pinMode(IIS3DWB_intPin2, INPUT); // enable IIS3DWB interrupt2

  // Read the IIS3DWB Chip ID register, this is a good test of communication
  Serial.println("IIS3DWB accel...");
  uint8_t c = IIS3DWB.getChipID();  // Read CHIP_ID register for IIS3DWB
  Serial.print("IIS3DWB "); Serial.print("I AM "); Serial.print(c, HEX); Serial.print(" I should be "); Serial.println(0x7B, HEX);
  Serial.println(" ");

  if(c == 0x7B) // check if all I2C sensors have acknowledged
  {
   Serial.println("IIS3DWB is online...");  
   Serial.println(" ");

   // reset IIS3DWB to start fresh
   IIS3DWB.reset();
    
   digitalWrite(myLed, LOW); // indicate passed the ID check

   // get accel sensor resolution, only need to do this once
   aRes = IIS3DWB.getAres(Ascale);

   IIS3DWB.selfTest();

   IIS3DWB.init(Ascale); // configure IIS3DWB  

   IIS3DWB.offsetBias(accelBias);
   Serial.println("accel biases (mg)"); Serial.println(1000.0f * accelBias[0]); Serial.println(1000.0f * accelBias[1]); Serial.println(1000.0f * accelBias[2]);
   Serial.println(" ");
   delay(1000); 
   
   digitalWrite(myLed, HIGH); // turn off led when sensor configuration is finished
  }
  else 
  {
  if(c != 0x6A) Serial.println(" IIS3DWB not functioning!"); 
  while(1){};
  }

 // Set the time
  SetDefaultRTC();
  
  /* Set up the RTC alarm interrupt */
  RTC.enableAlarm(RTC.MATCH_ANY); // alarm once a second
  
  RTC.attachInterrupt(alarmMatch); // interrupt every time the alarm sounds

  attachInterrupt(IIS3DWB_intPin1, myinthandler1, RISING);   // define interrupt for intPin1 output of IIS3DWB
  attachInterrupt(IIS3DWB_intPin2, myinthandler2, FALLING);  // define interrupt for intPin2 output of IIS3DWB
}

/* End of setup */

void loop() {

     if(IIS3DWB_DataReady)  // Handle data ready condition
     {
       IIS3DWB_DataReady = false;
      
//       IIS3DWBstatus = IIS3DWB.DRstatus(); // read data ready status
//       if (IIS3DWBstatus & 0x01) {         // if new accel data is available, read it

         IIS3DWB.readAccelData(IIS3DWBData);  
   
         // Now we'll calculate the accleration value into actual g's
         ax = (float)IIS3DWBData[0]*aRes - accelBias[0];  // get actual g value, this depends on scale being set
         ay = (float)IIS3DWBData[1]*aRes - accelBias[1];   
         az = (float)IIS3DWBData[2]*aRes - accelBias[2];  
//       }
     }  // end of data ready interrupt handling
     

      if (IIS3DWB_Wakeup) { // if activity change event FALLING detected
          IIS3DWB_Wakeup = false;
        
          Serial.println("IIS3DWB is awake");
     } // end activity change interrupt handling 
     
   // end sensor interrupt handling


    if(alarmFlag) { // update RTC output (serial display) whenever the RTC alarm condition is achieved  
       alarmFlag = false;
     
    // Read RTC
   if(SerialDebug)
    {
    Serial.println("RTC:");
    Day = RTC.getDay();
    Month = RTC.getMonth();
    Year = RTC.getYear();
    Seconds = RTC.getSeconds();
    Minutes = RTC.getMinutes();
    Hours   = RTC.getHours();     
    if(Hours < 10) {Serial.print("0"); Serial.print(Hours);} else Serial.print(Hours);
    Serial.print(":"); 
    if(Minutes < 10) {Serial.print("0"); Serial.print(Minutes);} else Serial.print(Minutes); 
    Serial.print(":"); 
    if(Seconds < 10) {Serial.print("0"); Serial.println(Seconds);} else Serial.println(Seconds);  

    Serial.print(Month); Serial.print("/"); Serial.print(Day); Serial.print("/"); Serial.println(Year);
    Serial.println(" ");
    }
    
    if(SerialDebug) { // report latest accel data
    Serial.print("ax = "); Serial.print((int)1000*ax);  
    Serial.print(" ay = "); Serial.print((int)1000*ay); 
    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
    }

    IIS3DWBData[3] = IIS3DWB.readTempData(); // get IIS3DWB chip temperature
    accelTemp = ((float) IIS3DWBData[3]) / 256.0f + 25.0f; // Accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    if(SerialDebug) {
      Serial.print("IIS3DWB temperature is ");  Serial.print(accelTemp, 1);  Serial.println(" degrees C"); // Print T values to tenths of a degree C
    }

    digitalWrite(myLed, LOW); delay(10); digitalWrite(myLed, HIGH);   // toggle the led
    }

    STM32.sleep(); // sleep while waiting for an interrupt
}
/*  End of main loop */


void myinthandler1()
{
  IIS3DWB_DataReady = true;
}


void myinthandler2()
{
  IIS3DWB_Wakeup = true;
}


void alarmMatch()
{
  alarmFlag = true;
}


void SetDefaultRTC()  // Sets the RTC to the FW build date-time...
{
  char Build_mo[3];

  // Convert month string to integer

  Build_mo[0] = build_date[0];
  Build_mo[1] = build_date[1];
  Build_mo[2] = build_date[2];

  String build_mo = Build_mo;

  if(build_mo == "Jan")
  {
    month = 1;
  } else if(build_mo == "Feb")
  {
    month = 2;
  } else if(build_mo == "Mar")
  {
    month = 3;
  } else if(build_mo == "Apr")
  {
    month = 4;
  } else if(build_mo == "May")
  {
    month = 5;
  } else if(build_mo == "Jun")
  {
    month = 6;
  } else if(build_mo == "Jul")
  {
    month = 7;
  } else if(build_mo == "Aug")
  {
    month = 8;
  } else if(build_mo == "Sep")
  {
    month = 9;
  } else if(build_mo == "Oct")
  {
    month = 10;
  } else if(build_mo == "Nov")
  {
    month = 11;
  } else if(build_mo == "Dec")
  {
    month = 12;
  } else
  {
    month = 1;     // Default to January if something goes wrong...
  }

  // Convert ASCII strings to integers
  day     = (build_date[4] - 48)*10 + build_date[5] - 48;  // ASCII "0" = 48
  year    = (build_date[9] - 48)*10 + build_date[10] - 48;
  hours   = (build_time[0] - 48)*10 + build_time[1] - 48;
  minutes = (build_time[3] - 48)*10 + build_time[4] - 48;
  seconds = (build_time[6] - 48)*10 + build_time[7] - 48;

  // Set the date/time

  RTC.setDay(day);
  RTC.setMonth(month);
  RTC.setYear(year);
  RTC.setHours(hours);
  RTC.setMinutes(minutes);
  RTC.setSeconds(seconds);
}
  
