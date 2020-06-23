/*
 * Copyright (c) 2020 Tlera Corp.  All rights reserved.
 *
 *  Created by Kris Winer
 *  FFT analysis augmentation by Greg Tomasch
 * 
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
 * Uses the SPI interface for fast data transfer.
 * 
 * Intended to be run on a Dragonfly development board:
 * 
 * https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/
 *  
 * 
 * Library may be used freely and without limit with attribution.
 */
#include <arm_const_structs.h>
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

// FFT definitions
#define SHOW_FFT_DATA                                                                                      // Uncomment to print FFT PSD data on the serial monitor
//#define NORMALIZE PSD                                                                                      // Uncomment to print PSD normalized to the largest bin

#define FIFO_WATERMARK       256

#define FFT_LENGTH          2048                                                                          // FFT length must be a power of 2! Allowed values are: 16, 32, 64, 128, 256, 512 
#define SCAN_AVG_COUNT       1                                                                          // Number of FFT "Scans" to average per PSD report; set to "1" generate report every "FFT_LENGTH" raw data points

// Analyze Ax, Ay, Az or Arms; uncomment one only
//#define ANALYZE_AX
//#define ANALYZE_AY
//#define ANALYZE_AZ
#define ANALYZE_RMS

uint32_t                     start_fft;                                                                    // Time immediately before calling FFT analysis
uint32_t                     finish_fft;                                                                   // Time immediately after completing FFT analysis
float32_t                    fft_io_buffer[2*FFT_LENGTH];                                                  // Complex FFT input/output buffer
float32_t                    fft_output_buffer[FFT_LENGTH];                                                // Real magnitude output buffer
float32_t                    psd_f32[FFT_LENGTH];                                                          // Scan-averaged complex magnitude PSD output buffer
const arm_cfft_instance_f32* fft_Instance;                                                                 // CMSIS f32 CFFT instance pointer for the selected FFT length
uint16_t                     fft_sample_count = 0;                                                         // Index to count the number of input samples in the fft I/O buffer                                                 
float                        freq_bin_width   = 26667.0f/2.0f;                                             // FFT Frequency bin width in Hz
float                        psd_max          = 0.0f;                                                      // PSD maximum for spectrum normalization
volatile uint8_t             fft_data_ready   = 0;                                                         // Flag for when FFT raw data buffer is full and ready for analysis
uint8_t                      psd_avg_count    = 0;                                                         // FFT scan counter for scan averaging
uint32_t                     m_ifft_flag      = 0;                                                         // Flag that selects forward (0) or inverse (1) CFFT transform.
uint32_t                     m_do_bit_reverse = 1;                                                         // Flag that enables (1) or disables (0) bit reversal of output. Bit reversal true i normal


//IIS3DWB definitions
#define IIS3DWB_intPin1 8  // interrupt1 pin definitions, data ready
#define IIS3DWB_intPin2 9  // interrupt2 pin definitions, activity detection

/* Specify sensor parameters (sample rate is same as the bandwidth 6.3 kHz by default)
 * choices are:  AFS_2G, AFS_4G, AFS_8G, AFS_16G  
*/ 
uint8_t Ascale = AFS_4G;
/*
 *FIFO modes include: Bypassmode(FIFO disabled), FIFOmode (stops collecting when FIFO full),
 * Cont_to_FIFOmode(Continuous mode until trigger deasserted, the FIFO mode),
 * Bypass_to_Contmode (Bypass  mode until trigger deasserted, the Continuous mode),
 * Contmode (If FIFO full, new sample overwrites older one),
 * Bypass_to_FIFOmode (Bypass mode until trigger deasserted, the FIFO mode).
 * 
 * FIFO size from 1 to 512 data samples.
 */
uint16_t fifo_count;                                               // FIFO buffer size variable 
int16_t AxyzData[FIFO_WATERMARK][3];                               // create FIFO data array
float Ax[FIFO_WATERMARK], Ay[FIFO_WATERMARK], Az[FIFO_WATERMARK];  // create accel data buffers
uint8_t fifo_mode = Contmode;
uint32_t start_dataRead, stop_dataRead;

float aRes;                              // scale resolutions per LSB for the accel 
float accelBias[3] = {0.0f, 0.0f, 0.0f}; // offset biases for the accel 
int16_t IIS3DWBData[4] = {0};            // Stores the 16-bit signed sensor output
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

  if(c == 0x7B) // check if all SPI sensors have acknowledged
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

   IIS3DWB.initFIFO(FIFO_WATERMARK, fifo_mode); // use FIFO to collect data
   
   digitalWrite(myLed, HIGH); // turn off led when sensor configuration is finished
  }
  else 
  {
  if(c != 0x6A) Serial.println(" IIS3DWB not functioning!"); 
  while(1){};
  }

  // Configure the FFT
    switch(FFT_LENGTH)                                                                                       // Define the appropriate CMSIS f32 CFFT instance pointer for the defined FFT length
  {
    case 16:
      fft_Instance = &arm_cfft_sR_f32_len16;
      break;
    case 32:
      fft_Instance = &arm_cfft_sR_f32_len32;
      break;
    case 64:
      fft_Instance = &arm_cfft_sR_f32_len64;
      break;
    case 128:
      fft_Instance = &arm_cfft_sR_f32_len128;
      break;
    case 256:
      fft_Instance = &arm_cfft_sR_f32_len256;
      break;
    case 512:
      fft_Instance = &arm_cfft_sR_f32_len512;
      break;
    case 1024:
      fft_Instance = &arm_cfft_sR_f32_len1024;
      break;
    case 2048:
      fft_Instance = &arm_cfft_sR_f32_len2048;
      break;
    case 4096:
      fft_Instance = &arm_cfft_sR_f32_len4096;
      break;
    default:
      Serial.println("Incorrect FFT length specified!");
      while(1) {;}
      break;
  }

  freq_bin_width /= (float)FFT_LENGTH;
  memset(fft_io_buffer, 0x00, sizeof(fft_io_buffer));                                                      // Initialize the fft I/O buffer
  

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

    if(IIS3DWB_DataReady)  // FIFO threshold condition
    {
       IIS3DWB_DataReady = false;
      
       fifo_count = IIS3DWB.FIFOstatus();        // read FIFO status
       if( fifo_count == FIFO_WATERMARK ) {      // if FIFO count valid

         start_dataRead = micros();                                                                                  // Diagnostic to track FFT calculation time; comment out when done
         IIS3DWB.readFIFOData(fifo_count, &AxyzData[0][0]);
         stop_dataRead = micros();                                                                                  // Diagnostic to track FFT calculation time; comment out when done
         Serial.print("SPI data read time: ");                                                                       // Diagnostic to track FFT calculation time; comment out when done
         Serial.print((stop_dataRead - start_dataRead)); Serial.print(" "); Serial.print("us");                       // Diagnostic to track FFT calculation time; comment out when done
         Serial.print(", FIFO Count: "); Serial.println(fifo_count);

         for (uint16_t ii = 0; ii < fifo_count; ii++)
         {
          Ax[ii] = (float)AxyzData[ii][0]*aRes - accelBias[0]; // properly scale data in terms of gs
          Ay[ii] = (float)AxyzData[ii][1]*aRes - accelBias[1];
          Az[ii] = (float)AxyzData[ii][2]*aRes - accelBias[2];

          // Now we'll load the new acceleration data into the FFT I/O buffer
        #if defined(ANALYZE_AX)
          fft_io_buffer[2*fft_sample_count+2*ii] = Ax[ii];
        #elif defined(ANALYZE_AY)
          fft_io_buffer[2*fft_sample_count+2*ii] = Ay[ii];
        #elif defined(ANALYZE_AZ)
          fft_io_buffer[2*fft_sample_count+2*ii] = Az[ii];
        #elif defined(ANALYZE_RMS)
          fft_io_buffer[2*fft_sample_count+2*ii] = sqrtf(Ax[ii]*Ax[ii] + Ay[ii]*Ay[ii] + Az[ii]*Az[ii]);
        #else
          fft_io_buffer[2*fft_sample_count+2*ii] = Ax[ii];                                                              // Default to ax in case the analysis definition is botched
        #endif
          fft_io_buffer[2*fft_sample_count+2*ii+1] =0.0f;                                                                   // Pad the imaginary component with 0.0f
          }

          fft_sample_count += fifo_count;
          if(fft_sample_count == FFT_LENGTH)
          {
            fft_sample_count = 0;
            fft_data_ready   = 1;
          }
       }
     }  // end of data ready interrupt handling


    if(fft_data_ready)                                                                                       // If the I/O buffer has a full FFT sample in it, build the FFT input buffer and calculate the FFT
  {
    fft_data_ready = 0;                                                                                    // Reset the FFT data ready flag
    start_fft = micros();                                                                                  // Diagnostic to track FFT calculation time; comment out when done
    fft_process(fft_io_buffer, fft_Instance, fft_output_buffer, FFT_LENGTH);                               // Calculate the PSD!
    finish_fft = micros();                                                                                 // Diagnostic to track FFT calculation time; comment out when done
    for(uint32_t i=0; i<FFT_LENGTH/2; i++)                                                                 // Add new PSD values to the avrages array element-by-element
    {
      psd_f32[i] += fft_output_buffer[i];
    }
    psd_avg_count++;                                                                                       // Increment the scan averaging counter
    Serial.print("FFT Calc time: ");                                                                       // Diagnostic to track FFT calculation time; comment out when done
    Serial.print((finish_fft - start_fft)); Serial.print(" "); Serial.println("us");                       // Diagnostic to track FFT calculation time; comment out when done
  }


  // Check if the defined number of FFT calculations have been averaged
  if(psd_avg_count >= SCAN_AVG_COUNT)                                                                      // If the requested number of PSD samples have been summed, calculate averages and report the PSD over serial
  {
    psd_avg_count = 0;                                                                                     // Reset the scan averaging counter flag
    psd_max       = 0.0f;
    for(uint32_t i=0; i<FFT_LENGTH/2; i++)                                                                 // The number of PSD bins is half the FFT length
    {
      psd_f32[i] /= (float)SCAN_AVG_COUNT;                                                                 // Divide PSD sum arrray element by the number of scans
      psd_f32[i] *= 2.0f;                                                                                  // Multiply by 2; symmetrical about the Nyquist frequency
      if(psd_f32[i] > psd_max) {psd_max = psd_f32[i];}                                                     // Track maximum amplitude bin as we go...
    }
    for(uint32_t i=0; i<FFT_LENGTH/2; i++)                                                                 // The number of PSD bins is half the FFT length
    {
      #ifdef SHOW_FFT_DATA
        Serial.print((float)i*freq_bin_width + freq_bin_width/2.0f);                                       // Calculate the center frequency of each PSD bin
        #ifdef NORMALIZE PSD
          Serial.print(","); Serial.println(psd_f32[i]/psd_max, 5);                                        // Print PSD data normalized to the largest bin
        #else
          Serial.print(","); Serial.println(psd_f32[i]);                                                   // Print raw PSD data
        #endif
      #endif
      psd_f32[i] = 0.0f;                                                                                   // Clear PSD sum array element after you print it so the psd_f32[] will be cleared and ready for the next sample
    }
    #ifdef SHOW_FFT_DATA
      Serial.println("");
    #endif
  }
     

      if (IIS3DWB_Wakeup) { // if activity change event FALLING detected
          IIS3DWB_Wakeup = false;
        
      if(SerialDebug) Serial.println("IIS3DWB is awake");
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
    
//    if(SerialDebug) { // report latest accel data
//    for(uint16_t ii = 0; ii < FIFO_WATERMARK; ii++)
//    {
//      Serial.print("Ax sample "); Serial.print(ii); Serial.print(" = "); Serial.print((int)1000*Ax[ii]);  
//      Serial.print(" Ay sample "); Serial.print(ii); Serial.print(" = "); Serial.print((int)1000*Ay[ii]);  
//      Serial.print("Az sample "); Serial.print(ii); Serial.print(" = "); Serial.print((int)1000*Az[ii]); Serial.println(" mg");
//    }
//    Serial.print("ax = "); Serial.print((int)1000*ax);  
//    Serial.print(" ay = "); Serial.print((int)1000*ay); 
//    Serial.print(" az = "); Serial.print((int)1000*az); Serial.println(" mg");
//    }

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


/**
* @fn: fft_process(float32_t * p_input, const arm_cfft_instance_f32 * p_input_struct, float32_t * p_output, uint16_t output_size)
*
* @brief: CMSIS Complex FFT evaluation function
* @params: Input data buffer pointer, arm_cfft_instance_f32 instance pointer,
*          output buffer pointer, output data buffer size
* @returns:
*/
void fft_process(float32_t * p_input, const arm_cfft_instance_f32 * p_input_struct,                    // Calculate complex FFT and complex magnitude using sp float CMSIS DSP library calls
                 float32_t * p_output, uint16_t output_size)
{
    arm_cfft_f32(p_input_struct, p_input, m_ifft_flag, m_do_bit_reverse);                              // Use 32bit float CFFT module to process the data
    arm_cmplx_mag_f32(p_input, p_output, output_size);                                                 // Calculate the magnitude at each bin using Complex Magnitude Module function
}

  
