# IIS3DWB
**Arduino sketches for ST's vibration analyzer accelerometer**

From the [data sheet]( https://www.st.com/resource/en/datasheet/iis3dwb.pdf):
"The IIS3DWB is a system-in-package featuring a 3-axis digital vibration sensor with low noise over an ultra-wide and 
flat frequency range. The wide bandwidth, low noise, very stable and repeatable sensitivity, together with the capability 
of operating over an extended temperature range (up to +105 °C), make the device particularly suitable for vibration 
monitoring in industrial applications."

The flat frequency range is 0.1 Hz to 6300 Hz (-3 dB bandwidth) with a 26667 Hz sample rate. The device has a large FIFO that can hold up to  3072 bytes of data, enough for 512 accelerometer data samples.

![breakout](https://user-images.githubusercontent.com/6698410/84727817-c078ca80-af44-11ea-98da-ecece12c1a06.jpg)
*IIS3DWB breakout board sitting atop a Dragonfly STM32L476 development board via female machine pin headers.*
 
The sketch is intended to be run on a [Dragonfly](https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/) development board at 80 MHz. The sensor uses the SPI interface for full functionality and fast data transfer. Maximum SPI clock is 10 MHz, which the Dragonfly can support at its 80 MHz CPU clock speed.

The basic sketch shows how to configure the sensor, including setting up a data ready interrupt on INT1 and an activity recognition (i.e., wakeup) interrupt on INT2, how to read the data, how to power down and power up the sensor, etc.

Second sketch shows how to set up the FIFO, collect accel data in the FIFO, set up the FIFO watermark and FIFO-full (watermark) interrupt, and then batch read the FIFO into a buffer for further processing on the host. Further processing includes FFT and power spectrum peak identification, etc.

The SPI read of the 512 data samples from the FIFO takes about 13.1 ms and the fft for the full 512 data sample FIFO takes about 800 us at 80 MHz STM32L4 clock speed. The FIFO interframe time is ~20 ms (512 data samples/26667 Hz ~ 19.2 ms), so this is a rather longer read/processing time than we would like but still works. The SPI FIFO read time dominates here, and on the latest L4 core this time is reduced to ~8.6 ms through a number of improvements in the system layer.

Next steps are to test the IIS3DWB against a known vibrational source for accuracy and, after that, add a PDM mic for an overlapping frequency range sensitivity between 100 Hz and 40 kHz. The challenge will be to maintain full data and fft throughput on both data streams without collisions.

Breakout board [design](https://oshpark.com/shared_projects/KyNfc7rT) is open source in the shared space at OSH Park.

FFT analysis code modifications from CMSIS library were done by Greg Tomasch.

L4 system layer and Arduino core designed by Thomas Roell.

Copyright for this work is owned by Tlera Corporation (2020) for use by anyone with proper attribution.
 

