# IIS3DWB
**Arduino sketches for ST's vibration analyzer accelerometer**

From the [data sheet]( https://www.st.com/resource/en/datasheet/iis3dwb.pdf):
"The IIS3DWB is a system-in-package featuring a 3-axis digital vibration sensor with low noise over an ultra-wide and 
flat frequency range. The wide bandwidth, low noise, very stable and repeatable sensitivity, together with the capability 
of operating over an extended temperature range (up to +105 °C), make the device particularly suitable for vibration 
monitoring in industrial applications."

The flat frequency range is ~50 Hz to 6300 Hz (maximum bandwidth) with a 26660 Hz sample rate. The device has a large FIFO that can hold up to  3072 bytes of data, enough for 512 accelerometer data samples.

![breakout](https://user-images.githubusercontent.com/6698410/84727817-c078ca80-af44-11ea-98da-ecece12c1a06.jpg)
*IIS3DWB breakout board sitting atop a Dragonfly STM32L476 development board via female machine pin headers.*
 
The sketch is intended to be run on a [Dragonfly](https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/) development board. The sensor uses the SPI interface for full functionality and fast data transfer. Maximum SPI clock is 10 MHz, which the Dragonfly can support at its 80 MHz CPU clock speed.

The basic sketch shows how to configure the sensor, including setting up a data ready interrupt on INT1 and an activity reognition (i.e., wakeup) interrupt on INT2, how to read the data, how to power down and power up the sensor, etc.

Additional sketches will show how to set up the FIFO, collect accel data in the FIFO, set up the FIFO watermark and FIFO-full (watermark) interrupt, and then batch read the FIFO into a buffer for further processing on the host. Further processing includes FFT and power spectrum peak identification, etc.

Breakout board [design](https://oshpark.com/shared_projects/KyNfc7rT) is open source in the shared space at OSH Park.
 

