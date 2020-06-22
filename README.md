# IIS3DWB
**Arduino sketches for ST's vibration analyzer accelerometer**

From the [data sheet]( https://www.st.com/resource/en/datasheet/iis3dwb.pdf):
"The IIS3DWB is a system-in-package featuring a 3-axis digital vibration sensor with low noise over an ultra-wide and 
flat frequency range. The wide bandwidth, low noise, very stable and repeatable sensitivity, together with the capability 
of operating over an extended temperature range (up to +105 °C), make the device particularly suitable for vibration 
monitoring in industrial applications."

The flat frequency range is 0 Hz to 6300 Hz (-3 dB bandwidth) with a 26667 Hz sample rate. The device has a large FIFO that can hold up to  3072 bytes of data, enough for 512 accelerometer data samples.

![breakout](https://user-images.githubusercontent.com/6698410/84727817-c078ca80-af44-11ea-98da-ecece12c1a06.jpg)
*IIS3DWB breakout board sitting atop a Dragonfly STM32L476 development board via female machine pin headers.*
 
The sketch is intended to be run on a [Dragonfly](https://www.tindie.com/products/tleracorp/dragonfly-stm32l47696-development-board/) development board at 80 MHz. The sensor uses the SPI interface for full functionality and fast data transfer. Maximum SPI clock is 10 MHz, which the Dragonfly can support at its 80 MHz CPU clock speed.

The basic sketch shows how to configure the sensor, including setting up a data ready interrupt on INT1 and an activity recognition (i.e., wakeup) interrupt on INT2, how to read the data, how to power down and power up the sensor, etc.

Second sketch shows how to set up the FIFO, collect accel data in the FIFO, set up the FIFO watermark and FIFO-full (watermark) interrupt, and then batch read the FIFO into a buffer for further processing on the host. Further processing includes FFT and power spectrum peak identification, etc.

We are reading the entire FIFO in one operation and subsequently constructing the data bytes rather than doing this all sequentially. Full (512 data samples) FIFO buffer read time is ~3.1 ms. Theoretical read time should be (512 samples x 7 bytes/sample x 8 bits/byte)/10 MHz = 2.87 ms so we are above 93% efficiency with > 15 ms of headroom.

Next steps are to test the IIS3DWB against a known vibrational source for accuracy and, after that, add a PDM mic for an overlapping frequency range sensitivity between 100 Hz and 40 kHz. The challenge will be to maintain full data and fft throughput on both data streams without collisions.

We tested the IIS3DWB against a hand-held Wahl Trimmer device. The IIS3DWB was mounted on top of the Dragonfly as shown above, and the Dragonfly was held by hand against the Wahl Trimmer. The 2048-bin FFT spectrum with the HPF filter set at 33 Hz (-3dB, ODR/800, maximum bandwidth setting) is shown below for both trimmer on and off:

![first_test results](https://user-images.githubusercontent.com/6698410/85237780-33b78c00-b3de-11ea-9d6c-0c1fe6a3c966.jpg)

The Wahl trimmer specs list the cutting frequency at 14400 cuts per minute, which we take to mean 7200 (two cuts per) motor revolutions per minute or 120 Hz vibration frequency. The dominant peak is at 3 Hz (first, DC bin) for reasons we do not understand when using an HPF. The next-highest-intensity frequency is at 114 Hz (3% of the integrated intensity) with harmonics at 230, 341, and 686 Hz. There are somewhat weaker peaks at 58.5 Hz (60 Hz AC), 286 Hz and 796 Hz. The spectrum continues to tail off at higher frequencies (not shown). The zero vibration spectrum is essentially zero (orange line) on this scale. The total FFT time for 2048 bins is 4.0 ms, 5x (expected from Nlog2(N) dependence of FFT [operations](https://blog.endaq.com/vibration-analysis-fft-psd-and-spectrogram) on bin size) the 800 us for 512 bins.  

Breakout board [design](https://oshpark.com/shared_projects/KyNfc7rT) is open source in the shared space at OSH Park.

FFT analysis code modifications from CMSIS library were done by Greg Tomasch.

L4 system layer and Arduino core designed (and SPI FIFO read strategy) by Thomas Roell.

Copyright for this work is owned by Tlera Corporation (2020) for use by anyone with proper attribution.
 

