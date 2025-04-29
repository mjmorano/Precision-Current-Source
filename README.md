# Overview
This is all of the design files and the code needed for my precision current source. The code for the microcontroller was made using the Arduino framework and PlatformIO so it's easy to modify and can be uploaded without neededing a programmer, just an Arduino. The serial interface features a few basic commands for setting the current and reading it back using the onboard ADC, as well as reading the internal temperature sensors. I added a lot of error checking to the firmware so it should be basically impossible to make the device do something that it shouldn't, and give the user feedback if their inputs are accepted or not. This design uses an external voltage source up to 100 volts, so it can be used with relatively high resistance loads. It is also suitable for high inductance loads, and has been verified stable into 250 mH. The KiCad folder contains all of the schematics and the .zip files with the gerbers needed to have the PCBs and the front panel manufactured. I have the front panels made as aluminum PCBs with no copper layers, which pretty much any manufacturer can do.

<p align="center">
  <img src=https://github.com/user-attachments/assets/c5d25695-98c0-4fc6-a0a8-a178284d2bb7 />
  <img src=https://github.com/user-attachments/assets/159fc946-f35d-427a-823e-23750d6b0d00 width="45%" height="250px" />
  <img src=https://github.com/user-attachments/assets/4047fc3a-19de-4ba4-a13d-b69d08dfd4a1 width="45%" height="250px" />
</p>

# Stability Measurement
Since this is meant to be used for driving gradient and shim coils for long experimental runs, temperature stability is critical. To measure the stability I rented a Keysight 3458A and logged data for a week at maximum output current and the result can be seen in the plot below. Allan deviations for this data are also shown. The current stability could be further improved by using a different voltage reference, the LTZ1000A or LM399 for example, but these would come with a significant accuracy penalty. Of course the accuracy can be calibrated in software, but that requires the user to have a very good DMM and makes the assembly process more complicated. I tried to balance initial accuracy and temperature stability as much as possible.

<p align="center">
  <img src=https://github.com/user-attachments/assets/0c841162-d0d3-4aff-b783-4f881fc1e3cc />
  <img src=https://github.com/user-attachments/assets/ae9102fe-52ee-4cb9-8865-a3c8f8de0e02 />
</p>

# Flashing the Firmware
To upload the program to the microcontroller, the Arduino bootloader needs to be burned. It's possible to buy the microcontroller with the bootloader already installed, but it's easy to do yourself. The best way to do this is in the Arduino IDE, and I'll outline how to do it with either an Uno or a Nano. First the ArduinoISP needs to be uploaded to Arduino and can be found under the example sketches as shown below.

<p align="center">
 <img src=https://github.com/user-attachments/assets/20c8d19b-1148-4c20-8540-05499a6f8e7e />
</p>

Once it's uploaded connect the Arduino pins to the 6 pin header on the current source as follows:


  | Arduino | Current Source |
  |:-------:|:--------------:|
  |    10   |    RESET       |
  |    11   |    MOSI        |
  |    12   |    MISO        |
  |    13   |    SCK         |
  |    5V   |    5V          |
  |   GND   |    GND         |


Next under tools set the programmer to be "Arduino as ISP", then click burn bootloader. If everything is working right a message should pop up in the bottom right corner of the Arduino IDE saying "Burning bootloader...", followed by a message saying "Done burning bootloader."
<p align="center">
  <img src=https://github.com/user-attachments/assets/8915f8ca-b316-46a5-80d0-961a422a1a06 />
  <img src=https://github.com/user-attachments/assets/d5e70085-ea2d-4b31-bed8-474886d6a417 />
  <img src=https://github.com/user-attachments/assets/261dac4a-f4db-41c6-ae22-c2dcec10fc9d />
</p>

Now that the bootloader is burned open the Code/Microcontroller folder from the repo in VS Code, making sure to have the PlatformIO extension installed. After PlatformIO initializes select the port for the Arduino that was used to burn the bootloader in the bottom left, for me it was COM5.
<p align="center">
  <Img src=https://github.com/user-attachments/assets/bcefc186-d4b8-4972-8e65-b901564c9881 />
</p>

After this all that needs to be done is to click the upload button and the terminal should look like the below image.
<p align="center">
  <Img src=https://github.com/user-attachments/assets/629b103b-d0d7-43b1-95bc-183c9badbdb2 />
  <Img src=https://github.com/user-attachments/assets/536357de-a0ee-4a5f-8675-916f626b76a4 />
</p>

Now the device is ready to be connected over USB to a computer.


# Serial Interface
When the device is connected to the computer it should show up as a COM port in windows and a ttyUSB device on Linux. On Linux the device will need to be chmodded in order to connect to it using pyserial. Usually the FTDI drivers automatically download or are already installed, but if not they can be downloaded [here](https://ftdichip.com/drivers/vcp-drivers/). The interface uses eight data bits, no parity, and one stop bit at 115200 baud. On powerup the device does some self checks and looks to see if the optional tmp117 temperature sensor is present. After connecting to the serial interface two readlines need to be done, the first says whether the tmp117 is detected or not, and the second is for any detected errors. If the device detects no errors the second readline will return "Ready." Examples of how to use pyserial with the device are shown in the Code/Serial_Interface_Examples.py file. There are some other commands not shown there, but they won't really be used.
