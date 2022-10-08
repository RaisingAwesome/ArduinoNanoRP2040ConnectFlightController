# Arduino Nano RP2040 Connect Flight Controller
## Introduction
The Arduino Nano RP2040 Connect comes with a built in IMU. It's fast processing speed and ability to program it with the Arduino IDE makes for a great microcontroller to serve as a quadcopter (drone) flight controller.

The code in the repository is MIT licensed open source.  Do whatever you like with it. If you'd like to simply contribute to this source, perform a pull request and get your 'git' on.  We'd appreciate it.

We have designed a printed circuit board (PCB) that goes along with this code.  It allows hookups to beefy motors and their ESCs.  You can find it here: 
[Arduino Nano RP2040 Connect Flight Controller PCB](https://www.pcbway.com/project/shareproject/First_Opensource_Arduino_Nano_RP2040_Connect_Drone_Flight_Controller_4f90bf4e.html)

What's special about this project is that at the time of writing this article, there are no search results via Google, Bing, or Yahoo that can find a published opensource code for the Arduino Nano RP2040 Connect for flight control. This makes this project the first published open-source version!
## Attribution
We got our starter code from nickrehm's repository dRehmFlight.  It was great code IMU focused code for the Teensy that we ported for our hardware and additional features such as proximity sensors and GPS.  You can find his repository here: [dRehmFlight](https://github.com/nickrehm/dRehmFlight)
## Hardware
These are the components we are using for our drone:

- Microcontroller:  Arduino Nano RP2040 Connect
- Motors: QWinOut A2212 1000KV Brushless Outrunner Motor 13T
- ESCs: QWinOut 2-4S 30A/40A RC Brushless ESC Simonk Firmware Electric Speed Controller with 5V 3A BEC
- Battery: HRB 4S 3300mAh 14.8v Lipo RC Battery 60C

This hardware works with the code out of the box.  The drone frame is up to the builder.
## PCB Design Considerations

The biggest design consideration for this board was the logic level. The RP2040 GPIO is designed for 3.3V and not 5V tolerant. However, the ESCs and the radio receiver communicate on 5V.  So, any pin to the RP2040 that was receiving a signal required a voltage divider to protect the RP2040.

The second consideration was battery management. Many of the LiPo batteries available are long lasting with a ton of amps to deliver quickly, but must not be allowed to go under their minimum cell voltage. We balooned a LiPo once in the house.  It's terrifying!

In turn, a means of letting the drone pilot know the battery is draining too low had to be designed.  We achieved this by applying a voltage divider here as well. In our case, we had a 14.8V divided to just under 3Vs. The closer the battery approaches undervoltage (12.8V), the beeper will increase its beep rate until it is too annoying to want to continue use.

Otherwise, the rest of the design was just focused on providing reliable wire hookups between the drone receiver and motor electronic speed controllers (ESCs).
## Code Considerations
As mentioned, we started with the dRehmFlight repository as our base code.  We altered many of the functions to work with our hardware.  At this time, we are awating our PCB to tune the PID Constants.
