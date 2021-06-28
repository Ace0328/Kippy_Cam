# Kipp-o-mat

## Libraries
The project uses modified version of NEXTION library downloaded from the official site. Only 2 files are modified:
1. NexConfig.h - disabled debug output for Arduino Nano\Uno boards
2. NextHardware.cpp - changed baud rate to 115200 and decrease delay within NexLoop() function to make the motor runs smoothly

## Building
Before the building, all necessary librabries should be include:
1. In the Arduino IDE call Sketch-Include library-Add .ZIP Library...
2. Select ITEADLIB_Arduino_Nextion folder and add it. Do the same for madleech-Button library.

The IDE will copy these libs into its own lib folder.

Finally, just compile the sketch.

## Configurations
All configurations are done within conf.h file.