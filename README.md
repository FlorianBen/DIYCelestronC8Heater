# DIYCelestronC8Heater (WIP)

## Disclaimer and read first

DIY Heater controller for the Celestron C8 telescope.

The main goal of this project is to provide a cheap solution for reducing the dew on my Celestron C8 tube.

The bill of material:

- Arduino Uno or Nano: 10€
- Case for the Arduino board: 5€
- Resistive wire: 5€
- Multipins connectors and cables, button, rotary encoder, OLED/LCD screen: less than 15€

This project is a homemade attempt to create a heater controller and it is primitive compare to a commercial off the shelf solution. Do not if you are not confident with what you are doing. Overheating your telescope may damage your telescope. **THEREFORE, THE PRESENT RESOURCES ARE PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY**

## 3D model

An example of . The parts were designed and assembled with FreeCAD.

## Arduino code

The Arduino code uses two libraries:

- [Vrekrer_scpi_parser](https://github.com/Vrekrer/Vrekrer_scpi_parser) to parse the incoming SCPI commands.

Be sure that both of these dependencies are installed in the Arduino library folder.

A state machine drives the motor while the serial interface are read between loop iteration. 
The command are sent in ASCII format through the Arduino serial interface, usually `/dev/ttyAMC0`. The table below tabulates each available commands:

<!-- Command                   | Description
--------------------------|-------------------------------------
*IDN?                     | Ask for the identification string
FOCus:STEPper:ACcel float | Set the acceleration of the stepper
FOCus:STEPper:ACcel?      | Ask for the acceleration
FOCus:STEPper:RPMspeed    | Set the maximum speed of the stepper
FOCus:STEPper:RPMspeed?   | Ask for the maximum speed
FOCus:STEPper:Go int      | Move according to the given step.
FOCus:STEPper:GOto int    | Move to the given position.
FOCus:STEPper:Go?         | Ask if the stepper is running.
FOCus:STEPper:ABort       | Abort motion.
FOCus:STEPper:POSition?   | Ask for the current position.
FOCus:STEPper:RAZPOSition | Reset the position to zero.
FOCus:STEPper:MAXpos int  | Set the maximum position.
FOCus:STEPper:MAXpos?     | Ask for the maximum position. -->


<!-- ## Indilib

An INDI driver is avaible for the focuser. The patch is avaible in the indi folder. First download INDI source files from here [INDI](https://github.com/indilib/indi) and apply the patch:

```sh
cd indi
git am --signoff -k < 0001-Support-for-an-poor-man-s-DIY-focuser-for-Celestron.patch
```

Then indi can be build as described in the [INDI Readme](https://github.com/FlorianBen/indi#building) -->