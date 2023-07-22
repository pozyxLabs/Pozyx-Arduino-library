# Pozyx-Arduino-library
An Arduino library in C to work with the [Pozyx Creator kit](https://www.pozyx.io/creator). 
The [Pozyx](https://www.pozyx.io) creator kit has everything required to achieve accurate indoor positioning for a hobby project and consists out of an Arduino-compatible shield equiped with the Qorvo DW1000 UWB chip, as well as several stand-alone reference anchors.

## Prerequisites
Please download a release version for a safely stable experience!

The library requires **firmware version 1.1** installed on the Pozyx devices.
If you're on an older version of the firmware and do not want to upgrade for some reason, please download an older release of the library according to their respective release description.

## Documentation and examples
You can find the Arduino tutorials on our documentation site: [https://docs.pozyx.io/creator/latest/arduino.](https://docs.pozyx.io/creator/arduino)

Documentation for the Arduino library can be found here: https://ardupozyx.readthedocs.io.

If you encounter any issues, please send a mail to support@pozyx.io instead of creating an issue here.

The following folders can be found together with this library:

1.  **examples**. These example scripts showcase some basic functionality of the Pozyx device, each example comes with a tutorial that can be found on the pozyx website https://docs.pozyx.io/creator/arduino
2.  **unit_test**. This folder contains a collection of Arduino scripts that can be run to test certain functionalities of the Pozyx device. They also serve as some good examples for some Arduino library functions.
3.  **useful**. This folder contains a number of useful Arduino sketches that provide basic functionality such as discovering all pozyx devices or configuring them.
