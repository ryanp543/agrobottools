# Agrobot Tools ROS Package

## Overview

This repository contains a ROS package with all of the nodes needed to run the tools developed for my Master's thesis in the MIT Bioinstrumentation Lab. While operating the tools requires ROS, the provided scripts in the `scripts` directory should at least demonstrate how one would communicate with the tools over the RS232 Bricklet-TOSLink PCB interface. 

In the `scripts\tool_specific_scripts` directory, I've added the firmware or files that are specifically uploaded to some of the tools. The impedance probe file is uploaded to the Teensy LC within the impedance probe tool via the Arduino IDE. The NIR Java Application is converted to a .jar file, uploaded to the Raspberry Pi Zero mounted to the NeoSpectra sensor, and then used to create a .sh file that can be run from the terminal. The Thermal Camera Open MV file is the Python file that is uploaded to the OpenMV Cam H7 carrier board that comes with MicroPython installed. 

The impedance probe and thermal camera "firmware" files generally just take commands from the main control box and then return the collected data. The Java application for the NIR spectroscopy tool is essentially a standalone application. To run it from the Intel NUC mounted on the rover, you'll have to SSH into the Raspberry Pi zero, as described in the `nir_base.py` file. 

## Contributors

#### Primary Contributors
Ryan Poon from the [MIT Bioinstrumentation Lab](https://bioinstrumentation.mit.edu/) is the primary contributor of this project.

#### Other Contributors
Professor Ian Hunter served as the main advisor.



