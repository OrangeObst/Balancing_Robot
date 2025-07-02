# Self-Balancing Robot Project

-------------------------------------
## Overview

This project showcases a self-balancing robot designed to demonstrate stability and control using two PID controllers. The first one maintains balance and the second one tries to return the robot to its starting point.

<!-- ## Key Features
----------------

* **Autonomous Balancing**: Utilizes a MPU6050 to measure the robot's tilt angle and adjust the motors to maintain balance.
* **Adjustable Control Algorithm**: Experiment with [PID, LQR, etc.] controllers for optimal performance
* **Modular Design**: Easily integrate new sensors or actuators for expanded capabilities
* **Cross-Platform Compatibility**: Designed to work with [list operating systems or platforms, if applicable] -->

-------------------------------------
## Hardware & Software Requirements

### Hardware:
* Raspberry Pi 4B
* Waveshare Stepper Motor HAT with two DRV8825 motor drivers
* MPU6050
* 2 NEMA 17 Stepper Motors with 1.8° per step, 1.5A
* 3D-printed parts for the robot's body

### Software:
*  Python 3.12
*  RPi.GPIO
*  smbus2
*  numpy
*  matplotlib

<!-- ## Getting Started
-------------------

1. **Cloning the Repository**: `git clone https://[your-repo-url].git`
2. **Setup Your Environment**:
	* Install [required software dependencies]
	* Configure your [microcontroller/board] as per the included `README.setup` file
3. **Upload and Test**: Follow `upload_instructions.md` for deploying the code to your robot -->

-------------------------------------
## Setup Steps:

1. **Hardware Assembly**
   - Ensure all parts are connected properly. Especially cables from MPU6050 to the Stepper Motor HAT may be loose
   - Ensure that the batteries have enough charge to power the robot. Low charge may cause strange behaviour

2. **Software Installation**
   - Clone this repository: `git clone https://github.com/OrangeObst/Balancing_Robot.git`
   - Install all required Python libraries by navigating to the projects root directory and running:
     ```bash
     pip install -r requirements.txt
     ```
     **Note:** Ensure you're in the project's root directory before executing the command.

3. **Calibration**
   - Stand the robot upright at the angle at which it is in equilibrium
   <!-- - Run the calibration script: `python calibration_script.py` -->
   - Run the mpu's calibration function (calibrate_sensor(t), t in seconds)
   - The MPU class has a function to set the digital lowpass filter and samplerate divisor according to the chosen time delay between measurements. (optimize_sample_settings(t), t in ms)
   - Adjust the PID values, which are at the top of the main.p file, to your liking. Higher values will result in more aggressive balancing attempts. Lower values will result in more subtle balancing attempts.
   **Note:** All settings, including PID values will be outsourced to  a config file in the future.


-------------------------------------
## Usage:

* **Power On**: The robot currently doesn't automatically start balancing. One has to connect via ssh to run the main.py file. Future updates will include an autostart feature.

* **Data logging**: As long as LOG_DATA is set to True the following values will be collected by the data_collector from data_collector.py:
    - The raw data output from the mpu6050 (ax, ay, az, gx, gy, gz)
    - Current angle, as well as the angle calculated from acceleration data and the angle calculated from the gyroscope data
    - Each indiviual PID term (P, I, D) and its output for both PID controllers
        - Angle PID outputs a speed between [-100, +100]
        - Position PID currently outputs a target angle, which is then fed into the angle PID as its setpoint
    - Motor step count

* **Troubleshooting**: 


------------------------------
## Project Status & Future Work

* **Current Issues**: There seem to be several issues at the moment..
    - If the robot tends to drift off into one direction the mpu's calibration may have been off
        - In theory the i-term of the angle PID should counteract this, but it seems to be too slow
    - The robot rarely starts in extreme oscillations but nearly always ends up in extreme oscillations, which are not tied to the PID constants.
    

* **Planned Enhancements**:
    + **Autostart Feature**: Implement a service to automatically start the balancing script on boot.
    
    + **Web Interface for Remote Monitoring and Control**: Develop a web interface to monitor the robot's status, adjust PID values, and perform other tasks remotely.

	+ **Outsource Control Logic**: Control algorithm shall be outsourced to another computer and communicate via  a network protocol (UDP, RTPS, Fast-DDS .. )

