# Sensor Fusion of Visual Odometry and Observer

This project combines Visual Odometry with a High-Gain observer, using a Kalman filter for sensor fusion. The system has been created for the application of vehicle localisation, especially for patchy, or GNSS-denied environments. The work is detailed in the paper currently stored in [Soton ePrints](https://eprints.soton.ac.uk/503449/).

# Setup Requirements
- **MATLAB** (Created in R2025a). 
- **Computer Vision Toolbox**.
- **Symbolic Math Toolbox**.
- **Phased Array System Toolbox**. This is only for the functions **rotx/roty/rotz** (*because I was too lazy to change them...*).
- **YALMIP** and **MOSEK** installed, with **MOSEK** path added in **FigureCreation.m**. This is used to solve the high-gain LMI gain. There are default values if desired, or you can solve the LMI using another solver etc.
- Correct file structure is needed for using your own images and sensor data. Requirements are detailed in **ExampleFileStructure/FileStructureRequirements.md**. You **MUST** change the filepath in MATLAB before the code will function, so check the file structure document!
# Code Usage
The main file is **FigureCreation.m**. This is the file to use when you want to use the system. This file contains various self contained tests, which evaluate different parts of the system, corresponding the different blocks in the whole system diagram:

![Full System Overview.](/image/SystemOverview.svg "Full System Overview.")


Each of the following sections is in **FigureCreation.m**, and are separated by a double percentage sign. 
### The three following setup routines **MUST** be run. 

**"Initial Setup"** MUST be run first, regardless of systems being run.

**"Perform Visual Odometry"** finds the estimated path from the visual odometry. As this takes a long time, a workspace containing the visual outputs (for the default route) is provided as **"SkipVisualOdometryWorkspace.mat"**. Therefore, if you open the workspace, you do not need to run **"Perform Visual Odometry"**. **IMPORTANT:** If you are opening the workspace, open it before you run initial setup, otherwise it will overwrite your custom initial setup settings (Filepath, Route, etc.)!

**"Post VO Setup"** MUST be performed after either performing Visual Odometry, or importing the visual odometry outputs from the workspace.

### The rest of the systems can then be run independently, and in any order as needed.

There are some example output images in this repo's **image** folder. For example, see the last section of this readme.

**"High-Gain Observer Only"** validates the high-gain observer, by comparing the estimated to the actual orientation.

**"GNSS Only"** plots the RTK GNSS data.

**"Camera Only"** plots the orientation and velocity (scale) corrected camera estimate of the Visual Odometry.

**"Camera and Observer"** plots the observer-corrected Visual Odometry. This also contains the absolute position from the GNSS, which has been "estimated" through (input to) the Observer.

**"Camera, Observer and GNSS"** is the full system, as described in the paper.

**"Camera, Observer and GNSS, with GNSS Failure"** is the full system, where there is a GNSS outage, as the vehicle approaches the roundabout.

**"All Systems"** shows the comparison of all the different systems, all plotted on the same axis to show their differences.

# File Overview and Code Walkthrough
As mentioned before the main file is **FigureCreation.m**. This deals with setup of the systems, as well as the running of the visual odometry algorithm in **VisualOdometry.m**. The visual odometry algorithm has been adapted from this [Mathworks Example](https://uk.mathworks.com/help/vision/ug/monocular-visual-simultaneous-localization-and-mapping.html), which details the process nicely. 

The Kalman Filter, in **KalmanFilter.m**, is then used in its setup runthrough to filter the RTK GNSS, ready for input to the high-gain observer. 

The high-gain observer, in **HighGainObserver.m**, can then be used to estimate the vehicle's orientation. This specific high-gain observer is courtesy of Hichem Bessafa, from [his paper](https://ieeexplore.ieee.org/abstract/document/10156219)  in American Control Conference (ACC). It has been adatpted for use in our system. It uses helper files **HighGainObserver_method2.m**, **Phi.m**, **Proj.m**, **NormalizeAngle.m** and **AbsoluteAngleDeg.m**.

By running these files first, during setup, the systems can each be run independently, allowing the individual figure creaton.


# CARLA Usage
This example, as in the paper, uses CARLA to produce the image and sensor outputs. The example camera output is given in **ExampleFileStructure/000006.png**. There are a few example CARLA files in **ExampleCARLACode**, which can be used to save the required data, ready to be used by the system. **ExampleCARLACode/CameraTest30HzSlow2.py** was used to produce the results in the paper, so probably a good idea to start there!

# Example image
The proposed system with GNSS Failure:

![Full System with GNSS Failure.](/image/CameraObserverGNSSwithGNSSFailure.png "Full System with GNSS Failure.")
