# Planar Cable-Driven Robot

**Authors**: Arzaq Khan, Hamdan Raashid  
**Date**: December 14, 2024  

## Project Overview

This project involves the design and implementation of a **planar cable-driven robot** capable of continuously tracking and following objects in real time. The original goal was to create a robot that could pick up and drop off objects in a logistics setting using a camera for target identification and an electromagnet for object manipulation.

### Key Features:
- **Real-time object tracking** using a stationary camera and OpenCV.
- **Inverse kinematics model** for cable length calculations.
- **Motion control** to smoothly position the robot's end-effector.
- **Homography matrix** for transforming pixel coordinates to physical coordinates in the robot’s workspace.

While the robot can track and follow objects, the **pick-and-place functionality** (using an electromagnet) was not implemented due to technical constraints.

---
## Project Methodology

### 1. **Inverse Kinematics Model**
The robot’s motion is controlled by the lengths of cables connected to winches. The inverse kinematics model calculates the required cable lengths based on the desired position of the end-effector. 

#### Key Formula:
To determine the cable lengths for each motor:
- \( LA = \| P - A \| \)
- \( LB = \| Q - B \| \)
- \( LC = \| R - C \| \)
- \( LD = \| S - D \| \)

Where \( P, Q, R, S \) are the coordinates of the end-effector, and \( A, B, C, D \) are the pulley positions.

### 2. **Homography for Vision**
Since the camera is not positioned overhead, a homography matrix is used to map pixel coordinates to physical coordinates. This matrix is calculated by correlating known points in the physical world with their corresponding points in the image.

### 3. **Vision and Object Tracking**
Using OpenCV’s CSRT tracker, the robot tracks a user-selected object in the video feed and calculates its physical position in the workspace. If tracking fails, the user is prompted to reselect the target.


### Setup

Clone the repository:
   ```bash
   git clone https://github.com/yourusername/planar-cable-driven-robot.git
   cd planar-cable-driven-robot
```

### Working Directory
```
src/
|-- ev3-client/
|   |-- sample_client.py         # Client Code for Visual Tracking
|   |-- System_constT.py         # System Attributes and Functions
|   |-- System_InverseKinematics.py # Client Code for Inverse Kinematics
|-- vision-server/
|-- myenv/
|-- vision_server              # Code for Homography computation
