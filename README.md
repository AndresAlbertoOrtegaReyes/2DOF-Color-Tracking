# 2DOF Color Tracking

![MATLAB](https://img.shields.io/badge/MATLAB-R2023a-orange) ![Arduino](https://img.shields.io/badge/Hardware-Arduino_UNO-blue) ![Status](https://img.shields.io/badge/Status-Completed-success)

## 📖 Overview
This project implements a **Automatic Visual Servoing** system using a 2-DOF (Pan/Tilt) robotic platform. The system utilizes computer vision to track a specific object based on color and employs a **PID controller** to keep the target centered in the camera's field of view.

The project integrates high-level image processing in **MATLAB** with low-level actuator control on an **Arduino UNO**, communicating via a serial interface.

## ✨ Key Features
* **HSV Color Segmentation:** Robust object detection resistant to lighting variations.
* **Frequency Domain Filtering:** Implementation of **Fast Fourier Transform (FFT)** with a Hanning Window to reduce Gaussian noise and improve segmentation accuracy.
* **PID Control:** A tuned Proportional-Integral-Derivative controller ($K_p, K_i, K_d$) for smooth and accurate servo movement.
* **Telemetry:** Live plotting of error signals ($dx, dy$) and trajectory tracking.

## ⚙️ Hardware Architecture
The physical system consists of a custom 3D-printed structure designed to hold the camera and motors.

* **Microcontroller:** Arduino UNO (ATmega328P).
* **Actuators:** 2x Standard Servomotors (Axis X: Pin D9, Axis Y: Pin D10).
* **Sensor:** FHD Webcam (Downsampled to 640x480 for processing efficiency).
* **Power:** External 5V supply for servos, USB for logic/comms.

> **Note:** The mechanical design files (`.stl`) can be found in the `cad/` directory.

![Hardware Setup](DOF2_Artificial_Vision.jpg)

*Figure 1: Physical implementation of the 2-DOF Tracking Turret.*

## 🚀 Software Strategy

### 1. Computer Vision Pipeline (MATLAB)
The vision algorithm processes frames in the following order:
1.  **Acquisition:** Capture frame from webcam.
2.  **Noise Filtering:** * Application of FFT to move to the frequency domain.
    * Multiplication with a **Hanning Window** to attenuate high-frequency noise.
    * Inverse FFT to reconstruct the smoothed image.
3.  **Segmentation:** Conversion from RGB to **HSV space** and binary masking based on user-selected color thresholds.
4.  **Centroid Calculation:** Determination of the object's center $(x, y)$ and calculation of the error relative to the image center.

### 2. Control Loop
The error vector is sent to the control block:
$$u(t) = K_p e(t) + K_i \int_{0}^{t} e(\tau) d\tau + K_d \frac{de(t)}{dt}$$

* **Saturation Limits:** Applied to output signals to prevent servo overdrive.
* **Serial Communication:** MATLAB transmits error data; Arduino computes servo angles and executes movement.

## 📊 Results & Performance
The system was tested with dynamic object movement. The graphs below show the error minimization over time, demonstrating the stability of the PID tuning.

![PID Error Graph](Error_Artificial_Vision.jpg)

*Figure 2: Tracking error (in pixels) for X and Y axes over time.*
