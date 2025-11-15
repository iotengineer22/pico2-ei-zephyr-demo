# pico2-ei-zephyr-demo
This repository presents a Zephyr Demo with Edge Impulse and Raspberry Pi Pico2(W)

## Introduction
This repository showcases examples of gesture and voice recognition using external sensors with the **Raspberry Pi Pico2(W)**.

The projects are built with the Zephyr RTOS and leverage machine learning models deployed from Edge Impulse.

> **Disclaimer:** This is a personal project created for quick demonstration purposes. Please consider it as a reference, as the code may not be fully polished or production-ready.


## Structure
    .  
    ├── src                                             # Program and board files 
    │    ├── pico2w                                     # Raspberry Pi Pico2(W)
    │    │    ├── pico-imu_dataforwarder                # Collecting accel sensor data
    │    │    ├── pico-imu_inference                    # Gesture Recognition
    ├── LICENSE
    └── README.md


## Demo Videos

See the examples in action on YouTube:
*   **Gesture Datafowarder Demo:** [https://youtu.be/5CdPXB99Jo0](https://youtu.be/5CdPXB99Jo0)
*   **Gesture Recognition Demo:** [https://youtu.be/kFZd0z9Mo5U](https://youtu.be/kFZd0z9Mo5U)

## Overview

The core logic for each example is straightforward:

*   **Gesture Recognition:**
    1.  The application captures 2 seconds of data from a 3-axis accelerometer (X, Y, Z) at 104Hz.
    2.  It runs inference on the collected data to classify the gesture.
    3.  This process repeats in a continuous loop.

## How to Build and Run

To get this running on your own board, follow these steps:

1.  **Export Model from Edge Impulse:**
    *   Go to your project on [Edge Impulse](https://www.edgeimpulse.com/).
    *   Navigate to the **Deployment** tab.
    *   Select the **C++ library** and click **Build**. This will download a `.zip` file containing the inference SDK and your trained model.

2.  **Copy SDK to Project:**
    *   Unzip the downloaded file.
    *   Copy all the contents (the `edge-impulse-sdk/`, `model-parameters/`, and `tflite-model/` directories) into the `src` folder of this Zephyr project, overwriting the existing files.

3.  **Build and Flash:**
    *   Open your terminal, navigate to the project directory, and run the standard Zephyr build and flash commands for your board. For the Raspberry Pi Pico W, it would be:

    ```bash
    # Build the application
    west build -p -b rpi_pico2/rp2350a/m33/w

    # Flash it to the device
    (*Flash uf2 file.)
