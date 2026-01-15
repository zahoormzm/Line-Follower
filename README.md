# VectorDrive: Adaptive PID High Speed Line Follower

## Overview
VectorDrive is a high-frequency, embedded autonomous navigation system designed for high-speed line tracking on complex geometries. It leverages **direct ADC register manipulation** for ultra-low latency sensing and implements **voltage-compensated adaptive PID control** to ensure consistent kinematic performance regardless of battery discharge curves.

## Demo
![VectorDrive Demo](https://github.com/user-attachments/assets/c74c64aa-6dd4-45f4-a26e-4c65b108e6ee)

## Key Engineering Features
* **Voltage-Agnostic Velocity Control:** Implements a dynamic `VOLTAGE_MULTIPLIER` derived from real-time battery monitoring (`ADC_REF / CURRENT_BAT`). This scales PWM output to maintain constant motor RPM as battery voltage drops, eliminating the "slowdown" effect during long runs.
* **Heuristic Adaptive PID:** Features a non-linear control loop that dynamically boosts $K_p$ and $K_d$ gains by **1.5x** when `abs(error) > 10`. This allows for smooth oscillation-free straight-line driving while providing aggressive torque response during acute cornering.
* **Optimized ADC Latency:** Bypasses standard Arduino `analogRead` overhead by directly manipulating the `ADCSRA` registers (`sbi`/`cbi` macros), setting prescalers to minimize sensor integration time without sacrificing resolution.
* **Robust Failsafe Recovery:** Implements a "Last Known State" memory buffer. If the line is lost, the system utilizes the `lastError` polarity to execute a hard recovery turn in the direction of the track, rather than drifting aimlessly.

## System Architecture
The system operates on a closed-loop feedback mechanism governed by a custom PID controller.

### 1. Perception Layer (8-Channel Array)
The sensor array output is normalized using a dynamic thresholding map. The error is calculated using a weighted average of active sensors:

$$Error = \frac{\sum (Sensor_i \times Weight_i)}{\sum ActiveSensors}$$

* **Weights:** Symmetric distribution `[20, 8, 2, 1, -1, -2, -8, -20]` prioritizes center alignment while penalizing deviation exponentially.

### 2. Control Algorithm (PID)
The corrected motor output is derived from the standard PID formulation with the adaptive gain injection:

$$u(t) = K_p^{adaptive}e(t) + K_i\int e(t)dt + K_d^{adaptive}\frac{de}{dt}$$

### 3. Actuation Layer
* **Base Speed:** 110 PWM (Voltage Compensated)
* **Differential Drive:** The PID output is added/subtracted from the base speed to generate differential thrust.
* **Drivers:** Supports Dual H-Bridge configuration (AIN/BIN logic).

## Getting Started

### Hardware Requirements
* **Microcontroller:** ATmega328P based board (Arduino Nano/Uno)
* **Sensors:** 8-Channel IR Reflectance Array (Digital/Analog)
* **Driver:** TB6612FNG or L298N Dual Motor Driver
* **Power:** 2S/3S LiPo Battery (Voltage divider on ADC 3 required)

### Installation
1.  **Clone the repository:**
    ```bash
    git clone [https://github.com/yourusername/vectordrive.git](https://github.com/yourusername/vectordrive.git)
    ```
2.  **Pin Configuration:**
    Ensure your hardware matches the pin definitions in the macro section:
    ```cpp
    #define AIN1 4  // Left Motor Logic 1
    #define BIN1 6  // Right Motor Logic 1
    #define CALIBRATE_BTN 11
    ```
3.  **Calibration:**
    * **Preset:** Set `USE_PRESET_CALIBRATION = true` if you have hardcoded values.
    * **Manual:** Set to `false`. Hold the `CALIBRATE_BTN` on boot and sweep the robot across the line for 3 seconds.
