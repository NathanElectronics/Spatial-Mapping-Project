# ToF Spatial Mapping Device

A lightweight, cost-effective 3D spatial mapping device designed for indoor exploration, using a Time-of-Flight (ToF) sensor as an alternative to traditional Light Detection and Ranging (LiDAR) systems. The system combines the MSP-EXP432E401Y microcontroller, VL53L1X ToF sensor, and a 28BYJ-48 stepper motor to generate layered y-z slices of the scanned environment, which are manually displaced along the x-axis to form a complete 3D representation.

---

## 📌 Author / Contact Information

**Nathan Chan**  
Email: *nathanchan.1738@gmail.com*  
GitHub: [github.com/NathanElectronics](https://github.com/NathanElectronics)

---

## 🐞 Bug Tracker

Please report bugs or enhancement requests at:  
[nathanchan.1738@gmail.com](nathanchan.1738@gmail.com)

---

## ⚠️ Known Issues

- Real-time plotting speed may vary based on PC specs and serial buffer performance.
- Limited range in strong ambient lighting conditions due to sensor saturation.
- Manual x-axis displacement requires user calibration between layers.

---

## 🛠️ Build Instructions

1. **Hardware Requirements:**
   - MSP-EXP432E401Y LaunchPad
   - VL53L1X Time-of-Flight sensor
   - 28BYJ-48 5V stepper motor
   - ULN2003 driver board
   - Power supply (e.g., USB or external regulated 5V)
   - Push buttons, onboard LEDs (preconfigured on LaunchPad)

2. **Software Setup:**
   - Install **Keil uVision** or compatible ARM compiler.
   - Clone this repository:
     ```bash
     git clone https://github.com/nathanchan/tof-spatial-mapper.git
     ```
   - Flash the provided firmware to the MSP432 via Keil or TI Code Composer Studio.
   - Ensure VL53L1X is connected via I2C and the UART pins are wired to the PC serial port.

---

## ▶️ Run Instructions

1. Power on the device — onboard firmware initializes LEDs and button input.
2. The VL53L1X sensor begins capturing distance readings by emitting pulses and processing reflected light using internal ADC and filtering.
3. Distance data is transmitted from sensor to MSP432 via **I2C**, and then from the MSP432 to the PC via **UART** at 18 MHz bus speed.
4. The PC reads UART data and computes y-z coordinates based on the stepper motor’s angular position.
5. After completing a layer, manually shift the x-axis for the next scan layer.
6. Use MATLAB or Python scripts to render a 3D spatial map from accumulated slices.

---

## ✅ Test Suite Instructions

Testing is currently manual and performed as follows:

- Validate stepper motor calibration by performing a full 180° sweep and checking scan spacing.
- Ensure UART values align with known distances using reference objects.
- Test the 3D plotting pipeline in MATLAB or Python:
  - Compare known wall distances or corners for visual accuracy.
  - Adjust trigonometric conversions if necessary.

Future work includes automated scanning and integration with motor encoders for x-axis stepping.

---

## 📎 License

This project is released under the MIT License. See `LICENSE` for details.
